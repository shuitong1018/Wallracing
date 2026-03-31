#include "error.h"
#include "sensor.h"            // 必须包含 sensor.h 才能获取到 extern 的归一化变量
#include "zf_common_headfile.h" // 逐飞库主头文件

// ================== 1. 差比和算法参数配置 ==================
// 这里的系数决定了你的车过弯有多“丝滑”，后续调参主要调这里
#define COEFF_A  1.0f   // L1和L4 (外侧/前侧电感) 的差值系数：决定直线寻迹的回归速度
#define COEFF_B  1.0f   // L2和L3 (内侧/后侧电感) 的差值系数：决定弯道内切的紧凑程度
#define COEFF_C  1.0f   // L2和L3差值的绝对值系数：用于分母补偿，防止分母过小导致误差突变

// ================== 2. 误差限幅与滤波配置 ==================
#define ERROR_MAX   100.0f   // 舵机打角的最大误差输出限幅
#define ERROR_MIN  -100.0f   // 舵机打角的最小误差输出限幅

#define FILTER_WINDOW_SIZE  5    // 滑动窗口大小 (去极值平均滤波)

// ================== 3. 静态局部变量 (C251规范) ==================
static float error_buffer[FILTER_WINDOW_SIZE];
static unsigned char buffer_index = 0;
static unsigned char buffer_full = 0;

// ================== 4. 辅助数学函数 ==================
// 自定义绝对值函数 (避免引入额外的 math.h 库导致单片机 Flash 占用增加)
float my_abs(float x)
{
    if(x < 0) return -x;
    return x;
}

// ================== 5. 去极值平均滤波算法 ==================
static float remove_extremes_average_filter(float new_value) {
    unsigned char i;
    float sum;
    float max_value, min_value;
    unsigned char valid_count;
    
    // 将新算出的误差存入环形缓冲区
    error_buffer[buffer_index] = new_value;
    buffer_index++;
    
    // 缓冲区游标循环
    if(buffer_index >= FILTER_WINDOW_SIZE) {
        buffer_index = 0;
        buffer_full = 1;  
    }
    
    // 确定当前可参与计算的有效数据个数
    if(buffer_full) {
        valid_count = FILTER_WINDOW_SIZE;
    } else {
        valid_count = buffer_index;
    }
    
    // 刚开机数据不足3个时，无法去极值，直接算普通平均
    if(valid_count < 3) {
        sum = 0.0f;
        for(i = 0; i < valid_count; i++) {
            sum += error_buffer[i];
        }
        return sum / valid_count;
    }
    
    // 寻找当前窗口内的最大值和最小值
    max_value = error_buffer[0];
    min_value = error_buffer[0];
    for(i = 1; i < valid_count; i++) {
        if(error_buffer[i] > max_value) {
            max_value = error_buffer[i];
        }
        if(error_buffer[i] < min_value) {
            min_value = error_buffer[i];
        }
    }
    
    // 累加所有值
    sum = 0.0f;
    for(i = 0; i < valid_count; i++) {
        sum += error_buffer[i];
    }
    
    // 减去一个最大值和一个最小值，剔除突变噪点
    sum = sum - max_value - min_value;
    
    // 返回去极值后的平均偏差
    return sum / (valid_count - 2);
}

// ================== 6. 核心算法：计算赛道偏差 ==================
float Error_Calculate(float l1_n, float l2_n, float l3_n, float l4_n) {
    float numerator;             // 公式分子
    float denominator;           // 公式分母
    float raw_error;             // 原始误差
    float L1, L2, L3, L4;        // 比例化后的电感值
    
    // 【关键对接步】：将 sensor.c 传过来的 0~128 的整数范围，等比例压缩到 0.0~1.0 的小数区间
    // 这是差比和算法的数学基础，保证各个电感的权重统一
    L1 = l1_n / 128.0f;
    L2 = l2_n / 128.0f;
    L3 = l3_n / 128.0f;
    L4 = l4_n / 128.0f;

    // 根据官方公式计算分子：A*(L1-L4) + B*(L2-L3)
    numerator = COEFF_A * (L1 - L4) + COEFF_B * (L2 - L3);
    
    // 根据官方公式计算分母：A*(L1+L4) + C*|L2-L3|
    denominator = COEFF_A * (L1 + L4) + COEFF_C * my_abs(L2 - L3);
    
    // 除零保护：如果车飞出赛道，电感全为0，分母极小，防止引发单片机硬件除零异常导致死机重启
    if(denominator < 0.001f) {
        raw_error = 0.0f; // 丢失赛道时，可以根据实际情况改为输出上一帧的误差 (或者保持直行)
    } else {
        // 算出原始偏差，并放大100倍，方便后续 PID 整数计算和观察
        raw_error = (numerator * 100.0f) / denominator;
    }
    
    // 绝对安全限幅：防止偏差过大导致舵机打死卡住齿轮
    if(raw_error > ERROR_MAX) {
        raw_error = ERROR_MAX;
    }
    if(raw_error < ERROR_MIN) {
        raw_error = ERROR_MIN;
    }
    
    // 经过滑动窗口滤波后，输出最终平滑的误差给舵机 PID 环
    return remove_extremes_average_filter(raw_error);
}

// 获取最近一次的误差值（如果需要在其他地方读取但不触发重新计算时使用）
float Get_Last_Error(void) {
    unsigned char last_index;
    if(buffer_index == 0) {
        last_index = buffer_full ? (FILTER_WINDOW_SIZE - 1) : 0;
    } else {
        last_index = buffer_index - 1;
    }
    return error_buffer[last_index];
}

// 冲出赛道或发生碰撞后，重置滤波器状态
void Reset_Error_Filter(void) {
    unsigned char i;
    for(i = 0; i < FILTER_WINDOW_SIZE; i++) {
        error_buffer[i] = 0.0f;
    }
    buffer_index = 0;
    buffer_full = 0;
}