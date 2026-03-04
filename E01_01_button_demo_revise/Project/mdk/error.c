#include "zf_common_headfile.h"        // 逐飞库主头文件 

// 定义系数（需要根据实际调试确定）
#define COEFF_A  1.0f   // L1-L4 的系数（左右电感差）
#define COEFF_B  0.5f   // L2-L3 的系数（前后电感差）  
#define COEFF_C  1.0f   // 分母中的系数（L2-L3的绝对值系数）

// 误差限幅值
#define ERROR_MAX   100.0f   // 最大误差
#define ERROR_MIN   -100.0f  // 最小误差

// 电感归一化处理
#define NORMALIZE_ENABLE     1   // 1:启用归一化, 0:禁用归一化
#define NORMALIZE_MAX        4096 // ADC最大值（12位ADC）

// 滤波器参数
#define FILTER_ALPHA         0.3f // 一阶低通滤波系数

static float last_error = 0.0f;   // 上一次的误差值（用于滤波）

float my_abs(float x)
{
    if(x < 0) return -x;
    return x;
}

/**
 * @brief   电感值归一化处理
 * @param   value   原始ADC值
 * @return  归一化后的值（0-1.0）
 */
static float normalize_sensor(int16 value) {
#if NORMALIZE_ENABLE
    if(value < 0) value = 0;
    if(value > NORMALIZE_MAX) value = NORMALIZE_MAX;
    return (float)value / NORMALIZE_MAX;
#else
    return (float)value;
#endif
}

/**
 * @brief   一阶低通滤波器
 * @param   new_value   新值
 * @param   last_value  上一次的值
 * @return  滤波后的值
 */
static float low_pass_filter(float new_value, float last_value) {
    return FILTER_ALPHA * new_value + (1.0f - FILTER_ALPHA) * last_value;
}

/**
 * @brief   计算智能车路径误差
 * @param   L1  电感1的值（左侧前电感）
 * @param   L2  电感2的值（左侧后电感）
 * @param   L3  电感3的值（右侧后电感）
 * @param   L4  电感4的值（右侧前电感）
 * @return  计算得到的误差值（已限幅和滤波）
 */
float Error_Calculate(int16 L1, int16 L2, int16 L3, int16 L4) {
    float fL1, fL2, fL3, fL4;   // 归一化后的电感值
    float numerator;             // 分子
    float denominator;           // 分母
    float error;                 // 误差值
    
    // 1. 对原始ADC值进行归一化处理
    fL1 = normalize_sensor(L1);
    fL2 = normalize_sensor(L2);
    fL3 = normalize_sensor(L3);
    fL4 = normalize_sensor(L4);
    
    // 2. 计算分子
    numerator = COEFF_A * (fL1 - fL4) + COEFF_B * (fL2 - fL3);
    
    // 3. 计算分母
    denominator = COEFF_A * (fL1 + fL4) + COEFF_C * my_abs(fL2 - fL3);
    
    // 4. 防止除零
    if(denominator < 0.001f) {  // 使用小值判断避免浮点精度问题
        error = 0.0f;
    } else {
        // 5. 计算误差（乘以100放大误差值）
        error = (numerator / denominator) * 100.0f;
    }
    
    // 6. 限幅
    if(error > ERROR_MAX) {
        error = ERROR_MAX;
    }
    if(error < ERROR_MIN) {
        error = ERROR_MIN;
    }
    
    // 7. 一阶低通滤波
    error = low_pass_filter(error, last_error);
    last_error = error;
    
    return error;
}

/**
 * @brief   获取当前误差值（不重新计算）
 * @return  上一次计算的误差值
 */
float Get_Last_Error(void) {
    return last_error;
}

/**
 * @brief   重置误差滤波器
 */
void Reset_Error_Filter(void) {
    last_error = 0.0f;
}