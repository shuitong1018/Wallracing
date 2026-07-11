#include "zf_common_headfile.h" 
#include "sensor.h"

// ================= 1. 归一化参数配置区 =================

#define L1_MAX  140
#define L1_MIN  10
#define L2_MAX  106
#define L2_MIN  15
#define L3_MAX  109
#define L3_MIN  10
#define L4_MAX  140
#define L4_MIN  10


// 全局变量声明（供外部文件调用计算误差）
float L1_norm, L2_norm, L3_norm, L4_norm;

// ================= 1.5 硬件引脚与初始化配置 =================
#define CHANNEL_NUMBER          (4)
// 电感传感器引脚配置 (请确认是否与你的实际接线一致)
#define ADC_CHANNEL1            (ADC_CH8_P00)  // L1接 P00 引脚
#define ADC_CHANNEL2            (ADC_CH9_P01)  // L2接 P01 引脚
#define ADC_CHANNEL3            (ADC_CH13_P05) // L3接 P05 引脚
#define ADC_CHANNEL4            (ADC_CH14_P06) // L4接 P06 引脚

// 定义通道数组供读取函数使用
adc_channel_enum channel_list[CHANNEL_NUMBER] =
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4
};

// 传感器硬件初始化函数
void Sensor_Init(void)
{
    // 将这四个引脚初始化为 8位 ADC 模式
    adc_init(ADC_CHANNEL1, ADC_8BIT);
    adc_init(ADC_CHANNEL2, ADC_8BIT);
    adc_init(ADC_CHANNEL3, ADC_8BIT);
    adc_init(ADC_CHANNEL4, ADC_8BIT);
}

// 传感器监控打印函数 (供 main.c 循环调用)
void Sensor_Monitor(void)
{
    // 直接打印刚刚归一化好的 0~128 的数据
    printf("Norm -> L1:%.1f, L2:%.1f, L3:%.1f, L4:%.1f\r\n", L1_norm, L2_norm, L3_norm, L4_norm);
}

// ================= 2. 核心归一化算法 =================
// 严格遵守官方公式：将ADC值映射到 0-128 范围的高效整型运算
int16 Normalize_ADC(int16 value, int16 Max, int16 Min)
{
    int16 buf;
    
    // 容错：如果宏定义填反了大小值，自动交换
    if (Min > Max) {
        buf = Min;
        Min = Max;
        Max = buf;
    }
    
    // 限幅保护：防止跑出边界导致负数或者超过128（这是防止车子一碰线就满偏锁死的关键）
    if (value <= Min) return 0;
    if (value >= Max) return 128;
    
    // 核心公式：利用左移7位 (<< 7) 代替乘以128，极大提升单片机运算速度，避免浮点灾难
    return ((value - Min) << 7) / (Max - Min);
}

// ================= 3. 数据读取与执行 =================
void Sensor_Read_Normalized(void)
{
    // 1. 获取 3 次平均滤波后的底层 8 位原始值 (在 0-255 范围内变化)
    uint16 raw1 = adc_mean_filter_convert(channel_list[0], 3);
    uint16 raw2 = adc_mean_filter_convert(channel_list[1], 3);
    uint16 raw3 = adc_mean_filter_convert(channel_list[2], 3);
    uint16 raw4 = adc_mean_filter_convert(channel_list[3], 3);

    // 2. 传入各自专属的 Max 和 Min 进行归一化，得到 0~128 的整数，再存入 float 变量供差比和使用
    L1_norm = (float)Normalize_ADC((int16)raw1, L1_MAX, L1_MIN);
    L2_norm = (float)Normalize_ADC((int16)raw2, L2_MAX, L2_MIN);
    L3_norm = (float)Normalize_ADC((int16)raw3, L3_MAX, L3_MIN);
    L4_norm = (float)Normalize_ADC((int16)raw4, L4_MAX, L4_MIN);
}