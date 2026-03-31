#include "zf_common_headfile.h" 
#include "sensor.h"

// ================= 1. 归一化参数配置区 =================
// 步骤 A：填入你在 0-100 量程下测出的老数据（请务必替换为你实际测量的真实数值）
#define L1_OLD_MAX  50
#define L1_OLD_MIN  9.2
#define L2_OLD_MAX  50
#define L2_OLD_MIN  15.4
#define L3_OLD_MAX  50
#define L3_OLD_MIN  7.6
#define L4_OLD_MAX  50
#define L4_OLD_MIN  5.3


// 步骤 B：编译器自动换算，还原为 0-255 硬件底层量程的近似值 (乘255除100)
#define L1_RAW_MAX  (L1_OLD_MAX * 255 / 100)
#define L1_RAW_MIN  (L1_OLD_MIN * 255 / 100)
#define L2_RAW_MAX  (L2_OLD_MAX * 255 / 100)
#define L2_RAW_MIN  (L2_OLD_MIN * 255 / 100)
#define L3_RAW_MAX  (L3_OLD_MAX * 255 / 100)
#define L3_RAW_MIN  (L3_OLD_MIN * 255 / 100)
#define L4_RAW_MAX  (L4_OLD_MAX * 255 / 100)
#define L4_RAW_MIN  (L4_OLD_MIN * 255 / 100)

// 步骤 C：严格套用官方算法，最大值扩大1.05倍(5%)，最小值除以2，留出充足的动态容错裕量
#define L1_MAX  (int16)(L1_RAW_MAX * 1.05f)
#define L1_MIN  (int16)(L1_RAW_MIN / 2)
#define L2_MAX  (int16)(L2_RAW_MAX * 1.05f)
#define L2_MIN  (int16)(L2_RAW_MIN / 2)
#define L3_MAX  (int16)(L3_RAW_MAX * 1.05f)
#define L3_MIN  (int16)(L3_RAW_MIN / 2)
#define L4_MAX  (int16)(L4_RAW_MAX * 1.05f)
#define L4_MIN  (int16)(L4_RAW_MIN / 2)


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
    // 1. 获取 10 次平均滤波后的底层 8 位原始值 (在 0-255 范围内变化)
    uint16 raw1 = adc_mean_filter_convert(channel_list[0], 10);
    uint16 raw2 = adc_mean_filter_convert(channel_list[1], 10);
    uint16 raw3 = adc_mean_filter_convert(channel_list[2], 10);
    uint16 raw4 = adc_mean_filter_convert(channel_list[3], 10);

    // 2. 传入各自专属的 Max 和 Min 进行归一化，得到 0~128 的整数，再存入 float 变量供差比和使用
    L1_norm = (float)Normalize_ADC((int16)raw1, L1_MAX, L1_MIN);
    L2_norm = (float)Normalize_ADC((int16)raw2, L2_MAX, L2_MIN);
    L3_norm = (float)Normalize_ADC((int16)raw3, L3_MAX, L3_MIN);
    L4_norm = (float)Normalize_ADC((int16)raw4, L4_MAX, L4_MIN);
}