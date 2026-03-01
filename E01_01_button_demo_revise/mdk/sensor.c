#include "config.h"
#include "zf_driver_adc.h"
#include "zf_driver_delay.h"

int L1, L2, L3, L4;

void Sensor_Init(void)
{
    // 使用正确的ADC通道枚举和分辨率
    adc_init(ADC_CH0_P10, ADC_12BIT);  // 初始化P10为ADC，12位分辨率
    adc_init(ADC_CH1_P11, ADC_12BIT);  // 初始化P11为ADC，12位分辨率
    adc_init(ADC_CH2_P12, ADC_12BIT);  // 初始化P12为ADC，12位分辨率
    adc_init(ADC_CH3_P13, ADC_12BIT);  // 初始化P13为ADC，12位分辨率
    
    system_delay_ms(10);
}

void Sensor_Read(void)
{
    // 读取ADC值
    L1 = adc_convert(ADC_CH0_P10);
    L2 = adc_convert(ADC_CH1_P11);
    L3 = adc_convert(ADC_CH2_P12);
    L4 = adc_convert(ADC_CH3_P13);
}