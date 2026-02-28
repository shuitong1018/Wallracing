#include "sensor.h"

int L1, L2, L3, L4;

// 读取指定ADC通道
u16 Get_ADC_Channel(u8 channel) {
    // 选择通道并启动转换
    ADC_CONTR = ADC_POWER | ADC_START | (channel << 1);
    
    _nop_(); _nop_();
    
    // 等待转换完成
    while(!(ADC_CONTR & ADC_FLAG));
    
    // 读取结果（10位）
    u16 value = (ADC_RES << 8) | ADC_RESL;
    
    // 清除标志位
    ADC_CONTR &= ~ADC_FLAG;
    
    return value;
}

void Sensor_Init(void) {
    // 1. 设置P1.0-P1.3为高阻输入
    P1M1 |= 0x0F;   // M1=1
    P1M0 &= ~0x0F;  // M0=0
    
    // 2. 配置ADC（右对齐，时钟分频）
    ADCCFG = 0x20;
    
    // 3. 使能ADC电源
    ADC_CONTR = ADC_POWER;
    
    // 4. 等待电源稳定
    for(int i = 0; i < 1000; i++);
    
    // 5. 进行一次空读
    Get_ADC_Channel(0);
}

void Sensor_Read(void) {
    // get_sensor() -> L1 L2 L3 L4
    L1 = Get_ADC_Channel(SENSOR_L1_CH);
    L2 = Get_ADC_Channel(SENSOR_L2_CH);
    L3 = Get_ADC_Channel(SENSOR_L3_CH);
    L4 = Get_ADC_Channel(SENSOR_L4_CH);
}