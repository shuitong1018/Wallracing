#ifndef __SENSOR_H
#define __SENSOR_H

#include "zf_common_typedef.h"  // 包含基础类型定义
#include "zf_driver_gpio.h"      // GPIO驱动
#include "zf_driver_adc.h"       // ADC驱动
#include "config.h"

extern int L1, L2, L3, L4;

void Sensor_Init(void);
void Sensor_Read(void);

#endif