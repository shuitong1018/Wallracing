#ifndef __SENSOR_H
#define __SENSOR_H

#include "config.h"

// 传感器原始值
extern int L1, L2, L3, L4;

// 函数声明
void Sensor_Init(void);
void Sensor_Read(void);

#endif