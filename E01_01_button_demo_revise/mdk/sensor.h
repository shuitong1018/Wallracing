#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "zf_common_headfile.h"

// 声明全局变量
extern float L1_norm, L2_norm, L3_norm, L4_norm;

// 声明对外开放的函数
void Sensor_Init(void);
void Sensor_Read_Normalized(void);
void Sensor_Monitor(void); 

#endif