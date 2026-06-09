#ifndef __HARDWARE_H
#define __HARDWARE_H

#include "config.h"

// 传感器值
extern volatile u16 L1, L2, L3, L4;

// 初始化函数
void System_Init(void);
void ADC_Init(void);
void PWM_Init(void);

// 传感器读取
void Get_Sensor_Raw(void);

// 电机控制
void Set_PWM(int left_speed, int left_dir, int right_speed, int right_dir);

// 延时
void delay_ms(u16 ms);

#endif