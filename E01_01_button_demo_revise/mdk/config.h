#ifndef __CONFIG_H
#define __CONFIG_H

#include "zf_common_typedef.h"

//==============================================================================
// 传感器引脚定义（使用逐飞库的枚举）
//==============================================================================
#define SENSOR_L1_ADC   ADC_CH0_P10
#define SENSOR_L2_ADC   ADC_CH1_P11
#define SENSOR_L3_ADC   ADC_CH2_P12
#define SENSOR_L4_ADC   ADC_CH3_P13

// 传感器引脚（用于GPIO）
#define SENSOR_L1_PIN    IO_P10
#define SENSOR_L2_PIN    IO_P11
#define SENSOR_L3_PIN    IO_P12
#define SENSOR_L4_PIN    IO_P13

// 电机引脚定义（使用逐飞库的枚举）
#define LEFT_MOTOR_PWM   PWMA_CH1P_P20
#define RIGHT_MOTOR_PWM  PWMA_CH2P_P22
#define LEFT_MOTOR_DIR   IO_P21
#define RIGHT_MOTOR_DIR  IO_P23

//==============================================================================
// 阈值定义
//==============================================================================
#define THRESHOLD_BLACK   100
#define THRESHOLD_EDGE    500
#define THRESHOLD_CROSS   800

//==============================================================================
// 速度参数（注意PWM_DUTY_MAX=10000）
//==============================================================================
#define BASE_SPEED_NORMAL    2000    // 20%占空比
#define BASE_SPEED_TURN      1500    // 15%占空比
#define BASE_SPEED_FAST      3000    // 30%占空比
#define MOTOR_SPEED_MAX      5000    // 50%占空比

//==============================================================================
// 控制周期
//==============================================================================
#define CONTROL_CYCLE_MS     20
#define FILTER_WINDOW_SIZE   5

#endif