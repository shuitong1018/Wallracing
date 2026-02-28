#ifndef __CONFIG_H
#define __CONFIG_H

#include "STC32G.h"

// ==================== 传感器引脚定义 ====================
// 使用P1.0-P1.3作为ADC输入
#define SENSOR_L1_CH  0  // P1.0
#define SENSOR_L2_CH  1  // P1.1
#define SENSOR_L3_CH  2  // P1.2
#define SENSOR_L4_CH  3  // P1.3

// ==================== 电机引脚定义 ====================
// PWM输出引脚
#define LEFT_MOTOR_PWM_PIN  PWM1P   // P2.0
#define RIGHT_MOTOR_PWM_PIN PWM2P   // P2.1

// 方向控制引脚
#define LEFT_MOTOR_DIR_PIN  P20
#define RIGHT_MOTOR_DIR_PIN P21

// ==================== 系统参数 ====================
#define FILTER_WINDOW_SIZE  5    // 滤波窗口大小
#define BASE_SPEED_NORMAL   150  // 正常速度
#define BASE_SPEED_TURN     80   // 转弯速度
#define BASE_SPEED_FLAT     180  // 平地速度
#define CONTROL_CYCLE_MS    20   // 控制周期(ms)

// ==================== 误差参数 ====================
#define ERROR_SCALE_FACTOR  10   // 误差放大系数
#define ERROR_MAX           100  // 最大误差
#define ERROR_MIN           -100 // 最小误差

// ==================== ADC寄存器位定义 ====================
#define ADC_POWER   0x80
#define ADC_START   0x40
#define ADC_FLAG    0x20

#endif