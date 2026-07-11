#ifndef __MOTOR_H
#define __MOTOR_H

#include "config.h"
#include "zf_common_typedef.h"

// º¯ÊýÉùÃ÷
void Motor_Init(void);
void Motor_SetLeft(int speed);
void Motor_SetRight(int speed);
void Motor_Control(int left_speed, int right_speed);
void Motor_Stop(void);

#endif