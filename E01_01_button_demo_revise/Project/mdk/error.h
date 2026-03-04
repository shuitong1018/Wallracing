
#ifndef _ERROR_H_
#define _ERROR_H_

#include "zf_common_headfile.h"

/**
 * @brief   计算智能车路径误差
 * @param   L1  电感1的值
 * @param   L2  电感2的值
 * @param   L3  电感3的值
 * @param   L4  电感4的值
 * @return  计算得到的误差值
 */
float Error_Calculate(int16 L1, int16 L2, int16 L3, int16 L4);

/**
 * @brief   获取当前误差值
 * @return  上一次计算的误差值
 */
float Get_Last_Error(void);

/**
 * @brief   重置误差滤波器
 */
void Reset_Error_Filter(void);

#endif