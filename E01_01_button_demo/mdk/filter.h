#ifndef __FILTER_H
#define __FILTER_H

#include "config.h"

// 滤波后的值
extern int L1_filt, L2_filt, L3_filt, L4_filt;

// 函数声明
void Filter_Init(void);
void Filter_Process(int raw_L1, int raw_L2, int raw_L3, int raw_L4);

#endif