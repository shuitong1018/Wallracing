#ifndef _ERROR_H_
#define _ERROR_H_

#include "zf_common_headfile.h"

// 暴露给外部（如 main.c 或 PID 控制文件）调用的核心函数
float Error_Calculate(float l1_n, float l2_n, float l3_n, float l4_n);
float Get_Last_Error(void);
void Reset_Error_Filter(void);

#endif