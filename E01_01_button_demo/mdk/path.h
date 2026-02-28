#ifndef __PATH_H
#define __PATH_H

#include "config.h"

typedef enum {
    PATH_FLAT = 0,        // 平地（无黑线）
    PATH_RIGHT_ANGLE = 1, // 直角弯
    PATH_STRAIGHT = 2     // 直道
} PathType_t;

// 函数声明
void Path_Init(void);
PathType_t Path_Detect(int L1, int L2, int L3, int L4);

#endif