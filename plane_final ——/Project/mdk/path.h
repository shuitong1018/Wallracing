#ifndef _PATH_H_
#define _PATH_H_

#include "zf_common_headfile.h" // 使用逐飞总头文件替代 stdint.h

// 枚举定义
typedef enum {
    PATH_NORMAL,
    PATH_CROSS,
    PATH_ROUNDABOUT
} path_state_enum;

typedef enum {
    ROUND_NONE,
    ROUND_FOUND,
	ROUND_DELAY,
    ROUND_IN,
    ROUND_MID,
    ROUND_OUT
} round_state_enum;

// 外部变量声明
extern path_state_enum current_path_type;
extern int8 round_direction; // 注意：使用 int8 而非 int8_t

// 函数声明必须带分号
void Path_Identify(void);
void Path_Reset(void);

#endif