#ifndef __PATH_H
#define __PATH_H

#include "zf_common_typedef.h"
#include "config.h"

// 路径类型枚举
typedef enum {
    PATH_STRAIGHT = 0,      // 直道
    PATH_LEFT_ANGLE,        // 左直角
    PATH_RIGHT_ANGLE,       // 右直角
    PATH_CIRCULAR,          // 圆形弯道
    PATH_CROSSROAD,         // 十字路口
    PATH_ZIGZAG,            // 折线
    PATH_ROUNDABOUT,        // 环岛
    PATH_FLAT,              // 平地
    PATH_UNKNOWN            // 未知
} PathType_t;

// 方向类型枚举
typedef enum {
    DIR_LEFT = 0,
    DIR_RIGHT,
    DIR_STRAIGHT,
    DIR_UNKNOWN
} PathDirection_t;

// 动作类型枚举
typedef enum {
    ACTION_TRACKING = 0,        // 循迹
    ACTION_TRACKING_FAST,       // 快速循迹
    ACTION_TURN_LEFT_SMOOTH,    // 平滑左转
    ACTION_TURN_RIGHT_SMOOTH,   // 平滑右转
    ACTION_TURN_LEFT_HARD,      // 急左转
    ACTION_TURN_RIGHT_HARD,     // 急右转
    ACTION_ROUNDABOUT,          // 环岛
    ACTION_STOP                 // 停止
} PathAction_t;

// 函数声明
void Path_Init(void);
PathType_t Path_Detect(int L1, int L2, int L3, int L4);
PathDirection_t Path_Get_Direction(int L1, int L2, int L3, int L4);
PathAction_t Path_Get_Action(PathType_t path, int L1, int L2, int L3, int L4);

#endif