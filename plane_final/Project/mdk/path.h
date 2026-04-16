#ifndef __PATH_H
#define __PATH_H

#include "zf_common_headfile.h"

//==============================================================================
// 路径类型枚举（去掉UNKNOWN）
//==============================================================================
typedef enum {
    PATH_STRAIGHT = 0,      // 直道
    PATH_LEFT_ANGLE,        // 左直角
    PATH_RIGHT_ANGLE,       // 右直角
    PATH_CIRCULAR,          // 圆形弯道
    PATH_CROSSROAD,         // 十字路口
    PATH_ZIGZAG,            // 折线
    PATH_ROUNDABOUT,        // 环岛
    PATH_FLAT,              // 平地（丢线）
    PATH_TYPE_COUNT
} PathType_t;

//==============================================================================
// 路径方向枚举
//==============================================================================
typedef enum {
    DIR_LEFT = -1,
    DIR_STRAIGHT = 0,
    DIR_RIGHT = 1
} PathDirection_t;

//==============================================================================
// 控制动作枚举
//==============================================================================
typedef enum {
    ACTION_TRACKING = 0,        // 正常循迹
    ACTION_TURN_LEFT_HARD,      // 左急转
    ACTION_TURN_RIGHT_HARD,     // 右急转
    ACTION_TURN_LEFT_SMOOTH,    // 左平滑转弯
    ACTION_TURN_RIGHT_SMOOTH,   // 右平滑转弯
    ACTION_TRACKING_FAST,       // 快速循迹
    ACTION_ROUNDABOUT,          // 环岛模式
    ACTION_STOP,                // 停止
    ACTION_FORWARD_STRAIGHT,    // 直行（不循迹）
    ACTION_COUNT
} PathAction_t;

//==============================================================================
// 函数声明
//==============================================================================
void Path_Init(void);
PathType_t Path_Detect(float L1, float L2, float L3, float L4);
PathDirection_t Path_Get_Direction(float L1, float L2, float L3, float L4);
PathAction_t Path_Get_Action(PathType_t path, float L1, float L2, float L3, float L4);
const char* Path_GetName(PathType_t path);
const char* Action_GetName(PathAction_t action);
PathType_t Path_GetCurrent(void);
int Path_NeedSlowDown(PathType_t path);
float Path_GetSpeedFactor(PathType_t path);
void Path_Reset(void);
void Path_Debug_Print(float L1, float L2, float L3, float L4);

#endif //