#include "path.h"
#include <stdlib.h>

//==============================================================================
// 静态变量
//==============================================================================
static PathType_t current_path;
static int path_counter;

//==============================================================================
// 路径初始化
//==============================================================================
void Path_Init(void)
{
    current_path = PATH_STRAIGHT;
    path_counter = 0;
}

//==============================================================================
// 路径类型检测
//==============================================================================
PathType_t Path_Detect(int L1, int L2, int L3, int L4)
{
    int edge_diff;
    int middle_sum;
    
    // 检测平地
    if(L1 < THRESHOLD_BLACK && L2 < THRESHOLD_BLACK && 
       L3 < THRESHOLD_BLACK && L4 < THRESHOLD_BLACK) {
        return PATH_FLAT;
    }
    
    // 检测十字路口
    if(L1 > THRESHOLD_CROSS && L2 > THRESHOLD_CROSS && 
       L3 > THRESHOLD_CROSS && L4 > THRESHOLD_CROSS) {
        return PATH_CROSSROAD;
    }
    
    // 检测直角弯
    edge_diff = abs(L1 - L4);
    middle_sum = L2 + L3;
    
    if(edge_diff > THRESHOLD_EDGE && middle_sum < THRESHOLD_BLACK * 2) {
        if(L1 > L4) {
            return PATH_LEFT_ANGLE;
        } else {
            return PATH_RIGHT_ANGLE;
        }
    }
    
    return PATH_STRAIGHT;
}

//==============================================================================
// 获取路径方向（偏离中心程度）
//==============================================================================
PathDirection_t Path_Get_Direction(int L1, int L2, int L3, int L4)
{
    int sum;
    int weighted;
    int position;
    
    sum = L1 + L2 + L3 + L4;
    if(sum == 0) return DIR_STRAIGHT;
    
    weighted = L1 * 1 + L2 * 2 + L3 * 3 + L4 * 4;
    position = (weighted * 100) / sum;
    
    if(position < 225) {
        return DIR_LEFT;
    } else if(position > 275) {
        return DIR_RIGHT;
    } else {
        return DIR_STRAIGHT;
    }
}

//==============================================================================
// 获取控制动作
//==============================================================================
PathAction_t Path_Get_Action(PathType_t path, int L1, int L2, int L3, int L4)
{
    PathAction_t action = ACTION_TRACKING;
    
    (void)L2;    // 消除未使用参数警告
    (void)L3;
    
    switch(path)
    {
        case PATH_STRAIGHT:
            action = ACTION_TRACKING;
            break;
            
        case PATH_LEFT_ANGLE:
        case PATH_RIGHT_ANGLE:
            path_counter++;
            if(path_counter < 20) {
                if(L1 > L4) {
                    action = ACTION_TURN_LEFT_HARD;
                } else {
                    action = ACTION_TURN_RIGHT_HARD;
                }
            } else {
                path_counter = 0;
                action = ACTION_TRACKING;
            }
            break;
            
        case PATH_CIRCULAR:
            if(L1 > L4) {
                action = ACTION_TURN_LEFT_SMOOTH;
            } else {
                action = ACTION_TURN_RIGHT_SMOOTH;
            }
            break;
            
        case PATH_CROSSROAD:
            action = ACTION_TRACKING;
            break;
            
        case PATH_ZIGZAG:
            action = ACTION_TRACKING_FAST;
            break;
            
        case PATH_ROUNDABOUT:
            action = ACTION_ROUNDABOUT;
            break;
            
        case PATH_FLAT:
        default:
            action = ACTION_TRACKING;
            break;
    }
    
    return action;
}