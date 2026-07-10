#include "path.h"
#include "sensor.h"
#include "error.h"

path_state_enum current_path_type = PATH_NORMAL;
round_state_enum round_step = ROUND_NONE;

static uint16 path_exit_timer = 0; 
static uint16 path_mid_timer = 0;
static uint16 round_cooldown = 0; // 出环冷却计数器，防二次误触
static uint16 path_timer = 0;
extern float angle_z;
uint16 L1_raw = 0, L2_raw = 0, L3_raw = 0, L4_raw = 0;

void Path_Reset(void) {
    current_path_type = PATH_NORMAL;
    round_step = ROUND_NONE;
    path_exit_timer = 0;
    round_cooldown = 60; // 出环后免疫 300ms，确保以 1.7m/s 远离环岛区域
}

void Path_Identify(void) {
    extern float L1_norm, L2_norm, L3_norm, L4_norm;
    extern int8 round_direction;
	extern float true_gyro_z;
	extern float true_gyro_x;
	
    // 冷却时间递减
    if (round_cooldown > 0) {
        round_cooldown--;
    }

    // 状态分支 1：当前已经是十字模式，专心处理十字的退出
    if (current_path_type == PATH_CROSS) {
        path_timer++;
        if (path_timer > 1) {
            current_path_type = PATH_NORMAL;
            path_timer = 0; // 退出时清零
        }
    }
    
    // 状态分支 2：当前已经是环岛模式，专心走环岛内部状态机
    else if (current_path_type == PATH_ROUNDABOUT) {
        switch (round_step) {
            case ROUND_FOUND:
            case ROUND_DELAY:
            case ROUND_IN:
                break;
                
            case ROUND_MID:
                // 【核心升级：角度死锁】
                // 六边形环岛，没有转够230度，绝对不可能是出口！
                
                if (round_direction == 1) {
                    // 右环岛：右转角度是累加为正
                    if (angle_z > 230.0f && L1_norm > 80) { 
                        round_step = ROUND_OUT;
                        angle_z = 0; // 找到出口瞬间，再次清零角度，给 ROUND_OUT 用！
                    }
                } 
                else if (round_direction == -1) {
                    // 左环岛：左转角度是累积为负
                    if (angle_z < -230.0f && L4_norm > 90) { 
                        round_step = ROUND_OUT;
                        angle_z = 0; 
                    }
                }
                break;
                
            case ROUND_OUT:
                // 等待主函数根据出环角度重置
                break;
		}
    }
    
    // 状态分支 3：只有在普通巡线状态下，才允许捕捉并切入新路径！
    else {
        // 优先判断十字
        if (L2_norm > 90 && L3_norm > 90 ) {        
            current_path_type = PATH_CROSS;
			path_timer = 0; // 【关键】：进入十字瞬间，清零计时器！
			angle_z = 0; // 【核心新增】：进十字瞬间，把角度清零！把它当指南针！
        } 
        // 必须在没有冷却的情况下，才允许捕捉环岛入口
        else if (round_cooldown == 0) {
            
            // 【右环岛高速大放水】
            if (L1_norm > 70 && L4_norm > 90 && L3_norm > 40 && true_gyro_z > -400.0f && true_gyro_z <400.0f && true_gyro_x > -500.0f && true_gyro_x < 500.0f) {
                current_path_type = PATH_ROUNDABOUT;
                round_step = ROUND_FOUND;
                round_direction = 1;  // 右环岛
            }
            // 【左环岛同步升级】
            else if (L1_norm > 90 && L4_norm > 60 && L2_norm > 40 && true_gyro_z > -400.0f && true_gyro_z <400.0f  && true_gyro_x > -500.0f && true_gyro_x < 500.0f) {
                current_path_type = PATH_ROUNDABOUT;
                round_step = ROUND_FOUND;
                round_direction = -1; // 左环岛
            }
        }
    }
}