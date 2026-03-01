#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_adc.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "zf_driver_uart.h"

#include "config.h"
#include "sensor.h"
#include "filter.h"
#include "path.h"
#include "motor.h"
#include "error.h"

// 全局变量
int base_speed = BASE_SPEED_NORMAL;
int target_speed_diff = 0;
int error_value = 0;


extern int L1, L2, L3, L4;           // 定义在 sensor.c
extern int L1_filt, L2_filt, L3_filt, L4_filt;  // 定义在 filter.c

//-------------------------------------------------------------------------------------------------------------------
// 系统初始化
//-------------------------------------------------------------------------------------------------------------------
void System_Init(void)
{
    // 逐飞库必须的初始化
    clock_init(SYSTEM_CLOCK_30M);      // 系统时钟初始化为30MHz
    debug_init();                        // 调试串口初始化
    
    // 各模块初始化
    Sensor_Init();
    Filter_Init();
    Path_Init();
    Motor_Init();
    
    system_delay_ms(100);                // 使用逐飞库的延时函数
}

//-------------------------------------------------------------------------------------------------------------------
// 主函数
//-------------------------------------------------------------------------------------------------------------------
void main(void)
{
    PathType_t path;
    PathAction_t action;
    int left_speed, right_speed;
    
    System_Init();
    
    while(1)
    {
        // 读取传感器
        Sensor_Read();
        
        // 滤波处理
        Filter_Process();
        
        // 检测路径类型
        path = Path_Detect(L1_filt, L2_filt, L3_filt, L4_filt);
        
        // 计算误差
        error_value = Error_Calculate();
        
        // 获取控制动作
        action = Path_Get_Action(path, L1_filt, L2_filt, L3_filt, L4_filt);
        
        // 执行控制
        switch(action)
        {
            case ACTION_TRACKING:
                // 正常循迹
                left_speed = BASE_SPEED_NORMAL + error_value;
                right_speed = BASE_SPEED_NORMAL - error_value;
                Motor_Control(left_speed, right_speed);
                break;
                
            case ACTION_TRACKING_FAST:
                // 快速循迹（如折线）
                left_speed = BASE_SPEED_FAST + error_value * 2;
                right_speed = BASE_SPEED_FAST - error_value * 2;
                Motor_Control(left_speed, right_speed);
                break;
                
            case ACTION_TURN_LEFT_HARD:
                // 急左转（直角）
                Motor_SetLeft(300);    // 30%占空比
                Motor_SetRight(700);   // 70%占空比
                break;
                
            case ACTION_TURN_RIGHT_HARD:
                // 急右转（直角）
                Motor_SetLeft(700);
                Motor_SetRight(300);
                break;
                
            case ACTION_TURN_LEFT_SMOOTH:
                // 平滑左转（圆形弯）
                Motor_SetLeft(400);
                Motor_SetRight(600);
                break;
                
            case ACTION_TURN_RIGHT_SMOOTH:
                // 平滑右转（圆形弯）
                Motor_SetLeft(600);
                Motor_SetRight(400);
                break;
                
            case ACTION_ROUNDABOUT:
                // 环岛模式
                Motor_SetLeft(450);
                Motor_SetRight(550);
                break;
                
            default:
                Motor_Stop();
                break;
        }
        
        printf("L1=%d L2=%d L3=%d L4=%d err=%d\r\n", L1_filt, L2_filt, L3_filt, L4_filt, error_value);
        
        system_delay_ms(CONTROL_CYCLE_MS);  // 使用逐飞库的延时函数
    }
}