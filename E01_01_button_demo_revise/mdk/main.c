#include "zf_common_headfile.h" 
#include "config.h"
#include "sensor.h"
#include "path.h"
#include "error.h"
#

// 全局变量
float error_value = 0.0f;
extern float L1_norm, L2_norm, L3_norm, L4_norm;           

//-------------------------------------------------------------------------------------------------------------------
// 系统初始化
//-------------------------------------------------------------------------------------------------------------------
void System_Init(void)
{
    // 逐飞库必须的初始化
    clock_init(SYSTEM_CLOCK_30M);      // 系统时钟初始化为30MHz
    debug_init();                       // 调试串口初始化 - 占用 P31(TX) 和 P30(RX)UART引脚
    
    // 各模块初始化
    Sensor_Init();
    
    system_delay_ms(100);                // 使用逐飞库的延时函数
}

//-------------------------------------------------------------------------------------------------------------------
// 主函数
//-------------------------------------------------------------------------------------------------------------------
void main(void)
{
    
    System_Init();
    while(1)
    {
        // 读取传感器
		Sensor_Read_Normalized();
		Sensor_Monitor();
	
        // 计算误差
        error_value = Error_Calculate(L1_norm, L2_norm, L3_norm, L4_norm);
		printf("the error is %.2f\r\n",error_value);
		system_delay_ms(500);  // 使用逐飞库的延时函数
	}
}
        
        // 执行控制
        /*switch(action)
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
        }*/
        