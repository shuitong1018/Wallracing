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
#include "my_function.h"


#define MAX_DUTY           (30)                                                // 最大占空比百分比，范围为 0%~30%

#define PWM_R              (PWMA_CH3P_P24)                                     // 右电机 PWM 控制引脚定义
#define DIR_R              (IO_P50)                                            // 右电机方向控制引脚定义

#define PWM_L              (PWMA_CH4P_P26)                                     // 左电机 PWM 控制引脚定义
#define DIR_L              (IO_P51)                                            // 左电机方向控制引脚定义

#define ENCODER_R          (TIM0_ENCOEDER)                                     // 右电机编码器定时器定义
#define ENCODER_DIR_R      (IO_P35)                                            // 右电机编码器方向引脚定义
#define ENCODER_COUNT_R    (TIM0_ENCOEDER_P34)                                 // 右电机编码器计数引脚定义

#define ENCODER_L          (TIM3_ENCOEDER)                                     // 左电机编码器定时器定义
#define ENCODER_DIR_L      (IO_P53)                                            // 左电机编码器方向引脚定义
#define ENCODER_COUNT_L    (TIM3_ENCOEDER_P04)                                 // 左电机编码器计数引脚定义
#define WHEEL_BASE          10
#define FILTER_COEFF       0.98f
#define DT                 0.001f

void IMU_Filter_Update(void);
// 全局变量
int base_speed = BASE_SPEED_NORMAL;
int target_speed_diff = 0;
int error_value = 0;


extern int L1, L2, L3, L4;           // 定义在 sensor.c
extern int L1_filt, L2_filt, L3_filt, L4_filt;  // 定义在 filter.c

void IMU_Filter_Update(void);

int16 right_encoder_data = 0;                                                  // 右电机编码器计数数据
int16 left_encoder_data = 0;                                                   // 左电机编码器计数数据

PID_t pid_speed=
{
	0,                                                               // 目标值
	0,                                                               // 实际值
	0,                                                                  // 输出值
	0.5,                                                                 // 比例系数
	0.1,                                                                  // 积分系数
	0.01,                                                                // 微分系数
	0,                                                                // 当前误差
	0,                                                                // 上一次误差
	0,                                                              // 积分误差
	MAX_DUTY,                                                        // 输出最大值
	-MAX_DUTY                                                        // 输出最小值
};                                                       
PID_t pid_dir=
{
	0,                                                              // 目标值
	0,                                                              // 实际值
	0,                                                                 // 输出值
	0.5,                                                               // 比例系数
	0.1,                                                               // 积分系数
	0.01,                                                              // 微分系数
	0,                                                            // 当前误差
	0,                                                            // 上一次误差
	0,                                                          // 积分误差
	100,                                                 // 输出最大值
	-100                                                // 输出最小值
};    
PID_t pid_balance=
{
	0,                                                              // 目标值
	0,                                                              // 实际值
	0,                                                                 // 输出值
	0.5,                                                               // 比例系数
	0.1,                                                               // 积分系数
	0.01,                                                              // 微分系数
	0,                                                            // 当前误差
	0,                                                            // 上一次误差
	0,                                                          // 积分误差
	100,                                                 // 输出最大值
	-100                                                // 输出最小值
};                                                                  

float pitch_angle = 0;														  // 车身倾角
float pitch_rate = 0;															  // 车身倾角角速度

unsigned char counter_dir = 0;                                                        // 方向计数器，用于控制PID最外环定时更新计数
unsigned char counter_speed = 0;                                                      // 速度计数器，用于控制PID中环定时更新计数
unsigned char counter_balance = 0;                                                       // 平衡计数器，用于控制平衡PID内环定时更新计数
unsigned char speed_r,speed_l,speed_diff;                                                           // 占空比变量，范围为 -MAX_DUTY ~ MAX_DUTY，正负表示方向
unsigned char speed_average = 0; // 速度平均值，用于方向控制的输入
unsigned char speed_target = 0; // 速度目标值，用于速度 PID 的目标值
unsigned char speed_out = 0; // 速度最终输出值，用于调整基础速度

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
    pit_init(TIM_0, 1);
	  irq_init();
  	if(imu660rb_init())
            printf("\r\nIMU660RB init error."); 
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
                left_speed = speed_out + speed_diff;
                right_speed = speed_out - speed_diff;
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

void PID_calculate()
{
	counter_dir++;
	counter_speed++;
	
	                                       			
	if(counter_balance >= 1) // 每 1 次中断更新一次平衡 PID
	{
		counter_balance = 0;            // 重置计数器
		imu660rb_get_acc();                                                         // 获取 IMU660RB 的加速度测量数值
 	    imu660rb_get_gyro();                                                        // 获取 IMU660RB 的角速度测量数值
		IMU_Filter_Update();                                                        // 更新 IMU 互补滤波，计算车身倾角
		pid_balance.actual = pitch_angle; // 设置实际值为车身倾角
		pid_balance.out = - pid_balance.Kp * (pid_balance.target - pid_balance.actual) - pid_balance.Kd * pitch_rate; // 计算平衡 PID 输出
		speed_out = pid_balance.out; // 将平衡 PID 输出调整基础速度，以保持车身平衡
		speed_r = speed_out + speed_diff; // 计算右轮速度
		speed_l = speed_out - speed_diff; // 计算左轮速度
	}

	if(counter_speed >= 2) // 每 2 次中断更新一次速度 PID
	{
		counter_speed = 0;            // 重置计数器
		right_encoder_data = encoder_get_count(ENCODER_R);	                    		// 获取右电机编码器计数数据
		left_encoder_data  = encoder_get_count(ENCODER_L);	                    		// 获取左电机编码器计数数据
		
		encoder_clear_count(ENCODER_R);	                                    			// 清零右电机编码器计数
		encoder_clear_count(ENCODER_L);                                       			// 清零左电机编码器计数

		speed_average = (right_encoder_data + left_encoder_data) / 2; // 计算左右轮平均速度，作为方向控制的输入
		pid_speed.target = speed_target; // 设置速度目标值
		pid_speed.actual = speed_average; // 设置实际值为平均速度
		PID_update(&pid_speed); // 更新速度 PID，计算输出
		pid_balance.target = pid_speed.out; // 将速度 PID 输出作为基础速度
	}
	
	if(counter_dir >= 5) // 每 5 次中断更新一次方向 PID
	{
		counter_dir = 0;            // 重置计数器
		pid_dir.actual = error_value; // 设置实际值为方向误差
		PID_update(&pid_dir);
		speed_diff = pid_dir.out; // 将方向 PID 输出作为左右轮速度差
	}
	
}

void IMU_Filter_Update(void)
{
    
    // 从加速度计算倾角 (单位: 度)
    float acc_angle = atan2(-imu660rb_acc_x, imu660rb_acc_z) * 57.29578f; // 57.29578 = 180/PI, 弧度转度

    // 3. 互补滤波核心公式
    pitch_rate = imu660rb_gyro_y; // 角速度直接用于D项
    pitch_angle = FILTER_COEFF * (pitch_angle + imu660rb_gyro_y * DT) + (1.0f - FILTER_COEFF) * acc_angle;
    // 公式解释: 用陀螺仪积分预测角度，再用加速度计角度进行微量修正
}
