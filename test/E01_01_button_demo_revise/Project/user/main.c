#include "zf_common_headfile.h" 
#include "config.h"
#include "sensor.h"
#include "path.h"
#include "error.h"

#define MAX_DUTY           (30)                                                // 最大占空比百分比，范围为 0%~30%

#define LED1               (IO_P52)

#define PWM_R              (PWMA_CH4P_P26)                                     // 右电机 PWM 控制引脚定义
#define DIR_R              (IO_P51)                                            // 右电机方向控制引脚定义

#define PWM_L              (PWMA_CH3P_P24)                                     // 左电机 PWM 控制引脚定义
#define DIR_L              (IO_P50)                                            // 左电机方向控制引脚定义

#define ENCODER_R          (TIM3_ENCOEDER)                                     // 右电机编码器定时器定义
#define ENCODER_DIR_R      (IO_P53)                                            // 右电机编码器方向引脚定义
#define ENCODER_COUNT_R    (TIM3_ENCOEDER_P04)                                 // 右电机编码器计数引脚定义

#define ENCODER_L          (TIM0_ENCOEDER)                                     // 左电机编码器定时器定义
#define ENCODER_DIR_L      (IO_P35)                                            // 左电机编码器方向引脚定义
#define ENCODER_COUNT_L    (TIM0_ENCOEDER_P34)                                 // 左电机编码器计数引脚定义

// 全局变量
float error_value = 0.0f;
extern float L1_norm, L2_norm, L3_norm, L4_norm;  

float duty_L = 0,duty_R = 0;                                                     // 占空比变量，范围为 -MAX_DUTY ~ MAX_DUTY，正负表示方向

int16 right_encoder_data = 0;                                                  // 右电机编码器计数数据
int16 left_encoder_data = 0;                                                   // 左电机编码器计数数据

float Kp_L = 0.8, Kd_L = 0.1;                                                     // 左电机 PID 控制参数
float Kp_R = 0.9, Kd_R = 0.1;                                                     // 右电机 PID 控制参数

float error_L=0,error_R=0;
/* 上一周期的（测量-目标）误差，用于计算微分项（delta error） */
float prev_e_L = 0.0f, prev_e_R = 0.0f;

float target_speed = 1;                                                     // 目标速度变量，单位为m/s


// 方向PID参数
float Kp_dir = 1.0f;
float Kd_dir = 0.0f;
float prev_e_dir = 0.0f;
float dir_output = 0.0f;           

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
    
    //电机模块初始化
    gpio_init(LED1, GPO, 1, GPO_PUSH_PULL);
	
	pwm_init(PWM_R, 1000, 0);      												// 初始化右电机 PWM 引脚，频率为 1000 Hz，初始占空比为 0
	gpio_init(DIR_R, GPO, 1, GPO_PUSH_PULL);  									// 初始化右电机方向控制引脚为推挽输出，初始电平为 1
				
	pwm_init(PWM_L, 1000, 0);      												// 初始化左电机 PWM 引脚，频率为 1000 Hz，初始占空比为 0
	gpio_init(DIR_L, GPO, 1, GPO_PUSH_PULL);  									// 初始化左电机方向控制引脚为推挽输出，初始电平为 1
				
	encoder_dir_init(ENCODER_R, ENCODER_DIR_R, ENCODER_COUNT_R);  				// 初始化右电机编码器，指定方向引脚和计数引脚
	encoder_dir_init(ENCODER_L, ENCODER_DIR_L, ENCODER_COUNT_L);  				// 初始化左电机编码器，指定方向引脚和计数引脚
				
	pwm_set_freq(PWM_R, 500, 5000);  											// 设置右电机 PWM 频率为 500 Hz，占空比为 50%（5000 对应 50%），用于驱动校准零点
	pwm_set_freq(PWM_L, 500, 5000);  											// 设置左电机 PWM 频率为 500 Hz，占空比为 50%（5000 对应 50%），用于驱动校准零点
				
	system_delay_ms(3000);  													// 延迟 3000 ms，等待驱动板完成零点校准
				
	pwm_set_freq(PWM_R, 1000, 0);  												// 恢复右电机 PWM 频率为 1000 Hz，占空比为 0，准备进入正常控制
	pwm_set_freq(PWM_L, 1000, 0);  												// 恢复左电机 PWM 频率为 1000 Hz，占空比为 0，准备进入正常控制
	
	// 此处编写用户代码 例如外设初始化代码等
	pit_ms_init(TIM1_PIT, 5);													// 初始化 PIT 定时器,定时周期为 5 ms
	// 初始化无线串口
    if(wireless_uart_init())
    {
        // 初始化失败时可以在此处理（当前仅简单断开循环）
        while(1);
    }
		
		
    system_delay_ms(100);                
}

//速度环相关函数声明
void speed_control();
void speed_get();
void speed_calculate();
void speed_out();
void speed_show();

//转向环相关函数声明
void dir_get();
void dir_calculate();
void dir_control();

//巡线函数
void patrol_line();

//-------------------------------------------------------------------------------------------------------------------
// 主函数
//-------------------------------------------------------------------------------------------------------------------
void main(void)
{
    
    System_Init();
	
	  tim1_irq_handler = patrol_line;
    while(1)
    {
    //     // 读取传感器
	// 	Sensor_Read_Normalized();
	// 	Sensor_Monitor();
	
    //     // 计算误差
    // error_value = Error_Calculate(L1_norm, L2_norm, L3_norm, L4_norm);
		//printf("the error is %.2f\r\n",error_value);
		speed_show();
		system_delay_ms(500);  
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
        
void dir_get(){
    // 读取传感器
		Sensor_Read_Normalized();
		Sensor_Monitor();
	
    // 计算误差
    error_value = Error_Calculate(L1_norm, L2_norm, L3_norm, L4_norm);
}

void dir_calculate(){
    float e = error_value; // 当前误差
    
    dir_output = Kp_dir * e + Kd_dir * (e - prev_e_dir);
    
    // 限幅控制量
    if (dir_output > 3.0f) dir_output = 3.0f;
    if (dir_output < -3.0f) dir_output = -3.0f;
    
    prev_e_dir = e;
}

void dir_control(){
    dir_get();
    dir_calculate();
}

void patrol_line(){
    dir_control();
    speed_control();
    duty_L += dir_output; // 将方向控制输出叠加到左轮占空比
    duty_R -= dir_output; // 将方向控制输出叠加到右轮占空比
    speed_out();
}
	
void speed_control(){
     speed_get();
     speed_calculate();
     
   //speed_show();
}

void speed_get(){
    right_encoder_data = encoder_get_count(ENCODER_R);    // 获取右电机编码器计数数据
    left_encoder_data  = -encoder_get_count(ENCODER_L);    // 获取左电机编码器计数数据

    encoder_clear_count(ENCODER_R);    // 清零右电机编码器计数
    encoder_clear_count(ENCODER_L);    // 清零左电机编码器计数
}

void speed_calculate(){
	
            float eL = (float)(target_speed - left_encoder_data*0.003518);   // 当前误差（左）
            float eR = (float)(target_speed - right_encoder_data*0.003518);  // 当前误差（右）

            // 左轮：比例 + 微分
            error_L = Kp_L * (eL) + Kd_L * (eL - prev_e_L) ;
            // 限幅控制量，保持与原逻辑一致的限幅 ±3
            if (error_L > 3.0f) error_L = 3.0f;
            if (error_L < -3.0f) error_L = -3.0f;
            duty_L += error_L;
            if (duty_L >= MAX_DUTY) duty_L = MAX_DUTY;
            if (duty_L <= -MAX_DUTY) duty_L = -MAX_DUTY;

            // 右轮：比例 + 微分
            error_R = Kp_R * (eR) + Kd_R * (eR - prev_e_R);
            if (error_R > 3.0f) error_R = 3.0f;
            if (error_R < -3.0f) error_R = -3.0f;
            duty_R += error_R;
            if (duty_R >= MAX_DUTY) duty_R = MAX_DUTY;
            if (duty_R <= -MAX_DUTY) duty_R = -MAX_DUTY;

            // 保存本周期误差以便下次计算微分项
            prev_e_L = eL;
            prev_e_R = eR;
}

void speed_out(){
    if(duty_L >= 0)                                                          			// 当占空比为非负数时，电机正转
    {
        pwm_set_duty(PWM_L, (int32)(duty_L * (PWM_DUTY_MAX / 100)) + (PWM_DUTY_MAX / 10));     // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_L, 1);                                          			// 输出电机旋转方向信号
    }
    else                                                                   			// 电机反转
    {
        pwm_set_duty(PWM_L, (int32)((-duty_L) * (PWM_DUTY_MAX / 100)) + (PWM_DUTY_MAX / 10));  // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_L, 1);                                          			// 输出电机旋转方向信号
                                                			// 输出电机旋转方向信号
    }
    if(duty_R >= 0)                                                          			// 当占空比为非负数时，电机正转
    {
        pwm_set_duty(PWM_R, (int32)(duty_R * (PWM_DUTY_MAX / 100)) + (PWM_DUTY_MAX / 10));     // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_R, 0);                                          			// 输出电机旋转方向信号
    }
    else                                                                   			// 电机反转
    {
        pwm_set_duty(PWM_R, (int32)((-duty_R) * (PWM_DUTY_MAX / 100)) + (PWM_DUTY_MAX / 10));  // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_R, 0);                                         			// 输出电机旋转方向信号
    }
}
void speed_show(){
    char buf[128];

    // 格式化并通过无线串口发送
    sprintf(buf, "%.2f,%.2f\r\n", right_encoder_data*0.003518, left_encoder_data*0.003518);
    wireless_uart_send_string(buf);

   // sprintf(buf, "duty_L:%d error_L:%.2f duty_R:%d error_R:%.2f\r\n", duty_L, error_L, duty_R, error_R);
   // wireless_uart_send_string(buf);

}