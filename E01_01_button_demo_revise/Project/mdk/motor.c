#include "motor.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

void Motor_Init(void)
{
    // 方向引脚初始化 - gpio_init需要4个参数
    gpio_init(IO_P21, GPO, 1, GPO_PUSH_PULL);   // 左电机方向，初始高电平，推挽输出
    gpio_init(IO_P23, GPO, 1, GPO_PUSH_PULL);   // 右电机方向，初始高电平，推挽输出
    
    // PWM初始化 - 使用正确的PWM通道枚举
    // 左电机使用PWMA_CH1P_P20 (P20输出PWM)
    pwm_init(PWMA_CH1P_P20, 20000, 0);  // 20kHz，初始占空比0
    
    // 右电机使用PWMA_CH2P_P22 (P22输出PWM)
    pwm_init(PWMA_CH2P_P22, 20000, 0);  // 20kHz，初始占空比0
		encoder_dir_init(ENCODER_R, ENCODER_DIR_R, ENCODER_COUNT_R);  				// 初始化右电机编码器，指定方向引脚和计数引脚
		encoder_dir_init(ENCODER_L, ENCODER_DIR_L, ENCODER_COUNT_L);  				// 初始化左电机编码器，指定方向引脚和计数引脚
				
		pwm_set_freq(PWM_R, 500, 5000);  											// 设置右电机 PWM 频率为 500 Hz，占空比为 50%（5000 对应 50%），用于驱动校准零点
		pwm_set_freq(PWM_L, 500, 5000);  											// 设置左电机 PWM 频率为 500 Hz，占空比为 50%（5000 对应 50%），用于驱动校准零点
				
		system_delay_ms(3000);  													// 延迟 3000 ms，等待驱动板完成零点校准
				
		pwm_set_freq(PWM_R, 1000, 0);  												// 恢复右电机 PWM 频率为 1000 Hz，占空比为 0，准备进入正常控制
		pwm_set_freq(PWM_L, 1000, 0);  												// 恢复左电机 PWM 频率为 1000 Hz，占空比为 0，准备进入正常控制
	
}

void Motor_SetLeft(int speed)
{
    uint8 dir = 1;
    uint32 pwm_value;
    
    // 速度限幅 (PWM范围0-10000，因为PWM_DUTY_MAX=10000)
    if(speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;
    if(speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;
    
    // 方向处理
    if(speed >= 0) {
        dir = 1;                        // 正转
        // 将0-255的速度映射到0-10000
        pwm_value = (uint32)speed * (PWM_DUTY_MAX / MOTOR_SPEED_MAX);
    } else {
        dir = 0;                        // 反转
        pwm_value = (uint32)(-speed) * (PWM_DUTY_MAX / MOTOR_SPEED_MAX);
    }
    
    gpio_set_level(IO_P21, dir);                 // 设置方向
    pwm_set_duty(PWMA_CH1P_P20, pwm_value);      // 设置PWM占空比
}

void Motor_SetRight(int speed)
{
    uint8 dir = 1;
    uint32 pwm_value;
    
    if(speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;
    if(speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;
    
    if(speed >= 0) {
        dir = 1;
        pwm_value = (uint32)speed * (PWM_DUTY_MAX / MOTOR_SPEED_MAX);
    } else {
        dir = 0;
        pwm_value = (uint32)(-speed) * (PWM_DUTY_MAX / MOTOR_SPEED_MAX);
    }
    
    gpio_set_level(IO_P23, dir);                 // 设置方向
    pwm_set_duty(PWMA_CH2P_P22, pwm_value);      // 设置PWM占空比
}

void Motor_Control(int left_speed, int right_speed)
{
    Motor_SetLeft(left_speed);
    Motor_SetRight(right_speed);
}

void Motor_Stop(void)
{
    pwm_set_duty(PWMA_CH1P_P20, 0);
    pwm_set_duty(PWMA_CH2P_P22, 0);
    gpio_set_level(IO_P21, 1);
    gpio_set_level(IO_P23, 1);
}