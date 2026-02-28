#include "motor.h"

void Motor_Init(void) {
    // 1. 设置P2.0-P2.1为推挽输出
    P2M1 &= ~0x03;
    P2M0 |= 0x03;
    
    // 2. 配置PWM
    PWMA_ENO = 0x00;
    PWMA_PS = 0x00;     // PWM输出到P2.0, P2.1
    
    // 3. 配置PWM通道
    PWMA_CCER1 = 0x00;
    PWMA_CCMR1 = 0x60;  // PWM1输出模式
    PWMA_CCMR2 = 0x60;  // PWM2输出模式
    
    // 4. 使能PWM输出
    PWMA_CCER1 |= 0x01;  // 使能PWM1
    PWMA_CCER1 |= 0x10;  // 使能PWM2
    PWMA_ENO |= 0x03;    // 使能IO输出
    
    // 5. 设置PWM频率（20KHz）
    PWMA_PSCR = 24 - 1;  // 预分频
    PWMA_ARR = 255;      // 自动重装值（0-255）
    PWMA_CR1 |= 0x01;    // 使能计数器
}

void Motor_Control(int base_speed, int speed_diff) {
    int left_speed, right_speed;
    int left_dir = 1, right_dir = 1;
    
    // set_pwm (left, 负速度 + target_seed_diff)
    // right, base_seed - target_seed_diff
    left_speed = base_speed - speed_diff;
    right_speed = base_speed + speed_diff;
    
    // 方向处理（如果速度为负，则反转）
    if(left_speed < 0) {
        left_speed = -left_speed;
        left_dir = 0;
    }
    if(right_speed < 0) {
        right_speed = -right_speed;
        right_dir = 0;
    }
    
    // 速度限幅
    if(left_speed > 255) left_speed = 255;
    if(right_speed > 255) right_speed = 255;
    
    // 设置方向
    LEFT_MOTOR_DIR_PIN = left_dir;
    RIGHT_MOTOR_DIR_PIN = right_dir;
    
    // 设置PWM占空比
    PWMA_CCR1 = left_speed;
    PWMA_CCR2 = right_speed;
}