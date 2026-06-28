#include "zf_common_headfile.h" 
#include "config.h"
#include "sensor.h"
#include "path.h"
#include "error.h"

#define MAX_DUTY           (75)                                                // 最大占空比百分比，范围为 0%~75%

#define LED1               (IO_P52)

// 【新增 1】：按键引脚与屏幕模式变量
#define KEY_PIN            (IO_P70)
uint8 display_mode = 0; // 0代表显示Norm归一化值，1代表显示Raw原始值

// 定义发车按键和状态变量 
#define RUN_KEY_PIN        (IO_P71)      // 发车/停车按键
uint8 car_running = 0;                   // 0: 停车看数据状态, 1: 狂飙状态

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
#define DEADZONE_L  1250  // 左轮死区补偿 
#define DEADZONE_R  1300  // 右轮死区补偿 

#define PWM_FREQ           (500)                      // PWM频率 
#define PWM_PIN            (PWMB_CH3_P33)              // 使用P33引脚
#define MOTOR_SPEED_DUTY   (4000)                      // 固定转速占空比（0-10000）

#define DATA_SIZE 800

//typedef struct {
//    int16 left_encoder;
//    int16 right_encoder;
//    int16 dir_output_scaled;
//} pid_data_t;

// 全局变量
float error_value = 0.0f;
extern float L1_norm, L2_norm, L3_norm, L4_norm;  
extern uint16 L1_raw, L2_raw, L3_raw, L4_raw;

float angle_z;
extern int8 round_direction = 0; // 1代表右环，-1代表左环
extern path_state_enum current_path_type;
extern round_state_enum round_step;

float edata duty_L = 0,duty_R = 0;                                                     // 速度环输出的占空比

uint16 lost_line_timer = 0;   // 修复：定义丢失线计时器
float last_stable_dir = 0;    // 修复：定义十字路口锁定的方向
int16 edata right_encoder_data = 0;                                                  // 右电机编码器计数数据
int16 edata left_encoder_data = 0;                                                   // 左电机编码器计数数据

uint16 action_timer = 0;

const float Kp_L = 11.8, Kd_L = 15;                                      					// 左电机 PID 控制参数
const float Kp_R = 11.3, Kd_R = 16;                                                     // 右电机 PID 控制参数
const float KF_L = 14.3,KF_R =14.4;
/* 上一周期的（测量-目标）误差，用于计算微分项（delta error） */
float edata prev_e_L = 0.0f, prev_e_R = 0.0f;


const float target_speed = 1.7;                                                 // 目标速度变量，单位为m/s
float edata target_speed_L = 0,target_speed_R = 0;                              // 左右轮目标速度变量，单位为m/s   

// 方向PID参数
const float Kp1_dir = 0.0008f;
const float Kp2_dir = 0.000091f;
const float Kd_dir = 0.063f;
float edata prev_e_dir = 0.0f;
float edata dir_output = 0.0f;           

// ================= 陀螺仪全局变量 =================
float gyro_z_offset = 0.0f; // Z轴静态零偏
float gyro_x_offset = 0.0f; // 【必须新增】x轴静态零偏
float angle_z = 0.0f;       // 积分得到的绝对角度（度）
float true_gyro_z = 0.0f;
float true_gyro_x = 0.0f;   // 【必须新增】真实的x轴角速度

void Gyro_Init_And_Calibrate(void) {
    int32 offset_sum_z = 0;
    int32 offset_sum_x = 0; // 【新增】
    int i;
    
    if(imu660rb_init()) {
        printf("\r\nIMU660RB init error.");
        while(1); 
    }
    
    system_delay_ms(100); 
    for(i = 0; i < 200; i++) {
        imu660rb_get_gyro();
        offset_sum_z += imu660rb_gyro_z;
        offset_sum_x += imu660rb_gyro_x; // 【新增】累加Y轴
        system_delay_ms(2);
    }
    gyro_z_offset = (float)offset_sum_z / 200.0f;
    gyro_x_offset = (float)offset_sum_x / 200.0f; // 【新增】算出Y轴零偏
    printf("\r\nGyro Z Offset: %.2f", gyro_z_offset);
}
void pit_hanlder (void) {
	   
    // 1. 获取最新数据
    imu660rb_get_gyro(); 
    
    // 2. 减去零偏，得到真实角速度
    true_gyro_z = (float)imu660rb_gyro_z - gyro_z_offset;
	true_gyro_x = (float)imu660rb_gyro_x - gyro_x_offset;
    
    // 3. 死区滤波：剔除极其微小的震动噪点
    if (true_gyro_z < 5.0f && true_gyro_z > -5.0f) {
        true_gyro_z = 0;
    }
    
	if (true_gyro_x < 10.0f && true_gyro_x > -10.0f) {
        true_gyro_x = 0;
    }
	
    // 4. 角度积分公式
    // 乘以 0.005 是因为中断是 5ms (0.005秒)
    angle_z += (true_gyro_z / 13.98f) * 0.005f; 
}
// 数据存储相关
//pid_data_t pid_data[DATA_SIZE];
//int data_index = 0;
//int collecting = 1;
//volatile int data_ready = 0;

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
    
	// 【新增 2】：屏幕与按键的初始化
    ips200_init();                                 // 初始化IPS200屏幕
    ips200_clear(RGB565_BLACK);                    // 预先清刷
    gpio_init(KEY_PIN, GPI, 1, GPI_PULL_UP);       // 初始化 P7.0 为按键输入，内部上拉
	gpio_init(RUN_KEY_PIN, GPI, 1, GPI_PULL_UP);   // 初始化 P7.1 发车按键
	
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
	Gyro_Init_And_Calibrate();                                                  //初始化陀螺仪
	
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

//调试函数声明
void data_show();

//转向环相关函数声明
void dir_get();
void dir_calculate();
void dir_control();

// PWM 缓启动函数声明
void pwm_soft_start(pwm_channel_enum pin, uint32 target_duty, uint16 step, uint16 interval_ms);

//void output_all_data();

//巡线函数
void patrol_line();

//-------------------------------------------------------------------------------------------------------------------
// 主函数
//-------------------------------------------------------------------------------------------------------------------
void main(void)
{
    System_Init();
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    adc_init(ADC_CH5_P15, ADC_8BIT);//测电池电压
    pwm_init(PWM_PIN, PWM_FREQ, 0);
    pwm_set_duty(PWM_PIN, 0);                          // 初始化风扇为关闭状态

    tim1_irq_handler = patrol_line;

	
    while(1)
    {
        data_show();
        system_delay_ms(5);
    }
}
void dir_get(){
    // 读取传感器
		Sensor_Read_Normalized();
		//Sensor_Monitor();
	
    // 计算误差
    error_value = Error_Calculate(L1_norm, L2_norm, L3_norm, L4_norm);
}

void dir_calculate(void) {
    // 1. 严格遵守 C89：先在最前面声明所有变量
    float e;
    float abs_e;
    
    // 2. 然后再进行赋值和计算
    e = error_value;
    abs_e = (e >= 0) ? e : -e;

    dir_output = Kp1_dir * e + Kp2_dir * e * abs_e + Kd_dir * (e - prev_e_dir);
    

    if (dir_output > 1.3f) dir_output = 1.3f;
    if (dir_output < -1.3f) dir_output = -1.3f;    
	   
    prev_e_dir = e;
}

void dir_control(){
    dir_get();
    dir_calculate();
}

void patrol_line() {
	
	static uint8 led_blink_timer = 0; 
    float min_speed = 0.05f;
	
    // 1. 动态基础速度：默认使用直道速度 1.7m/s
    float active_speed = target_speed; 
	
	pit_hanlder();
    dir_get(); 
	
	// 发车保护锁
if (car_running == 0) {
    pwm_set_duty(PWM_L, 0); 
    pwm_set_duty(PWM_R, 0); 
    pwm_set_duty(PWM_PIN, 0); // 彻底停车，全部断电
    return; 
} 
else if (car_running == 1) {
    // 正在缓启动风扇，强制锁住左右轮，但【不要】关风扇 PWM_PIN
    pwm_set_duty(PWM_L, 0); 
    pwm_set_duty(PWM_R, 0); 
    return; 
}
// 只有 car_running == 2 时，才会往下执行
    
    // 丢线保护
    if (L1_norm < 5 && L2_norm < 5 && L3_norm < 5 && L4_norm < 5) {
        lost_line_timer++;
        if (lost_line_timer > 100) { 
            target_speed_L = 0;
            target_speed_R = 0;
					  pwm_set_duty(PWM_PIN,0);
            speed_control();
            return; 
        }
    } else {
        lost_line_timer = 0; 
    }

    Path_Identify(); 

    // ================= 路径决策分支 =================

    if (current_path_type == PATH_CROSS) {
        //高级版 - 用陀螺仪压制转向
        dir_output = angle_z * 0.05f; // 如果车头偏了，产生反向抵抗力
    }

    else if (current_path_type == PATH_ROUNDABOUT) {
        if (round_step == ROUND_FOUND) {
            round_step = ROUND_DELAY;
            action_timer = 0; 
        }
        else if (round_step == ROUND_DELAY) {
            dir_output = last_stable_dir * 0.5f; 
            action_timer++;

            // 这里的直道滑行延时依然用 timer 最稳（因为车身还没开始转，陀螺仪没变化）
            if (round_direction == 1 && action_timer > 10) {  
                round_step = ROUND_IN; 
                angle_z = 0; // 【关键】开始转弯前，把角度清零！
            } 
            else if (round_direction == -1 && action_timer > 14) {  
                round_step = ROUND_IN; 
                angle_z = 0; // 【关键】开始转弯前，把角度清零！
            }
        }
        else if (round_step == ROUND_IN) {
            if (round_direction == 1) {
                dir_output = -4.0f; 
                // 【陀螺仪接管】：不用管时间，车头只要实打实地转了 45 度，立马结束入环！
                if (angle_z > 45.0f) { 
                    round_step = ROUND_MID; 
                }
            } else {
                dir_output = 3.5f; 
                if (angle_z < -45.0f) { 
                    round_step = ROUND_MID; 
                }
            }

            target_speed_L = active_speed - dir_output;
            target_speed_R = active_speed + dir_output;
            speed_control();
            return; 
        }
        else if (round_step == ROUND_MID) {
            dir_calculate(); 
            dir_output += (float)round_direction * 0.25f; 
        }
        else if (round_step == ROUND_OUT) {
            dir_calculate();
            dir_output -= (float)round_direction * 0.8f; 

            // 【陀螺仪出环接管】：出环时反向拉扯，只要车头甩回了大概 35 度，完美切回直道！
            if (round_direction == 1) {
                if (angle_z > 35.0f) Path_Reset(); // 右环出来往右甩
            } else {
                if (angle_z < -35.0f) Path_Reset();  // 左环出来往左甩
            }
        }
	}
    if (current_path_type == PATH_NORMAL) {
        dir_calculate(); 
		last_stable_dir = dir_output;
    }
    
    target_speed_L = active_speed - dir_output;
    target_speed_R = active_speed + dir_output;

    speed_control();
	
    // 【新增】非阻塞式 LED 环岛指示灯
    // ---------------------------------------------------------
    // 静态变量计数器，每次进 5ms 中断加 1

    // 如果当前状态是环岛（不管是哪个阶段）
    if (current_path_type == PATH_ROUNDABOUT) {
        // 假设低电平(0)点亮，高电平(1)熄灭。如果你的灯反了，就把 0 和 1 互换。
        P52 = 0; 
    } 
    else {
        P52 = 1; 
    }
}
	
void speed_control(){
     speed_get();
     speed_calculate();
	   speed_out();
}

void speed_get(){
    right_encoder_data = encoder_get_count(ENCODER_R);    // 获取右电机编码器计数数据
    left_encoder_data  = -encoder_get_count(ENCODER_L);    // 获取左电机编码器计数数据

    encoder_clear_count(ENCODER_R);    // 清零右电机编码器计数
    encoder_clear_count(ENCODER_L);    // 清零左电机编码器计数
}

void speed_calculate(void) {
    // 1. 严格遵守 C89：集中声明所有临时变量
    float eL, eR, ff_L, ff_R;

    // 2. 然后再进行计算 (带上 'f' 后缀，防止编译器把它当成 double 报错)
    eL = (float)(target_speed_L - left_encoder_data * 0.003518f);
    eR = (float)(target_speed_R - right_encoder_data * 0.003518f);

    ff_L = target_speed_L * KF_L;
    ff_R = target_speed_R * KF_R;

    // 3. 位置式 PD 公式
    duty_L = Kp_L * eL + Kd_L * (eL - prev_e_L) + ff_L;
    duty_R = Kp_R * eR + Kd_R * (eR - prev_e_R) + ff_R;

    // 全局限幅
    if (duty_L > MAX_DUTY)  duty_L = MAX_DUTY;
    if (duty_L < -MAX_DUTY) duty_L = -MAX_DUTY;
    if (duty_R > MAX_DUTY)  duty_R = MAX_DUTY;
    if (duty_R < -MAX_DUTY) duty_R = -MAX_DUTY;

    // 更新历史记录
    prev_e_L = eL;
    prev_e_R = eR;
}

void speed_out(){
//    if (!collecting) {
//        pwm_set_duty(PWM_L, 0);
//        pwm_set_duty(PWM_R, 0);
//        return;
//    }
    // 【终极融合逻辑】：动态防抱死限幅
    // 只有在普通赛道（长直道/缓弯）时，才强制轮子保持微弱正转，防止抖动和甩尾。
    if (current_path_type == PATH_NORMAL) {
        if (duty_L < 0.5f) duty_L = 0.5f; 
        if (duty_R < 0.5f) duty_R = 0.5f; 
    }
		
    if(duty_L >= 0)                                                          			// 当占空比为非负数时，电机正转
    {
        pwm_set_duty(PWM_L, (int32)(duty_L * (PWM_DUTY_MAX / 100)) + DEADZONE_L);     // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_L, 1);                                          			// 输出电机旋转方向信号
    }
    else                                                                   			// 电机反转
    {
        pwm_set_duty(PWM_L, (int32)((-duty_L) * (PWM_DUTY_MAX / 100)) + DEADZONE_L);  // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_L, 0);                                          			// 输出电机旋转方向信号
                                                			// 输出电机旋转方向信号
    }
    if(duty_R >= 0)                                                          			// 当占空比为非负数时，电机正转
    {
        pwm_set_duty(PWM_R, (int32)(duty_R * (PWM_DUTY_MAX / 100)) + DEADZONE_R);     // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_R, 0);                                          			// 输出电机旋转方向信号
    }
    else                                                                   			// 电机反转
    {
        pwm_set_duty(PWM_R, (int32)((-duty_R) * (PWM_DUTY_MAX / 100)) + DEADZONE_R);  // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_R, 1);                                         			// 输出电机旋转方向信号
    }
}

void pwm_soft_start(pwm_channel_enum pin, uint32 target_duty, uint16 step, uint16 interval_ms)
{
    uint32 duty;

    pwm_set_duty(pin, 0);
    for (duty = 800; duty < target_duty; duty += step)
    {
        pwm_set_duty(pin, duty);
        system_delay_ms(interval_ms);
    }
    pwm_set_duty(pin, target_duty);
}

void data_show(){
//   static uint16 voltage_print_timer = 0;
    char buf[128];
//    uint16 adc_p15_value;
//    float v_p15;
//    float battery_voltage;

    if(gpio_get_level(RUN_KEY_PIN) == 0)            
    {
        system_delay_ms(20); // 消抖 
        if(gpio_get_level(RUN_KEY_PIN) == 0)        
        {
            if (car_running == 0) {
                // 1. 设置为 1，告诉中断：“我要缓启动了，别把风扇关了，但轮子依然锁住”
                car_running = 1; 
                
                // 2. 真正的缓启动，此时中断不会来捣乱了
                pwm_soft_start(PWM_PIN, MOTOR_SPEED_DUTY, 100, 10);  
                
                // 3. 清除所有历史积累的“脏数据”，防止起步瞬间PID爆炸
                encoder_clear_count(ENCODER_R);
                encoder_clear_count(ENCODER_L);
                prev_e_L = 0.0f; 
                prev_e_R = 0.0f;
                prev_e_dir = 0.0f;
                angle_z = 0.0f;     // 清零陀螺仪积分
                lost_line_timer = 0; // 清除丢线计时

                // 4. 正式放开发车权限
                car_running = 2;  
            } else {
                // 停车逻辑
                car_running = 0;
                pwm_set_duty(PWM_PIN, 0); 
            }
            
            ips200_clear(RGB565_BLACK); 
            while(gpio_get_level(RUN_KEY_PIN) == 0); 
        }
    }

//    // 读取 P15 的 ADC 数据
//    adc_p15_value = adc_convert(ADC_CH5_P15);
//    v_p15 = (float)adc_p15_value * 3.3 / 255.0f;
//    battery_voltage = v_p15 * 11.0f;

//    if (++voltage_print_timer >= 5) {
//        voltage_print_timer = 0;
//        sprintf(buf, "%.2f\r\n", battery_voltage);
//        wireless_uart_send_string(buf);
//    }

   sprintf(buf, "%.2f,%.2f,%.2f,%.2f,%.2f\r\n", right_encoder_data*0.003518, left_encoder_data*0.003518,target_speed_R,target_speed_L,dir_output);
//    sprintf(buf, "%.1f,%.1f,%.1f,%.1f,%.2f,%.2f\n", 
//            L1_norm, 
//            L2_norm, 
//            L3_norm, 
//            L4_norm, 
//            true_gyro_z, 
//            true_gyro_x);
     wireless_uart_send_string(buf);
		 
//	// 2. 新增：按键扫描与状态切换
//    // --------------------------------------------------------
//    if(gpio_get_level(KEY_PIN) == 0)            
//    {
//        system_delay_ms(20); // 延时20ms消抖                    
//        if(gpio_get_level(KEY_PIN) == 0)        
//        {
//            display_mode = !display_mode;       // 切换显示模式
//            ips200_clear(RGB565_BLACK);         // 清屏防止残影
//            
//            // 死循环等待松手（因为寻线在定时器中断里，所以这里死等不会让车失控）
//            while(gpio_get_level(KEY_PIN) == 0); 
//        }
//    }
	
//    // --------------------------------------------------------
//    // 3. 新增：屏幕分屏渲染逻辑
//    // --------------------------------------------------------
//    if(display_mode == 0)
//    {
//        // 模式0：显示归一化值 (Norm 0 ~ 128)
//        ips200_show_string(0, 16*0, "--- Mode: NORM ---"); 
//        ips200_show_string(0, 16*1, "L1_Norm: "); ips200_show_float(72, 16*1, L1_norm, 3, 1);     
//        ips200_show_string(0, 16*2, "L2_Norm: "); ips200_show_float(72, 16*2, L2_norm, 3, 1);     
//        ips200_show_string(0, 16*3, "L3_Norm: "); ips200_show_float(72, 16*3, L3_norm, 3, 1);     
//        ips200_show_string(0, 16*4, "L4_Norm: "); ips200_show_float(72, 16*4, L4_norm, 3, 1);     
//    }
//    else
//    {
//        // 模式1：显示底层原始 ADC 值 (Raw)
//        ips200_show_string(0, 16*0, "--- Mode: RAW  ---"); 
//        ips200_show_string(0, 16*1, "L1_Raw : "); ips200_show_int32(72, 16*1, L1_raw, 3);     
//        ips200_show_string(0, 16*2, "L2_Raw : "); ips200_show_int32(72, 16*2, L2_raw, 3);     
//        ips200_show_string(0, 16*3, "L3_Raw : "); ips200_show_int32(72, 16*3, L3_raw, 3);     
//        ips200_show_string(0, 16*4, "L4_Raw : "); ips200_show_int32(72, 16*4, L4_raw, 3);     
//    }

//    // --------------------------------------------------------
//    // 4. 新增：把陀螺仪核心数据挂在最下方，方便随时查看
//    // --------------------------------------------------------
//    ips200_show_string(0, 16*6, "Angle Z: "); 
//    ips200_show_float(72, 16*6, angle_z, 4, 1);       // 实时显示绝对角度
//    
//    ips200_show_string(0, 16*7, "Gyro  Z: "); 
//    ips200_show_float(72, 16*7, true_gyro_z, 4, 1);   // 实时角速度，晃动车身看它跳动
//	
//	// 【新增的 Y 轴屏幕打印】
//    // 你可以用手猛烈抬起车头，看看这个数字会不会瞬间飙升到 150 以上！
//    ips200_show_string(0, 16*8, "Gyro  X: "); 
//    ips200_show_float(72, 16*8, true_gyro_x, 4, 1);
//	
//   // sprintf(buf, "duty_L:%d error_L:%.2f duty_R:%d error_R:%.2f\r\n", duty_L, error_L, duty_R, error_R);
//   // wireless_uart_send_string(buf);

}