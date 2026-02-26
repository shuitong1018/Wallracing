/*********************************************************************************************************************
* STC32G Opensourec Library 即（STC32G 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是STC 开源库的一部分
*
* STC32G 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2024-08-01        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "my_function.h"

// *************************** 例程硬件连接说明 ***************************
// 使用 STC32G 缩微学习主板进行测试
//      将模块的电源接线端子与主板的驱动供电端子连接
//      将模块的信号接口使用配套排线与主板电机信号接口连接 引脚参考上方核心板连接
//      将主板与供电电池正确连接


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 主板电池供电 连接STC-FOC无刷驱动
// 2.若正常连接电机 可以看到电机周期正反转  若未连接电机  则会发现驱动板蓝灯闪烁(校准零点失败)
// 3.本例程将从调试串口输出电机转速信息，若正常驱动电机旋转，则可在串口助手看到以下示例内容：
//   right speed:0, left speed:0
//   right speed:-259, left speed:-219
//   right speed:-1111, left speed:-1037
//   right speed:-1917, left speed:-1848
//   right speed:-2676, left speed:-2620
//   .......
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

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
//int8 duty = 0;                                                                 // 占空比变量，范围为 -MAX_DUTY ~ MAX_DUTY，正负表示方向
//uint8 dir = 1;                                                                 // 方向标志位，0 表示占空比递减，1 表示占空比递增

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
																			 //双串级PID变量定义
float error_dir = 0;                                                           // 方向误差

unsigned char counter_dir = 0;                                                        // 方向计数器，用于控制PID外环定时更新计数
unsigned char counter_speed = 0;                                                      // 速度计数器，用于控制PID内环定时更新计数
unsigned char duty,speed_r,speed_l;                                                           // 占空比变量，范围为 -MAX_DUTY ~ MAX_DUTY，正负表示方向
unsigned char base_speed = 20; // 基础速度

void main()
{
    clock_init(SYSTEM_CLOCK_30M);  												// 初始化系统时钟为 30 MHz
	debug_init();                  												// 初始化调试串口，用于输出调试信息
	
	// 此处编写用户代码 例如外设初始化代码等
	
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
	pit_init(TIM_0, 1);
	system_init();
	
    while(1)
    {
		// 此处编写需要循环执行的代码
        
		//读取工字电感数据
		//get_encoder();
		//滤波，计算
		//得到error_dir
		





		// if(duty >= 0)                                                          			// 当占空比为非负数时，电机正转
        // {
        //     pwm_set_duty(PWM_L, duty * (PWM_DUTY_MAX / 100) + (PWM_DUTY_MAX / 10));     // 计算并输出占空比（加上 10% 信号死区）
        //     gpio_set_level(DIR_L, 0);                                          			// 输出电机旋转方向信号

        //     pwm_set_duty(PWM_R, duty * (PWM_DUTY_MAX / 100) + (PWM_DUTY_MAX / 10));     // 计算并输出占空比（加上 10% 信号死区）
        //     gpio_set_level(DIR_R, 0);                                          			// 输出电机旋转方向信号
        // }
        // else                                                                   			// 电机反转
        // {
        //     pwm_set_duty(PWM_L, (-duty) * (PWM_DUTY_MAX / 100) + (PWM_DUTY_MAX / 10));  // 计算并输出占空比（加上 10% 信号死区）
		// 	gpio_set_level(DIR_L, 1);                                          			// 输出电机旋转方向信号

        //     pwm_set_duty(PWM_R, (-duty) * (PWM_DUTY_MAX / 100) + (PWM_DUTY_MAX / 10));  // 计算并输出占空比（加上 10% 信号死区）
		// 	gpio_set_level(DIR_R, 1);                                          			// 输出电机旋转方向信号
        // }
		
		
        // if(dir)                                                                 		// 当方向标志为 1 时，占空比递增（加速）
        // {		
        //     duty ++;                                                            		// 正向计数
        //     if(duty >= MAX_DUTY)                                                		// 达到最大值
        //     {		
		// 		dir = 0;                                                    			// 改变方向标志，后续占空比将递减
		// 	}
        // }
        // else																			// 当方向标志为 0 时，占空比递减（减速至反转）
        // {
        //     duty --;                                                            		// 反向计数
        //     if(duty <= -MAX_DUTY)                                               		// 达到最小值
        //     {
		// 		dir = 1;                                                     			// 改变方向标志，后续占空比将递增
		// 	}
        // }
		
		// right_encoder_data = encoder_get_count(ENCODER_R);	                    		// 获取右电机编码器计数数据
		// left_encoder_data  = encoder_get_count(ENCODER_L);	                    		// 获取左电机编码器计数数据
		
		// encoder_clear_count(ENCODER_R);	                                    			// 清零右电机编码器计数
		// encoder_clear_count(ENCODER_L);                                       			// 清零左电机编码器计数
		
		
		// printf("right speed:%d, left speed:%d\r\n", right_encoder_data, left_encoder_data);	// 通过调试串口输出左右电机转速信息（编码器计数反映转速）
		
        // system_delay_ms(50);                                                  		 	// 延迟 50 ms，控制循环周期
		
		// 此处编写需要循环执行的代码
    }
}


void PID_calculate()
{
	counter_dir++;
	counter_speed++;
	if(counter_dir >= 5) // 每 5 次中断更新一次方向 PID
	{
		counter_dir = 0;            // 重置计数器
		pid_dir.actual = error_dir; // 设置实际值为方向误差
		PID_update(&pid_dir);
		pid_speed.target = pid_dir.out; // 将方向 PID 输出作为速度 PID 的目标值
	}
	if(counter_speed >= 2) // 每 2 次中断更新一次速度 PID
	{
		counter_speed = 0;            // 重置计数器
		pid_speed.actual = (encoder_get_count(ENCODER_R) - encoder_get_count(ENCODER_L)) / WHEEL_BASE; // 设置实际值为左右电机编码器计数的平均值
		PID_update(&pid_speed);
		duty = (unsigned char)pid_speed.out; // 获取速度 PID 输出作为两轮速度差
		speed_r = base_speed + duty; // 计算右轮速度
		speed_l = base_speed - duty; // 计算左轮速度
		if(duty > 0){
			pwm_set_duty(PWM_L, speed_l * (PWM_DUTY_MAX / 100) + (PWM_DUTY_MAX / 10));     // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_L, 0);                                          			// 输出电机旋转方向信号

        pwm_set_duty(PWM_R, speed_r * (PWM_DUTY_MAX / 100) + (PWM_DUTY_MAX / 10));     // 计算并输出占空比（加上 10% 信号死区）
        gpio_set_level(DIR_R, 0); // 输出电机旋转方向信号
		}
		else //应该用不上
		{
			pwm_set_duty(PWM_L, (-duty) * (PWM_DUTY_MAX / 100) + (PWM_DUTY_MAX / 10));  // 计算并输出占空比（加上 10% 信号死区）
			gpio_set_level(DIR_L, 1);                                          			// 输出电机旋转方向信号

			pwm_set_duty(PWM_R, (-duty) * (PWM_DUTY_MAX / 100) + (PWM_DUTY_MAX / 10));  // 计算并输出占空比（加上 10% 信号死区）
			gpio_set_level(DIR_R, 1);                                          			// 输出电机旋转方向信号
		}                                       			
	}
}
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：电机不转或者转速输出不正确
//      如果使用主板测试，主板必须要用电池供电
//      检查模块是否正确连接供电 必须使用电源线供电 不能使用杜邦线
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      联系咨询技术客服


