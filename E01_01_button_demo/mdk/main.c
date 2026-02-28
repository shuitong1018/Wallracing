#include "config.h"
#include "sensor.h"
#include "filter.h"
#include "path.h"
#include "error.h"
#include "motor.h"

// 全局变量
int base_speed = BASE_SPEED_NORMAL;
int target_turn = 0;
int target_speed_diff = 0;
int error = 0;

// 延时函数
void delay_ms(u16 ms) {
    u16 i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 4000; j++);
}

void System_Init(void) {
    // 初始化各个模块
    Sensor_Init();
    Filter_Init();
    Path_Init();
    Error_Init();
    Motor_Init();
    Antenna_Init();
    Stereo_Init();
    
    // 等待系统稳定
    delay_ms(100);
}

void main(void) {
    // 1. 系统初始化
    System_Init();
    
    // 主循环
    while(1) {
        // 2. 获取传感器数据
        Sensor_Read();
        
        // 3. 滤波处理
        Filter_Process(L1, L2, L3, L4);
        
        // 4. 判断路径类型
        PathType_t path = Path_Detect(L1_filt, L2_filt, L3_filt, L4_filt);
        
        // 5. 根据路径调整基础速度
        switch(path) {
            case PATH_RIGHT_ANGLE:
                base_speed = BASE_SPEED_TURN;   // 直角弯减速
                break;
            case PATH_FLAT:
                base_speed = BASE_SPEED_FLAT;   // 平地加速
                break;
            default:
                base_speed = BASE_SPEED_NORMAL; // 正常速度
                break;
        }
        
        // 6. 计算误差（差比和算法）
        error = Error_Calculate(L1_filt, L2_filt, L3_filt, L4_filt);
        
        // 7. PDD（外环右移）（） -> target_turn
        // PID控制算法部分 
        // target_turn = PDD_Outer_Loop(error);
        target_turn = 0;  // 临时赋值
        
        // 8. PDD（内环速度环）（） -> target_seed_diff
        // PID控制算法部分
        // target_speed_diff = PDD_Inner_Loop(target_turn);
        target_speed_diff = 0;  // 临时赋值
        
        // 9. 电机控制
        Motor_Control(base_speed, target_speed_diff);
       
        
        // 10. 控制周期延时
        delay_ms(CONTROL_CYCLE_MS);
    }
}