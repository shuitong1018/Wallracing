#include "filter.h"
#include <stdio.h>
#include <stdlib.h>

int L1_filt, L2_filt, L3_filt, L4_filt;

// 历史数据缓存
static u16 history[4][FILTER_WINDOW_SIZE] = {0};
static u8 index = 0;

// 突变检测阈值（如果新值与上次滤波值的差值超过此值，认为是突变）
#define MUTATION_THRESHOLD 200

// 上一次的滤波值，用于突变检测
static int last_filt[4] = {0, 0, 0, 0};

void Filter_Init(void) {
    // 清空历史数据
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < FILTER_WINDOW_SIZE; j++) {
            history[i][j] = 0;
        }
        last_filt[i] = 0;
    }
    index = 0;
}

// 冒泡排序（用于中值滤波）
void Sort_Array(int arr[], int size) {
    for(int i = 0; i < size-1; i++) {
        for(int j = 0; j < size-1-i; j++) {
            if(arr[j] > arr[j+1]) {
                int temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}
int Comprehensive_Filter(int new_value, int sensor_index) {
    int limited_value;
    int temp_buffer[FILTER_WINDOW_SIZE];
    
    // 1. 第一步：限幅滤波（去除瞬间大跳变）
    if(abs(new_value - last_filt[sensor_index]) > MUTATION_THRESHOLD) {
        limited_value = last_filt[sensor_index];
    } else {
        limited_value = new_value;
    }
    
    // 2. 更新历史数据（使用限幅后的值）
    history[sensor_index][index] = limited_value;
    
    // 3. 第二步：中值滤波（去除脉冲噪声）
    for(int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        temp_buffer[i] = history[sensor_index][i];
    }
    Sort_Array(temp_buffer, FILTER_WINDOW_SIZE);
    
    // 4. 去掉最小值和最大值，对中间的值求平均
    u32 sum = 0;
    for(int i = 1; i < FILTER_WINDOW_SIZE-1; i++) {
        sum += temp_buffer[i];
    }
    int filtered_value = sum / (FILTER_WINDOW_SIZE - 2);
    
    // 5. 保存本次滤波值
    last_filt[sensor_index] = filtered_value;
    
    return filtered_value;
}
// 主滤波函数
void Filter_Process(int raw_L1, int raw_L2, int raw_L3, int raw_L4) {
    
    L1_filt = Comprehensive_Filter(raw_L1, 0);
    L2_filt = Comprehensive_Filter(raw_L2, 1);
    L3_filt = Comprehensive_Filter(raw_L3, 2);
    L4_filt = Comprehensive_Filter(raw_L4, 3);
    
    // 更新索引（必须在所有传感器处理完成后）
    index = (index + 1) % FILTER_WINDOW_SIZE;
    
    // 打印滤波后的值（调试用）
    // printf("L1:%d L2:%d L3:%d L4:%d\n", L1_filt, L2_filt, L3_filt, L4_filt);
}

// 如果需要单独获取某个传感器的滤波值
int Get_Filtered_Value(int sensor_index) {
    switch(sensor_index) {
        case 0: return L1_filt;
        case 1: return L2_filt;
        case 2: return L3_filt;
        case 3: return L4_filt;
        default: return 0;
    }
}

// 突变检测函数（返回是否有突变）
int Check_Mutation(int new_value, int sensor_index) {
    if(abs(new_value - last_filt[sensor_index]) > MUTATION_THRESHOLD) {
        return 1;  // 有突变
    }
    return 0;  // 无突变
}