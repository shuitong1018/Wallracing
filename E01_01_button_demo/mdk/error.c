#include "error.h"

// 传感器位置权重
#define POS_L1  -3
#define POS_L2  -1
#define POS_L3   1
#define POS_L4   3

void Error_Init(void) {
    // 无需初始化
}

int Error_Calculate(int L1, int L2, int L3, int L4) {
    // 差比和算法：error = (Σ(值×位置)) / (Σ值)
    
    // 1. 计算加权和（分子）
    int weighted_sum = L1 * POS_L1 + L2 * POS_L2 + L3 * POS_L3 + L4 * POS_L4;
    
    // 2. 计算总和（分母）
    int total_value = L1 + L2 + L3 + L4;
    
    // 3. 防止除零
    if(total_value == 0) {
        return 0;
    }
    
    // 4. 计算差比和误差，并放大
    int error = weighted_sum * ERROR_SCALE_FACTOR / total_value;
    
    // 5. 限幅
    if(error > ERROR_MAX) error = ERROR_MAX;
    if(error < ERROR_MIN) error = ERROR_MIN;
    
    return error;
}