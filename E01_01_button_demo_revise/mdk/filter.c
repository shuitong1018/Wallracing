#include "filter.h"

//==============================================================================
// 全局变量
//==============================================================================
int L1_filt, L2_filt, L3_filt, L4_filt;
static uint16 history[4][FILTER_WINDOW_SIZE];
static uint8 index;

//==============================================================================
// 滤波初始化
//==============================================================================
void Filter_Init(void)
{
    uint8 i, j;
    
    for(i = 0; i < 4; i++) {
        for(j = 0; j < FILTER_WINDOW_SIZE; j++) {
            history[i][j] = 0;
        }
    }
    index = 0;
    L1_filt = L2_filt = L3_filt = L4_filt = 0;
}

//==============================================================================
// 滑动平均滤波处理
//==============================================================================
void Filter_Process(void)
{
    uint8 i;
    uint32 sum1, sum2, sum3, sum4;
    
    // 更新历史数据
    history[0][index] = L1_filt;
    history[1][index] = L2_filt;
    history[2][index] = L3_filt;
    history[3][index] = L4_filt;
    
    // 计算总和
    sum1 = sum2 = sum3 = sum4 = 0;
    for(i = 0; i < FILTER_WINDOW_SIZE; i++) {
        sum1 += history[0][i];
        sum2 += history[1][i];
        sum3 += history[2][i];
        sum4 += history[3][i];
    }
    
    // 计算平均值
    L1_filt = sum1 / FILTER_WINDOW_SIZE;
    L2_filt = sum2 / FILTER_WINDOW_SIZE;
    L3_filt = sum3 / FILTER_WINDOW_SIZE;
    L4_filt = sum4 / FILTER_WINDOW_SIZE;
    
    // 更新索引
    index = (index + 1) % FILTER_WINDOW_SIZE;
}