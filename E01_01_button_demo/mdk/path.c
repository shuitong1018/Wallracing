#include "path.h"
#include <stdlib.h>

void Path_Init(void) {
    // 无需初始化
}

PathType_t Path_Detect(int L1, int L2, int L3, int L4) {
    // 判断是否平地（所有传感器都检测不到黑线）
    if(L1 < 100 && L2 < 100 && L3 < 100 && L4 < 100) {
        return PATH_FLAT;
    }
    
    // 判断是否直角弯（L1和L4差值大）
    if(abs(L1 - L4) > 500) {
        return PATH_RIGHT_ANGLE;
    }
    
    // 默认是直道
    return PATH_STRAIGHT;
}