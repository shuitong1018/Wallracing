/*#include "path.h"
#include "zf_common_headfile.h"

//==============================================================================
// 常量定义
//==============================================================================
// 传感器数量
#define SENSOR_COUNT            4

// 历史数据深度
#define HISTORY_DEPTH           5

// 阈值常量 - 需要根据实际调试调整
#define THRESHOLD_LINE_PRESENT      30.0f   // 检测到线的阈值
#define THRESHOLD_STRONG_SIGNAL     60.0f   // 强信号阈值
#define THRESHOLD_WEAK_SIGNAL       15.0f   // 弱信号阈值
#define THRESHOLD_LOST_LINE         10.0f   // 丢线阈值
#define THRESHOLD_SIDE_DIFF         40.0f   // 左右侧差异阈值

// 状态计数阈值
#define STRAIGHT_COUNT_THRESHOLD    3
#define ANGLE_COUNT_MAX             30
#define CIRCLE_COUNT_THRESHOLD      5
#define CROSS_COUNT_THRESHOLD       3
#define ROUNDABOUT_EDGE_MAX         6
#define LOST_LINE_MAX                20

// 误差阈值
#define ERROR_STRAIGHT_THRESHOLD    20.0f   // 直道误差阈值
#define ERROR_CIRCLE_THRESHOLD      50.0f   // 弯道误差阈值

//==============================================================================
// 数据结构定义
//==============================================================================
typedef struct {
    float values[4];          // L1,L2,L3,L4
    float error;              // 对应的误差值
    PathType_t detected_type;
    uint32_t timestamp;
} SensorHistory_t;

typedef struct {
    // 传感器历史
    SensorHistory_t history[HISTORY_DEPTH];
    int history_index;
    
    // 路径状态
    PathType_t current_path;
    PathType_t previous_path;
    int path_stable_count;
    
    // 特殊路况计数器
    int angle_counter;
    int cross_counter;
    int circle_counter;
    int roundabout_edge_counter;
    int lost_line_counter;
    int straight_counter;
    
    // 特征标志
    int left_bias_count;      // 左侧偏置计数
    int right_bias_count;     // 右侧偏置计数
    float last_error;
    float last_valid_error;    // 上次有效误差
    
    // 环岛状态
    int in_roundabout;
    int roundabout_phase;      // 0:入口,1:岛内,2:出口
    int roundabout_edge_count;
    float roundabout_entry_error;
    
    // 调试信息
    uint32_t last_print_time;
    
} PathController_t;

//==============================================================================
// 静态变量
//==============================================================================
static PathController_t controller;
static const char* path_type_names[] = {
    "STRAIGHT", "LEFT_ANGLE", "RIGHT_ANGLE", "CIRCULAR",
    "CROSSROAD", "ZIGZAG", "ROUNDABOUT", "FLAT"
};

static const char* action_names[] = {
    "TRACKING", "TURN_LEFT_HARD", "TURN_RIGHT_HARD", "TURN_LEFT_SMOOTH",
    "TURN_RIGHT_SMOOTH", "TRACKING_FAST", "ROUNDABOUT", "STOP", "FORWARD_STRAIGHT"
};

//==============================================================================
// 内部函数声明
//==============================================================================
static void update_history(float L1, float L2, float L3, float L4, float error, PathType_t type);
static PathType_t get_stable_path_type(void);
static int detect_lost_line(float L1, float L2, float L3, float L4);
static int detect_crossroad(float L1, float L2, float L3, float L4);
static int detect_angle(float L1, float L2, float L3, float L4);
static int detect_circular(float L1, float L2, float L3, float L4, float error);
static int detect_zigzag(float L1, float L2, float L3, float L4, float error);
static int detect_roundabout(float L1, float L2, float L3, float L4);
static int get_bias_direction(float L1, float L2, float L3, float L4);
static int is_exiting_special_element(void);

//==============================================================================
// 外部函数引用
//==============================================================================
extern float Error_Calculate(float L1_norm, float L2_norm, float L3_norm, float L4_norm);
extern float my_abs(float x);

//==============================================================================
// 路径初始化
//==============================================================================
void Path_Init(void)
{
    // 初始化控制器
    memset(&controller, 0, sizeof(PathController_t));
    
    // 初始化历史数据
    for(int i = 0; i < HISTORY_DEPTH; i++) {
        controller.history[i].values[0] = 0;
        controller.history[i].values[1] = 0;
        controller.history[i].values[2] = 0;
        controller.history[i].values[3] = 0;
        controller.history[i].error = 0;
        controller.history[i].detected_type = PATH_STRAIGHT;  // 默认直道
        controller.history[i].timestamp = 0;
    }
    
    controller.history_index = 0;
    controller.current_path = PATH_STRAIGHT;
    controller.previous_path = PATH_STRAIGHT;
    controller.path_stable_count = 0;
    
    controller.angle_counter = 0;
    controller.cross_counter = 0;
    controller.circle_counter = 0;
    controller.roundabout_edge_counter = 0;
    controller.lost_line_counter = 0;
    controller.straight_counter = 0;
    
    controller.left_bias_count = 0;
    controller.right_bias_count = 0;
    controller.last_error = 0;
    controller.last_valid_error = 0;
    
    controller.in_roundabout = 0;
    controller.roundabout_phase = 0;
    controller.roundabout_edge_count = 0;
    controller.roundabout_entry_error = 0;
    
    controller.last_print_time = 0;
}

//==============================================================================
// 增强版路径类型检测
//==============================================================================
PathType_t Path_Detect(float L1, float L2, float L3, float L4)
{
    PathType_t detected = PATH_STRAIGHT;  // 默认直道
    float error = Error_Calculate(L1, L2, L3, L4);
    
    // 1. 检测是否丢线（所有电感信号都很弱）
    if(detect_lost_line(L1, L2, L3, L4)) {
        controller.lost_line_counter++;
        
        // 在特殊路况中丢线是正常的，需要结合历史状态判断
        if(controller.current_path == PATH_LEFT_ANGLE || 
           controller.current_path == PATH_RIGHT_ANGLE ||
           controller.current_path == PATH_ROUNDABOUT) {
            // 过弯或环岛时丢线正常，保持状态
            detected = controller.current_path;
            update_history(L1, L2, L3, L4, error, detected);
            return detected;
        }
        
        // 短暂丢线可能只是晃动
        if(controller.lost_line_counter < 5) {
            detected = controller.current_path;
        } else {
            detected = PATH_FLAT;
        }
        
        update_history(L1, L2, L3, L4, error, detected);
        return detected;
    } else {
        controller.lost_line_counter = 0;
    }
    
    // 2. 检测十字路口（四个电感都有较强信号）
    if(detect_crossroad(L1, L2, L3, L4)) {
        controller.cross_counter++;
        if(controller.cross_counter > CROSS_COUNT_THRESHOLD) {
            detected = PATH_CROSSROAD;
            update_history(L1, L2, L3, L4, error, detected);
            return detected;
        }
    } else {
        controller.cross_counter = 0;
    }
    
    // 3. 检测环岛
    if(detect_roundabout(L1, L2, L3, L4)) {
        detected = PATH_ROUNDABOUT;
        update_history(L1, L2, L3, L4, error, detected);
        return detected;
    }
    
    // 4. 检测直角弯
    if(detect_angle(L1, L2, L3, L4)) {
        int direction = get_bias_direction(L1, L2, L3, L4);
        if(direction < 0) {
            detected = PATH_LEFT_ANGLE;
        } else if(direction > 0) {
            detected = PATH_RIGHT_ANGLE;
        }
        
        if(detected != PATH_STRAIGHT) {
            controller.angle_counter++;
            update_history(L1, L2, L3, L4, error, detected);
            return detected;
        }
    }
    
    // 5. 检测圆形弯道
    if(detect_circular(L1, L2, L3, L4, error)) {
        detected = PATH_CIRCULAR;
        update_history(L1, L2, L3, L4, error, detected);
        return detected;
    }
    
    // 6. 检测折线
    if(detect_zigzag(L1, L2, L3, L4, error)) {
        detected = PATH_ZIGZAG;
        update_history(L1, L2, L3, L4, error, detected);
        return detected;
    }
    
    // 7. 默认返回直道
    detected = PATH_STRAIGHT;
    controller.straight_counter++;
    update_history(L1, L2, L3, L4, error, detected);
    
    return detected;
}

//==============================================================================
// 检测丢线情况
//==============================================================================
static int detect_lost_line(float L1, float L2, float L3, float L4)
{
    return (L1 < THRESHOLD_LOST_LINE && L2 < THRESHOLD_LOST_LINE && 
            L3 < THRESHOLD_LOST_LINE && L4 < THRESHOLD_LOST_LINE);
}

//==============================================================================
// 检测十字路口
//==============================================================================
static int detect_crossroad(float L1, float L2, float L3, float L4)
{
    // 十字路口特征：四个电感都有较强信号，且中间两个电感信号强于两边
    float middle_avg = (L2 + L3) / 2.0f;
    float side_avg = (L1 + L4) / 2.0f;
    
    return (L1 > THRESHOLD_STRONG_SIGNAL && 
            L2 > THRESHOLD_STRONG_SIGNAL && 
            L3 > THRESHOLD_STRONG_SIGNAL && 
            L4 > THRESHOLD_STRONG_SIGNAL &&
            middle_avg > side_avg * 1.2f);
}

//==============================================================================
// 检测直角弯
//==============================================================================
static int detect_angle(float L1, float L2, float L3, float L4)
{
    // 直角弯特征：某一侧电感信号很强，另一侧很弱
    float left_sum = L1 + L2;
    float right_sum = L3 + L4;
    float side_diff = my_abs(left_sum - right_sum);
    
    // 左右差异大，且单侧信号强
    if(side_diff > THRESHOLD_SIDE_DIFF * 2) {
        if((L1 > THRESHOLD_STRONG_SIGNAL && L4 < THRESHOLD_WEAK_SIGNAL) ||
           (L4 > THRESHOLD_STRONG_SIGNAL && L1 < THRESHOLD_WEAK_SIGNAL)) {
            return 1;
        }
    }
    
    return 0;
}

//==============================================================================
// 检测圆形弯道
//==============================================================================
static int detect_circular(float L1, float L2, float L3, float L4, float error)
{
    // 圆形弯道特征：持续的较大误差，且误差方向稳定
    static int same_direction_count = 0;
    static float last_error_direction = 0;
    
    float current_direction = (error > 0) ? 1 : ((error < 0) ? -1 : 0);
    
    // 误差足够大且方向一致
    if(my_abs(error) > ERROR_CIRCLE_THRESHOLD) {
        if(current_direction == last_error_direction) {
            same_direction_count++;
        } else {
            same_direction_count = 1;
        }
    } else {
        same_direction_count = 0;
    }
    
    last_error_direction = current_direction;
    
    return (same_direction_count > CIRCLE_COUNT_THRESHOLD);
}

//==============================================================================
// 检测折线
//==============================================================================
static int detect_zigzag(float L1, float L2, float L3, float L4, float error)
{
    // 折线特征：误差快速变化
    static float last_error = 0;
    static int change_count = 0;
    
    float error_change = my_abs(error - last_error);
    
    if(error_change > 50.0f) {  // 误差变化大
        change_count++;
        if(change_count > 3) {
            change_count = 0;
            last_error = error;
            return 1;
        }
    } else {
        change_count = 0;
    }
    
    last_error = error;
    return 0;
}

//==============================================================================
// 检测环岛
//==============================================================================
static int detect_roundabout(float L1, float L2, float L3, float L4)
{
    // 环岛特征：长时间的单侧偏置
    int direction = get_bias_direction(L1, L2, L3, L4);
    
    if(direction < 0) {  // 左偏
        controller.left_bias_count++;
        controller.right_bias_count = 0;
    } else if(direction > 0) {  // 右偏
        controller.right_bias_count++;
        controller.left_bias_count = 0;
    } else {
        // 短暂回中不立即清零
        if(controller.left_bias_count > 0) {
            controller.left_bias_count = controller.left_bias_count > 2 ? controller.left_bias_count - 1 : 0;
        }
        if(controller.right_bias_count > 0) {
            controller.right_bias_count = controller.right_bias_count > 2 ? controller.right_bias_count - 1 : 0;
        }
    }
    
    // 连续多次同向偏置，可能是环岛
    if(controller.left_bias_count > 8 || controller.right_bias_count > 8) {
        return 1;
    }
    
    return 0;
}

//==============================================================================
// 获取偏置方向
//==============================================================================
static int get_bias_direction(float L1, float L2, float L3, float L4)
{
    float left_strength = L1 + L2;
    float right_strength = L3 + L4;
    
    if(left_strength > right_strength + THRESHOLD_SIDE_DIFF) {
        return -1;  // 左偏
    } else if(right_strength > left_strength + THRESHOLD_SIDE_DIFF) {
        return 1;   // 右偏
    } else {
        return 0;   // 居中
    }
}

//==============================================================================
// 更新传感器历史
//==============================================================================
static void update_history(float L1, float L2, float L3, float L4, float error, PathType_t type)
{
    controller.history_index = (controller.history_index + 1) % HISTORY_DEPTH;
    
    controller.history[controller.history_index].values[0] = L1;
    controller.history[controller.history_index].values[1] = L2;
    controller.history[controller.history_index].values[2] = L3;
    controller.history[controller.history_index].values[3] = L4;
    controller.history[controller.history_index].error = error;
    controller.history[controller.history_index].detected_type = type;
    controller.history[controller.history_index].timestamp = systick_get_ms();
}

//==============================================================================
// 获取稳定的路径类型
//==============================================================================
static PathType_t get_stable_path_type(void)
{
    int count[PATH_TYPE_COUNT] = {0};
    int max_count = 0;
    PathType_t stable_type = PATH_STRAIGHT;
    
    // 统计历史数据
    for(int i = 0; i < HISTORY_DEPTH; i++) {
        PathType_t type = controller.history[i].detected_type;
        if(type < PATH_TYPE_COUNT) {
            count[type]++;
        }
    }
    
    // 找出现次数最多的类型
    for(int i = 0; i < PATH_TYPE_COUNT; i++) {
        if(count[i] > max_count) {
            max_count = count[i];
            stable_type = (PathType_t)i;
        }
    }
    
    // 需要达到一定置信度，否则保持当前路径
    if(max_count < HISTORY_DEPTH / 2) {
        return controller.current_path;
    }
    
    return stable_type;
}

//==============================================================================
// 获取路径方向
//==============================================================================
PathDirection_t Path_Get_Direction(float L1, float L2, float L3, float L4)
{
    float error = Error_Calculate(L1, L2, L3, L4);
    
    if(error < -ERROR_STRAIGHT_THRESHOLD) {
        return DIR_LEFT;
    } else if(error > ERROR_STRAIGHT_THRESHOLD) {
        return DIR_RIGHT;
    } else {
        return DIR_STRAIGHT;
    }
}

//==============================================================================
// 获取控制动作
//==============================================================================
PathAction_t Path_Get_Action(PathType_t path, float L1, float L2, float L3, float L4)
{
    PathAction_t action = ACTION_TRACKING;
    float error = Error_Calculate(L1, L2, L3, L4);
    int direction = get_bias_direction(L1, L2, L3, L4);
    
    // 更新路径状态
    controller.previous_path = controller.current_path;
    PathType_t stable_path = get_stable_path_type();
    if(stable_path != controller.current_path) {
        // 路径变化时，适当延时确认
        controller.path_stable_count++;
        if(controller.path_stable_count > STRAIGHT_COUNT_THRESHOLD) {
            controller.current_path = stable_path;
            controller.path_stable_count = 0;
        }
    } else {
        controller.path_stable_count = 0;
    }
    
    // 保存有效误差（非丢线时）
    if(!detect_lost_line(L1, L2, L3, L4)) {
        controller.last_valid_error = error;
    }
    
    // 根据不同路径类型生成控制动作
    switch(controller.current_path)
    {
        case PATH_STRAIGHT:
            // 直道：正常循迹
            action = ACTION_TRACKING;
            controller.angle_counter = 0;
            controller.circle_counter = 0;
            controller.roundabout_edge_counter = 0;
            break;
            
        case PATH_LEFT_ANGLE:
            // 左直角弯
            controller.angle_counter++;
            
            if(controller.angle_counter < 10) {
                // 入弯初期：强转向
                action = ACTION_TURN_LEFT_HARD;
            } else if(controller.angle_counter < 25) {
                // 弯道中：保持转向
                action = ACTION_TURN_LEFT_HARD;
            } else {
                // 准备出弯
                action = ACTION_TRACKING;
            }
            
            // 出弯判断
            if(controller.angle_counter > ANGLE_COUNT_MAX || 
               (my_abs(error) < ERROR_STRAIGHT_THRESHOLD && controller.angle_counter > 15)) {
                controller.angle_counter = 0;
            }
            break;
            
        case PATH_RIGHT_ANGLE:
            // 右直角弯
            controller.angle_counter++;
            
            if(controller.angle_counter < 10) {
                action = ACTION_TURN_RIGHT_HARD;
            } else if(controller.angle_counter < 25) {
                action = ACTION_TURN_RIGHT_HARD;
            } else {
                action = ACTION_TRACKING;
            }
            
            if(controller.angle_counter > ANGLE_COUNT_MAX || 
               (my_abs(error) < ERROR_STRAIGHT_THRESHOLD && controller.angle_counter > 15)) {
                controller.angle_counter = 0;
            }
            break;
            
        case PATH_CIRCULAR:
            // 圆形弯道：平滑转向
            controller.circle_counter++;
            if(error < -ERROR_CIRCLE_THRESHOLD) {
                action = ACTION_TURN_LEFT_SMOOTH;
            } else if(error > ERROR_CIRCLE_THRESHOLD) {
                action = ACTION_TURN_RIGHT_SMOOTH;
            } else {
                action = ACTION_TRACKING;
            }
            break;
            
        case PATH_CROSSROAD:
            // 十字路口：保持直行
            action = ACTION_FORWARD_STRAIGHT;
            controller.cross_counter++;
            
            // 出十字判断
            if(controller.cross_counter > CROSS_COUNT_THRESHOLD * 3) {
                controller.cross_counter = 0;
            }
            break;
            
        case PATH_ZIGZAG:
            // 折线：快速响应
            if(my_abs(error) > ERROR_CIRCLE_THRESHOLD * 1.5f) {
                if(error < 0) {
                    action = ACTION_TURN_LEFT_HARD;
                } else {
                    action = ACTION_TURN_RIGHT_HARD;
                }
            } else {
                action = ACTION_TRACKING_FAST;
            }
            break;
            
        case PATH_ROUNDABOUT:
            // 环岛处理
            if(!controller.in_roundabout) {
                // 首次进入环岛
                controller.in_roundabout = 1;
                controller.roundabout_phase = 0;  // 入口
                controller.roundabout_edge_count = 0;
                controller.roundabout_entry_error = error;
            }
            
            // 环岛内循迹
            if(controller.roundabout_phase == 0) {
                // 入口阶段：根据进入方向转向
                if(controller.left_bias_count > controller.right_bias_count) {
                    action = ACTION_TURN_LEFT_SMOOTH;
                } else {
                    action = ACTION_TURN_RIGHT_SMOOTH;
                }
                
                // 进入环岛后切换状态
                if(my_abs(error) > ERROR_CIRCLE_THRESHOLD * 1.5f) {
                    controller.roundabout_phase = 1;  // 进入岛内
                }
            } else if(controller.roundabout_phase == 1) {
                // 岛内阶段：稳定循迹
                if(direction < 0) {
                    action = ACTION_TURN_LEFT_SMOOTH;
                } else if(direction > 0) {
                    action = ACTION_TURN_RIGHT_SMOOTH;
                } else {
                    action = ACTION_TRACKING;
                }
                
                // 检测环岛边沿（丢线计数）
                if(detect_lost_line(L1, L2, L3, L4)) {
                    controller.roundabout_edge_count++;
                }
            }
            
            // 环岛出口检测
            if(controller.roundabout_edge_count >= ROUNDABOUT_EDGE_MAX) {
                // 准备出环岛
                controller.roundabout_phase = 2;
                action = ACTION_FORWARD_STRAIGHT;
                
                // 完全退出环岛
                if(my_abs(error) < ERROR_STRAIGHT_THRESHOLD) {
                    controller.in_roundabout = 0;
                    controller.roundabout_phase = 0;
                    controller.roundabout_edge_count = 0;
                }
            }
            break;
            
        case PATH_FLAT:
            // 平地/丢线：紧急处理
            controller.lost_line_counter++;
            if(controller.lost_line_counter < 10) {
                // 短暂丢线，使用上次有效误差进行循迹
                action = ACTION_TRACKING;
            } else {
                // 长时间丢线，停止
                action = ACTION_STOP;
            }
            break;
            
        default:
            action = ACTION_TRACKING;
            break;
    }
    
    controller.last_error = error;
    return action;
}

//==============================================================================
// 获取当前路径名称
//==============================================================================
const char* Path_GetName(PathType_t path)
{
    if(path >= 0 && path < PATH_TYPE_COUNT) {
        return path_type_names[path];
    }
    return "UNKNOWN";
}

//==============================================================================
// 获取动作名称
//==============================================================================
const char* Action_GetName(PathAction_t action)
{
    if(action >= 0 && action < ACTION_COUNT) {
        return action_names[action];
    }
    return "UNKNOWN";
}

//==============================================================================
// 获取当前路径状态
//==============================================================================
PathType_t Path_GetCurrent(void)
{
    return controller.current_path;
}

//==============================================================================
// 判断是否需要减速
//==============================================================================
int Path_NeedSlowDown(PathType_t path)
{
    switch(path) {
        case PATH_LEFT_ANGLE:
        case PATH_RIGHT_ANGLE:
        case PATH_ZIGZAG:
            return 2;  // 大幅减速
        case PATH_ROUNDABOUT:
            return 3;  // 最大减速
        case PATH_CIRCULAR:
            return 1;  // 轻微减速
        case PATH_CROSSROAD:
            return 1;  // 轻微减速
        default:
            return 0;  // 不需要减速
    }
}

//==============================================================================
// 获取推荐速度系数
//==============================================================================
float Path_GetSpeedFactor(PathType_t path)
{
    switch(path) {
        case PATH_STRAIGHT:
            return 1.0f;      // 全速
        case PATH_CIRCULAR:
            return 0.7f;      // 70%速度
        case PATH_CROSSROAD:
            return 0.6f;      // 60%速度
        case PATH_LEFT_ANGLE:
        case PATH_RIGHT_ANGLE:
            return 0.4f;      // 40%速度
        case PATH_ZIGZAG:
            return 0.35f;     // 35%速度
        case PATH_ROUNDABOUT:
            return 0.3f;      // 30%速度
        case PATH_FLAT:
            return 0.2f;      // 20%速度
        default:
            return 0.6f;
    }
}

//==============================================================================
// 重置路径检测状态
//==============================================================================
void Path_Reset(void)
{
    Path_Init();
}

//==============================================================================
// 路径调试信息打印
//==============================================================================
void Path_Debug_Print(float L1, float L2, float L3, float L4)
{
    uint32_t current_time = systick_get_ms();
    
    // 每200ms打印一次
    if(current_time - controller.last_print_time > 200) {
        controller.last_print_time = current_time;
        
        float error = Error_Calculate(L1, L2, L3, L4);
        PathType_t current = Path_GetCurrent();
        PathDirection_t dir = Path_Get_Direction(L1, L2, L3, L4);
        
        printf("Path: %s, Dir: %d, Error: %.1f, L1:%.1f L2:%.1f L3:%.1f L4:%.1f\r\n",
               Path_GetName(current), dir, error, L1, L2, L3, L4);
        
        // 打印特殊状态
        if(controller.in_roundabout) {
            printf("Roundabout: Phase=%d, Edge=%d\r\n", 
                   controller.roundabout_phase, controller.roundabout_edge_count);
        }
        
        // 打印偏置计数
        if(controller.left_bias_count > 0 || controller.right_bias_count > 0) {
            printf("Bias: L=%d, R=%d\r\n", controller.left_bias_count, controller.right_bias_count);
        }
    }
}*/