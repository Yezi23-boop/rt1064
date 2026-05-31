# 路径跟踪PID实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将执行器的分段速度控制替换为位置式PID控制，实现0.5cm到点精度，消除稳态误差。

**Architecture:** 在motion_math.c中添加位置式PID模块，executor.c中调用PID替代原有分段速度控制。PID输出车体速度vx/vy，通过set_motion()送入现有控制链路，不破坏现有架构。

**Tech Stack:** C语言，RT1064 MCU，20ms控制周期，现有惯导系统

---

## 文件结构

### 修改的文件
- `project/user/src/motion_math.c` - 添加path_pid结构体和函数
- `project/user/inc/motion_math.h` - 添加path_pid声明
- `project/user/src/executor.c` - 使用path_pid替代分段速度控制
- `project/user/inc/drive_config.h` - 添加path_pid参数定义

### 新增的文件
- 无（在现有文件中添加）

---

## Task 1: 在motion_math.h中添加path_pid结构体声明

**Files:**
- Modify: `project/user/inc/motion_math.h`

- [ ] **Step 1: 读取当前motion_math.h内容**

```c
// 读取文件，了解当前结构
```

- [ ] **Step 2: 添加path_pid_struct定义**

在motion_math.h中添加以下内容：

```c
/** 路径跟踪位置式PID状态，输入输出节拍固定为20ms。 */
typedef struct
{
    float kp;                            /**< 比例系数。 */
    float ki;                            /**< 积分系数。 */
    float kd;                            /**< 微分系数。 */
    float integral;                      /**< 积分累积。 */
    float last_error;                    /**< 上一次误差。 */
    float last_output;                   /**< 上一次输出（用于限幅）。 */
    float max_output;                    /**< 最大输出限幅。 */
    float max_integral;                  /**< 积分限幅（防积分饱和）。 */
} path_pid_struct;
```

- [ ] **Step 3: 添加path_pid函数声明**

在motion_math.h中添加以下函数声明：

```c
/**
 * @brief 初始化路径跟踪位置式PID实例。
 * @param[out] pid 待初始化的PID状态。
 * @param[in] kp 比例系数。
 * @param[in] ki 积分系数。
 * @param[in] kd 微分系数。
 * @param[in] max_output 最大输出限幅。
 * @param[in] max_integral 积分限幅。
 */
void path_pid_init(path_pid_struct *pid, float kp, float ki, float kd, 
                   float max_output, float max_integral);

/**
 * @brief 清除PID历史状态和积分累积。
 * @param[in,out] pid 待重置的PID状态。
 */
void path_pid_reset(path_pid_struct *pid);

/**
 * @brief 按一次20ms采样更新路径跟踪位置式PID。
 * @param[in,out] pid 路径跟踪PID状态。
 * @param[in] error 位置误差，单位为cm。
 * @param[in] dt_s 采样周期，单位为秒。
 * @return 限幅后的速度输出，范围[-max_output, max_output]。
 */
float path_pid_update(path_pid_struct *pid, float error, float dt_s);

/**
 * @brief 获取当前积分值（用于调试）。
 * @param[in] pid PID状态。
 * @return 当前积分值。
 */
float path_pid_get_integral(const path_pid_struct *pid);
```

- [ ] **Step 4: 验证修改**

检查motion_math.h文件，确保：
1. path_pid_struct定义正确
2. 函数声明完整
3. 注释清晰

---

## Task 2: 在motion_math.c中实现path_pid函数

**Files:**
- Modify: `project/user/src/motion_math.c`

- [ ] **Step 1: 读取当前motion_math.c内容**

```c
// 读取文件，了解当前结构
```

- [ ] **Step 2: 添加path_pid_init函数**

在motion_math.c中添加以下函数：

```c
void path_pid_init(path_pid_struct *pid, float kp, float ki, float kd, 
                   float max_output, float max_integral)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->max_integral = max_integral;
    path_pid_reset(pid);
}
```

- [ ] **Step 3: 添加path_pid_reset函数**

```c
void path_pid_reset(path_pid_struct *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
}
```

- [ ] **Step 4: 添加path_pid_update函数**

```c
float path_pid_update(path_pid_struct *pid, float error, float dt_s)
{
    float p_term, i_term, d_term;
    float output;

    /* 1. 比例项 */
    p_term = pid->kp * error;

    /* 2. 积分项（带限幅，防积分饱和） */
    pid->integral += error * dt_s;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    i_term = pid->ki * pid->integral;

    /* 3. 微分项 */
    d_term = pid->kd * (error - pid->last_error) / dt_s;
    pid->last_error = error;

    /* 4. 计算总输出 */
    output = p_term + i_term + d_term;

    /* 5. 输出限幅 */
    output = limit_float(output, -pid->max_output, pid->max_output);

    pid->last_output = output;
    return output;
}
```

- [ ] **Step 5: 添加path_pid_get_integral函数**

```c
float path_pid_get_integral(const path_pid_struct *pid)
{
    return pid->integral;
}
```

- [ ] **Step 6: 验证修改**

检查motion_math.c文件，确保：
1. 函数实现正确
2. 使用了已有的limit_float函数
3. 注释清晰

---

## Task 3: 在drive_config.h中添加path_pid参数定义

**Files:**
- Modify: `project/user/inc/drive_config.h`

- [ ] **Step 1: 读取当前drive_config.h内容**

```c
// 读取文件，了解当前参数定义
```

- [ ] **Step 2: 添加path_pid参数定义**

在drive_config.h中添加以下参数：

```c
/** 路径跟踪PID比例系数；误差单位为cm，输出为归一化速度。 */
#define PATH_KP (0.5f)
/** 路径跟踪PID积分系数；用于消除稳态误差。 */
#define PATH_KI (0.01f)
/** 路径跟踪PID微分系数；用于减少超调。 */
#define PATH_KD (0.1f)
/** 路径跟踪PID最大输出速度，归一化到MAX_WHEEL_TARGET_COUNT。 */
#define PATH_MAX_SPEED (0.6f)
/** 路径跟踪PID积分限幅，防积分饱和。 */
#define PATH_MAX_INTEGRAL (0.3f)
/** 路径跟踪PID到点阈值，单位cm。 */
#define PATH_ARRIVAL_THRESHOLD_CM (0.5f)
```

- [ ] **Step 3: 验证修改**

检查drive_config.h文件，确保：
1. 参数定义正确
2. 注释清晰
3. 参数值合理

---

## Task 4: 修改executor.c使用path_pid替代分段速度控制

**Files:**
- Modify: `project/user/src/executor.c`

- [ ] **Step 1: 读取当前executor.c内容**

```c
// 读取文件，了解当前实现
```

- [ ] **Step 2: 添加path_pid头文件引用**

在executor.c顶部添加：

```c
#include "motion_math.h"  // 包含path_pid定义
```

- [ ] **Step 3: 添加path_pid实例**

在executor.c中添加静态变量：

```c
/* 路径跟踪PID实例 */
static path_pid_struct x_pid;
static path_pid_struct y_pid;
```

- [ ] **Step 4: 添加executor_init函数**

在executor.c中添加初始化函数：

```c
void executor_init(void)
{
    /* 初始化路径跟踪PID */
    path_pid_init(&x_pid, PATH_KP, PATH_KI, PATH_KD, PATH_MAX_SPEED, PATH_MAX_INTEGRAL);
    path_pid_init(&y_pid, PATH_KP, PATH_KI, PATH_KD, PATH_MAX_SPEED, PATH_MAX_INTEGRAL);
}
```

- [ ] **Step 5: 修改is_axis_arrived函数**

将到点阈值从2cm改为0.5cm：

```c
static uint8 is_axis_arrived(float target_x, float target_y, char action)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;

    if (0 != action_is_x_axis(action)) {
        return ((abs_float(dx) < PATH_ARRIVAL_THRESHOLD_CM) &&
                (abs_float(dy) < PATH_ARRIVAL_THRESHOLD_CM));
    }
    if (0 != action_is_y_axis(action)) {
        return ((abs_float(dy) < PATH_ARRIVAL_THRESHOLD_CM) &&
                (abs_float(dx) < PATH_ARRIVAL_THRESHOLD_CM));
    }
    return (sqrtf(dx * dx + dy * dy) < PATH_ARRIVAL_THRESHOLD_CM);
}
```

- [ ] **Step 6: 替换move_to_target函数**

将原有的分段速度控制替换为PID控制：

```c
static void move_to_target(float target_x, float target_y, char action)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;
    float vx_world, vy_world;
    float vx_body, vy_body;

    /* 使用PID计算世界坐标速度 */
    vx_world = path_pid_update(&x_pid, dx, CONTROL_DT_S);
    vy_world = path_pid_update(&y_pid, dy, CONTROL_DT_S);

    /* 世界坐标转车体坐标 */
    world_velocity_to_body(vx_world, vy_world, pose->yaw_deg, &vx_body, &vy_body);

    /* 设置运动 */
    set_motion(vx_body, vy_body);
}
```

- [ ] **Step 7: 修改executor_start函数**

在executor_start中初始化PID：

```c
void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row_param, uint8 start_col_param, uint8 single_step)
{
    /* 参数检查 */
    if (waypoints == NULL || count == 0) {
        exec_state = EXEC_STATE_ERROR;
        exec_error = EXEC_ERROR_MAP;
        return;
    }

    /* 保存路径数据 */
    exec_waypoints = waypoints;
    exec_waypoint_count = count;
    start_row = start_row_param;
    start_col = start_col_param;
    single_step_mode = single_step;

    /* 重置状态 */
    current_step = 0;
    exec_error = EXEC_ERROR_NONE;

    /* 重置PID */
    path_pid_reset(&x_pid);
    path_pid_reset(&y_pid);

    /* 重置位姿，以起点为原点 */
    drive_pose_reset(0.0f, 0.0f, 0.0f);

    /* 设置初始状态 */
    if (single_step_mode) {
        exec_state = EXEC_STATE_PAUSED;
    } else {
        exec_state = EXEC_STATE_RUNNING;
    }
}
```

- [ ] **Step 8: 修改executor_stop函数**

在executor_stop中重置PID：

```c
void executor_stop(void)
{
    stop_motion();
    exec_state = EXEC_STATE_IDLE;
    exec_error = EXEC_ERROR_NONE;
    exec_waypoints = NULL;
    exec_waypoint_count = 0;
    current_step = 0;

    /* 重置PID */
    path_pid_reset(&x_pid);
    path_pid_reset(&y_pid);
}
```

- [ ] **Step 9: 修改executor_update_20ms函数**

在到达目标点时重置PID：

```c
void executor_update_20ms(void)
{
    /* 只在运行状态执行 */
    if (exec_state != EXEC_STATE_RUNNING) {
        return;
    }

    /* 检查是否完成所有步骤 */
    if (current_step >= exec_waypoint_count) {
        stop_motion();
        exec_state = EXEC_STATE_DONE;
        return;
    }

    /* 获取当前目标 */
    const waypoint_struct *wp = &exec_waypoints[current_step];
    float target_x, target_y;
    grid_to_physical(wp->row, wp->col, &target_x, &target_y);

    if (is_axis_arrived(target_x, target_y, wp->action)) {
        /* 到达目标点 */
        set_motion(0.0f, 0.0f);
        current_step++;

        /* 重置PID（换下一个waypoint时） */
        path_pid_reset(&x_pid);
        path_pid_reset(&y_pid);

        if (single_step_mode) {
            stop_motion();
            exec_state = EXEC_STATE_PAUSED;
        }
    } else {
        /* 向目标移动 */
        move_to_target(target_x, target_y, wp->action);
    }
}
```

- [ ] **Step 10: 验证修改**

检查executor.c文件，确保：
1. 头文件引用正确
2. PID实例定义正确
3. 初始化和重置逻辑正确
4. move_to_target使用PID
5. 到点阈值使用PATH_ARRIVAL_THRESHOLD_CM

---

## Task 5: 在main.c中调用executor_init

**Files:**
- Modify: `project/user/src/main.c`

- [ ] **Step 1: 读取当前main.c内容**

```c
// 读取文件，了解当前初始化流程
```

- [ ] **Step 2: 添加executor头文件引用**

在main.c顶部添加：

```c
#include "executor.h"  // 包含executor_init声明
```

- [ ] **Step 3: 在main函数中调用executor_init**

在control_init()之后调用executor_init()：

```c
int main(void)
{
    uint8 control_init_state;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    wireless_uart_init();
    openart_uart_init();
    control_init_state = control_init();
    executor_init();  // 初始化执行器PID

    app_init();

    while (1)
    {
        app_poll();
    }
}
```

- [ ] **Step 4: 验证修改**

检查main.c文件，确保：
1. 头文件引用正确
2. executor_init()调用位置正确
3. 初始化顺序正确

---

## Task 6: 添加调试接口

**Files:**
- Modify: `project/user/src/executor.c`

- [ ] **Step 1: 添加调试输出函数**

在executor.c中添加调试函数：

```c
void executor_debug_output(void)
{
    const drive_pose_struct *pose = drive_pose_get();
    const waypoint_struct *wp;
    float target_x, target_y;

    if (exec_state != EXEC_STATE_RUNNING) {
        return;
    }

    wp = &exec_waypoints[current_step];
    grid_to_physical(wp->row, wp->col, &target_x, &target_y);

    printf("EXEC: target=(%.2f,%.2f) current=(%.2f,%.2f) error=(%.2f,%.2f)\r\n",
           target_x, target_y, pose->x_cm, pose->y_cm,
           target_x - pose->x_cm, target_y - pose->y_cm);
    printf("PID: x_integral=%.3f y_integral=%.3f\r\n",
           path_pid_get_integral(&x_pid), path_pid_get_integral(&y_pid));
}
```

- [ ] **Step 2: 在executor.h中添加调试函数声明**

在executor.h中添加：

```c
/**
 * @brief 输出执行器调试信息。
 * @note 用于调试路径跟踪PID。
 */
void executor_debug_output(void);
```

- [ ] **Step 3: 验证修改**

确保调试函数可以正常调用。

---

## Task 7: 编译验证

**Files:**
- 无

- [ ] **Step 1: 更新Keil工程文件**

如果添加了新的头文件引用，需要更新Keil工程文件。

- [ ] **Step 2: 编译项目**

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

- [ ] **Step 3: 检查编译结果**

确保：
1. 0 Error(s)
2. 0 Warning(s)
3. 没有类型不匹配
4. 没有未定义符号

---

## Task 8: 测试验证

**Files:**
- 无

- [ ] **Step 1: 下载程序**

```powershell
D:\Keil_v5\UV4\UV4.exe -f "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

- [ ] **Step 2: 复位MCU**

```powershell
uvx pyocd reset -t mimxrt1064
```

- [ ] **Step 3: 测试基本运动**

1. 选择一个简单的地图
2. 执行求解和执行
3. 观察车模运动是否平滑
4. 观察是否能精确到达目标点

- [ ] **Step 4: 测试到点精度**

1. 选择一个需要精确到点的地图
2. 执行求解和执行
3. 测量实际到点误差
4. 确认误差<0.5cm

- [ ] **Step 5: 调整PID参数**

如果精度不够或运动不平滑：
1. 增加PATH_KP：提高响应速度
2. 增加PATH_KI：消除稳态误差
3. 增加PATH_KD：减少超调
4. 调整PATH_MAX_SPEED：限制最大速度

---

## 参数调优指南

### 初始参数
```c
#define PATH_KP (0.5f)
#define PATH_KI (0.01f)
#define PATH_KD (0.1f)
#define PATH_MAX_SPEED (0.6f)
#define PATH_MAX_INTEGRAL (0.3f)
#define PATH_ARRIVAL_THRESHOLD_CM (0.5f)
```

### 调参步骤
1. **先调KP**：从小到大，直到响应快且不振荡
2. **再调KI**：从小到大，消除稳态误差
3. **最后调KD**：从小到大，减少超调

### 预期效果
1. **消除稳态误差**：积分项会累积误差，最终输出足够大的速度克服摩擦
2. **运动平滑**：PID输出连续变化，不会阶跃
3. **高精度**：通过积分项消除1-2cm的稳态误差，达到0.5cm精度

---

## 注意事项

1. **不破坏现有架构**：path_pid输出vx, vy，通过set_motion()送入现有控制链路
2. **符合惯导要求**：只输出vx, vy，不输出vz，保持无旋转
3. **保守设计**：使用保守的PID参数，优先稳定性和精度
4. **可调试**：添加调试接口，方便参数调优
