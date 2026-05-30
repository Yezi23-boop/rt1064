# Waypoint Executor 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现路径执行器，将 BFS 求解器输出的网格路径转换为实际控制指令，驱动麦轮底盘完成推箱子任务。

**Architecture:** 两层状态机（外层任务级，内层当前箱子执行），在 PIT 20ms 中断中运行，通过 `set_motion()` 控制底盘，使用 `drive_pose` 反馈判断到位。

**Tech Stack:** C (Keil MDK), NXP i.MX RT1064, SeekFree 库, 麦轮底盘

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `project/user/inc/executor.h` | 新增 | 类型定义、函数声明 |
| `project/user/src/executor.c` | 新增 | 状态机逻辑、坐标转换、到位判断 |
| `project/user/src/menu.c` | 修改 | 添加 RUN_MODE_STEP、K3/K4 处理、调用 executor |
| `project/user/src/screen.c` | 修改 | 显示 executor 状态 |
| `project/user/src/isr.c` | 修改 | 在 PIT 20ms 中断调用 executor_update_20ms() |
| `project/mdk/rt1064.uvprojx` | 修改 | 添加 executor.c 到 Keil 工程 |

---

## Task 1: 创建 executor.h

**Files:**
- Create: `project/user/inc/executor.h`

- [ ] **Step 1: 创建头文件，定义所有类型和函数声明**

```c
#ifndef EXECUTOR_H
#define EXECUTOR_H

#include "common.h"
#include "map_types.h"

/* 外层状态：任务级 */
typedef enum {
    EXEC_STATE_IDLE,      /* 等待启动 */
    EXEC_STATE_RUNNING,   /* 正在执行 */
    EXEC_STATE_PAUSED,    /* 单步暂停 */
    EXEC_STATE_DONE,      /* 全部完成 */
    EXEC_STATE_ERROR      /* 出错 */
} executor_state_enum;

/* 内层状态：当前箱子执行 */
typedef enum {
    STEP_MOVING,     /* 移动到下一个格子 */
    STEP_PUSHING,    /* 推箱子 */
    STEP_RETURNING   /* 推完回起点 */
} executor_step_enum;

/* 错误类型 */
typedef enum {
    EXEC_ERROR_NONE,
    EXEC_ERROR_TIMEOUT,    /* 单步超时（3秒） */
    EXEC_ERROR_MAP,        /* 地图数据异常 */
    EXEC_ERROR_POSE        /* pose 系统异常 */
} executor_error_enum;

/* 启动执行器
 * waypoints: 求解器输出的路径点数组
 * count: 路径点数量
 * start_row, start_col: 起点网格坐标
 * single_step: 是否单步模式
 */
void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row, uint8 start_col, uint8 single_step);

/* 停止执行器（急停） */
void executor_stop(void);

/* 20ms 中断调用（在 update_control_20ms() 里调用） */
void executor_update_20ms(void);

/* 单步模式下，恢复执行（K3 按键调用） */
void executor_resume(void);

/* 获取当前状态 */
executor_state_enum executor_get_state(void);

/* 获取错误类型 */
executor_error_enum executor_get_error(void);

/* 获取状态字符串（用于显示） */
const char *executor_state_name(void);

/* 获取当前进度 */
uint16 executor_get_current_step(void);
uint16 executor_get_total_steps(void);
uint16 executor_get_current_box(void);
uint16 executor_get_total_boxes(void);

#endif /* EXECUTOR_H */
```

- [ ] **Step 2: 验证头文件语法**

运行 Keil 编译，确认无语法错误。

---

## Task 2: 实现 executor.c 核心状态机

**Files:**
- Create: `project/user/src/executor.c`

- [ ] **Step 1: 创建文件，包含头文件和静态变量**

```c
#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "motion_math.h"
#include <math.h>

/* 执行器内部状态 */
static executor_state_enum exec_state = EXEC_STATE_IDLE;
static executor_error_enum exec_error = EXEC_ERROR_NONE;
static executor_step_enum step_state = STEP_MOVING;

/* 路径数据 */
static const waypoint_struct *exec_waypoints = NULL;
static uint16 exec_waypoint_count = 0;
static uint16 current_step = 0;
static uint8 start_row = 0;
static uint8 start_col = 0;

/* 单步模式 */
static uint8 single_step_mode = 0;

/* 超时检测 */
static uint32 step_start_ms = 0;
#define EXEC_TIMEOUT_MS 3000

/* 到位阈值 */
#define ARRIVAL_THRESHOLD_CM 2.0f
#define ARRIVAL_THRESHOLD_DEG 5.0f

/* 速度 */
#define EXEC_MOVE_SPEED 1.0f
```

- [ ] **Step 2: 实现 executor_start()**

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
    step_state = STEP_MOVING;
    exec_error = EXEC_ERROR_NONE;

    /* 重置位姿，以起点为原点 */
    drive_pose_reset(0.0f, 0.0f, 0.0f);

    /* 设置初始状态 */
    if (single_step_mode) {
        exec_state = EXEC_STATE_PAUSED;
    } else {
        exec_state = EXEC_STATE_RUNNING;
        step_start_ms = HAL_GetTick();
    }
}
```

- [ ] **Step 3: 实现 executor_stop()**

```c
void executor_stop(void)
{
    stop_motion();
    exec_state = EXEC_STATE_IDLE;
    exec_error = EXEC_ERROR_NONE;
    exec_waypoints = NULL;
    exec_waypoint_count = 0;
    current_step = 0;
}
```

- [ ] **Step 4: 实现 executor_resume()**

```c
void executor_resume(void)
{
    if (exec_state == EXEC_STATE_PAUSED) {
        exec_state = EXEC_STATE_RUNNING;
        step_start_ms = HAL_GetTick();
    }
}
```

- [ ] **Step 5: 实现坐标转换辅助函数**

```c
/* 将网格坐标转换为物理坐标（以起点为原点） */
static void grid_to_physical(uint8 row, uint8 col, float *x_cm, float *y_cm)
{
    *x_cm = (float)(col - start_col) * 20.0f;
    *y_cm = -(float)(row - start_row) * 20.0f;
}
```

- [ ] **Step 6: 实现到位判断辅助函数**

```c
/* 检查是否到达目标位置 */
static uint8 is_arrived(float target_x, float target_y)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;
    float distance = sqrtf(dx * dx + dy * dy);
    return (distance < ARRIVAL_THRESHOLD_CM);
}
```

- [ ] **Step 7: 实现移动控制辅助函数**

```c
/* 向目标位置移动 */
static void move_to_target(float target_x, float target_y)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;

    /* 选择主移动方向 */
    if (fabsf(dx) > fabsf(dy)) {
        /* X 方向误差大，左右移动 */
        if (dx > 0) {
            set_motion_command(MOTION_RIGHT, EXEC_MOVE_SPEED, 0.0f);
        } else {
            set_motion_command(MOTION_LEFT, EXEC_MOVE_SPEED, 0.0f);
        }
    } else {
        /* Y 方向误差大，前后移动 */
        if (dy > 0) {
            set_motion_command(MOTION_FORWARD, EXEC_MOVE_SPEED, 0.0f);
        } else {
            set_motion_command(MOTION_BACKWARD, EXEC_MOVE_SPEED, 0.0f);
        }
    }
}
```

- [ ] **Step 8: 实现 executor_update_20ms() 核心逻辑**

```c
void executor_update_20ms(void)
{
    /* 只在运行状态执行 */
    if (exec_state != EXEC_STATE_RUNNING) {
        return;
    }

    /* 检查超时 */
    if (HAL_GetTick() - step_start_ms > EXEC_TIMEOUT_MS) {
        stop_motion();
        exec_state = EXEC_STATE_ERROR;
        exec_error = EXEC_ERROR_TIMEOUT;
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

    /* 根据内层状态执行 */
    switch (step_state) {
        case STEP_MOVING:
            if (is_arrived(target_x, target_y)) {
                /* 到达目标格子 */
                if (wp->action >= 'A' && wp->action <= 'Z') {
                    /* 大写字母 = 推箱动作 */
                    step_state = STEP_PUSHING;
                } else {
                    /* 小写字母 = 普通移动，进入下一步 */
                    current_step++;
                    step_start_ms = HAL_GetTick();

                    /* 单步模式暂停 */
                    if (single_step_mode) {
                        stop_motion();
                        exec_state = EXEC_STATE_PAUSED;
                    }
                }
            } else {
                /* 继续移动 */
                move_to_target(target_x, target_y);
            }
            break;

        case STEP_PUSHING:
            /* 推箱动作：在 STEP_MOVING 到达目标位置时已完成 */
            /* 直接过渡到返回起点 */
            step_state = STEP_RETURNING;
            step_start_ms = HAL_GetTick();
            break;

        case STEP_RETURNING:
            /* 返回起点 (0, 0) */
            if (is_arrived(0.0f, 0.0f)) {
                /* 到达起点 */
                stop_motion();
                current_step++;
                step_state = STEP_MOVING;
                step_start_ms = HAL_GetTick();

                /* 单步模式暂停 */
                if (single_step_mode) {
                    exec_state = EXEC_STATE_PAUSED;
                }
            } else {
                /* 继续返回起点 */
                move_to_target(0.0f, 0.0f);
            }
            break;
    }
}
```

- [ ] **Step 9: 实现状态查询函数**

```c
executor_state_enum executor_get_state(void)
{
    return exec_state;
}

executor_error_enum executor_get_error(void)
{
    return exec_error;
}

const char *executor_state_name(void)
{
    switch (exec_state) {
        case EXEC_STATE_IDLE:    return "Idle";
        case EXEC_STATE_RUNNING: return "Running";
        case EXEC_STATE_PAUSED:  return "Paused";
        case EXEC_STATE_DONE:    return "Done";
        case EXEC_STATE_ERROR:   return "Error";
        default:                 return "Unknown";
    }
}

uint16 executor_get_current_step(void)
{
    return current_step;
}

uint16 executor_get_total_steps(void)
{
    return exec_waypoint_count;
}

uint16 executor_get_current_box(void)
{
    /* 计算当前是第几个箱子 */
    uint16 box = 0;
    for (uint16 i = 0; i < current_step && i < exec_waypoint_count; i++) {
        if (exec_waypoints[i].action >= 'A' && exec_waypoints[i].action <= 'Z') {
            box++;
        }
    }
    return box;
}

uint16 executor_get_total_boxes(void)
{
    /* 计算总共有多少个箱子 */
    uint16 box = 0;
    for (uint16 i = 0; i < exec_waypoint_count; i++) {
        if (exec_waypoints[i].action >= 'A' && exec_waypoints[i].action <= 'Z') {
            box++;
        }
    }
    return box;
}
```

- [ ] **Step 10: 验证编译**

运行 Keil 编译，确认无语法错误。

---

## Task 3: 修改 menu.c 添加 RUN_MODE_STEP

**Files:**
- Modify: `project/user/src/menu.c`
- Modify: `project/user/inc/settings.h`

- [ ] **Step 1: 在 settings.h 添加 RUN_MODE_STEP**

找到 `run_mode_enum` 定义，添加 `RUN_MODE_STEP`：

```c
typedef enum {
    RUN_MODE_SOLVE,     // 求解+回放
    RUN_MODE_RUN,       // 连续执行
    RUN_MODE_STEP,      // 单步执行
    RUN_MODE_COUNT
} run_mode_enum;
```

- [ ] **Step 2: 在 menu.c 添加 executor 头文件包含**

在文件顶部添加：

```c
#include "executor.h"
```

- [ ] **Step 3: 修改 execute_current_selection() 支持 executor**

找到 `execute_current_selection()` 函数，修改为：

```c
static void execute_current_selection(void)
{
    candidate_map = current_map;
    candidate_mode = run_mode;
    solve_current_map();

    if (last_result.solved) {
        if (run_mode == RUN_MODE_SOLVE) {
            /* 求解+回放模式 */
            enter_page(MENU_PAGE_RUN_PLAYBACK);
        } else {
            /* RUN 或 STEP 模式 */
            uint8 single_step = (run_mode == RUN_MODE_STEP) ? 1 : 0;
            executor_start(last_result.waypoints, last_result.waypoint_count,
                           start_row, start_col, single_step);
            run_state = "Running";
        }
    }
}
```

- [ ] **Step 4: 在 handle_run_event() 添加 K3 处理 executor 恢复**

找到 `handle_run_event()` 函数，在 `K3_SHORT` 处添加：

```c
case K3_SHORT:
    if (executor_get_state() == EXEC_STATE_PAUSED) {
        executor_resume();
        run_state = "Running";
    } else if (executor_get_state() == EXEC_STATE_IDLE) {
        execute_current_selection();
    }
    break;
```

- [ ] **Step 5: 在 handle_run_event() 添加 K4 处理 executor 停止**

找到 `handle_run_event()` 函数，在 `K4_SHORT` 处添加：

```c
case K4_SHORT:
    if (executor_get_state() == EXEC_STATE_ERROR ||
        executor_get_state() == EXEC_STATE_DONE) {
        executor_stop();
        run_state = "Idle";
    } else {
        /* 切换地图来源 */
        /* ... 现有代码 ... */
    }
    break;
```

- [ ] **Step 6: 验证编译**

运行 Keil 编译，确认无语法错误。

---

## Task 4: 修改 screen.c 显示 executor 状态

**Files:**
- Modify: `project/user/src/screen.c`

- [ ] **Step 1: 在 screen_draw_run_workbench() 添加 executor 状态显示**

找到 `screen_draw_run_workbench()` 函数，添加 executor 状态显示逻辑：

```c
void screen_draw_run_workbench(uint8 current_map, run_mode_enum mode,
                               map_source_enum source, save_state_enum save_state,
                               const char *state, uint32 elapsed_ms)
{
    /* ... 现有代码 ... */

    /* 显示 executor 状态 */
    if (executor_get_state() != EXEC_STATE_IDLE) {
        /* 状态 */
        ips200_show_string(0, 160, "State:");
        ips200_show_string(48, 160, executor_state_name());

        /* 进度 */
        char buf[32];
        sprintf(buf, "Step: %d/%d", executor_get_current_step(), executor_get_total_steps());
        ips200_show_string(0, 180, buf);

        /* 箱子数 */
        sprintf(buf, "Box: %d/%d", executor_get_current_box(), executor_get_total_boxes());
        ips200_show_string(0, 200, buf);

        /* 位置 */
        const drive_pose_struct *pose = drive_pose_get();
        sprintf(buf, "X: %.1f  Y: %.1f", pose->x_cm, pose->y_cm);
        ips200_show_string(0, 220, buf);

        /* 错误信息 */
        if (executor_get_state() == EXEC_STATE_ERROR) {
            const char *err_msg = "Unknown";
            switch (executor_get_error()) {
                case EXEC_ERROR_TIMEOUT: err_msg = "TIMEOUT"; break;
                case EXEC_ERROR_MAP:     err_msg = "MAP ERR"; break;
                case EXEC_ERROR_POSE:    err_msg = "POSE ERR"; break;
                default: break;
            }
            ips200_show_string(0, 240, err_msg);
        }
    }

    /* ... 现有代码 ... */
}
```

- [ ] **Step 2: 验证编译**

运行 Keil 编译，确认无语法错误。

---

## Task 5: 修改 isr.c 调用 executor_update_20ms()

**Files:**
- Modify: `project/user/src/isr.c`

- [ ] **Step 1: 在 isr.c 添加 executor 头文件包含**

在文件顶部添加：

```c
#include "executor.h"
```

- [ ] **Step 2: 在 update_control_20ms() 末尾调用 executor_update_20ms()**

找到 `update_control_20ms()` 函数，在末尾（`drive_output_update_and_output()` 之后）添加：

```c
    drive_output_update_and_output(&control_status);

    /* 执行器更新 */
    executor_update_20ms();
```

- [ ] **Step 3: 验证编译**

运行 Keil 编译，确认无语法错误。

---

## Task 6: 更新 Keil 工程文件

**Files:**
- Modify: `project/mdk/rt1064.uvprojx`

- [ ] **Step 1: 添加 executor.c 到 Keil 工程**

打开 `rt1064.uvprojx`，在 `<Groups>` 中找到 `<Group>` 名为 `user` 或 `src` 的组，添加：

```xml
<File>
  <FileName>executor.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\..\project\user\src\executor.c</FilePath>
</File>
```

- [ ] **Step 2: 验证编译**

运行 Keil 编译，确认 0 Error, 0 Warning。

---

## Task 7: 测试验证

**Files:**
- 无新增文件

- [ ] **Step 1: 测试单箱任务**

1. 选择一个离线地图（比如 map 00）
2. 模式选择 Run
3. 按 K3 启动
4. 观察小车是否：
   - 移动到箱子位置
   - 推箱子到目标
   - 返回起点
   - 状态显示 "Done"

- [ ] **Step 2: 测试多箱任务**

1. 选择一个多箱地图
2. 模式选择 Run
3. 按 K3 启动
4. 观察小车是否：
   - 完成第一个箱子后返回起点
   - 继续第二个箱子
   - 所有箱子完成后状态显示 "Done"

- [ ] **Step 3: 测试单步模式**

1. 模式选择 Step
2. 按 K3 启动
3. 观察小车是否：
   - 立即暂停，状态显示 "Paused"
   - 按 K3 后走一格再暂停
   - 重复直到完成

- [ ] **Step 4: 测试超时处理**

1. 人为制造障碍（比如挡住小车）
2. 观察 3 秒后是否：
   - 小车停止
   - 状态显示 "Error"
   - 错误显示 "TIMEOUT"
3. 按 K4 回到 Idle

- [ ] **Step 5: 测试急停**

1. 执行过程中按 K4
2. 观察小车是否立即停止
3. 状态回到 Idle

---

## 完成

所有任务完成后，executor 功能即可使用：

1. 选择地图（K1/K2）
2. 选择来源（K4 切换 Offline/ART）
3. 选择模式（K3 长按进入模式选择）
   - Solve：求解+回放
   - Run：连续执行
   - Step：单步执行
4. 按 K3 启动
5. 单步模式下，按 K3 继续
6. 出错或完成后，按 K4 回到 Idle
