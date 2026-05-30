# Waypoint Executor 设计文档

## 概述

设计一个路径执行器（executor），将 BFS 求解器输出的网格路径转换为实际控制指令，驱动麦轮底盘在物理场地上完成推箱子任务。

## 需求总结

| 需求 | 决策 |
|------|------|
| 任务类型 | 多箱任务（求解器输出用 `\|` 分隔） |
| 执行模式 | 真实执行 + 单步调试 |
| 坐标系统 | 以起点为原点，用 pose 反馈 |
| 速度控制 | 统一速度 1.0（满速，移动和推箱相同） |
| 单步触发 | 菜单里加 Step 选项 |
| 超时处理 | 固定 3 秒超时 |
| 回起点 | 每个箱子推完都回起点 |

## 架构设计

### 文件结构

```
project/user/src/executor.c   ← 新增
project/user/inc/executor.h   ← 新增
```

### 职责划分

- `executor.c`：状态机逻辑，坐标转换，到位判断
- `executor.h`：对外接口（start/stop/get_status）
- `menu.c`：调用 executor 的 start/stop，显示状态
- `isr.c`：在 PIT 20ms 中断里调用 `executor_update_20ms()`

### 数据流

```
求解器输出 (waypoints[])
    ↓
executor_start(waypoints, count, start_row, start_col, single_step)
    ↓
executor_update_20ms()  ← PIT 20ms 中断调用
    ↓
set_motion_command()  ← 驱动底盘
    ↓
drive_pose_get()  ← 反馈到位判断
```

## 状态机设计

### 外层状态（任务级）

```c
typedef enum {
    EXEC_STATE_IDLE,      // 等待启动
    EXEC_STATE_RUNNING,   // 正在执行
    EXEC_STATE_PAUSED,    // 单步暂停
    EXEC_STATE_DONE,      // 全部完成
    EXEC_STATE_ERROR      // 出错（超时等）
} executor_state_enum;
```

### 内层状态（当前箱子执行）

```c
typedef enum {
    STEP_MOVING,     // 移动到下一个格子
    STEP_PUSHING,    // 推箱子（到达目标格子，action 是大写）
    STEP_RETURNING   // 推完回起点
} executor_step_enum;
```

### 状态转换

```
IDLE → [executor_start()] → RUNNING
RUNNING → [所有箱子完成] → DONE
RUNNING → [超时/错误] → ERROR
RUNNING → [单步模式 && 一格走完] → PAUSED
PAUSED → [K3 按键] → RUNNING
ERROR → [K4 按键] → IDLE
DONE  → [K4 按键] → IDLE
```

### 内层循环（在 RUNNING 状态下）

```
MOVING → [到位] → 有 PUSH 动作？→ PUSHING → RETURNING → 下一个箱子
                  ↓ 没有
                  下一个格子 → MOVING
```

## 坐标映射与移动逻辑

### 坐标转换

```c
// 起点为原点，row↓ 对应 y 负方向
float target_x = (col - start_col) * 20.0f;
float target_y = -(row - start_row) * 20.0f;
```

### 到位判断

```c
pose = drive_pose_get();
dx = target_x - pose->x_cm;
dy = target_y - pose->y_cm;
distance = sqrt(dx*dx + dy*dy);

if (distance < 2.0f) {
    // 到位，进入下一步
}
```

### 移动方向决策

```c
// 根据 dx, dy 决定移动方向
if (fabs(dx) > fabs(dy)) {
    // X 方向误差大，左右移动
    if (dx > 0) set_motion_command(MOTION_RIGHT, speed, 0);
    else        set_motion_command(MOTION_LEFT, speed, 0);
} else {
    // Y 方向误差大，前后移动
    if (dy > 0) set_motion_command(MOTION_FORWARD, speed, 0);
    else        set_motion_command(MOTION_BACKWARD, speed, 0);
}
```

### 超时处理

```c
// 每格最多 3 秒
if (HAL_GetTick() - step_start_ms > 3000) {
    stop_motion();
    state = EXEC_STATE_ERROR;
}
```

## 菜单集成

### Run 模式选项扩展

```c
typedef enum {
    RUN_MODE_SOLVE,     // 求解+回放
    RUN_MODE_RUN,       // 连续执行
    RUN_MODE_STEP       // 单步执行
} run_mode_enum;
```

### K3 触发逻辑

在 `execute_current_selection()` 中：

```c
if (run_mode == RUN_MODE_SOLVE) {
    solve_current_map();  // 现有逻辑
    enter_page(MENU_PAGE_RUN_PLAYBACK);
} else {
    // RUN 或 STEP 模式
    solve_current_map();  // 先求解
    if (last_result.solved) {
        executor_start(last_result.waypoints, last_result.waypoint_count,
                       start_row, start_col,
                       run_mode == RUN_MODE_STEP);
        run_state = "Running";
    }
}
```

### Run 页面显示

```
Map: 03  Source: Offline
Mode: Run  State: Running
Step: 5/12  Box: 1/2
X: 40.2  Y: -20.1
```

### 按键处理

- K3：单步模式下，从 PAUSED 恢复执行
- K4：从 ERROR/DONE 回到 IDLE

## 公共 API

```c
// 启动执行器
// waypoints: 求解器输出的路径点数组
// count: 路径点数量
// start_row, start_col: 起点网格坐标
// single_step: 是否单步模式
void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row, uint8 start_col, uint8 single_step);

// 停止执行器（急停）
void executor_stop(void);

// 20ms 中断调用（在 update_control_20ms() 里调用）
void executor_update_20ms(void);

// 单步模式下，恢复执行（K3 按键调用）
void executor_resume(void);

// 获取当前状态
executor_state_enum executor_get_state(void);

// 获取状态字符串（用于显示）
const char *executor_state_name(void);

// 获取当前进度
uint16 executor_get_current_step(void);
uint16 executor_get_total_steps(void);
uint16 executor_get_current_box(void);
uint16 executor_get_total_boxes(void);
```

## 错误处理与边界情况

### 错误类型

```c
typedef enum {
    EXEC_ERROR_NONE,
    EXEC_ERROR_TIMEOUT,    // 单步超时（3秒）
    EXEC_ERROR_MAP,        // 地图数据异常
    EXEC_ERROR_POSE        // pose 系统异常
} executor_error_enum;
```

### 错误处理策略

- 超时：立即停车，状态设为 ERROR，显示 "TIMEOUT"
- 地图异常：waypoint 数据无效，显示 "MAP ERR"
- pose 异常：pose 系统故障，显示 "POSE ERR"

### 边界情况处理

1. **起点即终点**：waypoint 只有一个点，直接标记完成
2. **重复格子**：连续两个 waypoint 在同一格，跳过移动
3. **推箱后回起点**：RETURNING 阶段用 pose 反馈，目标坐标为起点 (0, 0)
4. **单步模式第一格**：start 后立即 PAUSED，等 K3 按键才开始走第一格

### 急停行为（executor_stop）

- 立即调用 `stop_motion()`
- 状态设为 IDLE
- 清除所有执行状态

## 实现计划

### 第 1 步：创建 executor.h

定义所有类型和函数声明。

### 第 2 步：实现 executor.c 核心状态机

- 实现 `executor_start()`
- 实现 `executor_update_20ms()`
- 实现 `executor_stop()`
- 实现 `executor_resume()`
- 实现状态查询函数

### 第 3 步：修改 menu.c

- 添加 `RUN_MODE_STEP` 到 `run_mode_enum`
- 修改 `execute_current_selection()` 支持 executor
- 添加 K3/K4 按键处理
- 更新 `screen_draw_run_workbench()` 显示 executor 状态

### 第 4 步：修改 isr.c

- 在 `update_control_20ms()` 里调用 `executor_update_20ms()`

### 第 5 步：更新 Keil 工程文件

- 将 `executor.c` 添加到 `rt1064.uvprojx`

### 第 6 步：测试

- 用离线地图测试单箱任务
- 测试多箱任务
- 测试单步模式
- 测试超时处理
- 测试错误恢复
