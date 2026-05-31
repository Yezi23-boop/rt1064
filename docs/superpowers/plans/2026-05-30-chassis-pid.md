# 底盘级 PID 控制器实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 添加底盘级 PID 控制器，用正运动学解算的底盘实际速度作为反馈，修正轮子目标，解决麦轮走直线时的横向漂移问题。

**Architecture:** 在 `update_control_20ms()` 中，`drive_pose_update_20ms()` 之后、`mecanum_mix()` 之前插入底盘级 PID。底盘 PID 用编码器正运动学解算的 `body_vx`, `body_vy` 作为反馈，输出 `vx/vy` 修正量，修正后再进入混控。

**Tech Stack:** C (Keil MDK), NXP i.MX RT1064, 麦轮底盘

---

## 现有控制链路

```
update_control_20ms()
    │
    ├─ 1. read_encoder_counts()          ← 读编码器（单位：count/20ms）
    ├─ 2. drive_imu_sync_status()        ← 读 IMU
    ├─ 3. drive_pose_update_20ms()       ← 正运动学（输出 body_vx_count, body_vy_count，单位：count/20ms）
    ├─ 4. drive_imu_update_attitude_20ms() ← 姿态 PD → vzt（归一化 [-1,1]）
    ├─ 5. mecanum_mix(vx, vy, vz, vzt)   ← 混控（输入归一化 [-1,1]）
    ├─ 6. wheel_targets_from_norm()      ← 轮子目标（输出 count/20ms）
    └─ 7. drive_output_update_and_output() ← 轮子 PID → PWM
```

**单位说明**：
- `control_status.vx/vy`：归一化值，范围 [-1, 1]
- `body_vx_count/body_vy_count`：编码器脉冲/20ms，范围 [-100, 100]
- `body_vx_cm/body_vy_cm`：cm/20ms，范围约 [-0.86, 0.86]

**插入点：第 3 步之后、第 4 步之前**

```
    ├─ 3. drive_pose_update_20ms()       ← 正运动学
    │       ↓
    │   body_vx_count, body_vy_count     ← 反馈量（count/20ms）
    │       ↓
    │   / MAX_WHEEL_TARGET_COUNT         ← 归一化到 [-1, 1]
    │       ↓
    ├─ 3.5 【新增】chassis_pid_update()  ← 底盘级 PID
    │       ↓
    │   vx_corrected, vy_corrected       ← 修正后的速度（归一化）
    │       ↓
    ├─ 4. drive_imu_update_attitude_20ms()
    ├─ 5. mecanum_mix(vx_corrected, vy_corrected, vz, vzt)
    ...
```

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `project/user/inc/chassis_pid.h` | 新增 | 底盘 PID 结构体和函数声明 |
| `project/user/src/chassis_pid.c` | 新增 | 底盘 PID 实现 |
| `project/user/inc/drive_config.h` | 修改 | 添加底盘 PID 参数宏 |
| `project/user/src/drive_control.c` | 修改 | 在 update_control_20ms() 中调用底盘 PID |
| `project/user/src/drive_pose.c` | 修改 | 导出 body_vx/body_vy 供底盘 PID 使用 |
| `project/user/inc/drive_pose.h` | 修改 | 添加 body_vx/body_vy 的 getter 函数声明 |
| `project/mdk/rt1064.uvprojx` | 修改 | 添加 chassis_pid.c 到工程 |

---

## Task 1: 创建 chassis_pid.h

**Files:**
- Create: `project/user/inc/chassis_pid.h`

- [ ] **Step 1: 创建头文件**

```c
#ifndef _chassis_pid_h_
#define _chassis_pid_h_

#include "zf_common_typedef.h"

/**
 * @brief 底盘级 PID 控制器结构体
 * 
 * 用于修正 vx/vy，使底盘实际速度跟踪目标速度。
 * 只用 P 控制，Ki/Kd 留作扩展。
 */
typedef struct
{
    float kp;           /**< 比例增益 */
    float err_sum;      /**< 误差累加（积分项，预留） */
    float prev_error;   /**< 上次误差（微分项，预留） */
} chassis_pid_t;

/**
 * @brief 初始化底盘 PID 控制器
 * @param[in] pid   PID 实例指针
 * @param[in] kp    比例增益
 */
void chassis_pid_init(chassis_pid_t *pid, float kp);

/**
 * @brief 重置底盘 PID 控制器状态
 * @param[in] pid   PID 实例指针
 */
void chassis_pid_reset(chassis_pid_t *pid);

/**
 * @brief 底盘 PID 更新
 * @param[in] pid      PID 实例指针
 * @param[in] target   目标速度（归一化 [-1, 1]）
 * @param[in] actual   实际速度（归一化 [-1, 1]，从正运动学获取并归一化）
 * @return 修正量（归一化 [-1, 1]）
 */
float chassis_pid_update(chassis_pid_t *pid, float target, float actual);

#endif /* _chassis_pid_h_ */
```

- [ ] **Step 2: 验证编译**

---

## Task 2: 实现 chassis_pid.c

**Files:**
- Create: `project/user/src/chassis_pid.c`

- [ ] **Step 1: 实现底盘 PID**

```c
#include "chassis_pid.h"
#include "motion_math.h"

void chassis_pid_init(chassis_pid_t *pid, float kp)
{
    pid->kp = kp;
    pid->err_sum = 0.0f;
    pid->prev_error = 0.0f;
}

void chassis_pid_reset(chassis_pid_t *pid)
{
    pid->err_sum = 0.0f;
    pid->prev_error = 0.0f;
}

float chassis_pid_update(chassis_pid_t *pid, float target, float actual)
{
    float error = target - actual;
    
    /* 纯 P 控制，Ki/Kd 留作扩展 */
    return pid->kp * error;
}
```

- [ ] **Step 2: 验证编译**

---

## Task 3: 在 drive_config.h 添加底盘 PID 参数

**Files:**
- Modify: `project/user/inc/drive_config.h`

- [ ] **Step 1: 添加参数宏**

在 `WHEEL_PID_*` 参数之后添加：

```c
/* 底盘级 PID 参数 */
#define CHASSIS_PID_KP (0.3f)    /* 比例增益，初始值，需调试 */
```

- [ ] **Step 2: 验证编译**

---

## Task 4: 在 drive_pose.c 导出 body_vx/body_vy

**Files:**
- Modify: `project/user/src/drive_pose.c`
- Modify: `project/user/inc/drive_pose.h`

**目的：** 当前 `body_vx_count` 和 `body_vy_count` 是 `drive_pose_update_20ms()` 的局部变量，需要导出供底盘 PID 使用。

- [ ] **Step 1: 在 drive_pose.c 添加静态变量**

在 `drive_pose` 静态变量之后添加：

```c
/* 底盘速度反馈（从正运动学解算） */
static float chassis_vx_fb = 0.0f;  /* 车体右向速度，单位：count/20ms */
static float chassis_vy_fb = 0.0f;  /* 车体前向速度，单位：count/20ms */
```

- [ ] **Step 2: 在 drive_pose_update_20ms() 中保存速度**

在 `body_vx_count` 和 `body_vy_count` 计算之后，添加：

```c
/* 保存底盘速度反馈 */
chassis_vx_fb = body_vx_count;
chassis_vy_fb = body_vy_count;
```

- [ ] **Step 3: 在 drive_pose.c 添加 getter 函数**

```c
float drive_pose_get_chassis_vx_fb(void)
{
    return chassis_vx_fb;
}

float drive_pose_get_chassis_vy_fb(void)
{
    return chassis_vy_fb;
}
```

- [ ] **Step 4: 在 drive_pose.h 添加声明**

```c
/**
 * @brief 获取底盘右向速度反馈（正运动学解算）
 * @return 右向速度，单位：count/20ms
 */
float drive_pose_get_chassis_vx_fb(void);

/**
 * @brief 获取底盘前向速度反馈（正运动学解算）
 * @return 前向速度，单位：count/20ms
 */
float drive_pose_get_chassis_vy_fb(void);
```

- [ ] **Step 5: 验证编译**

---

## Task 5: 在 drive_control.c 集成底盘 PID

**Files:**
- Modify: `project/user/src/drive_control.c`

- [ ] **Step 1: 添加头文件包含**

在文件顶部添加：

```c
#include "chassis_pid.h"
```

- [ ] **Step 2: 添加底盘 PID 实例**

在 `control_status` 静态变量之后添加：

```c
/* 底盘级 PID 实例 */
static chassis_pid_t chassis_vx_pid;
static chassis_pid_t chassis_vy_pid;
```

- [ ] **Step 3: 在 control_init() 中初始化底盘 PID**

在 `drive_output_init()` 之后添加：

```c
/* 初始化底盘 PID */
chassis_pid_init(&chassis_vx_pid, CHASSIS_PID_KP);
chassis_pid_init(&chassis_vy_pid, CHASSIS_PID_KP);
```

- [ ] **Step 4: 在 update_control_20ms() 中调用底盘 PID**

在 `drive_pose_update_20ms()` 之后、`drive_imu_update_attitude_20ms()` 之前添加：

```c
/* 底盘级 PID：用正运动学反馈修正 vx/vy
 * body_vx_count/body_vy_count 单位是 count/20ms，范围 [-100, 100]
 * control_status.vx/vy 单位是归一化值，范围 [-1, 1]
 * 需要将反馈量归一化后再比较 */
{
    float vx_fb = drive_pose_get_chassis_vx_fb() / MAX_WHEEL_TARGET_COUNT;
    float vy_fb = drive_pose_get_chassis_vy_fb() / MAX_WHEEL_TARGET_COUNT;
    float vx_correction = chassis_pid_update(&chassis_vx_pid, control_status.vx, vx_fb);
    float vy_correction = chassis_pid_update(&chassis_vy_pid, control_status.vy, vy_fb);
    
    control_status.vx = limit_float(control_status.vx + vx_correction, -1.0f, 1.0f);
    control_status.vy = limit_float(control_status.vy + vy_correction, -1.0f, 1.0f);
}
```

- [ ] **Step 5: 在 stop_motion() 中重置底盘 PID**

在 `drive_output_stop()` 之后添加：

```c
/* 重置底盘 PID */
chassis_pid_reset(&chassis_vx_pid);
chassis_pid_reset(&chassis_vy_pid);
```

- [ ] **Step 6: 验证编译**

---

## Task 6: 更新 Keil 工程文件

**Files:**
- Modify: `project/mdk/rt1064.uvprojx`

- [ ] **Step 1: 添加 chassis_pid.c 到工程**

在 `<Groups>` 中找到 `user` 或 `src` 组，添加：

```xml
<File>
  <FileName>chassis_pid.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\..\project\user\src\chassis_pid.c</FilePath>
</File>
```

- [ ] **Step 2: 验证编译**

运行 Keil 编译，确认 0 Error, 0 Warning。

---

## Task 7: 测试验证

**Files:**
- 无新增文件

- [ ] **Step 1: 测试走直线**

1. 让车前进 1 米（`set_motion_command(MOTION_FORWARD, 0.5, 0)`）
2. 观察是否走直线
3. 如果还是偏，调整 `CHASSIS_PID_KP`

- [ ] **Step 2: 调参**

```
初始值：CHASSIS_PID_KP = 0.3f

测试方法：
1. 让车前进，观察是否走直线
2. 如果偏右 → Kp 加大（0.3 → 0.5）
3. 如果抖动 → Kp 减小（0.3 → 0.1）
4. 找到让车走直线的最小 Kp
```

- [ ] **Step 3: 测试推箱子**

1. 选择 Run 模式
2. 按 K3 启动推箱子任务
3. 观察小车是否走直线、到位精度是否提高

---

## 调参指南

### 初始值

```c
#define CHASSIS_PID_KP (0.3f)
```

### 调参步骤

1. **只调 Kp**：Ki=0, Kd=0
2. **从小到大**：0.1 → 0.2 → 0.3...
3. **找到临界点**：车开始抖动时的 Kp
4. **取 50%**：临界点的一半作为最终值

### 现象判断

| 现象 | 调整 |
|------|------|
| 还是走偏 | Kp 加大 |
| 车抖动 | Kp 减小 |
| 修过头（左右晃） | Kp 减小 |
| 修正太慢 | Kp 加大 |

---

## 完成

所有任务完成后，底盘级 PID 即可使用：

1. 车走直线时不再横向漂移
2. 到位精度提高
3. 推箱子更稳
