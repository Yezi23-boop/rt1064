# 科目二车头朝向与观察中心修正 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让科目二在观察箱子图案或目标数字前先将车头转向对象，再利用 OpenART #1 小车中心最多执行一次真实回中心与复核，最后才请求 OpenART #2 分类。

**Architecture:** 保留现有位置 BFS、executor、双 OpenART 协议和分类绑定结构。`subject2_logic` 只负责为观察计划补充绝对目标 yaw；`subject2` 主循环状态机负责转向稳定计时、请求式中心采样、一次位置修正、READY 重试和错误收敛。箱子中心检测及 `OBSERVE_REQ/OBSERVE_SAMPLE` 协议保留，但从科目二扫描状态机退出。

**Tech Stack:** RT1064 C99、逐飞 UART/PIT、现有 yaw PD 与位置 PID、PowerShell + GCC 主机测试、Keil MDK、OpenART MicroPython 回归测试。

---

## 文件结构

### 修改

- `project/user/inc/subject2_logic.h`：观察计划增加绝对目标 yaw。
- `project/user/src/subject2_logic.c`：根据观察格相对对象的位置生成四方向 yaw。
- `project/user/inc/subject2.h`：增加箱子/目标转向和小车回中心状态，删除旧箱子联合中心状态。
- `project/user/src/subject2.c`：实现 VTurn、统一 VCtr、一次 CAdj/CChk、READY 重试和超时；移除扫描阶段箱子中心调用。
- `project/user/inc/drive_config.h`：增加科目二转向及 READY 时序参数。
- `project/user/inc/executor.h`：增加科目二 yaw 错误码并修正 ART 中心错误注释。
- `project/user/src/screen.c`：为科目二 yaw 错误显示 `E:Yaw`。
- `tests/subject2_logic_test.c`：验证四个观察方向的 yaw。
- `tests/subject2_scan_test.c`：重写扫描状态机测试，覆盖转向、中心、回中心、READY 和错误路径。
- `docs/superpowers/specs/2026-07-13-subject2-heading-observation-design.md`：实现后同步最终状态名和行为，无新增需求。

### 保持不变

- `openmv/main_see.py`：保留箱子中心检测与请求式中心输出。
- `project/user/inc/openart_uart.h`、`project/user/src/openart_uart.c`：保留 `OBSERVE_REQ/OBSERVE_SAMPLE` 接口。
- `project/user/src/executor.c`：继续复用已实现的 `executor_start_position_correction(0,0)`。
- `project/user/src/solver.c`：位置 BFS 和指定配对 BFS 不变。
- `openmv/视觉/main.py`、`project/user/src/vision_uart.c`：ART #2 分类协议不变。

---

### Task 1: 让观察计划携带正确目标 yaw

**Files:**
- Modify: `project/user/inc/subject2_logic.h:30-38`
- Modify: `project/user/src/subject2_logic.c:6-10,108-194`
- Test: `tests/subject2_logic_test.c`

- [ ] **Step 1: 为四方向 yaw 编写失败测试**

在 `tests/subject2_logic_test.c` 增加一个只开放指定相邻观察格的辅助案例。小车直接放在候选格，使导航路径长度为 0，分别验证对象在上、右、下、左时的绝对 yaw：

```c
static uint8 observation_yaw_matches_four_directions(void)
{
    static const uint8 car_rows[4] = {6u, 5u, 4u, 5u};
    static const uint8 car_cols[4] = {5u, 4u, 5u, 6u};
    static const float expected_yaw[4] = {0.0f, 270.0f, 180.0f, 90.0f};
    uint8 index;

    for(index = 0u; index < 4u; index++)
    {
        test_map_struct map;
        subject2_object_struct objects[MAX_BOXES];
        subject2_observation_plan_struct plan;
        solve_result_struct path;
        uint8 object_count = 0u;

        init_map(&map, car_rows[index], car_cols[index]);
        map.rows[5][5] = 'B';
        map.rows[4][5] = (car_rows[index] == 4u) ? 'C' : '#';
        map.rows[6][5] = (car_rows[index] == 6u) ? 'C' : '#';
        map.rows[5][4] = (car_cols[index] == 4u) ? 'C' : '#';
        map.rows[5][6] = (car_cols[index] == 6u) ? 'C' : '#';

        if((0u == subject2_collect_objects(&map.source, 'B', objects, &object_count)) ||
           (0u == subject2_select_observation(&map.source, objects, object_count,
                                               &plan, &path)) ||
           (expected_yaw[index] != plan.target_yaw_deg))
        {
            return 0u;
        }
    }
    return 1u;
}
```

在 `main()` 中加入：

```c
passed &= run_case("observation-yaw", observation_yaw_matches_four_directions());
```

- [ ] **Step 2: 运行测试确认失败**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
```

Expected: GCC 报告 `subject2_observation_plan_struct` 没有 `target_yaw_deg`。

- [ ] **Step 3: 给观察计划增加 yaw 字段并在选择时赋值**

在 `project/user/inc/subject2_logic.h` 修改结构：

```c
typedef struct
{
    uint8 object_index;
    uint8 observation_bit;
    uint8 row;
    uint8 col;
    float target_yaw_deg;
} subject2_observation_plan_struct;
```

在 `project/user/src/subject2_logic.c` 与现有方向数组保持相同顺序增加：

```c
/* 候选格依次位于对象上、下、左、右；yaw 必须让车头反向面对对象。 */
static const float observation_target_yaw_deg[SUBJECT2_OBSERVATION_COUNT] = {
    180.0f, 0.0f, 270.0f, 90.0f
};
```

选中最优候选时同时写入：

```c
plan->target_yaw_deg = observation_target_yaw_deg[direction];
```

- [ ] **Step 4: 运行纯逻辑测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
```

Expected: 原 6 项和新增 `observation-yaw` 全部 `PASS`。

- [ ] **Step 5: 提交纯逻辑改动**

```powershell
git add project/user/inc/subject2_logic.h project/user/src/subject2_logic.c tests/subject2_logic_test.c
git commit -m "feat: add heading to subject2 observation plans"
```

---

### Task 2: 增加 VTurn 与 ART #2 READY 可靠等待

**Files:**
- Modify: `project/user/inc/drive_config.h:105-120`
- Modify: `project/user/inc/executor.h:24-33`
- Modify: `project/user/inc/subject2.h:8-31`
- Modify: `project/user/src/subject2.c:1-60,587-668,1135-1171`
- Modify: `project/user/src/screen.c:730-741`
- Test: `tests/subject2_scan_test.c`

- [ ] **Step 1: 扩展测试桩以观察 yaw 和模式重发**

在 `tests/subject2_scan_test.c` 增加 `drive_control` 测试状态：

```c
static control_status_struct fake_control_status;
static uint16 set_motion_count;
static uint16 set_target_yaw_count;
static float last_target_yaw;
static uint16 vision_mode_send_count;

const control_status_struct *get_control_status(void)
{
    return &fake_control_status;
}

void set_motion(float vx, float vy)
{
    (void)vx;
    (void)vy;
    set_motion_count++;
}

void set_target_yaw(float yaw)
{
    set_target_yaw_count++;
    last_target_yaw = yaw;
}
```

把现有 `vision_uart_set_mode()` 测试桩改为：

```c
void vision_uart_set_mode(vision_mode_enum mode)
{
    last_mode = mode;
    vision_mode_send_count++;
}
```

并在 `build_map()` 清零这些字段。

为仍在验证旧箱子中心分支的现有案例增加转向完成辅助函数，使 Task 2 引入 VTurn 后原回归案例继续通过：

```c
static void complete_observation_turn(const subject2_context_struct *context,
                                      subject2_update_struct *update)
{
    fake_control_status.yaw_error = 0.0f;
    subject2_tick(context, update);
    fake_time_ms += SUBJECT2_TURN_STABLE_MS;
    subject2_tick(context, update);
}
```

现有 `scan_to_select_push()` 和箱子观察错误案例在 PLAN/MOVE 后先断言进入对应 TURN，再调用该辅助函数，然后继续断言当前旧 `VObs/VAdj/VChk` 行为。Task 3 才整体替换这些旧断言。

- [ ] **Step 2: 编写 VTurn 稳定时间、复位和超时失败测试**

新增测试必须覆盖：

```c
static uint8 turn_requires_continuous_100ms(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);

    if((SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) ||
       (270.0f != last_target_yaw)) return 0u;

    fake_control_status.yaw_error = 1.0f;
    subject2_tick(&context, &update);
    fake_time_ms += 80u;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) return 0u;

    fake_control_status.yaw_error = 3.0f;
    subject2_tick(&context, &update); /* 越界必须清除稳定计时。 */
    fake_control_status.yaw_error = 1.0f;
    fake_time_ms += 99u;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) return 0u;

    fake_time_ms += 1u;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_OBSERVE == subject2_get_state()) &&
            (1u == observation_request_count)) ? 1u : 0u;
}
```

再增加 `turn_timeout_stops()`：进入 `SUBJECT2_SCAN_BOX_TURN` 后把 `fake_time_ms` 推进 `SUBJECT2_TURN_TIMEOUT_MS`，期望：

```c
SUBJECT2_ERROR == subject2_get_state()
EXEC_ERROR_SUBJECT2_YAW == fake_executor_error
0 == strcmp(update.run_state, "E:Yaw")
```

- [ ] **Step 3: 编写 READY 每秒重发与 10 秒超时测试**

新增测试：

```c
static uint8 vision_ready_retries_then_times_out(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    if(1u != vision_mode_send_count) return 0u;

    fake_time_ms = SUBJECT2_VISION_READY_RETRY_MS;
    subject2_tick(&context, &update);
    if(2u != vision_mode_send_count) return 0u;

    fake_time_ms = SUBJECT2_VISION_READY_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_SUBJECT2_CLASS == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:Class"))) ? 1u : 0u;
}
```

- [ ] **Step 4: 运行扫描测试确认失败**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: 编译因新宏、`SUBJECT2_SCAN_BOX_TURN` 和 `EXEC_ERROR_SUBJECT2_YAW` 不存在而失败。

- [ ] **Step 5: 增加配置、错误码和转向状态**

在 `drive_config.h` 增加：

```c
/** 科目二观察转向允许误差，单位 deg。 */
#define SUBJECT2_TURN_TOLERANCE_DEG (2.0f)
/** yaw 连续处于允许误差内的时间，单位 ms。 */
#define SUBJECT2_TURN_STABLE_MS (100u)
/** 科目二观察转向最长时间，单位 ms。 */
#define SUBJECT2_TURN_TIMEOUT_MS (10000u)
/** 等待 ART #2 READY 时的模式命令重发周期。 */
#define SUBJECT2_VISION_READY_RETRY_MS (1000u)
/** 等待 ART #2 READY 的总超时时间。 */
#define SUBJECT2_VISION_READY_TIMEOUT_MS (10000u)
```

在 `executor_error_enum` 增加：

```c
EXEC_ERROR_SUBJECT2_YAW, /**< 科目二观察转向无法在时限内稳定。 */
```

并在 `screen.c` 映射：

```c
case EXEC_ERROR_SUBJECT2_YAW: return "E:Yaw";
```

在 `subject2_state_enum` 增加：

```c
SUBJECT2_SCAN_BOX_TURN,
SUBJECT2_SCAN_TARGET_TURN,
```

- [ ] **Step 6: 实现基于真实时间的 VTurn**

`subject2.c` 显式包含：

```c
#include "drive_control.h"
```

增加状态数据：

```c
static uint32 turn_start_ms;
static uint32 turn_stable_start_ms;
static uint8 turn_stable_active;
static uint8 turn_for_center_verify;
```

Task 2 先增加一个兼容现有箱子/目标分支的转向后入口，确保该任务独立完成时旧中心流程仍可测试；Task 3 再统一改为小车 `VCtr`：

```c
static void subject2_begin_post_turn(subject2_update_struct *update)
{
    if(SUBJECT2_SCAN_BOX_TURN == subject2_state)
    {
        subject2_begin_box_observation(SUBJECT2_SCAN_BOX_OBSERVE,
                                       "VObs", update);
    }
    else
    {
        subject2_begin_center(SUBJECT2_SCAN_TARGET_CENTER,
                              "VCtr", update);
    }
}
```

进入转向的最小实现：

```c
static void subject2_begin_turn(uint8 for_center_verify,
                                subject2_update_struct *update)
{
    set_motion(0.0f, 0.0f);
    set_target_yaw(current_observation.target_yaw_deg);
    turn_start_ms = time_ms();
    turn_stable_start_ms = 0u;
    turn_stable_active = 0u;
    turn_for_center_verify = for_center_verify;
    subject2_state = (box_objects == current_objects()) ?
        SUBJECT2_SCAN_BOX_TURN : SUBJECT2_SCAN_TARGET_TURN;
    if(0 != update)
    {
        update->run_state = "VTurn";
        update->redraw = 1u;
    }
}
```

转向轮询必须用 `time_ms()` 而不是主循环调用次数：

```c
static void subject2_tick_turn(subject2_update_struct *update)
{
    const control_status_struct *status = get_control_status();
    uint32 now_ms = time_ms();

    if(subject2_abs_float(status->yaw_error) <= SUBJECT2_TURN_TOLERANCE_DEG)
    {
        if(0u == turn_stable_active)
        {
            turn_stable_active = 1u;
            turn_stable_start_ms = now_ms;
        }
        else if((now_ms - turn_stable_start_ms) >= SUBJECT2_TURN_STABLE_MS)
        {
            (void)turn_for_center_verify;
            subject2_begin_post_turn(update);
            return;
        }
    }
    else
    {
        turn_stable_active = 0u;
    }

    if((now_ms - turn_start_ms) >= SUBJECT2_TURN_TIMEOUT_MS)
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_YAW, "E:Yaw", update);
    }
    else if(0 != update)
    {
        update->run_state = "VTurn";
    }
}
```

`subject2_tick_move()` 完成观察导航后调用 `subject2_begin_turn(0u, update)`，零 waypoint 分支也直接进入 VTurn。不要在转向完成前发送 `CENTER_REQ` 或 `VISION_REQ`。

在 `subject2_tick()` 分派中加入：

```c
case SUBJECT2_SCAN_BOX_TURN:
case SUBJECT2_SCAN_TARGET_TURN:
    subject2_tick_turn(update);
    break;
```

- [ ] **Step 7: 实现 READY 重发与超时**

增加：

```c
static uint32 vision_mode_start_ms;
static uint32 vision_mode_last_send_ms;
static vision_mode_enum active_vision_mode;
```

模式入口统一为：

```c
static void subject2_begin_vision_mode(vision_mode_enum mode,
                                       subject2_state_enum state)
{
    active_vision_mode = mode;
    vision_mode_start_ms = time_ms();
    vision_mode_last_send_ms = vision_mode_start_ms;
    vision_uart_set_mode(mode);
    subject2_state = state;
}
```

模式轮询顺序固定为先检查 READY，再检查总超时，最后按 1 秒重发：

```c
if(0u != vision_uart_mode_ready(active_vision_mode))
{
    subject2_state = (VISION_MODE_BOX == active_vision_mode) ?
        SUBJECT2_SCAN_BOX_PLAN : SUBJECT2_SCAN_TARGET_PLAN;
}
else if((now_ms - vision_mode_start_ms) >= SUBJECT2_VISION_READY_TIMEOUT_MS)
{
    subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Class", update);
}
else if((now_ms - vision_mode_last_send_ms) >= SUBJECT2_VISION_READY_RETRY_MS)
{
    vision_uart_set_mode(active_vision_mode);
    vision_mode_last_send_ms = now_ms;
}
```

`subject2_begin()`、箱子扫描结束切换 TARGET、绑定重扫切换模式都必须使用该入口。分类期间 `VISION_REQ` 保持只发送一次，不增加重发。

- [ ] **Step 8: 运行扫描测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: 新增转向和 READY 案例以及经 VTurn 衔接的原有扫描回归案例全部 `PASS`。

- [ ] **Step 9: 提交转向和 READY 改动**

```powershell
git add project/user/inc/drive_config.h project/user/inc/executor.h project/user/inc/subject2.h project/user/src/subject2.c project/user/src/screen.c tests/subject2_scan_test.c
git commit -m "feat: orient subject2 car before classification"
```

---

### Task 3: 统一小车中心回正流程并移除扫描箱子中心依赖

**Files:**
- Modify: `project/user/inc/subject2.h:8-31`
- Modify: `project/user/src/subject2.c:22-48,177-499,587-668,1135-1190`
- Test: `tests/subject2_scan_test.c`

- [ ] **Step 1: 用统一中心流程重写扫描主路径测试**

删除 `tests/subject2_scan_test.c` 中仅服务旧 `OBSERVE_SAMPLE` 的样本数组、读取桩和 `feed_observation()`。保留一个只累加 `observation_request_count` 的 `openart_request_observation()` 桩，用来断言科目二扫描不再发出请求；保留 `openart_uart.h`，因为正式接口仍存在。

把箱子扫描主路径改为：

```c
/* 小车已经位于箱子左侧观察格，目标 yaw=270deg。 */
fake_ready_box = 1u;
subject2_tick(context, update); /* MODE -> PLAN */
subject2_tick(context, update); /* PLAN -> TURN */
if(SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) return 0u;

fake_control_status.yaw_error = 0.0f;
subject2_tick(context, update);
fake_time_ms += SUBJECT2_TURN_STABLE_MS;
subject2_tick(context, update);
if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
   (1u != center_request_count)) return 0u;

feed_center(550u, 550u); /* 当前 C=(5,5)，中心正好为 550,550。 */
subject2_tick(context, update);
if((SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) ||
   (0u != observation_request_count) ||
   (0u != correction_start_count)) return 0u;
```

目标扫描也必须经过 `SUBJECT2_SCAN_TARGET_TURN -> SUBJECT2_SCAN_TARGET_CENTER -> SUBJECT2_SCAN_TARGET_CLASSIFY`，不能从 MOVE 直接进入 CENTER。

增加 `step_mode_only_pauses_navigation()`：将 `context.run_mode=RUN_MODE_STEP`，验证导航调用收到 `single_step=1`；模拟最终 waypoint 已由用户恢复并执行到 `EXEC_STATE_DONE` 后，状态机必须自动完成 TURN、CENTER 和 CLASSIFY，不再调用 `executor_resume()`，也不等待额外 K3。

- [ ] **Step 2: 增加一次回中心与二次复核测试**

构造小车字符格 `C=(5,5)`，第一次中心中值为 `(560,550)`，即 X 偏 `+2cm`：

```c
feed_center(560u, 550u);
subject2_tick(&context, &update);
if((SUBJECT2_SCAN_BOX_CENTER_ADJUST != subject2_get_state()) ||
   (1u != correction_start_count) ||
   (correction_target_x != 0.0f) ||
   (correction_target_y != 0.0f)) return 0u;

fake_executor_state = EXEC_STATE_DONE;
subject2_tick(&context, &update);
if(SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) return 0u;

fake_control_status.yaw_error = 0.0f;
subject2_tick(&context, &update);
fake_time_ms += SUBJECT2_TURN_STABLE_MS;
subject2_tick(&context, &update);
if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
   (2u != center_request_count) ||
   (0 != strcmp(update.run_state, "CChk"))) return 0u;

feed_center(550u, 550u);
subject2_tick(&context, &update);
return ((SUBJECT2_SCAN_BOX_CLASSIFY == subject2_get_state()) &&
        (1u == correction_start_count)) ? 1u : 0u;
```

- [ ] **Step 3: 增加中心失败边界测试**

分别覆盖并期望 `SUBJECT2_ERROR + EXEC_ERROR_ART_CENTER + E:Ctr`：

1. `CENTER_REQ` 后推进 `EXEC_ART_SYNC_TIMEOUT_MS` 仍不足 3 帧。
2. 收样期间把字符地图 `C` 移出 `current_observation.row/col`。
3. 中心任一轴偏差超过 `EXEC_ART_CENTER_ABNORMAL_CM`。
4. 第一次回中心完成后第二组中心仍有任一轴偏差大于 `PATH_ARRIVAL_THRESHOLD_CM`。
5. `SUBJECT2_SCAN_*_CENTER_ADJUST` 持续 `EXEC_ART_SYNC_TIMEOUT_MS` 未完成。

验证分类请求数在所有失败案例中保持 0，确保错误中心不会进入 ART #2 分类。

增加 `classification_request_is_not_resent()`：进入 `VWait` 后记录 `vision_request_id`，在 20 秒超时前多次调用 `subject2_tick()`，断言请求号和请求发送计数不再增加；达到 `SUBJECT2_VIEW_TIMEOUT_MS` 后只执行一次 CANCEL 并进入 `VRetry`。

- [ ] **Step 4: 运行测试确认旧实现失败**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: 失败点应表明当前箱子仍发送 `OBSERVE_REQ`、缺少统一 `CENTER_ADJUST` 状态或中心失败仍走 `VRetry`。

- [ ] **Step 5: 用统一扫描中心状态替换旧箱子观察状态**

`subject2_state_enum` 删除：

```c
SUBJECT2_SCAN_BOX_OBSERVE,
SUBJECT2_SCAN_BOX_ADJUST,
SUBJECT2_SCAN_BOX_VERIFY,
```

增加：

```c
SUBJECT2_SCAN_BOX_CENTER_ADJUST,
SUBJECT2_SCAN_TARGET_CENTER_ADJUST,
```

`subject2.c` 删除扫描专用箱子中心数组和函数：

```text
observation_car_*_samples
observation_box_*_samples
observation_sample_count
observation_correction_count
observation_request_start_ms
subject2_begin_box_observation()
subject2_collect_observation()
subject2_prepare_observation_targets()
subject2_tick_box_observation()
subject2_tick_box_adjust()
```

不得删除 `openart_uart` 或 `main_see.py` 中的箱子中心接口。

- [ ] **Step 6: 实现统一中心入口、可信校验和 pose 重置**

增加状态数据：

```c
static uint32 center_request_start_ms;
static uint32 center_adjust_start_ms;
static uint8 center_verifying;
static uint8 center_correction_used;
```

新观察任务开始时把 `center_correction_used=0u`。转向完成后统一调用：

```c
static void subject2_begin_scan_center(uint8 verifying,
                                       subject2_update_struct *update)
{
    center_sample_count = 0u;
    center_verifying = verifying;
    center_request_start_ms = time_ms();
    openart_request_player_center();
    subject2_state = (box_objects == current_objects()) ?
        SUBJECT2_SCAN_BOX_CENTER : SUBJECT2_SCAN_TARGET_CENTER;
    if(0 != update)
    {
        update->run_state = verifying ? "CChk" : "VCtr";
        update->redraw = 1u;
    }
}
```

删除 Task 2 的 `subject2_begin_post_turn()`，并把 `subject2_tick_turn()` 的稳定完成分支改为：

```c
subject2_begin_scan_center(turn_for_center_verify, update);
```

把扫描中心应用函数改为输出格内偏移，并严格检查观察格：

```c
static uint8 subject2_apply_scan_center(const subject2_context_struct *context,
                                        float *offset_x_cm,
                                        float *offset_y_cm)
{
    const map_source_struct *source = openart_map_get();
    const drive_pose_struct *pose;
    uint8 car_row;
    uint8 car_col;
    uint16 col_q = subject2_median_u16(center_col_samples);
    uint16 row_q = subject2_median_u16(center_row_samples);

    if((0 == source) || (0 == subject2_map_objects_unchanged(source)) ||
       (0 == map_find_car(source, &car_row, &car_col, 0)) ||
       (car_row != current_observation.row) ||
       (car_col != current_observation.col) ||
       (col_q >= MAP_COLS * 100u) || (row_q >= MAP_ROWS * 100u))
    {
        return 0u;
    }

    *offset_x_cm = ((float)((int32)col_q - (int32)(car_col * 100u + 50u)) /
                    100.0f) * GRID_SIZE_CM;
    *offset_y_cm = -((float)((int32)row_q - (int32)(car_row * 100u + 50u)) /
                     100.0f) * GRID_SIZE_CM;
    if((subject2_abs_float(*offset_x_cm) > EXEC_ART_CENTER_ABNORMAL_CM) ||
       (subject2_abs_float(*offset_y_cm) > EXEC_ART_CENTER_ABNORMAL_CM))
    {
        return 0u;
    }

    pose = drive_pose_get();
    drive_pose_reset(*offset_x_cm, *offset_y_cm, pose->yaw_deg);
    current_pose_offset_x_cm = *offset_x_cm;
    current_pose_offset_y_cm = *offset_y_cm;
    navigation_start_row = car_row;
    navigation_start_col = car_col;
    *context->start_row = car_row;
    *context->start_col = car_col;
    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    *context->snapshot_valid = 1u;
    return 1u;
}
```

现有重规划 `RCtr` 继续使用允许非观察格的原逻辑，不要被严格观察格校验影响。

- [ ] **Step 7: 实现一次 CAdj、yaw 复核和 CChk**

扫描中心轮询顺序：

```c
uint8 is_box_scan = (SUBJECT2_SCAN_BOX_CENTER == subject2_state) ? 1u : 0u;

if(0u == subject2_collect_center())
{
    if((time_ms() - center_request_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:Ctr", update);
    return;
}
if(0u == subject2_apply_scan_center(context, &offset_x_cm, &offset_y_cm))
{
    subject2_fail(EXEC_ERROR_ART_CENTER, "E:Ctr", update);
    return;
}
if((subject2_abs_float(offset_x_cm) <= PATH_ARRIVAL_THRESHOLD_CM) &&
   (subject2_abs_float(offset_y_cm) <= PATH_ARRIVAL_THRESHOLD_CM))
{
    subject2_begin_classification(classify_state, update);
    return;
}
if((0u != center_verifying) || (0u != center_correction_used) ||
   (0u == executor_start_position_correction(0.0f, 0.0f)))
{
    subject2_fail(EXEC_ERROR_ART_CENTER, "E:Ctr", update);
    return;
}
center_correction_used = 1u;
center_adjust_start_ms = time_ms();
subject2_state = is_box_scan ? SUBJECT2_SCAN_BOX_CENTER_ADJUST :
                               SUBJECT2_SCAN_TARGET_CENTER_ADJUST;
update->run_state = "CAdj";
```

位置修正轮询：

```c
if(EXEC_STATE_DONE == executor_get_state())
{
    subject2_begin_turn(1u, update); /* 复核同一目标 yaw，随后进入 CChk。 */
}
else if((EXEC_STATE_ERROR == executor_get_state()) ||
        ((time_ms() - center_adjust_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS))
{
    subject2_fail(EXEC_ERROR_ART_CENTER, "E:Ctr", update);
}
else
{
    update->run_state = "CAdj";
}
```

第二组中心仍超出 0.5cm 时不得再次调用位置修正，直接 `E:Ctr`。

- [ ] **Step 8: 清理取消路径和当前对象判断**

`subject2_cancel()` 必须清零所有新增计时、验证和修正标志。`current_objects()` 必须把 BOX/TARGET 的 TURN、CENTER、CENTER_ADJUST、CLASSIFY 状态分别归到正确对象数组，避免目标扫描误用箱子对象。

分类失败仍调用 `subject2_mark_observation_failed()` 进入 `VRetry`；中心失败全部调用 `subject2_fail(EXEC_ERROR_ART_CENTER, "E:Ctr", update)`，不得进入 `VRetry`。

- [ ] **Step 9: 运行科目二扫描测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: 扫描、转向、READY、中心回正、分类超时、地图变化、推箱重试和 PCtr 回归案例全部 `PASS`。

- [ ] **Step 10: 静态确认扫描代码不再调用箱子中心**

Run:

```powershell
rg -n "SUBJECT2_SCAN_BOX_(OBSERVE|ADJUST|VERIFY)|subject2_(begin_box_observation|collect_observation|prepare_observation_targets|tick_box_observation|tick_box_adjust)|openart_request_observation|get_observation_sample" project/user/src/subject2.c project/user/inc/subject2.h
```

Expected: 无输出。

再确认保留能力：

```powershell
rg -n "openart_request_observation|openart_get_observation_sample|OBSERVE_REQ|OBSERVE_SAMPLE|detect_box_center" project/user openmv/main_see.py
```

Expected: `openart_uart.h/.c` 和 `openmv/main_see.py` 仍有匹配。

- [ ] **Step 11: 提交统一中心流程**

```powershell
git add project/user/inc/subject2.h project/user/src/subject2.c tests/subject2_scan_test.c
git commit -m "feat: recenter subject2 car before classification"
```

---

### Task 4: 回归测试、注释同步和完整构建

**Files:**
- Modify: `project/user/inc/executor.h`
- Modify: `docs/superpowers/specs/2026-07-13-subject2-heading-observation-design.md`
- Verify: all touched files and related host tests

- [ ] **Step 1: 同步注释和设计状态名**

把 `executor_start_position_correction()` 注释明确为“科目二观察格小车中心回正”，并把 `EXEC_ERROR_ART_CENTER` 注释从仅“推箱前”改为：

```c
EXEC_ERROR_ART_CENTER, /**< ART 小车中心采样、可信校验或位置修正失败。 */
```

检查设计文档最终状态链与代码一致：

```text
VPos -> VTurn -> VCtr -> [CAdj -> VTurn -> CChk] -> VWait
```

- [ ] **Step 2: 运行科目二和 executor 直接相关测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_position_correction_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
```

Expected: 所有案例 `PASS`。

- [ ] **Step 3: 运行双 OpenART、求解和比赛流程回归**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_vision_uart_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_solver_navigation_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_competition_flow_test.ps1
python tests/openart_observation_test.py
python tests/openart_request_flow_test.py
python tests/art2_protocol_test.py
python -m py_compile openmv/main_see.py
python -m py_compile "openmv/视觉/main.py"
```

Expected: C/PowerShell 测试全部 `PASS`；Python 测试退出码为 0；两次 `py_compile` 无输出。

- [ ] **Step 4: 运行 Keil 完整编译**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
Select-String -Path project/mdk/Objects/rt1064.build_log.htm -Pattern 'Program Size|Error\(s\)|Warning\(s\)' | ForEach-Object { $_.Line -replace '<[^>]+>','' }
```

Expected:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 5: 检查差异卫生**

Run:

```powershell
git diff --check
git status --short
```

Expected: `git diff --check` 无空白错误；`git status` 中只识别并报告本任务文件，不回退工作区已有修改。

- [ ] **Step 6: 提交验证后的最终改动**

```powershell
git add project/user/inc/drive_config.h project/user/inc/executor.h project/user/inc/subject2_logic.h project/user/inc/subject2.h project/user/src/subject2_logic.c project/user/src/subject2.c project/user/src/screen.c tests/subject2_logic_test.c tests/subject2_scan_test.c docs/superpowers/specs/2026-07-13-subject2-heading-observation-design.md docs/superpowers/plans/2026-07-13-subject2-heading-observation.md
git commit -m "feat: complete subject2 heading-aware observation flow"
```

---

## 上板验收顺序

1. 保持 `VISION_UART_BOARD_TEST_ENABLE=0`、`COMPETITION_MODE=COMPETITION_MODE_SUBJECT2_DEBUG`。
2. 仅放置一个箱子和一个目标，分别从对象下、左、上、右四个观察格测试 yaw=`0/270/180/90deg`。
3. 在观察格中心测试：确认只出现 `VTurn -> VCtr -> VWait`，不进入 `CAdj/CChk`。
4. 人为让小车偏离观察格中心 2~5cm：确认只执行一次 `CAdj`，随后 `VTurn -> CChk -> VWait`。
5. 让 OpenART #1 停止回复中心：10 秒后必须 `E:Ctr` 停车。
6. 让 yaw 无法进入 `+/-2deg`：10 秒后必须 `E:Yaw` 停车。
7. 暂停 OpenART #2：确认 `VISION_MODE` 每秒重发，10 秒后 `E:Class`。
8. 恢复双 ART，完成箱子图案和目标数字扫描，确认 `VISION_REQ` 每个观察任务只发送一次。
9. 运行完整指定配对推箱，确认 PCtr、ART Wait、RCtr 和返航逻辑没有回归。
