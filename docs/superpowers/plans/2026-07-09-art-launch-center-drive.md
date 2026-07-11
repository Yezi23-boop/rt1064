# ART Launch Center Drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the MCU use OpenART's launch-zone `PLAYER_CENTER_GRID` to drive from the left launch zone to the first playable grid center before waiting for the real Sokoban map.

**Architecture:** Split ART execution into two independent phases. Launch-zone localization uses only `PLAYER_CENTER_GRID` and a local X-axis motion target; formal Sokoban solving still waits for a valid 16x12 map with one inner `C` and matching nonzero `B/T`. The launch phase must not call `solve_map()`, mark Done, or trust launch-zone character maps.

**Tech Stack:** RT1064 C firmware, OpenART MicroPython UART protocol, existing `art_replan.c`, `openart_uart.c`, `drive_pose.c`, `drive_control.c`, `screen.c`, Keil MDK build.

---

## Current State

- `project/user/src/art_replan.c` currently uses the old ART flow:
  `WAIT_LAUNCH -> INITIAL -> stable map -> solve_map() -> Ready/Run`.
- `project/user/src/openart_uart.c` already parses:
  `PLAYER_CENTER_GRID col_q,row_q valid`.
- `project/user/src/executor.c` currently resets local pose to `(0,0)` at `executor_start()`.
- `project/user/inc/drive_config.h` currently has no launch-zone movement constants.
- Debug `printf` is not reliable, so verification must use screen state and visible vehicle behavior.

## Desired Runtime Flow

```text
K3
-> Wait 5s
-> Wait Center: collect 3 fresh valid PLAYER_CENTER_GRID samples
-> Launch: move right to x=30cm, computed from ART center
-> Wait Map: wait for true Sokoban map
-> solve_map()
-> executor_start()
```

Launch-zone rules:

- Do not solve from launch-zone map.
- Do not Done from launch-zone `B=0,T=0`.
- Do not require `C=1,B>0,T>0` during launch center collection.
- Only formal `Wait Map` requires `C=1,B>0,T>0,B==T,C in inner area`.
- OpenART must send launch-zone `PLAYER_CENTER_GRID` independently of the 16x12 character-map `C`; MCU cannot compute launch motion if the center stream depends on recognizing an outer-wall `C`.

---

### Task 1: Add Launch Configuration Constants

**Files:**
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/inc/drive_config.h`

- [ ] **Step 1: Add launch constants after ART stable-frame settings**

Insert after `EXEC_ART_STABLE_FRAMES`:

```c
/** ART 左发车区目标：第一个可走格中心 X，单位 cm；col=1.5, grid=20cm -> 30cm。 */
#define ART_LAUNCH_TARGET_X_CM (30.0f)
/** ART 发车中心采样有效帧数；使用中值滤波。 */
#define ART_LAUNCH_CENTER_SAMPLE_COUNT (3u)
/** ART 发车中心等待最长时间，单位 ms；无中心时停在 Wait Ctr，不盲跑。 */
#define ART_LAUNCH_CENTER_TIMEOUT_MS (3000u)
/** ART 发车移动最大归一化速度。 */
#define ART_LAUNCH_MOVE_MAX_SPEED (0.5f)
/** ART 发车移动到点阈值，单位 cm。 */
#define ART_LAUNCH_MOVE_ARRIVAL_CM (0.8f)
/** ART 发车移动连续到点 20ms 周期数。 */
#define ART_LAUNCH_MOVE_STABLE_TICKS (5u)
/** ART 发车移动距离最大允许值，单位 cm；防止中心异常导致跑太远。 */
#define ART_LAUNCH_MOVE_MAX_CM (35.0f)
/** ART 发车移动距离最小有效值，单位 cm；小于该值直接进入 Wait Map。 */
#define ART_LAUNCH_MOVE_MIN_CM (0.5f)
/** ART 发车移动最长时间，单位 ms；超时停车并进入 Wait Map。 */
#define ART_LAUNCH_MOVE_TIMEOUT_MS (5000u)
```

- [ ] **Step 2: Static check**

Run:

```powershell
rg -n "ART_LAUNCH_" C:\Users\ye\Desktop\rt1064\project\user\inc\drive_config.h
```

Expected: all constants above appear once.

---

### Task 2: Add ART Launch State Machine Fields

**Files:**
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/art_replan.c`

- [ ] **Step 1: Add phases**

Replace the enum with:

```c
typedef enum
{
    ART_REPLAN_IDLE = 0,         /**< 空闲态：不主动等待 ART，只响应 executor 触发的同步请求。 */
    ART_REPLAN_WAIT_LAUNCH,      /**< 发车前等待态：持续收图，但只计时不求解。 */
    ART_REPLAN_WAIT_CENTER,      /**< 发车区中心等待态：只收 PLAYER_CENTER_GRID，不求解地图。 */
    ART_REPLAN_LAUNCH_MOVE,      /**< 按 ART 中心计算距离，移动到外面第一可走格中心。 */
    ART_REPLAN_INITIAL,          /**< 出发车区后等待真实推箱地图并首次求解。 */
    ART_REPLAN_SEGMENT,          /**< waypoint 段末同步：按 ART 最新地图做确认。 */
} art_replan_phase_enum;
```

- [ ] **Step 2: Add static launch fields**

After existing static fields:

```c
static uint16 art_launch_center_col_samples[ART_LAUNCH_CENTER_SAMPLE_COUNT];
static uint16 art_launch_center_row_samples[ART_LAUNCH_CENTER_SAMPLE_COUNT];
static uint8 art_launch_center_sample_count = 0;
static uint16 art_launch_center_col_q = 0;
static uint16 art_launch_center_row_q = 0;
static uint8 art_launch_center_valid = 0;
static uint32 art_launch_center_start_ms = 0;
static float art_launch_start_x_cm = 0.0f;
static float art_launch_target_x_cm = 0.0f;
static uint8 art_launch_arrival_ticks = 0;
static uint32 art_launch_move_start_ms = 0;
```

- [ ] **Step 3: Add small helpers**

Add near existing static helpers:

```c
static float art_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16 art_median_u16(const uint16 *values, uint8 count)
{
    uint16 sorted[ART_LAUNCH_CENTER_SAMPLE_COUNT];
    uint8 i;
    uint8 j;

    for(i = 0; i < count; i++)
    {
        sorted[i] = values[i];
    }
    for(i = 1; i < count; i++)
    {
        uint16 key = sorted[i];
        j = i;
        while((j > 0u) && (sorted[j - 1u] > key))
        {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = key;
    }
    return sorted[count / 2u];
}

static void art_launch_clear_center(void)
{
    art_launch_center_sample_count = 0;
    art_launch_center_col_q = 0;
    art_launch_center_row_q = 0;
    art_launch_center_valid = 0;
}

static float art_grid_q_to_x_cm(uint16 col_q)
{
    return ((float)col_q * GRID_SIZE_CM) / 100.0f;
}
```

- [ ] **Step 4: Static check**

Run:

```powershell
rg -n "ART_REPLAN_WAIT_CENTER|ART_REPLAN_LAUNCH_MOVE|art_launch_center|art_grid_q_to_x_cm" C:\Users\ye\Desktop\rt1064\project\user\src\art_replan.c
```

Expected: new phase and helpers appear.

---

### Task 3: Gate Formal Map Solving to True Sokoban Maps

**Files:**
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/art_replan.c`

- [ ] **Step 1: Add inner-area check**

Add near `art_stats_done()`:

```c
static uint8 art_stats_car_inside_playable_area(const map_scan_stats_struct *stats)
{
    if((0u == stats->car_row) || (stats->car_row >= (MAP_ROWS - 1u)))
    {
        return 0;
    }
    if((0u == stats->car_col) || (stats->car_col >= (MAP_COLS - 1u)))
    {
        return 0;
    }
    return 1;
}
```

- [ ] **Step 2: Strengthen solve validity**

Replace `art_stats_valid_for_solve()` with:

```c
static uint8 art_stats_valid_for_solve(const map_scan_stats_struct *stats)
{
    if(1u != stats->car_count)
    {
        return 0;
    }
    if(0 == art_stats_car_inside_playable_area(stats))
    {
        return 0;
    }
    if((0u == stats->box_count) || (0u == stats->target_count))
    {
        return 0;
    }
    if(stats->box_count != stats->target_count)
    {
        return 0;
    }
    if((stats->box_count > MAX_BOXES) || (stats->target_count > MAX_BOXES))
    {
        return 0;
    }
    return 1;
}
```

- [ ] **Step 3: Prevent initial Done from launch-zone empty map**

Replace the Done branch condition:

```c
if((1u == stats.car_count) && (0 != art_stats_done(&stats)))
```

with:

```c
if((ART_REPLAN_SEGMENT == phase) &&
   (1u == stats.car_count) &&
   (0 != art_stats_done(&stats)))
```

- [ ] **Step 4: Static check**

Run:

```powershell
rg -n "art_stats_car_inside_playable_area|ART_REPLAN_SEGMENT == phase|box_count\\) \\|\\|" C:\Users\ye\Desktop\rt1064\project\user\src\art_replan.c
```

Expected: inner check exists, Done branch is segment-only, zero B/T maps are not valid for solve.

---

### Task 4: Implement Launch Center Collection

**Files:**
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/art_replan.c`

- [ ] **Step 1: Add wait-center transition**

Add:

```c
static void art_replan_begin_wait_center(art_replan_update_struct *update)
{
    art_replan_phase = ART_REPLAN_WAIT_CENTER;
    art_launch_center_start_ms = time_ms();
    art_launch_clear_center();
    art_last_player_center_sample = openart_get_player_center(0, 0, 0);
    if(0 != update)
    {
        update->run_state = "Wait Ctr";
        update->redraw = 1;
    }
}
```

- [ ] **Step 2: Add center collection function**

Add:

```c
static uint8 art_replan_collect_launch_center(void)
{
    uint16 center_col_q;
    uint16 center_row_q;
    uint8 center_valid;
    uint32 sample_count;

    sample_count = openart_get_player_center(&center_col_q, &center_row_q, &center_valid);
    if((sample_count == art_last_player_center_sample) || (0 == center_valid))
    {
        return 0;
    }

    art_last_player_center_sample = sample_count;
    if(art_launch_center_sample_count < ART_LAUNCH_CENTER_SAMPLE_COUNT)
    {
        art_launch_center_col_samples[art_launch_center_sample_count] = center_col_q;
        art_launch_center_row_samples[art_launch_center_sample_count] = center_row_q;
        art_launch_center_sample_count++;
    }

    if(art_launch_center_sample_count >= ART_LAUNCH_CENTER_SAMPLE_COUNT)
    {
        art_launch_center_col_q = art_median_u16(art_launch_center_col_samples,
                                                 ART_LAUNCH_CENTER_SAMPLE_COUNT);
        art_launch_center_row_q = art_median_u16(art_launch_center_row_samples,
                                                 ART_LAUNCH_CENTER_SAMPLE_COUNT);
        art_launch_center_valid = 1;
        return 1;
    }

    return 0;
}
```

- [ ] **Step 3: Add center-state tick**

Add:

```c
static void art_replan_tick_wait_center(art_replan_update_struct *update)
{
    if(0 != art_replan_collect_launch_center())
    {
        const drive_pose_struct *pose = drive_pose_get();
        float current_x_cm = art_grid_q_to_x_cm(art_launch_center_col_q);
        float move_cm = ART_LAUNCH_TARGET_X_CM - current_x_cm;

        if(move_cm < 0.0f)
        {
            move_cm = 0.0f;
        }
        if(move_cm > ART_LAUNCH_MOVE_MAX_CM)
        {
            move_cm = ART_LAUNCH_MOVE_MAX_CM;
        }

        if(move_cm <= ART_LAUNCH_MOVE_MIN_CM)
        {
            art_replan_begin(ART_REPLAN_INITIAL, update);
            return;
        }

        drive_pose_reset(0.0f, 0.0f, pose->yaw_deg);
        reset_motion_segment();
        art_launch_start_x_cm = 0.0f;
        art_launch_target_x_cm = move_cm;
        art_launch_arrival_ticks = 0;
        art_launch_move_start_ms = time_ms();
        art_replan_phase = ART_REPLAN_LAUNCH_MOVE;
        if(0 != update)
        {
            update->run_state = "Launch";
            update->redraw = 1;
        }
        return;
    }

    if((time_ms() - art_launch_center_start_ms) >= ART_LAUNCH_CENTER_TIMEOUT_MS)
    {
        if(0 != update)
        {
            update->run_state = "Wait Ctr";
            update->redraw = 1;
        }
        return;
    }

    if(0 != update)
    {
        update->run_state = "Wait Ctr";
    }
}
```

- [ ] **Step 4: Static check**

Run:

```powershell
rg -n "Wait Ctr|art_replan_collect_launch_center|ART_LAUNCH_TARGET_X_CM|ART_LAUNCH_MOVE_MAX_CM" C:\Users\ye\Desktop\rt1064\project\user\src\art_replan.c
```

Expected: launch center state and move computation exist.

---

### Task 5: Implement Launch Move

**Files:**
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/art_replan.c`

- [ ] **Step 1: Add includes**

At the top of `art_replan.c`, add:

```c
#include "drive_control.h"
#include "motion_math.h"
```

- [ ] **Step 2: Add launch movement tick**

Add:

```c
static void art_replan_tick_launch_move(art_replan_update_struct *update)
{
    const drive_pose_struct *pose = drive_pose_get();
    float error_x = art_launch_target_x_cm - (pose->x_cm - art_launch_start_x_cm);
    float vx;

    if((time_ms() - art_launch_move_start_ms) >= ART_LAUNCH_MOVE_TIMEOUT_MS)
    {
        stop_motion();
        art_replan_begin(ART_REPLAN_INITIAL, update);
        return;
    }

    if(art_abs_float(error_x) <= ART_LAUNCH_MOVE_ARRIVAL_CM)
    {
        reset_motion_segment();
        if(art_launch_arrival_ticks < ART_LAUNCH_MOVE_STABLE_TICKS)
        {
            art_launch_arrival_ticks++;
        }
        if(art_launch_arrival_ticks >= ART_LAUNCH_MOVE_STABLE_TICKS)
        {
            stop_motion();
            art_replan_begin(ART_REPLAN_INITIAL, update);
        }
        return;
    }

    art_launch_arrival_ticks = 0;
    vx = limit_float(error_x * PATH_KP,
                     -ART_LAUNCH_MOVE_MAX_SPEED,
                     ART_LAUNCH_MOVE_MAX_SPEED);
    set_motion(vx, 0.0f);

    if(0 != update)
    {
        update->run_state = "Launch";
    }
}
```

- [ ] **Step 3: Route phases in `art_replan_tick()`**

In `ART_REPLAN_WAIT_LAUNCH`, replace:

```c
art_replan_begin(ART_REPLAN_INITIAL, update);
```

with:

```c
art_replan_begin_wait_center(update);
```

Before ART timeout handling, add:

```c
if(ART_REPLAN_WAIT_CENTER == art_replan_phase)
{
    art_replan_tick_wait_center(update);
    return;
}

if(ART_REPLAN_LAUNCH_MOVE == art_replan_phase)
{
    art_replan_tick_launch_move(update);
    return;
}
```

- [ ] **Step 4: Static check**

Run:

```powershell
rg -n "ART_REPLAN_WAIT_CENTER|ART_REPLAN_LAUNCH_MOVE|art_replan_tick_launch_move|set_motion\\(vx, 0.0f\\)" C:\Users\ye\Desktop\rt1064\project\user\src\art_replan.c
```

Expected: both phases route before normal ART map timeout.

---

### Task 6: Minimal Screen-Based Observability

**Files:**
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/menu.c`
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/screen.c`
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/inc/screen.h`

- [ ] **Step 1: Add execute-page state text**

In `screen_execute_view_struct`, add:

```c
    const char *state_text;             /**< 菜单层运行状态，用于 ART 发车等待阶段显示。 */
```

- [ ] **Step 2: Fill execute-page state text**

In `build_execute_view()`, add:

```c
    view->state_text = run_state;
```

- [ ] **Step 3: Show state text even before a solved snapshot exists**

In `screen_draw_execute()`, show `state_text` near the top before the `No Map` early return:

```c
    ips200_show_string(0, LINE_H * 2, "S:");
    show_text_value(16, LINE_H * 2,
                    (0 != view->state_text) ? view->state_text : executor_state_text(view->state),
                    10);
```

When a map snapshot exists, reuse the same `state_text` in the existing `S:` line instead of only showing executor state.

- [ ] **Step 4: Static check**

Run:

```powershell
rg -n "state_text|Wait Ctr|Launch|Wait Map" C:\Users\ye\Desktop\rt1064\project\user
```

Expected: execute page receives and displays menu run state.

---

### Task 7: Verify Build and Runtime Behavior

**Files:**
- Build only; no source changes unless previous tasks fail.

- [ ] **Step 1: Build**

Run:

```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Then check:

```powershell
Select-String -Path "C:\Users\ye\Desktop\rt1064\project\mdk\Objects\rt1064.build_log.htm" -Pattern "Error\(s\)|Warning\(s\)|Program Size" | Select-Object -Last 10
```

Expected:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 2: Bench launch-center test**

Setup:

- ART source mode.
- Car in left launch zone.
- OpenART must send valid `PLAYER_CENTER_GRID` from a launch-zone detector that does not depend on 16x12 character-map `C`.

Expected screen sequence:

```text
Wait 5s
Wait Ctr
Launch
Wait ART / Wait Map
```

Expected movement:

- If `PLAYER_CENTER_GRID col_q≈70`, MCU computes about `30 - 14 = 16cm`.
- Car moves right about 16cm, not fixed 20cm.
- Car stops before solving.

- [ ] **Step 3: True map test**

After launch movement, let upper computer show real Sokoban map.

Expected:

- If map has `C=1,B>0,T>0,B==T,C inner`, MCU solves and enters Running/Paused.
- If map is still launch-zone blue map with `B=0,T=0`, screen stays waiting and does not Done.

- [ ] **Step 4: Failure behavior**

Block OpenART center output.

Expected:

- Screen stays `Wait Ctr`.
- Car does not blindly move.
- K4 long press still exits safely.

---

## Self-Review

- Spec coverage: launch center collection, ART-to-MCU center data, computed movement, wait-map gate, no initial Done, and screen observability are all covered.
- Placeholder scan: no `TBD` or unspecified implementation steps remain.
- Type consistency: plan uses existing project types (`uint8`, `uint16`, `int16`, `map_scan_stats_struct`, `art_replan_update_struct`) and existing functions (`openart_get_player_center`, `drive_pose_reset`, `set_motion`, `reset_motion_segment`, `art_replan_get_debug_status`).
