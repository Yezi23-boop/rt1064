# ART Waypoint Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Decouple ART waypoint pose correction from full ART map replanning, so normal movement can continue after trusted low-frequency center correction while push actions still use ART map truth.

**Architecture:** MCU remains responsible for continuous 20ms motion control. ART provides two low-frequency observations at waypoint settle time: player center for pose correction, and stable RT map for virtual-state confirmation only when needed. The executor exposes explicit sync results so `art_replan.c` can choose continue, confirm push, or replan.

**Tech Stack:** RT1064 C firmware, OpenART MicroPython UART protocol, Keil MDK build, existing `executor.c`, `art_replan.c`, `openart_uart.c`, and `drive_pose.c`.

---

## File Structure

- Modify `project/user/inc/drive_config.h`
  - Add ART center correction parameters, gate thresholds, and optional debug switch.
- Modify `project/user/inc/executor.h`
  - Add a small sync result enum and functions for ART center correction status.
- Modify `project/user/src/executor.c`
  - Keep 20ms path tracking unchanged.
  - Add gated/fused pose correction from ART center samples.
  - Add "continue current path after normal waypoint" support.
- Modify `project/user/src/art_replan.c`
  - Split segment sync into normal move, push move, and abnormal map-confirm paths.
  - Use stable ART map for push confirmation and abnormal cases, not every ordinary waypoint.
- Modify `project/user/src/openart_uart.c`
  - Keep protocol compatible; no new required fields in first implementation.
  - Optionally expose latest center sample age/count through existing accessors only if needed.
- Modify `project/user/src/screen.c` and related view structs only if debug display is needed after the core logic works.

---

### Task 1: Add ART Center Correction Parameters

**Files:**
- Modify: `project/user/inc/drive_config.h`

- [ ] **Step 1: Add config constants near existing ART executor config**

Insert after `EXEC_ART_STABLE_FRAMES`:

```c
/** ART 中心点校正开关；只在 waypoint 停稳后使用，不参与 20ms 实时闭环。 */
#define EXEC_ART_CENTER_CORRECT_ENABLE (1)
/** ART 中心点需要收集的有效样本数；3 帧中值兼顾稳定性和 600ms 停稳窗口。 */
#define EXEC_ART_CENTER_SAMPLE_COUNT (3u)
/** ART 中心点小于该偏差不修正，单位 cm，避免原地小抖动反复写 pose。 */
#define EXEC_ART_CENTER_IGNORE_CM (1.0f)
/** ART 中心点允许直接融合的最大偏差，单位 cm；超过后不立刻相信。 */
#define EXEC_ART_CENTER_FUSE_MAX_CM (5.0f)
/** ART 中心点超过该偏差认为异常，触发地图确认/重算，单位 cm。 */
#define EXEC_ART_CENTER_ABNORMAL_CM (10.0f)
/** ART 中心点融合比例；0.30 表示本地 pose 保留 70%，ART 观测占 30%。 */
#define EXEC_ART_CENTER_FUSE_ALPHA (0.30f)
/** ART 中心点所在格允许与当前 C 格一致；0 表示不接受相邻格中心直接校正。 */
#define EXEC_ART_CENTER_ALLOW_NEIGHBOR_CELL (0)
```

- [ ] **Step 2: Static verify constants are visible**

Run:

```powershell
rg -n "EXEC_ART_CENTER" project/user/inc/drive_config.h
```

Expected: all seven constants appear once.

- [ ] **Step 3: Keil compile**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: `0 Error(s), 0 Warning(s)`.

---

### Task 2: Make Executor Report ART Center Correction Result

**Files:**
- Modify: `project/user/inc/executor.h`
- Modify: `project/user/src/executor.c`

- [ ] **Step 1: Add correction result enum to `executor.h`**

Add before function declarations:

```c
typedef enum {
    EXEC_ART_CENTER_NONE = 0,      /**< 没有足够中心点样本。 */
    EXEC_ART_CENTER_IGNORED,       /**< 中心偏差很小，不修正 pose。 */
    EXEC_ART_CENTER_APPLIED,       /**< 已按融合比例修正 pose。 */
    EXEC_ART_CENTER_REJECTED,      /**< 中心点不可信，没有修正 pose。 */
    EXEC_ART_CENTER_ABNORMAL       /**< 偏差过大，需要 ART 地图确认/重算。 */
} executor_art_center_result_enum;
```

Replace:

```c
uint8 executor_commit_art_player_center(void);
```

with:

```c
executor_art_center_result_enum executor_commit_art_player_center(void);
```

- [ ] **Step 2: Update `executor.c` return type and basic returns**

Change the function signature:

```c
executor_art_center_result_enum executor_commit_art_player_center(void)
```

Use these direct return replacements:

```c
if(0 == art_player_center_median_valid)
{
    return EXEC_ART_CENTER_NONE;
}
```

```c
art_player_center_median_valid = 0;
return EXEC_ART_CENTER_REJECTED;
```

- [ ] **Step 3: Add distance helpers in `executor.c`**

Add near `abs_float()`:

```c
static float sqrt_distance_cm(float x_cm, float y_cm)
{
    return sqrtf((x_cm * x_cm) + (y_cm * y_cm));
}
```

- [ ] **Step 4: Change commit logic from direct reset to gated fusion**

Inside `executor_commit_art_player_center()`, after `grid_q_to_physical(...)`, replace the direct reset block with:

```c
    pose = drive_pose_get();
    pose_yaw = pose->yaw_deg;

    {
        float dx = corrected_x_cm - pose->x_cm;
        float dy = corrected_y_cm - pose->y_cm;
        float diff_cm = sqrt_distance_cm(dx, dy);

        art_player_center_median_valid = 0;

        if(diff_cm < EXEC_ART_CENTER_IGNORE_CM)
        {
            return EXEC_ART_CENTER_IGNORED;
        }

        if(diff_cm > EXEC_ART_CENTER_ABNORMAL_CM)
        {
            return EXEC_ART_CENTER_ABNORMAL;
        }

        if(diff_cm > EXEC_ART_CENTER_FUSE_MAX_CM)
        {
            return EXEC_ART_CENTER_REJECTED;
        }

        corrected_x_cm = pose->x_cm + (dx * EXEC_ART_CENTER_FUSE_ALPHA);
        corrected_y_cm = pose->y_cm + (dy * EXEC_ART_CENTER_FUSE_ALPHA);
        drive_pose_reset(corrected_x_cm, corrected_y_cm, pose_yaw);
    }
    return EXEC_ART_CENTER_APPLIED;
```

- [ ] **Step 5: Static verify old return type is gone**

Run:

```powershell
rg -n "uint8 executor_commit_art_player_center|executor_art_center_result_enum executor_commit_art_player_center" project/user/inc project/user/src
```

Expected: no `uint8 executor_commit_art_player_center`; one declaration and one definition using `executor_art_center_result_enum`.

- [ ] **Step 6: Keil compile**

Run Keil build.

Expected: `0 Error(s), 0 Warning(s)`.

---

### Task 3: Add Normal-Waypoint Continue Path

**Files:**
- Modify: `project/user/inc/executor.h`
- Modify: `project/user/src/executor.c`

- [ ] **Step 1: Declare a function to continue after ART sync**

Add to `executor.h` near ART sync functions:

```c
/**
 * @brief ART 普通 waypoint 同步完成后继续当前路径。
 * @return 1 表示已切到下一 waypoint 或任务完成；0 表示当前没有等待 ART。
 */
uint8 executor_continue_after_art_sync(void);
```

- [ ] **Step 2: Implement it in `executor.c`**

Add after `executor_get_art_sync_action()`:

```c
uint8 executor_continue_after_art_sync(void)
{
    if(0 == executor_art_sync_pending())
    {
        return 0;
    }

    executor_advance_after_segment();
    return 1;
}
```

If `executor_advance_after_segment()` is still below this new function, move `executor_continue_after_art_sync()` to after `executor_advance_after_segment()` to avoid adding a forward declaration.

- [ ] **Step 3: Keil compile**

Run Keil build.

Expected: `0 Error(s), 0 Warning(s)`.

---

### Task 4: Split ART Segment Sync Decisions

**Files:**
- Modify: `project/user/src/art_replan.c`

- [ ] **Step 1: Add a helper to decide whether normal move needs map confirmation**

Add near `art_action_is_push()`:

```c
static uint8 art_center_result_needs_map(executor_art_center_result_enum center_result)
{
    return (EXEC_ART_CENTER_ABNORMAL == center_result) ? 1u : 0u;
}
```

- [ ] **Step 2: Capture center correction result before segment decision**

In `art_handle_stable_map()`, before checking `ART_REPLAN_SEGMENT`, add:

```c
    executor_art_center_result_enum center_result = EXEC_ART_CENTER_NONE;
```

In the segment branch, before `char sync_action = ...`, add:

```c
        center_result = executor_commit_art_player_center();
```

- [ ] **Step 3: For normal waypoint, continue without solve when center is acceptable**

Replace the normal waypoint block:

```c
        if(0 == art_action_is_push(sync_action))
        {
            // 普通 waypoint：只要 ART 稳定帧来了，就认为虚拟状态已同步，刷新基线即可。
            art_update_confirmed_counts(&stats);
        }
```

with:

```c
        if(0 == art_action_is_push(sync_action))
        {
            art_update_confirmed_counts(&stats);
            if(0 == art_center_result_needs_map(center_result))
            {
                (void)executor_continue_after_art_sync();
                art_replan_cancel();
                if(0 != update)
                {
                    update->run_state = "Running";
                    update->redraw = 1;
                }
                return;
            }
            if(0 != update)
            {
                update->run_state = "ART Replan";
            }
        }
```

- [ ] **Step 4: Remove duplicate center commit after replan**

In the segment solve success branch, remove:

```c
            (void)executor_commit_art_player_center();
```

because center correction is now committed once before action handling.

- [ ] **Step 5: Static verify normal move can continue without solve**

Run:

```powershell
rg -n "executor_continue_after_art_sync|ART Replan|Box OK|Push Retry" project/user/src/art_replan.c
```

Expected: all strings appear.

- [ ] **Step 6: Keil compile**

Run Keil build.

Expected: `0 Error(s), 0 Warning(s)`.

---

### Task 5: Keep Push Actions Map-Confirmed

**Files:**
- Modify: `project/user/src/art_replan.c`

- [ ] **Step 1: Review push branch semantics**

Confirm this logic remains in `ART_REPLAN_SEGMENT`:

```c
        else if(0 != art_stats_count_decreased(&stats))
        {
            art_update_confirmed_counts(&stats);
            if(0 != update)
            {
                update->run_state = "Box OK";
            }
        }
        else
        {
            if(0 != update)
            {
                update->run_state = "Push Retry";
            }
            printf("ART_PUSH_NOT_CONFIRMED action=%c frame=%lu B=%d/%d T=%d/%d C=%d,%d\r\n",
```

- [ ] **Step 2: Ensure push branch still falls through to solve**

Verify there is no `return` inside push confirmed or push retry branches before `solve_map(...)`.

Run:

```powershell
rg -n "ART_PUSH_NOT_CONFIRMED|solve_map|return;" project/user/src/art_replan.c
```

Expected: push branch logs, then later `solve_map(...)`; no early return in push branch.

- [ ] **Step 3: Keil compile**

Run Keil build.

Expected: `0 Error(s), 0 Warning(s)`.

---

### Task 6: Add Minimal Debug Prints for Field Validation

**Files:**
- Modify: `project/user/src/art_replan.c`

- [ ] **Step 1: Print center correction result at segment sync**

After `center_result = executor_commit_art_player_center();`, add:

```c
        printf("ART_CENTER result=%d action=%c\r\n", center_result, sync_action);
```

If `sync_action` is declared after the new line, reorder to:

```c
        char sync_action = executor_get_art_sync_action();
        center_result = executor_commit_art_player_center();
        printf("ART_CENTER result=%d action=%c\r\n", center_result, sync_action);
```

- [ ] **Step 2: Keil compile**

Run Keil build.

Expected: `0 Error(s), 0 Warning(s)`.

- [ ] **Step 3: On-board observe normal waypoint**

Expected wireless/debug output for a normal move:

```text
ART_CENTER result=1 action=r
```

or:

```text
ART_CENTER result=2 action=r
```

Then no `ART_REPLAN_OK` is required for that normal waypoint.

- [ ] **Step 4: On-board observe push waypoint**

Expected push output still includes either:

```text
Box OK
```

or:

```text
ART_PUSH_NOT_CONFIRMED
```

and then an `ART_REPLAN_OK` after solve succeeds.

---

### Task 7: Final Verification

**Files:**
- Verify only; no expected source edits.

- [ ] **Step 1: Static search for old always-replan behavior**

Run:

```powershell
rg -n "executor_commit_art_player_center|executor_continue_after_art_sync|solve_map" project/user/src/art_replan.c project/user/src/executor.c project/user/inc/executor.h
```

Expected:
- `executor_commit_art_player_center()` called once in `art_replan.c`.
- `executor_continue_after_art_sync()` exists and is called for normal waypoint path.
- `solve_map()` remains in `art_replan.c` for initial, push, and abnormal replan.

- [ ] **Step 2: Full Keil build**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: `0 Error(s), 0 Warning(s)`.

- [ ] **Step 3: ART mode field test**

Set ART source and Run mode. Expected behavior:
- Start waits 5 seconds, then solves from stable ART map.
- Ordinary movement waypoint: car stops 600ms, ART center is sampled, pose correction result prints, then execution continues without full replan unless center is abnormal.
- Push waypoint: ART stable map confirms B/T decrease or triggers push retry/replan.
- If ART center is invalid or too different, the system does not blindly reset pose.

- [ ] **Step 4: Failure observation checklist**

If normal waypoints still pause too long:
- Check whether `ART_CENTER result=4` appears often; that means center is classified abnormal and forcing map confirmation.
- Increase `EXEC_ART_CENTER_ABNORMAL_CM` only after confirming OpenART center is noisy but map cell is correct.

If pose correction seems ineffective:
- Check whether result is mostly `EXEC_ART_CENTER_NONE`.
- Confirm OpenART is sending `PLAYER_CENTER_GRID ... 1` during the 600ms settle window.

If pose jumps:
- Lower `EXEC_ART_CENTER_FUSE_ALPHA` from `0.30f` to `0.20f`.
- Increase `EXEC_ART_CENTER_IGNORE_CM` from `1.0f` to `1.5f`.

---

## Self-Review

- Spec coverage: The plan separates center correction from map confirmation, keeps MCU as continuous controller, keeps ART map as virtual-state truth for push actions, and avoids normal waypoint full replan unless abnormal.
- Placeholder scan: No TBD/TODO placeholders remain.
- Type consistency: `executor_art_center_result_enum` is introduced in `executor.h`, returned by `executor_commit_art_player_center()`, and consumed by `art_replan.c`.
- Scope check: This plan intentionally does not change OpenART image recognition, UART wire format, solver BFS, or PID tuning.
