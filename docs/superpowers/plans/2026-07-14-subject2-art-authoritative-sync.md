# Subject2 ART Authoritative Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让科目二在关键同步点以 ART 冻结地图和5帧中心中值为全局权威，增量更新小车与对象位置，并避免合法位置变化误报 `E:CMap` 或触发全量重新识别。

**Architecture:** 在 `subject2_logic` 增加可主机测试的中心地图归一化和对象增量同步纯逻辑；`subject2` 状态机负责冻结、等待和应用结果；`executor` 只补一个只读中心中值接口，供推箱前提交使用。连续运动期间仍由 MCU pose 控制，不引入逐帧视觉闭环。

**Tech Stack:** C99、RT1064/Keil MDK、PowerShell GCC 主机测试、现有 OpenART UART 文本协议。

---

## Scope And File Map

- Modify: `project/user/inc/subject2_logic.h` — 声明中心地图归一化、对象增量同步结果和纯逻辑接口。
- Modify: `project/user/src/subject2_logic.c` — 实现地图归一化、对象差异匹配、局部失效和多完成判定。
- Modify: `project/user/inc/subject2.h` — 增加扫描地图同步状态。
- Modify: `project/user/src/subject2.c` — 接入扫描中心、推箱前中心、扫描对象变化和任务完成重规划。
- Modify: `project/user/inc/executor.h` — 声明只读的 ART 中心中值接口。
- Modify: `project/user/src/executor.c` — 在线程安全临界区内返回当前5帧中心中值。
- Modify: `tests/subject2_logic_test.c` — 覆盖中心归一化和对象身份增量同步。
- Modify: `tests/subject2_scan_test.c` — 覆盖 `VReplan`、扫描同步、局部重识别、多完成和返航。
- Modify: `tests/executor_pre_push_test.c` — 覆盖中心中值只读接口不消费提交状态。
- Modify: `docs/superpowers/specs/2026-07-14-subject2-art-authoritative-sync-design.md` — 按最终实现同步 `E:CMap` 和 ART 权威同步说明。

`openmv/main_see.py`、UART 文本协议、`art_replan.c`、PID、BFS 和20cm格距不在本次修改范围内。

## Execution Safety

当前工作树包含用户尚未提交的固件修改。执行时必须在当前工作树继续，不得 reset、checkout 或格式化无关文件。每次提交只使用明确路径的 `git commit --only`，避免把用户已暂存的其他修改带入提交。

### Task 1: Establish The Baseline

**Files:**
- Test: `tests/run_subject2_logic_test.ps1`
- Test: `tests/run_subject2_scan_test.ps1`
- Test: `tests/run_executor_pre_push_test.ps1`
- Test: `tests/run_openart_center_request_test.ps1`

- [ ] **Step 1: Record the current dirty state**

Run:

```powershell
git status --short
git diff -- project/user/src/subject2.c project/user/src/subject2_logic.c project/user/src/executor.c
git diff --cached -- project/user/src/subject2.c project/user/src/subject2_logic.c project/user/src/executor.c
```

Expected: existing user changes are visible and are not reverted.

- [ ] **Step 2: Run the focused baseline tests**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
```

Expected: all four scripts exit `0`. If an existing failure appears, record it before changing source and do not silently rewrite the expected behavior.

### Task 2: Normalize A Frozen Map To The Five-Sample Median

**Files:**
- Modify: `project/user/inc/subject2_logic.h`
- Modify: `project/user/src/subject2_logic.c`
- Test: `tests/subject2_logic_test.c`

- [ ] **Step 1: Add failing pure-logic tests**

Add cases that call the new interface:

```c
uint8 subject2_normalize_center_map(
    const map_source_struct *source,
    uint16 center_col_q,
    uint16 center_row_q,
    map_source_struct *normalized,
    char normalized_rows[MAP_ROWS][MAP_COLS + 1],
    uint8 *car_row,
    uint8 *car_col);
```

The tests must verify:

```c
/* Same cell: keep C and all B/T cells unchanged. */
subject2_normalize_center_map(&map.source, 550u, 550u,
                              &normalized, rows, &row, &col) == 1u;
row == 5u && col == 5u;

/* Boundary jitter: source C=(5,6), median=(5,5), move C to median. */
/* If old C was '+', restore old cell to 'T'. */
/* If new cell was 'T', write '+' instead of 'C'. */

/* Reject out-of-map center, a delta greater than one cell, and B/#/X destination. */
```

- [ ] **Step 2: Run the logic test and verify failure**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
```

Expected: compile failure because `subject2_normalize_center_map` is not declared.

- [ ] **Step 3: Declare the normalization interface**

Add the exact declaration above to `subject2_logic.h`, with a comment stating that `source` must contain exactly one `C/+`, the center may be in that cell or an eight-neighbor cell, and the destination map is an owned snapshot.

- [ ] **Step 4: Implement the minimal normalization logic**

Implement this sequence in `subject2_logic.c`:

```c
if((0 == source) || (0 == normalized) || (0 == normalized_rows) ||
   (0 == car_row) || (0 == car_col) ||
   (center_col_q >= MAP_COLS * 100u) ||
   (center_row_q >= MAP_ROWS * 100u) ||
   (0 == map_find_car(source, &source_row, &source_col, 0)))
{
    return 0u;
}

median_col = (uint8)(center_col_q / 100u);
median_row = (uint8)(center_row_q / 100u);
if((absolute_difference(source_row, median_row) > 1u) ||
   (absolute_difference(source_col, median_col) > 1u))
{
    return 0u;
}

destination = source->rows[median_row][median_col];
if(('.' != destination) && ('T' != destination) &&
   ('C' != destination) && ('+' != destination))
{
    return 0u;
}

map_source_snapshot(normalized, normalized_rows, source);
if((source_row != median_row) || (source_col != median_col))
{
    normalized_rows[source_row][source_col] =
        ('+' == normalized_rows[source_row][source_col]) ? 'T' : '.';
    normalized_rows[median_row][median_col] =
        ('T' == destination) ? '+' : 'C';
}
*car_row = median_row;
*car_col = median_col;
return 1u;
```

- [ ] **Step 5: Run the test and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
git diff --check -- project/user/inc/subject2_logic.h project/user/src/subject2_logic.c tests/subject2_logic_test.c
```

Expected: PASS and no whitespace errors.

Commit only these paths:

```powershell
git add project/user/inc/subject2_logic.h project/user/src/subject2_logic.c tests/subject2_logic_test.c
git commit --only -m "feat: normalize subject2 center snapshots" -- project/user/inc/subject2_logic.h project/user/src/subject2_logic.c tests/subject2_logic_test.c
```

### Task 3: Make Scan Center Use The Normalized Frozen Snapshot

**Files:**
- Modify: `project/user/src/subject2.c:23-70,275-387,567-675`
- Test: `tests/subject2_scan_test.c:1614-1699`

- [ ] **Step 1: Replace the old CMap expectation with failing authority tests**

Add two tests:

```c
/* Five samples have median in (5,5), but the frozen map's last C is (5,6). */
/* Expected: VAdj from authoritative median, never E:CMap. */

/* Median and normalized C move from observation (5,5) to (5,6), B/T unchanged. */
/* Expected: SUBJECT2_SCAN_BOX_PLAN + "VReplan", start=(5,6), no correction. */
```

Retain the existing tests for paired-map isolation, abnormal center, `E:CRef`, executor busy and center-adjust timeout.

- [ ] **Step 2: Run and verify the new cases fail**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: the boundary case reaches `E:CMap` or uses the last map C instead of the median C.

- [ ] **Step 3: Add one owned normalized snapshot to subject2**

Add static storage next to the current center arrays:

```c
static char center_map_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct center_map_source;
```

Initialize its row pointers through `map_source_snapshot()` only when a complete center result is available; no separate init function is needed.

- [ ] **Step 4: Pass the source explicitly to center application**

Change the internal helper from reading `openart_get_requested_center_map()` itself to receiving the already normalized source:

```c
static uint8 subject2_apply_center(
    const subject2_context_struct *context,
    const map_source_struct *source,
    uint8 require_observation_match,
    float *applied_x_cm,
    float *applied_y_cm);
```

Update its callers in scan and replan code. Replan must normalize its frozen map first; timeout fallback may continue using the unique live map cell with offset `(0,0)`.

- [ ] **Step 5: Rewrite the successful center branch**

After `subject2_collect_center()` succeeds:

```c
center_col_q = subject2_median_u16(center_col_samples);
center_row_q = subject2_median_u16(center_row_samples);
source = openart_get_requested_center_map();
if(0 == subject2_normalize_center_map(source,
                                      center_col_q, center_row_q,
                                      &center_map_source, center_map_rows,
                                      &car_row, &car_col))
{
    subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRef", update);
    return;
}
```

If `B/T` are unchanged and `(car_row,car_col)` equals the observation cell, apply the normalized center and start correction. If only the car cell changed, apply the center, preserve `box_objects`, `target_objects` and `bindings`, then enter the appropriate `*_PLAN` state with `VReplan`.

Do not call `subject2_observation_map_matches()` to reject a C-only change. Keep object-change handling for Task 6.

- [ ] **Step 6: Run focused tests and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
git diff --check -- project/user/src/subject2.c tests/subject2_scan_test.c
```

Expected: all tests pass; C-only changes no longer produce `E:CMap`.

Commit:

```powershell
git add project/user/src/subject2.c tests/subject2_scan_test.c
git commit --only -m "fix: trust normalized ART car position in subject2" -- project/user/src/subject2.c tests/subject2_scan_test.c
```

### Task 4: Use The Frozen Center Snapshot Before Push

**Files:**
- Modify: `project/user/inc/executor.h:220-245`
- Modify: `project/user/src/executor.c:670-803`
- Modify: `project/user/src/subject2.c:1374-1463`
- Test: `tests/executor_pre_push_test.c`
- Test: `tests/subject2_scan_test.c`

- [ ] **Step 1: Add failing executor and subject2 tests**

Add an executor test proving this read-only interface returns the median without consuming it:

```c
uint8 executor_get_art_player_center_median(uint16 *col_q, uint16 *row_q);
```

Test sequence:

```c
/* Push five samples, read median, then commit successfully. */
executor_get_art_player_center_median(&col_q, &row_q) == 1u;
executor_commit_art_player_center((uint8)(row_q / 100u),
                                  (uint8)(col_q / 100u)) != EXEC_ART_CENTER_NONE;
```

Add a subject2 test where the center request's frozen map has one valid C, then the live map gains a second C before `subject2_tick()`. Expected: pre-push uses the frozen map and does not enter `E:CPsh`.

- [ ] **Step 2: Run both tests and verify failure**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: missing getter compile failure and/or pre-push rejects the live double-C map.

- [ ] **Step 3: Implement the read-only median getter**

Use the existing critical-section pattern:

```c
uint8 executor_get_art_player_center_median(uint16 *col_q, uint16 *row_q)
{
    uint8 valid;
    uint32 primask = interrupt_global_disable();

    valid = art_player_center_median_valid;
    if((0u != valid) && (0 != col_q) && (0 != row_q))
    {
        *col_q = art_player_center_median_col_q;
        *row_q = art_player_center_median_row_q;
    }
    interrupt_global_enable(primask);
    return valid;
}
```

Do not clear `art_player_center_median_valid`; only `executor_commit_art_player_center()` consumes it.

- [ ] **Step 4: Normalize and use the frozen map in pre-push**

In `subject2_tick_pre_push_center()`:

```c
source = openart_get_requested_center_map();
if((0 == executor_get_art_player_center_median(&center_col_q, &center_row_q)) ||
   (0 == subject2_normalize_center_map(source,
                                       center_col_q, center_row_q,
                                       &center_map_source, center_map_rows,
                                       &car_row, &car_col)))
{
    subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
    return;
}
result = executor_commit_art_player_center(car_row, car_col);
```

When commit reports rejected or abnormal, pass `&center_map_source` to `subject2_accept_confirmed_map()`. Do not read `openart_map_get()` in this successful request path.

- [ ] **Step 5: Run tests and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
git diff --check -- project/user/inc/executor.h project/user/src/executor.c project/user/src/subject2.c tests/executor_pre_push_test.c tests/subject2_scan_test.c
```

Expected: PASS.

Commit:

```powershell
git add project/user/inc/executor.h project/user/src/executor.c project/user/src/subject2.c tests/executor_pre_push_test.c tests/subject2_scan_test.c
git commit --only -m "fix: pair pre-push center with frozen ART map" -- project/user/inc/executor.h project/user/src/executor.c project/user/src/subject2.c tests/executor_pre_push_test.c tests/subject2_scan_test.c
```

### Task 5: Add Pure Incremental Object Reconciliation

**Files:**
- Modify: `project/user/inc/subject2_logic.h`
- Modify: `project/user/src/subject2_logic.c`
- Test: `tests/subject2_logic_test.c`

- [ ] **Step 1: Define and test the reconciliation contract**

Add this result type and interface:

```c
typedef enum
{
    SUBJECT2_SYNC_OK = 0,
    SUBJECT2_SYNC_RESCAN,
    SUBJECT2_SYNC_AMBIGUOUS
} subject2_sync_result_enum;

typedef struct
{
    uint8 need_box_scan;
    uint8 need_target_scan;
    uint8 completed_count;
} subject2_sync_update_struct;

subject2_sync_result_enum subject2_reconcile_objects(
    const map_source_struct *source,
    subject2_object_struct box_objects[MAX_BOXES],
    uint8 *box_count,
    subject2_object_struct target_objects[MAX_BOXES],
    uint8 *target_count,
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
    uint8 strict_push_tracking,
    uint8 active_class,
    subject2_sync_update_struct *update);
```

Add failing tests for:

- C-only change: all objects and bindings byte-for-byte unchanged, result `SYNC_OK`.
- One recognized box moves: transfer its `cell` and binding `box_cell`, result `SYNC_OK`.
- One unrecognized box moves: transfer object state and keep it unrecognized.
- Multiple ambiguous box moves during scanning: preserve unaffected objects, clear only affected bindings, mark only affected new objects unrecognized, result `SYNC_RESCAN` with `need_box_scan=1`.
- Same ambiguous change with `strict_push_tracking=1`: result `SYNC_AMBIGUOUS` and do not partially mutate caller state.
- One or multiple bound target cells and corresponding boxes disappear: mark those bindings completed and increment `completed_count`.
- A target appears or moves: result `SYNC_AMBIGUOUS` because virtual targets are stationary.
- `B!=T`: result `SYNC_AMBIGUOUS`.

- [ ] **Step 2: Run and verify compile failure**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
```

Expected: missing type/function compile failure.

- [ ] **Step 3: Implement reconciliation transactionally**

Implement with local temporary arrays and commit mutations only after validation:

```c
subject2_object_struct next_boxes[MAX_BOXES];
subject2_object_struct next_targets[MAX_BOXES];
subject2_binding_struct next_bindings[SUBJECT2_CLASS_COUNT];
```

Required order:

1. Collect new `B` and `T/+` cells and require equal counts.
2. Copy unchanged target objects by cell.
3. Treat a missing bound target plus its missing bound box as completion; keep the binding and set `completed=1`.
4. Reject any newly appearing target cell.
5. Copy unchanged box objects by cell.
6. If exactly one unmatched old box and one unmatched new box remain, transfer identity and update its binding.
7. If multiple unmatched boxes remain in scan mode, clear only bindings belonging to unmatched recognized old boxes, create unrecognized objects for unmatched new cells and request box rescan.
8. In strict push mode, require all non-active uncompleted boxes to remain at their bound cells; use `subject2_track_active_box()` for the active box and return ambiguous on any other movement.
9. Copy temporary arrays and bindings to callers only for `SYNC_OK` or `SYNC_RESCAN`.

Do not clear the full binding table anywhere in this function.

- [ ] **Step 4: Run the logic tests and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
git diff --check -- project/user/inc/subject2_logic.h project/user/src/subject2_logic.c tests/subject2_logic_test.c
```

Expected: all reconciliation cases PASS.

Commit:

```powershell
git add project/user/inc/subject2_logic.h project/user/src/subject2_logic.c tests/subject2_logic_test.c
git commit --only -m "feat: reconcile subject2 ART object changes" -- project/user/inc/subject2_logic.h project/user/src/subject2_logic.c tests/subject2_logic_test.c
```

### Task 6: Add Scan Map Sync Without Full Rebinding

**Files:**
- Modify: `project/user/inc/subject2.h:8-39`
- Modify: `project/user/src/subject2.c:53-70,880-929,1465-1602,1733-1820`
- Test: `tests/subject2_scan_test.c:1336-1351`

- [ ] **Step 1: Add failing scan-sync state tests**

Replace `map_change_stops_scan()` with tests that verify:

```text
single box move -> ScanSync -> preserve class -> resume planning
C-only move -> VReplan directly, never ScanSync
ambiguous box moves -> ScanSync -> only affected box classifications cleared
B=T=0 -> RETURN_REQUESTED
B!=T -> ERROR after stable confirmation
```

The single-box test must record `vision_request_id` and prove no new classification request is sent when the moved box was already recognized and uniquely tracked.

- [ ] **Step 2: Run and verify the old code fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: current code enters `SUBJECT2_ERROR` from `subject2_tick_plan()` or center handling.

- [ ] **Step 3: Add one scan-sync state and an internal phase**

Add `SUBJECT2_SCAN_MAP_SYNC` to `subject2_state_enum`. In `subject2.c` add:

```c
typedef enum
{
    SUBJECT2_SCAN_SYNC_WAIT_MAP = 0,
    SUBJECT2_SCAN_SYNC_WAIT_CENTER
} subject2_scan_sync_phase_enum;

static subject2_scan_sync_phase_enum scan_sync_phase;
```

Reuse `confirm_last_frame`, `confirm_candidate_rows`, `confirm_candidate_valid` and `confirm_stable_count`; scan sync and push confirmation are mutually exclusive states.

- [ ] **Step 4: Enter sync instead of failing on B/T changes**

Create:

```c
static void subject2_begin_scan_map_sync(subject2_update_struct *update)
{
    executor_stop();
    vision_uart_cancel();
    confirm_last_frame = openart_uart_get_frame_count();
    confirm_candidate_valid = 0u;
    confirm_stable_count = 0u;
    scan_sync_phase = SUBJECT2_SCAN_SYNC_WAIT_MAP;
    subject2_state = SUBJECT2_SCAN_MAP_SYNC;
    update->run_state = "VSync";
    update->redraw = 1u;
}
```

Call it from scan planning and center handling when normalized `B/T` sets differ from current objects. Do not call it for a C-only difference.

- [ ] **Step 5: Implement the two-phase sync tick**

`WAIT_MAP` uses the existing frame-count and `map_rows_equal()` pattern. Once `EXEC_ART_STABLE_FRAMES` is reached and the map has one car with `B==T`, send `CENTER_REQ`, clear center samples and switch to `WAIT_CENTER`.

`WAIT_CENTER` collects five samples, normalizes the frozen map, calls:

```c
sync_result = subject2_reconcile_objects(
    &center_map_source,
    box_objects, &box_object_count,
    target_objects, &target_object_count,
    bindings, 0u, active_class, &sync_update);
```

Then apply the normalized center and choose the next state:

```text
B=T=0                         -> SUBJECT2_RETURN_REQUESTED
need_box_scan                 -> VISION_MODE_BOX / SUBJECT2_SCAN_BOX_MODE
need_target_scan              -> VISION_MODE_TARGET / SUBJECT2_SCAN_TARGET_MODE
all objects recognized        -> SUBJECT2_VALIDATE_BINDINGS
otherwise current scan phase  -> corresponding *_PLAN
```

`SYNC_AMBIGUOUS`, invalid map or center timeout uses existing `E:Map`, `E:Track`, `E:CTmo`/fallback policy as appropriate. Do not introduce a new configuration macro.

- [ ] **Step 6: Reset the new phase on begin/cancel**

Set `scan_sync_phase = SUBJECT2_SCAN_SYNC_WAIT_MAP` in `subject2_begin()` and `subject2_cancel()`. Add the new state to the main `subject2_tick()` switch.

- [ ] **Step 7: Run tests and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
git diff --check -- project/user/inc/subject2.h project/user/src/subject2.c tests/subject2_scan_test.c
```

Expected: scan changes synchronize without full rebinding; invalid maps still stop.

Commit:

```powershell
git add project/user/inc/subject2.h project/user/src/subject2.c tests/subject2_scan_test.c
git commit --only -m "feat: synchronize subject2 scan maps incrementally" -- project/user/inc/subject2.h project/user/src/subject2.c tests/subject2_scan_test.c
```

### Task 7: Accept Multiple Host-Completions And Preserve Remaining Bindings

**Files:**
- Modify: `project/user/src/subject2.c:1479-1568`
- Test: `tests/subject2_scan_test.c:651-676,893-930`

- [ ] **Step 1: Add failing push reconciliation tests**

Add cases for:

- one active binding completes: existing behavior remains.
- two bound box/target pairs disappear in one stable map: both bindings become completed, remaining bindings unchanged.
- active box moves with unchanged counts: existing active binding cell updates and no yaw restore repeats.
- a non-active box moves during strict push reconciliation: `E:Track`.
- all `B/T` disappear: center request then `RETURN_REQUESTED`, no classification mode request.

- [ ] **Step 2: Run and verify multi-completion fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: current `stats.box_count + 1 == task_start_box_count` branch rejects a reduction greater than one.

- [ ] **Step 3: Replace exact-one completion logic**

In `subject2_accept_confirmed_map()`:

```c
sync_result = subject2_reconcile_objects(
    source,
    box_objects, &box_object_count,
    target_objects, &target_object_count,
    bindings, 1u, active_class, &sync_update);
```

Accept `SYNC_OK` when:

- counts are unchanged and only the active box moved, or
- box and target counts decreased equally by one or more, and every disappearance maps to completed bindings.

On acceptance, snapshot the new map, stop the old executor and enter `subject2_begin_replan_center()`. Pass `for_return=1` only when both counts are zero. Do not call `subject2_bindings_clear()` and do not start a vision mode.

Return `E:Track` for `SYNC_AMBIGUOUS` or `SYNC_RESCAN` in strict push mode.

- [ ] **Step 4: Keep the host monitor cheap**

Leave `subject2_handle_host_completion()` active only in `SUBJECT2_EXECUTE_PUSH` and `SUBJECT2_PRE_PUSH_CENTER`. It may trigger only when `B/T` counts both decrease by the same positive amount. Do not monitor same-count box movement every periodic frame; that remains a task-end/turn synchronization concern.

- [ ] **Step 5: Run tests and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
git diff --check -- project/user/src/subject2.c tests/subject2_scan_test.c
```

Expected: multi-completion and active-box retry PASS without extra classification.

Commit:

```powershell
git add project/user/src/subject2.c tests/subject2_scan_test.c
git commit --only -m "fix: reconcile ART host completion in subject2" -- project/user/src/subject2.c tests/subject2_scan_test.c
```

### Task 8: Update Error Semantics And Run Full Verification

**Files:**
- Modify: `docs/superpowers/specs/2026-07-14-subject2-art-authoritative-sync-design.md`
- Verify: all modified source and tests

- [ ] **Step 1: Search for stale CMap behavior**

Run:

```powershell
rg -n 'E:CMap|center_map_mismatch|map_change_stops_scan|全量.*绑定|清空.*绑定' project/user tests docs/competition docs/superpowers
```

Expected: no runtime branch uses `E:CMap` for a C-only mismatch. Historical plans may mention it; do not rewrite unrelated historical documents.

- [ ] **Step 2: Update the active subject2 usage/error table**

Document these final meanings:

```text
E:CTmo = center samples timed out under strict policy
E:CRef = center result is out of range or not adjacent to its frozen map
E:Map  = stable ART map structure is invalid
E:Track = object identity cannot be recovered uniquely
VReplan = ART changed only the authoritative car position
VSync = ART changed B/T and incremental synchronization is running
```

- [ ] **Step 3: Run all C host tests**

Run:

```powershell
Get-ChildItem tests -Filter "run_*.ps1" | ForEach-Object {
    powershell -ExecutionPolicy Bypass -File $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Expected: every script exits `0`.

- [ ] **Step 4: Run all Python tests and syntax checks**

Run:

```powershell
Get-ChildItem tests -Filter "*_test.py" | ForEach-Object {
    python $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
python -m py_compile openmv/main_see.py "openmv/视觉/main.py"
```

Expected: all tests and both syntax checks pass. If the known legacy assertion in `openart_request_flow_test.py` still conflicts with the approved timeout-fallback design, report it separately; do not weaken the new center-map tests to satisfy that obsolete assertion.

- [ ] **Step 5: Build Keil**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: build log reports `0 Error(s), 0 Warning(s)`. Do not download to the board unless the user explicitly asks.

- [ ] **Step 6: Final diff audit**

Run:

```powershell
git diff --check
git status --short
git diff --stat
```

Expected: no whitespace errors; all unrelated dirty files remain untouched.

- [ ] **Step 7: Commit documentation only if changed**

```powershell
git add docs/superpowers/specs/2026-07-14-subject2-art-authoritative-sync-design.md
git commit --only -m "docs: describe subject2 ART synchronization" -- docs/superpowers/specs/2026-07-14-subject2-art-authoritative-sync-design.md
```

Skip this commit when neither file changed.

## On-Board Acceptance Checklist

The implementation is not considered complete until these board checks are performed or explicitly deferred by the user:

```text
1. Observation center crosses a grid boundary: VReplan, no E:CMap.
2. Previously recognized box moves one grid: binding follows it, no repeated classification.
3. Unrecognized box moves: only that object is still pending recognition.
4. Push path stays continuous between real direction changes.
5. Upstream removes one or multiple B/T pairs: old path stops and remaining bindings survive.
6. All B/T disappear: return flow starts without rescanning images.
7. Invalid B!=T or ambiguous non-active box movement: vehicle stops with E:Map/E:Track.
```
