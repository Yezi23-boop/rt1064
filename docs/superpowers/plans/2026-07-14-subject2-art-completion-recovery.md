# Subject2 ART Completion Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让科目二任务结束时接受 ART 权威的合法完成结果，并在剩余箱子身份不唯一时局部重识别，而不是立即 `E:Track`。

**Architecture:** `subject2_logic.c` 负责把新旧 B/T 集合协调为“直接接受、局部重扫或不可恢复”；`subject2.c` 负责等待合法稳定图、提交协调结果并在中心更新后进入局部箱子扫描；`screen.c` 只补齐错误文本。保留现有 UART、BFS、executor 和 PID 边界。

**Tech Stack:** C99、GCC 主机测试、Keil MDK、现有 `map_source_struct`/`subject2_*` 状态机。

---

### Task 1: ART 权威对象协调

**Files:**
- Modify: `tests/subject2_logic_test.c`
- Modify: `project/user/src/subject2_logic.c`

- [ ] **Step 1: 写唯一迁移的失败测试**

在 `tests/subject2_logic_test.c` 增加场景：旧图有两个已绑定箱子和两个目标，活动类别的目标与箱子消失，另一个箱子从旧格移动到唯一新格。调用 `subject2_reconcile_objects(..., strict_push_tracking=1, ...)` 后期望：

```c
SUBJECT2_SYNC_OK
bindings[active_class].completed == 1u
bindings[remaining_class].box_cell == moved_cell
box_count == 1u
target_count == 1u
```

- [ ] **Step 2: 运行测试确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1`

Expected: 新增用例 FAIL，现有严格分支因非活动箱离开旧格返回 `SUBJECT2_SYNC_AMBIGUOUS`。

- [ ] **Step 3: 写多个歧义箱子的失败测试**

增加三个箱子场景：一个绑定完成，两个剩余箱子都移动到新格，无法仅靠集合恢复身份。期望返回 `SUBJECT2_SYNC_RESCAN`，并且：

```c
update.need_box_scan == 1u
未变化的目标 binding 保持有效
两个受影响 box binding 失效
新箱子对象 recognized == 0u
```

- [ ] **Step 4: 运行测试确认第二个 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1`

Expected: 歧义用例当前返回 `SUBJECT2_SYNC_AMBIGUOUS`。

- [ ] **Step 5: 最小实现剩余箱子协调**

调整 `subject2_reconcile_objects()`：

```c
目标格消失 -> 对应 binding.completed = 1
仅对未完成 binding 匹配新 B
先匹配原格不变项
一旧一新未匹配 -> 更新该 binding.box_cell
多旧多新未匹配 -> 仅失效受影响 box binding，创建未识别新箱子并返回 RESCAN
```

删除“已完成类别旧 box_cell 仍出现在新 B 集合就立即 AMBIGUOUS”的假设；ART 只提供无类别 B，旧格可能已被另一个箱子占据。

- [ ] **Step 6: 运行逻辑测试确认 GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1`

Expected: 所有用例 PASS，`-Wall -Wextra -Werror` 无警告。

### Task 2: 合法稳定图等待与局部重识别状态

**Files:**
- Modify: `tests/subject2_scan_test.c`
- Modify: `project/user/src/subject2.c`

- [ ] **Step 1: 写过渡图继续等待的失败测试**

在任务结束确认状态依次提供：

```text
frame A: B=1,T=0
frame B: car_count=2
frame C/D: B=1,T=1,car_count=1
```

前两种图期望保持 `SUBJECT2_CONFIRM_MAP`，合法稳定图达到帧数后才进入 `SUBJECT2_REPLAN_CENTER`，不得进入 `SUBJECT2_ERROR`。

- [ ] **Step 2: 运行扫描测试确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1`

Expected: 当前实现接受两帧结构非法候选后调用协调并进入 `E:Track`。

- [ ] **Step 3: 实现合法候选过滤**

在 `subject2_tick_confirm_map()` 中，只有以下地图进入候选稳定计数：

```c
stats.car_count == 1u
stats.box_count == stats.target_count
stats.box_count <= task_start_box_count
stats.target_count <= task_start_target_count
```

非法帧清空 `confirm_candidate_valid/confirm_stable_count` 并继续 `ART Wait`；总等待计时不重置，持续非法最终仍走 `E:ATim`。

- [ ] **Step 4: 写局部重识别的失败测试**

构造协调结果需要箱子重扫的任务结束图，期望流程：

```text
CONFIRM_MAP -> REPLAN_CENTER -> VISION_MODE BOX -> SCAN_BOX_PLAN
```

验证目标识别结果和未受影响 binding 保留，且不进入 `E:Track`。

- [ ] **Step 5: 运行测试确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1`

Expected: 当前 `subject2_accept_confirmed_map()` 将 `SUBJECT2_SYNC_RESCAN` 当作错误。

- [ ] **Step 6: 实现重规划后的局部箱子扫描**

在 `subject2.c` 增加一个内部 pending 标志保存协调结果：

```c
sync_result == SUBJECT2_SYNC_RESCAN && sync_update.need_box_scan
    -> 提交最新对象、binding 和 snapshot
    -> 请求中心并应用最新 pose
    -> 切换 VISION_MODE_BOX
    -> 从未识别 box 继续扫描
```

`SUBJECT2_SYNC_AMBIGUOUS`、目标重扫请求或最终绑定冲突仍停车；`B=0,T=0` 保持返航路径。

- [ ] **Step 7: 运行扫描测试确认 GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1`

Expected: 宏开启和关闭两轮全部 PASS。

### Task 3: 错误显示和验证

**Files:**
- Modify: `project/user/src/screen.c`

- [ ] **Step 1: 补全 Track 错误文本**

在 `executor_error_text()` 增加：

```c
case EXEC_ERROR_SUBJECT2_TRACK: return "E:Trk";
```

- [ ] **Step 2: 运行全量主机测试**

Run:

```powershell
Get-ChildItem tests -Filter "run_*.ps1" | ForEach-Object {
    powershell -ExecutionPolicy Bypass -File $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
Get-ChildItem tests -Filter "*_test.py" | ForEach-Object {
    python $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Expected: 除仓库已知缺少训练样本的数据测试外，其余测试 PASS；任何新增失败必须修复。

- [ ] **Step 3: 运行 Keil 编译**

Run: `D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"`

Expected: `0 Error(s), 0 Warning(s)`。

- [ ] **Step 4: 最终静态检查**

Run: `git diff --check`

Expected: 无空白错误；确认只修改规格涉及的逻辑、测试和错误显示。
