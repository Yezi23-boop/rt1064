# Subject2 Duplicate Classes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让科目二支持多个同类卡通箱子与多个同数字目标，并按同类别任意互配完成扫描、规划、ART 同步和重试。

**Architecture:** 以 `box_objects[]` 和 `target_objects[]` 作为类别与位置的唯一事实来源，使用每类别数量校验代替单实例 `bindings[class]`。规划器枚举同类箱子与目标组合；ART 同步按目标剩余类别数量恢复箱子类别，无法唯一恢复时只局部重扫未匹配对象。

**Tech Stack:** C99、现有 `subject2` 状态机、Sokoban BFS、GCC 主机测试、Keil MDK。

---

## File Structure

- `project/user/inc/subject2_logic.h`：定义对象级类别校验与同步接口，移除单实例 binding API。
- `project/user/src/subject2_logic.c`：实现按类别计数、异常类别失效和重复类别对象协调。
- `project/user/src/subject2.c`：分类结果写入对象、动态配对、具体任务重试和同步状态集成。
- `tests/subject2_logic_test.c`：纯逻辑重复类别与对象协调测试。
- `tests/subject2_scan_test.c`：完整扫描、最短配对、重试和局部重扫状态测试。
- `docs/superpowers/specs/2026-07-15-subject2-duplicate-classes-design.md`：实现后同步最终接口名称和错误语义。

### Task 1: 类别数量校验

**Files:**
- Modify: `tests/subject2_logic_test.c`
- Modify: `project/user/inc/subject2_logic.h`
- Modify: `project/user/src/subject2_logic.c`

- [ ] **Step 1: 写重复类别数量匹配的失败测试**

构造两个 `class_id=8` 箱子和两个 `class_id=8` 目标，期望新接口返回匹配：

```c
uint8 subject2_object_class_counts_match(
    const subject2_object_struct *boxes, uint8 box_count,
    const subject2_object_struct *targets, uint8 target_count);
```

同时验证 `2 个 class 8 箱子 / 1 个 class 8 目标` 返回不匹配，存在未识别对象也返回不匹配。

- [ ] **Step 2: 运行测试确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1`

Expected: 编译失败，提示 `subject2_object_class_counts_match` 尚未定义。

- [ ] **Step 3: 实现最小类别计数校验**

在 `subject2_logic.c` 使用两个固定数组统计类别：

```c
uint8 box_counts[SUBJECT2_CLASS_COUNT] = {0};
uint8 target_counts[SUBJECT2_CLASS_COUNT] = {0};
```

任一对象未识别、类别越界、总数为零或任一类别数量不同都返回 `0`；其余返回 `1`。

- [ ] **Step 4: 写异常类别局部失效的失败测试**

新增接口：

```c
void subject2_invalidate_mismatched_classes(
    subject2_object_struct *boxes, uint8 box_count,
    subject2_object_struct *targets, uint8 target_count,
    uint8 *need_box_scan, uint8 *need_target_scan);
```

测试 `box={8,8,3,5}`、`target={8,3,3,5}`：类别 3 和 8 的相关对象全部变为未识别；数量一致的类别 5 保持不变。每个被失效对象必须重置 `class_id` 和 `tried_observation_mask`。

- [ ] **Step 5: 运行测试确认 RED，再实现并确认 GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1`

Expected before implementation: FAIL。实现后所有逻辑测试 PASS，且 GCC 无警告。

### Task 2: 重复类别 ART 对象协调

**Files:**
- Modify: `tests/subject2_logic_test.c`
- Modify: `project/user/inc/subject2_logic.h`
- Modify: `project/user/src/subject2_logic.c`

- [ ] **Step 1: 定义对象级同步结果字段并写失败测试**

扩展 `subject2_sync_update_struct`：

```c
uint8 active_box_valid;
uint16 active_box_cell;
```

把协调接口改为对象级参数：

```c
subject2_sync_result_enum subject2_reconcile_objects(
    const map_source_struct *source,
    subject2_object_struct box_objects[MAX_BOXES], uint8 *box_count,
    subject2_object_struct target_objects[MAX_BOXES], uint8 *target_count,
    uint16 active_box_cell, uint16 active_target_cell,
    subject2_sync_update_struct *update);
```

测试场景：两个 `class 8` 箱子和两个 `class 8` 目标，其中一对消失。期望剩余箱子仍为 `class 8`，`completed_count=1`，不要求区分同类实例。

- [ ] **Step 2: 运行测试确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1`

Expected: 旧 binding 签名无法满足新调用或结果错误。

- [ ] **Step 3: 实现目标按格继承**

对每个新目标格：

```text
旧目标同格存在 -> 复制旧对象
没有旧目标同格 -> 创建 recognized=0 的新目标对象并 need_target_scan=1
```

`completed_count = old_target_count - new_target_count`；不再写 `binding.completed`。

- [ ] **Step 4: 写箱子类别缺额恢复测试**

覆盖以下用例：

```text
同类两个箱子交换位置 -> SYNC_OK，均保持同类
一个未匹配箱子且只有 class 8 缺额 -> 自动赋 class 8
两个未匹配箱子但缺额类别为 {3,8} -> SYNC_RESCAN，只失效未匹配箱子
已完成箱子旧格被剩余同类箱子占用 -> 不继承已完成对象身份
```

- [ ] **Step 5: 实现按剩余目标数量恢复箱子类别**

算法顺序固定为：

```text
统计剩余已识别目标的 required_count[class]
原格匹配箱子在不超过 required_count[class] 时继承类别
计算 assigned_count[class] 与 deficit[class]
只有一个类别存在缺额 -> 所有未匹配箱子赋该类别
多个类别存在缺额 -> 未匹配箱子标记未识别并返回 RESCAN
```

若活动箱原格仍存在，设置 `active_box_valid=1`。若活动箱原格消失但只有一个可对应的新箱子，也更新 `active_box_cell`；同类多个实例无法区分但类别已确定时不报错，后续重试可重新选择同类箱子。

- [ ] **Step 6: 运行逻辑测试确认 GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1`

Expected: 所有旧的有效同步语义和新增重复类别用例 PASS。

### Task 3: 分类与数量校验状态机

**Files:**
- Modify: `tests/subject2_scan_test.c`
- Modify: `project/user/src/subject2.c`

- [ ] **Step 1: 写第二个同类识别结果被接受的失败测试**

构造两箱两目标地图，依次给两个箱子返回同一个类别、两个目标返回同一个类别。期望：

```text
第二个同类结果发送 VISION_ACK
不进入 View Backoff
全部扫描后进入绑定校验
```

- [ ] **Step 2: 运行测试确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1`

Expected: 第二个同类结果被 `subject2_bind_box/target` 拒绝并触发回退。

- [ ] **Step 3: 分类成功直接写对象**

删除 `subject2_tick_classify()` 对 `subject2_bind_box/target()` 的调用。确认分类后直接设置当前对象的 `class_id/recognized`，保留现有 ACK、最近识别显示和回退返回流程。

- [ ] **Step 4: 用类别数量替换 binding 校验**

`subject2_tick_validate()` 改用 `subject2_object_class_counts_match()`。数量不匹配时调用 `subject2_invalidate_mismatched_classes()`，并按 `need_box_scan/need_target_scan` 进入现有局部扫描；保留 `SUBJECT2_VALIDATION_MAX_RETRIES`。

- [ ] **Step 5: 运行扫描测试确认 GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1`

Expected: 重复类别扫描通过；原唯一类别、分类超时、回退和恢复航向测试继续 PASS。

### Task 4: 动态同类配对与具体目标重试

**Files:**
- Modify: `tests/subject2_scan_test.c`
- Modify: `project/user/src/subject2.c`

- [ ] **Step 1: 写最短同类组合选择的失败测试**

使用两个 `class 8` 箱子和两个 `class 8` 目标，构造只有交叉组合或其中一个组合动作更短的地图。完成扫描后调用选择状态，验证 executor 使用最短可行组合对应的起始箱子格和目标格。

- [ ] **Step 2: 运行测试确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1`

Expected: 旧代码每类只尝试一个 binding，无法选择正确组合。

- [ ] **Step 3: 实现箱子乘目标的候选枚举**

`subject2_tick_select_push()` 双层遍历：

```c
for(box_index = 0; box_index < box_object_count; box_index++)
for(target_index = 0; target_index < target_object_count; target_index++)
```

只对已识别且类别相同的组合调用 `solve_bound_box_path()`，按 `action_count` 选择最短结果。保存：

```c
active_class
active_box_cell
active_target_cell
```

- [ ] **Step 4: 写重复类别重试目标固定的失败测试**

模拟推箱未完成并刷新 ART 地图。期望下一次规划仍只考虑 `active_target_cell`，不会切换到另一个同类目标；活动箱能够唯一跟踪时优先使用更新后的 `active_box_cell`。

- [ ] **Step 5: 实现具体任务重试**

保留现有 `retry_active_only` 标志，但筛选条件改为：

```text
target.cell == active_target_cell
box.class_id == active_class
active_box_cell 仍存在时优先且只使用该格
活动箱无法唯一跟踪时允许从同类箱子中重新选择，但目标格不变
```

任务完成或目标消失时清除重试限制。

- [ ] **Step 6: 运行扫描与 solver 测试确认 GREEN**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_solver_navigation_test.ps1
```

Expected: 重复类别最短组合与固定目标重试用例 PASS，原求解器用例不变。

### Task 5: 状态机切换到对象级 ART 同步

**Files:**
- Modify: `tests/subject2_scan_test.c`
- Modify: `project/user/src/subject2.c`

- [ ] **Step 1: 写重复类别完成与局部重扫失败测试**

覆盖：

```text
两个 class 8 中完成一个 -> 剩余 class 8 继续规划
两个同类箱子位置交换 -> 不报 E:Track
class 3 与 class 8 的未匹配箱子同时变化 -> 只进入 BScan 重识别未匹配箱子
B=0,T=0 -> 进入返航
```

- [ ] **Step 2: 运行测试确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1`

Expected: `subject2_accept_confirmed_map()` 和 `subject2_finish_scan_map_sync()` 仍依赖 binding，新增用例失败。

- [ ] **Step 3: 切换两个同步入口**

两个入口都复制临时对象数组并调用对象级 `subject2_reconcile_objects()`。只有 `SYNC_OK/RESCAN` 时提交；`RESCAN` 继续复用现有 `VSync -> CENTER_REQ -> BScan/TScan` 路径。

同步提交后：

```text
更新 box_objects/target_objects 及数量
按 sync_update 更新 active_box_cell
数量未减少 -> retry_active_only=1
活动目标消失或数量减少 -> retry_active_only=0
全部对象消失 -> 返航
```

- [ ] **Step 4: 运行状态机测试确认 GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1`

Expected: 超时宏开启和关闭两轮全部 PASS。

### Task 6: 删除旧 binding 路径并完成验证

**Files:**
- Modify: `project/user/inc/subject2_logic.h`
- Modify: `project/user/src/subject2_logic.c`
- Modify: `project/user/src/subject2.c`
- Modify: `tests/subject2_logic_test.c`
- Modify: `tests/subject2_scan_test.c`
- Modify: `docs/superpowers/specs/2026-07-15-subject2-duplicate-classes-design.md`

- [ ] **Step 1: 删除本次迁移产生的旧接口孤儿**

删除：

```text
subject2_binding_struct
subject2_bindings_clear
subject2_bind_box
subject2_bind_target
subject2_binding_sets_match
subject2_track_active_box
subject2.c 的 bindings[]
```

搜索确认无残留：

Run: `rg -n "subject2_binding|subject2_bind_|subject2_track_active_box|bindings\[" project/user tests`

Expected: 无运行时代码或测试引用。

- [ ] **Step 2: 同步规格中的最终接口名称**

检查设计文档与实际 `subject2_logic.h` 一致，不加入实现快照或临时调试字段。

- [ ] **Step 3: 运行全量 C 主机测试**

Run:

```powershell
Get-ChildItem tests -Filter "run_*.ps1" | ForEach-Object {
    powershell -ExecutionPolicy Bypass -File $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Expected: 全部 PASS。

- [ ] **Step 4: 运行 Keil 编译**

Run: `D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"`

Expected: 构建日志包含 `0 Error(s), 0 Warning(s)`。

- [ ] **Step 5: 最终静态检查**

Run: `git diff --check`

Expected: 无空白错误；修改只涉及重复类别规格范围，未改变 UART、PID、BFS 或 OpenART 脚本。
