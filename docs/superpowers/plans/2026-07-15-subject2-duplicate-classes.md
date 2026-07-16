# 科目二重复类别支持实施计划

## 目标

让科目二支持多个同类卡通箱子和多个同数字目标。同类箱子可与任意同类目标配对；RT1064 不为同类对象建立永久实例编号，而是以当前 ART 地图、对象所在格和类别数量为准进行扫描、规划、同步和重试。

## 当前仓库事实

- `subject2_object_struct` 已能为每个箱子或目标独立保存 `cell/class_id/recognized`。
- 当前 `bindings[SUBJECT2_CLASS_COUNT]` 每类只能保存一个箱子格和一个目标格，是重复类别失败的直接原因。
- `subject2_tick_classify()` 会调用 `subject2_bind_box/target()`；第二个同类结果因此被拒绝。
- `subject2_tick_select_push()` 当前每类只尝试一组固定 `box_cell -> target_cell`。
- `subject2_reconcile_objects()` 当前依赖单实例 binding 恢复对象身份。
- 当前求解器是逐箱贪心，不是多箱全局最优搜索；本次保持该架构边界。

## 最终语义

```text
识别阶段：
  每个对象独立保存类别，允许重复

绑定校验：
  所有对象都已识别
  每个类别的箱子数量 == 目标数量

选择推箱：
  枚举同类别 box x target
  对每组调用现有 solve_bound_box_path()
  选择当前动作数最少的可行组合

ART 同步：
  目标格消失是上位机完成任务的权威事实
  由剩余目标的类别数量恢复剩余箱子类别

重试：
  固定 active_target_cell
  active_box_cell 可唯一追踪时固定箱子
  无法唯一追踪时允许从同类箱子中重新选择
```

同类实例本身不需要区分。两个同类箱子位置互换时，只要求类别数量正确且流程不中断，不要求恢复原来的物理实例身份。

## 不变范围

- 不修改 OpenART UART 文本协议。
- 不修改 OpenART 脚本、分类模型和置信度规则。
- 不修改 BFS、20cm 格距、PID、中心校正、发车和返航逻辑。
- 不引入完整多箱 Sokoban 全局搜索。
- 不增加屏幕调试字段。

## Task 1：类别数量校验

**文件**

- `tests/subject2_logic_test.c`
- `project/user/inc/subject2_logic.h`
- `project/user/src/subject2_logic.c`

1. 先写失败测试：
   - 两个 class 8 箱子和两个 class 8 目标返回匹配。
   - `box={8,8}`、`target={8}` 返回不匹配。
   - 任一对象 `recognized=0` 返回不匹配。
   - 任一类别越界返回不匹配。
   - 空对象集合返回不匹配。
2. 新增并实现：

```c
uint8 subject2_object_class_counts_match(
    const subject2_object_struct *boxes,
    uint8 box_count,
    const subject2_object_struct *targets,
    uint8 target_count);
```

3. 新增异常类别局部失效接口及测试：

```c
void subject2_invalidate_mismatched_classes(
    subject2_object_struct *boxes,
    uint8 box_count,
    subject2_object_struct *targets,
    uint8 target_count,
    uint8 *need_box_scan,
    uint8 *need_target_scan);
```

对箱子数与目标数不同的类别，两侧该类别的已识别对象全部重置为：

```text
recognized=0
class_id=SUBJECT2_INVALID_CLASS
tried_observation_mask=0
```

数量一致的其他类别保持不变。已有未识别对象只设置对应扫描标志，不得被误判为匹配。

4. 运行 `tests/run_subject2_logic_test.ps1`，确认先 RED、实现后 GREEN。

## Task 2：扫描状态机接受重复类别

**文件**

- `tests/subject2_scan_test.c`
- `project/user/src/subject2.c`

1. 先写失败测试：两箱两目标均识别为同一类别，第二个同类结果必须 ACK，不进入 View Backoff，扫描最终进入绑定校验。
2. `subject2_tick_classify()` 分类成功后直接写当前 `box_objects[]` 或 `target_objects[]`，不调用单实例 bind API。
3. `subject2_tick_validate()` 改用对象类别数量校验。
4. 数量不匹配时调用局部失效接口，并复用现有 BScan/TScan 路径重新识别；保留当前一次校验重试边界，第二次仍不一致才报 `E:BSet`。
5. 回归分类超时、回退观察、恢复发车航向和唯一类别场景。

## Task 3：动态同类配对

**文件**

- `tests/subject2_logic_test.c`
- `tests/subject2_scan_test.c`
- `project/user/inc/subject2_logic.h`
- `project/user/src/subject2_logic.c`
- `project/user/src/subject2.c`

1. 先写失败测试，覆盖：
   - 两个 class 8 箱子与两个 class 8 目标选择当前最短可行组合。
   - 不可达组合被跳过。
   - 动作数相同时按对象数组顺序稳定选择，保证结果可复现。
2. 新增纯逻辑选路接口，枚举所有已识别且类别相同的 `box x target` 组合，内部复用 `solve_bound_box_path()`。
3. 状态机保存具体活动任务：

```c
active_class
active_box_cell
active_target_cell
active_box_valid
```

4. 普通选择从所有同类组合中取当前最短路径。
5. 重试时始终限定 `target.cell == active_target_cell`：
   - `active_box_valid=1` 时只尝试当前活动箱格。
   - `active_box_valid=0` 时允许从同类别剩余箱子重新选择。
6. 明确限制：当前最短配对仍是逐箱贪心，可能不是所有箱子的全局最优组合；每个任务结束后根据 ART 地图重新规划，不在本次增加全局匹配搜索。

## Task 4：对象级 ART 同步

**文件**

- `tests/subject2_logic_test.c`
- `project/user/inc/subject2_logic.h`
- `project/user/src/subject2_logic.c`

### 4.1 原子提交

同步函数只操作临时对象数组。只有结果为 `SYNC_OK` 或 `SYNC_RESCAN` 时，`subject2.c` 才提交新数组；`SYNC_AMBIGUOUS` 不得发布部分修改。

### 4.2 目标处理

目标在任务过程中不会移动，只会被上位机消除：

```text
新目标格存在于旧目标列表 -> 继承原类别
新目标格不在旧目标列表 -> 地图矛盾，SYNC_AMBIGUOUS
旧目标格消失 -> 以上位机为准，计为对应类别完成一个
```

由剩余目标计算：

```c
required_count[class_id]
```

这组数量是恢复剩余箱子类别的权威配额。

### 4.3 箱子处理

按以下固定顺序处理：

1. 新旧箱子同格且旧对象已识别时，只有该类别尚未超过 `required_count[class]` 才继承类别。
2. 统计已分配数量 `assigned_count[class]`。
3. 计算 `deficit[class] = required_count[class] - assigned_count[class]`。
4. 若只剩一个类别有缺额，且缺额总数等于未匹配箱子数，则把所有未匹配箱子恢复为该类别。
5. 若多个类别同时有缺额，无法从几何状态判断实例类别，只把未匹配箱子标为未识别并返回 `SYNC_RESCAN`。
6. 不得因为同类实例身份无法区分而报错；只要类别配额唯一即可继续。

### 4.4 活动任务跟踪

- `active_target_cell` 已消失：当前或其他任务已经被上位机完成，清除重试限制并基于最新 ART 地图重新规划。
- `active_target_cell` 仍存在且 B/T 总数未减少：进入 Push Retry，目标格保持不变。
- 活动箱旧格仍有箱子且类别配额允许：`active_box_valid=1`。
- 活动箱旧格消失，但只有一个未匹配箱子可对应活动类别：更新 `active_box_cell` 并保持有效。
- 无法唯一确定活动箱时：`active_box_valid=0`，不报错；重试时重新选择同类箱子。
- 任意非活动目标意外消失也以上位机为准，不要求重新识别已经能够由类别配额唯一恢复的对象。

### 4.5 必测场景

- 两个 class 8 中完成一个，剩余 class 8 继续规划。
- 两个同类箱子位置变化，不因实例不可区分报错。
- 一个未匹配箱子且只有 class 8 缺额，自动恢复 class 8。
- 两个未匹配箱子且缺额类别为 class 3 和 class 8，只重扫未匹配箱子。
- 活动目标未消失且活动箱唯一移动，更新 `active_box_cell`。
- 活动目标已消失，清除 Push Retry。
- 非活动目标意外消失，接受 ART 结果并重规划。
- `B=0,T=0` 正常进入返航。

## Task 5：状态机接入对象级同步

**文件**

- `tests/subject2_scan_test.c`
- `project/user/src/subject2.c`

1. 先写端到端失败测试：重复类别完成、Push Retry 固定目标、非活动目标意外消失、局部箱子重扫和全部完成返航。
2. `subject2_accept_confirmed_map()` 与 `subject2_finish_scan_map_sync()` 都调用同一个对象级同步接口。
3. ART 地图稳定且 B/T 合法后，目标消失优先于 MCU 旧任务判断；不得因为旧 `active_class` 不一致拒绝上位机结果。
4. `SYNC_RESCAN` 复用现有 `VSync -> CENTER_REQ -> BScan/TScan`，只扫描失效对象。
5. 初始中心、段末中心、推箱前中心和恢复航向流程保持不变。

## Task 6：删除旧单实例 binding 路径

对象级扫描、选路和同步测试全部通过后再删除：

```text
subject2_binding_struct
subject2_bindings_clear
subject2_bind_box
subject2_bind_target
subject2_binding_sets_match
subject2_track_active_box
subject2.c 中的 bindings[]
```

搜索确认无运行时代码或测试残留：

```powershell
rg -n "subject2_binding|subject2_bind_|subject2_track_active_box|bindings\[" project/user tests
```

不要在迁移中途让新对象逻辑与旧 binding 校验同时生效。

## 验证

每个任务均遵循“先失败测试，再最小实现，再回归测试”。最终执行：

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_solver_navigation_test.ps1

Get-ChildItem tests -Filter "run_*.ps1" | ForEach-Object {
    powershell -ExecutionPolicy Bypass -File $_.FullName
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
git diff --check
```

验收标准：

- 重复类别扫描不会因第二个同类结果进入回退。
- 每类别箱子数量必须与目标数量一致，未识别对象不能通过校验。
- 同类对象选择当前最短可行组合。
- ART 目标消失始终作为完成事实被接受。
- 重试固定具体目标，不依赖永久箱子实例编号。
- 多类别无法恢复时只局部重扫，不清空全部绑定结果。
- 全量主机测试 PASS。
- Keil `0 Error(s), 0 Warning(s)`。
- 不改变 UART、OpenART、PID、BFS、发车和返航行为。
