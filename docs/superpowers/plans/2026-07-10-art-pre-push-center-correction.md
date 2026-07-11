# ART Pre-Push Center Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 ART 模式每个大写推箱 waypoint 开始前停车采集三帧中心，以 ART 90% 比例修正 pose；失败时保持当前 waypoint 并停车报错。

**Architecture:** executor 在 20ms 周期发现尚未校正的大写 waypoint 后只置位等待标志并保持停车。主循环 `art_replan` 接管三帧完整地图配套中心的采集、校验和提交，成功后放行同一个 waypoint，失败或超时进入独立中心错误状态。

**Tech Stack:** C99、RT1064/Keil MDK、PowerShell + GCC 主机测试。

---

### Task 1: Executor 推箱前等待行为测试

**Files:**
- Create: `tests/executor_pre_push_test.c`
- Create: `tests/run_executor_pre_push_test.ps1`
- Modify: `tests/solver_navigation_test.c`
- Modify: `project/user/inc/executor.h`
- Modify: `project/user/src/executor.c`
- Modify: `project/user/src/solver.c`

- [ ] **Step 1: 编写失败测试**

使用真实 `executor.c` 和 `motion_math.c`，在测试文件中提供 drive pose/底盘输出桩。验证：ART 模式大写 waypoint 首周期进入等待；放行后才输出运动；下一个大写 waypoint 再次等待；小写和离线大写不等待；错误后保留 waypoint。

在 solver 主机测试中构造连续两次右推的地图，要求 `RR` 输出两个独立大写 waypoint，避免连续推箱在求解结果中提前合并。

- [ ] **Step 2: 运行测试确认失败**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
```

预期：因 `executor_art_pre_push_pending()` 等接口尚不存在而编译失败。

- [ ] **Step 3: 实现最小 executor 状态机**

增加 `pre_push_center_waiting` 和 `pre_push_center_confirmed`。`executor_update_20ms()` 在段间等待处理之后、实际位置环运动之前拦截 ART 模式的大写 waypoint。新增：

```c
uint8 executor_art_pre_push_pending(void);
uint8 executor_continue_after_pre_push_center(void);
```

进入等待时清空三帧中心历史；成功放行只标记当前 waypoint 已校正，不推进 `current_step`。

修改 `solver.c` 的 waypoint 合并条件：任一动作是大写推箱时不合并；连续同方向小写移动仍合并。

- [ ] **Step 4: 运行测试确认状态机通过**

运行同一脚本，预期全部 executor 行为用例 `PASS`。

### Task 2: ART 三帧中心处理与失败停车

**Files:**
- Modify: `project/user/inc/drive_config.h`
- Modify: `project/user/inc/executor.h`
- Modify: `project/user/src/executor.c`
- Modify: `project/user/src/art_replan.c`
- Modify: `project/user/src/screen.c`

- [ ] **Step 1: 增加独立配置和错误码**

```c
#define EXEC_ART_PRE_PUSH_CENTER_CORRECT_ENABLE (1)
#define EXEC_ART_CENTER_FUSE_ALPHA (0.90f)
```

增加 `EXEC_ERROR_ART_CENTER`，屏幕错误文本映射为 `E:Ctr`。

- [ ] **Step 2: 让中心提交函数按调用方决定是否启用**

保留 `EXEC_ART_CENTER_CORRECT_ENABLE` 对原段末校正的控制；推箱前流程由新开关调用同一个三帧中值与 pose 融合实现。`IGNORED` 和 `APPLIED` 视为成功，`NONE/REJECTED/ABNORMAL` 视为失败。

- [ ] **Step 3: 增加 PRE_PUSH_CENTER 主循环阶段**

当 `art_replan` 空闲且 executor 报告推箱前等待时，丢弃旧半帧并从下一张完整地图开始计数。收到三个新有效配套中心后，对最新地图执行 `map_scan_stats()`，要求唯一 `C`，然后提交中心：

```text
APPLIED/IGNORED -> 放行当前 waypoint，状态 Running
其他结果       -> EXEC_ERROR_ART_CENTER，状态 E:Ctr
等待超时       -> EXEC_ERROR_ART_CENTER，状态 E:Ctr
```

该阶段不调用 `solve_map()`，不更新 confirmed B/T。

- [ ] **Step 4: 扩充主机测试验证 90% 融合**

给三帧中值构造 4cm 的视觉偏差，验证提交后 pose 修正为约 3.6cm；重新运行测试脚本预期全部 `PASS`。

### Task 3: 回归验证

**Files:**
- Verify: `project/user/inc/drive_config.h`
- Verify: `project/user/src/executor.c`
- Verify: `project/user/src/art_replan.c`
- Verify: `project/user/src/screen.c`

- [ ] **Step 1: 运行主机测试**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_solver_navigation_test.ps1
```

预期：全部用例 `PASS`，临时 exe 被删除。

- [ ] **Step 2: 静态检查触发边界**

确认只有 `art_sync_enabled && action_is_push()` 触发推箱前等待；普通小写、返航 `art_sync=0` 和离线执行不触发。确认 PRE_PUSH_CENTER 分支没有 `solve_map()` 或 confirmed B/T 更新。

- [ ] **Step 3: Keil 编译**

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

预期：`0 Error(s), 0 Warning(s)`。

- [ ] **Step 4: 补丁检查**

```powershell
git diff --check
git status --short
```

确认没有测试 `.exe` 留在仓库，并且没有改动 BFS、PID、里程计、发车或返航路径逻辑。
