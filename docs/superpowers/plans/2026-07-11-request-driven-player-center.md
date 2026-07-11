# ART 请求式精确中心识别 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 普通 OpenART 帧只识别 16x12 字符地图，发车、初次求解、任务重规划、推箱前和返航全部通过 `CENTER_REQ` 获取3个精确中心样本。

**Architecture:** 保留现有 `CENTER_REQ` / `CENTER_SAMPLE` 协议，由 MCU 的 `art_replan_phase` 决定样本用途。OpenART 仅在请求激活时运行 blob 粗定位和逐像素精确定位；MCU 用一个通用三样本中值收集器服务所有中心等待阶段，地图稳定与中心采样分成两个明确阶段。

**Tech Stack:** OpenART Plus MicroPython、RT1064 C99、Keil MDK、PowerShell/GCC 主机测试。

---

## 文件范围

- Modify: `openmv/main_see.py` — 将中心检测限制到 `CENTER_REQ` 活跃期间，普通地图帧固定发送无效中心。
- Modify: `project/user/src/art_replan.c` — 增加通用请求中心收集器和初始/重规划中心阶段，迁移发车及返航采样。
- Modify: `project/user/inc/openart_uart.h` — 更新协议注释，明确普通地图中心固定无效、精确中心只来自请求队列。
- Modify: `project/user/src/openart_uart.c` — 只做必要注释同步；现有请求样本解析和队列行为保持不变。
- Modify: `tests/openart_request_flow_test.py` — 覆盖普通帧不检测中心、请求时才检测以及无超时语义。
- Modify: `tests/openart_player_anchor_test.py` — 覆盖请求开始时粗 ROI 和连续3帧锚点更新。
- Create: `tests/art_replan_requested_center_test.c` — 主机验证发车、初始求解、任务重规划和返航均发送请求并无限等待。
- Create: `tests/run_art_replan_requested_center_test.ps1` — 编译运行 ART 状态机主机测试。
- Modify: `docs/superpowers/specs/2026-07-11-request-driven-player-center-design.md` — 实现后更新状态和实际函数名。

## Task 1: 锁定 OpenART 按需识别行为

**Files:**
- Modify: `tests/openart_request_flow_test.py`
- Modify: `tests/openart_player_anchor_test.py`

- [ ] **Step 1: 添加普通帧不运行中心检测的失败测试**

在 `tests/openart_request_flow_test.py` 解析 `main()` AST，确认 `detect_player_center()` 和 `detect_player_center_precise()` 只位于以 `center_request_active` 为条件的分支内，并确认普通 UART 发送固定为：

```python
send_map_uart(map_uart, char_matrix, None)
```

增加源码约束：

```python
assert "send_map_uart(map_uart, char_matrix, None)" in OPENMV_SOURCE
assert "player_center_grid = player_center_to_grid_q" not in main_body
assert "process_center_request(map_uart, precise_player_center" in main_body
```

- [ ] **Step 2: 添加请求状态重置测试**

在测试提取的全局状态中加入 `center_request_generation`。连续解析两条 `CENTER_REQ` 后应从新一轮 `CENTER_SAMPLE 1` 开始，上一轮未完成的样本不得继续编号。

```python
namespace["parse_map_uart_line"]("CENTER_REQ")
first_generation = namespace["center_request_generation"]
namespace["process_center_request"](uart, (568, 550), True, None)
namespace["parse_map_uart_line"]("CENTER_REQ")
assert namespace["center_request_generation"] == first_generation + 1
namespace["process_center_request"](uart, (570, 550), True, None)
assert uart.tx[-1] == "CENTER_SAMPLE 1,570,550\n"
```

- [ ] **Step 3: 运行测试并确认先失败**

Run:

```powershell
python tests/openart_request_flow_test.py
python tests/openart_player_anchor_test.py
```

Expected: 普通帧仍无条件调用中心检测、且 `center_request_generation` 尚不存在，因此测试失败。

## Task 2: OpenART 只在请求期间识别中心

**Files:**
- Modify: `openmv/main_see.py:925-987`
- Modify: `openmv/main_see.py:1075-1200`

- [ ] **Step 1: 为每轮请求增加代号**

在请求状态旁增加：

```python
center_request_generation = 0
```

收到请求时同时重置样本并递增代号：

```python
def parse_map_uart_line(line):
    global center_request_active
    global center_request_sample_count
    global center_request_generation

    if line == "CENTER_REQ":
        center_request_active = True
        center_request_sample_count = 0
        center_request_generation += 1
```

- [ ] **Step 2: 将中心检测放入请求分支**

在 `main()` 保存 `handled_center_request_generation` 和请求锚点。只有请求活跃时执行：

```python
precise_player_center = None
if center_request_active:
    if handled_center_request_generation != center_request_generation:
        player_center_anchor = None
        handled_center_request_generation = center_request_generation

    blob_player_center = detect_player_center(
        recognition_img, recognition_points,
        element_matrix, raw_element_matrix,
        player_center_anchor)
    precise_anchor = (blob_player_center if blob_player_center is not None
                      else player_center_anchor)
    precise_player_center = detect_player_center_precise(
        recognition_img, precise_anchor)
    if precise_player_center is not None:
        player_center_anchor = precise_player_center

process_center_request(map_uart, precise_player_center,
                       rectified_recognition_active, raw_grid_transform)
```

删除普通帧的 `player_center_history`、`last_player_center`、丢失计数和多帧中心更新；请求响应已经由 MCU 对3帧取中值，不再需要 OpenART 再做一层普通帧滤波。

- [ ] **Step 3: 普通地图帧固定发送无效中心**

替换普通发送路径：

```python
send_map_uart(map_uart, char_matrix, None)
```

调试十字只在请求活跃且本帧得到 `precise_player_center` 时绘制；调试打印普通帧固定显示 `PLAYER_CENTER_GRID 0,0 0`。

- [ ] **Step 4: 运行 OpenART 测试**

Run:

```powershell
python -m py_compile openmv/main_see.py
python tests/openart_request_flow_test.py
python tests/openart_player_anchor_test.py
python tests/openart_precise_center_test.py
```

Expected: 全部输出 `PASS`，语法检查退出码为0。

## Task 3: 建立 MCU 通用三样本请求收集器

**Files:**
- Modify: `project/user/src/art_replan.c:15-98`
- Modify: `project/user/src/art_replan.c:358-402`
- Test: `tests/art_replan_requested_center_test.c`
- Create: `tests/run_art_replan_requested_center_test.ps1`

- [ ] **Step 1: 创建状态机主机测试骨架**

测试桩提供 `time_ms()`、`openart_request_player_center()`、`openart_get_requested_center_sample()`、executor、地图和运动接口。记录请求次数、停止次数和当前状态文本。

核心队列接口：

```c
static uint16 queued_col_q[3];
static uint16 queued_row_q[3];
static uint8 queued_count;
static uint8 queued_read;

uint8 openart_get_requested_center_sample(uint16 *col_q, uint16 *row_q)
{
    if(queued_read >= queued_count)
    {
        return 0u;
    }
    *col_q = queued_col_q[queued_read];
    *row_q = queued_row_q[queued_read];
    queued_read++;
    return queued_read;
}
```

- [ ] **Step 2: 添加通用收集器测试**

验证进入任意中心等待阶段只发送一次请求；前2个样本保持等待；第3个样本后中值为第二个排序值；长时间无样本不产生超时错误。

```c
feed_center_sample(568u, 550u);
feed_center_sample(572u, 551u);
feed_center_sample(570u, 549u);
/* 期望中值 col=570,row=550。 */
```

- [ ] **Step 3: 运行测试并确认先失败**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
```

Expected: 通用请求阶段和测试接口尚不存在，编译或断言失败。

- [ ] **Step 4: 在 art_replan.c 实现通用收集器**

将现有 `art_launch_center_*` 改名为通用请求中心缓存：

```c
static uint16 art_requested_center_col_samples[ART_CENTER_SAMPLE_COUNT];
static uint16 art_requested_center_row_samples[ART_CENTER_SAMPLE_COUNT];
static uint8 art_requested_center_sample_count;
static uint16 art_requested_center_col_q;
static uint16 art_requested_center_row_q;
static uint8 art_requested_center_valid;
```

提供内部函数：

```c
static void art_requested_center_clear(void);
static void art_replan_begin_center_request(art_replan_phase_enum phase,
                                            const char *state,
                                            art_replan_update_struct *update);
static uint8 art_replan_collect_requested_center(void);
```

`art_replan_begin_center_request()` 必须先停车、清缓存、设置 phase，再调用一次 `openart_request_player_center()`；tick 期间不得重复发送请求。

- [ ] **Step 5: 运行通用收集器测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
```

Expected: 请求次数、中值和无限等待用例通过。

## Task 4: 迁移发车和返航到请求队列

**Files:**
- Modify: `project/user/src/art_replan.c:512-610`
- Modify: `project/user/src/art_replan.c:684-786`
- Test: `tests/art_replan_requested_center_test.c`

- [ ] **Step 1: 添加发车与返航失败测试**

发车5秒结束后应进入 `WCTR` 并发送一次请求；没有样本时即使推进时间超过 `EXEC_ART_SYNC_TIMEOUT_MS` 仍保持 `WCTR`。返航 `RetCtr` 和 `RetChk` 同样无限等待且不进入 `EXEC_ERROR_ART_TIMEOUT`。

- [ ] **Step 2: 改造发车中心阶段**

`art_replan_begin_wait_center()` 改为调用：

```c
art_replan_begin_center_request(ART_REPLAN_WAIT_CENTER, "WCTR", update);
```

`art_replan_tick_wait_center()` 只读取通用请求中值并计算：

```c
current_x_cm = art_grid_q_to_x_cm(art_requested_center_col_q);
move_cm = ART_LAUNCH_TARGET_X_CM - current_x_cm;
```

删除该函数中的 `openart_get_player_center()` 和中心等待超时判断。保存 home 时使用通用请求中值。

- [ ] **Step 3: 改造返航中心阶段**

`art_replan_begin_return_center()` 使用通用请求入口。`art_replan_tick_return_center()` 使用通用中值完成 Y/X 偏移和 home 容差判断，并删除中心采样超时分支。

返航轴向移动超时仍保留；用户只取消“中心采样超时”，不取消运动卡死保护。

- [ ] **Step 4: 运行状态机测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
```

Expected: `WCTR`、`RetCtr`、`RetChk` 均只请求一次并无限等待，轴向移动超时测试仍通过。

## Task 5: 初次求解和任务重规划增加中心阶段

**Files:**
- Modify: `project/user/src/art_replan.c:15-29`
- Modify: `project/user/src/art_replan.c:105-149`
- Modify: `project/user/src/art_replan.c:926-1070`
- Modify: `project/user/src/art_replan.c:1118-1242`
- Test: `tests/art_replan_requested_center_test.c`

- [ ] **Step 1: 添加初始/重规划顺序测试**

断言顺序必须为：

```text
稳定有效地图
-> 冻结 snapshot
-> ICtr 或 RCtr
-> CENTER_REQ
-> 3个样本中值
-> 计算相对 C 格 offset
-> solve_map
-> executor_start
```

2个样本时 `solve_map()` 和 `executor_start()` 调用次数必须仍为0。`Box OK` 与 `Push Retry` 都进入 `RCtr`。

- [ ] **Step 2: 增加明确 phase**

在枚举中增加：

```c
ART_REPLAN_INITIAL_CENTER,
ART_REPLAN_SEGMENT_CENTER,
```

并保存中心完成后要恢复的求解来源语义：初次求解使用 `INITIAL_CENTER`，任务结束重规划及最后箱子消失返航使用 `SEGMENT_CENTER`。

- [ ] **Step 3: 让 pose offset 接收请求中值**

将：

```c
art_replan_get_pose_offset(const map_scan_stats_struct *stats, ...)
```

改为：

```c
art_replan_calculate_pose_offset(const map_scan_stats_struct *stats,
                                 uint16 center_col_q,
                                 uint16 center_row_q,
                                 float *offset_x_cm,
                                 float *offset_y_cm)
```

保留地图范围和 C 八邻域检查，删除 `openart_get_player_center()`、普通中心帧号和 valid 判断。

- [ ] **Step 4: 拆分稳定地图确认和求解执行**

`art_handle_stable_map()` 在地图合法、B/T 语义处理完成后只保存 snapshot，并进入：

```c
art_replan_begin_center_request(ART_REPLAN_INITIAL_CENTER, "ICtr", update);
```

或：

```c
art_replan_begin_center_request(ART_REPLAN_SEGMENT_CENTER, "RCtr", update);
```

新增 `art_replan_solve_snapshot_after_center()`：重新扫描冻结 snapshot，使用请求中值算 offset，再调用 `solve_map()` 和带 offset 的 `executor_start()`。最后箱子已消失时，用同一 offset 启动返航 BFS。

- [ ] **Step 5: tick 中处理两个新阶段**

两个阶段都调用通用收集器；不足3帧时只更新 `ICtr/RCtr` 并返回，不检查 `EXEC_ART_SYNC_TIMEOUT_MS`。收满后调用 `art_replan_solve_snapshot_after_center()`。

- [ ] **Step 6: 运行状态机测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
```

Expected: 初始、Box OK、Push Retry 和最后箱子返航用例全部按“地图后请求中心、中心后求解”顺序通过。

## Task 6: 推箱前流程接入通用收集器并清理普通中心依赖

**Files:**
- Modify: `project/user/src/art_replan.c:164-200`
- Modify: `project/user/src/art_replan.c:358-510`
- Modify: `project/user/src/art_replan.c:1127-1158`
- Modify: `project/user/inc/openart_uart.h:74-103`
- Modify: `project/user/src/openart_uart.c:147-176`
- Test: `tests/openart_request_flow_test.py`
- Test: `tests/executor_pre_push_test.c`

- [ ] **Step 1: 保持推箱前行为但改用统一缓存**

`ART_REPLAN_PRE_PUSH_CENTER` 进入时调用通用请求入口。收满后将3个缓存样本依次交给现有 `executor_apply_art_player_center()`，再调用 `executor_commit_art_player_center()`；这样不改变当前90%融合、参考 C/waypoint 回退和 `E:Ctr` 语义。

- [ ] **Step 2: 删除 art_replan 对普通中心 getter 的全部调用**

Run:

```powershell
rg -n "openart_get_player_center" project/user/src/art_replan.c
```

Expected: 无结果。

`art_replan_collect_player_center()`、`art_last_player_center_sample` 和 `art_center_sampling_was_active` 随之删除。`EXEC_ART_CENTER_CORRECT_ENABLE` 保留配置但注释更新为当前普通帧无中心，因此关闭状态是唯一受支持的运行配置；本次不扩展普通 waypoint 请求。

- [ ] **Step 3: 同步 UART 注释**

说明地图帧中的 `PLAYER_CENTER_GRID` 为协议占位且固定 `valid=0`；精确中心只能通过 `openart_request_player_center()` 和 `openart_get_requested_center_sample()` 获取。保留解析兼容代码，不扩大协议改动。

- [ ] **Step 4: 运行推箱前与协议回归测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
python tests/openart_request_flow_test.py
```

Expected: 请求顺序、首推等待、连续推箱和最终任务 ART 同步全部通过。

## Task 7: 文档、静态检查与完整验证

**Files:**
- Modify: `docs/superpowers/specs/2026-07-11-request-driven-player-center-design.md`
- Verify: `openmv/main_see.py`
- Verify: `project/user/`
- Verify: `tests/`

- [ ] **Step 1: 更新设计文档为实际实现**

将文档标记为“已实现”，记录最终 phase 名称和普通帧固定无效中心行为。不得重新引入普通帧中心开关，因为本次需求已经确定普通运行不识别中心。

- [ ] **Step 2: 静态搜索普通中心依赖**

Run:

```powershell
rg -n "openart_get_player_center|player_center_history|last_player_center|player_center_lost_frames" project/user openmv/main_see.py
```

Expected: `art_replan.c` 和 OpenART 主循环无普通中心依赖；若 UART 兼容 getter 被保留，只允许出现在声明/定义中，不得有业务调用。

- [ ] **Step 3: 运行全部主机和 Python 测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_solver_navigation_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
python -m py_compile openmv/main_see.py
python tests/openart_precise_center_test.py
python tests/openart_request_flow_test.py
python tests/openart_player_anchor_test.py
```

Expected: 所有用例 `PASS`，Python 语法检查退出码为0。

- [ ] **Step 4: Keil 编译**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

随后检查：

```powershell
Get-Content project\mdk\Objects\rt1064.build_log.htm |
    Select-String "Error\(s\)|Warning\(s\)"
```

Expected: `0 Error(s), 0 Warning(s)`。

- [ ] **Step 5: 上板状态验证**

按以下顺序观察：

```text
普通地图运行：LOOP_FPS 提升，PLAYER_CENTER_GRID 0,0 0
发车：W5S -> WCTR -> 收满3帧 -> LCH
初次地图：WMAP -> ICtr -> Running
推箱前：PCtr -> Running
任务结束：ART Sync -> RCtr -> Running/RetGrid
返航：RetCtr -> RetY/RetX -> RetChk -> Done
```

在任一中心等待阶段遮挡车标，车辆必须持续停车且不自动超时；恢复识别后收满3帧才继续。K4 必须能立即停止并退出。
