# ART1 箱子中心与车头方向辅助观察 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 科目二观察黄色箱子时，用 ART1 的小车中心和箱子中心修正 MCU pose 与实际观察位置，并同步获得车头四方向；本版保持固定 yaw。

**Architecture:** ART1 只在调试开关开启或收到指定箱子请求时运行局部 ROI 识别，通过独立 UART1 观察样本协议返回三帧数据。MCU 对中心取中值，将小车格内 offset 写入 pose，并让 executor 用现有20ms位置环移动到箱子格内 offset 对应的真实观察点，复测通过后才请求 ART2 分类。

**Tech Stack:** OpenART Plus MicroPython、RT1064 C99、逐飞 UART、现有 executor/path PID、Python/PowerShell 主机测试、Keil MDK。

---

## 文件结构

- 修改 `openmv/main_see.py`：箱子局部中心、绿色/青色车身几何、四方向、调试开关和观察请求协议。
- 修改 `project/user/src/openart_uart.c` 与对应头文件：发送观察请求并解析三帧观察样本。
- 修改 `project/user/src/executor.c` 与对应头文件：增加任意 X/Y offset 的短距离位置环入口。
- 修改 `project/user/src/subject2.c` 与对应头文件：箱子扫描接入观察、微调和复测状态；目标扫描保持原流程。
- 扩展现有 OpenART/subject2/executor 主机测试，不新增屏幕页面。

### Task 1: ART1 箱子中心、车身方向与调试开关

**Files:**
- Modify: `openmv/main_see.py`
- Create: `tests/openart_observation_test.py`

- [ ] **Step 1: 编写纯函数与协议失败测试**

测试通过 AST 提取纯函数，覆盖：绿色中心到青色中心分别得到 `U/D/L/R`；对角差过小得到 `?`；`OBSERVE_REQ 5,8` 能保存指定格并清零样本号；第3个有效样本后请求结束；无请求时不发送 UART 数据。

期望协议固定为：

```text
OBSERVE_REQ row,col
OBSERVE_SAMPLE index,car_col_q,car_row_q,box_col_q,box_row_q,heading
```

Run:

```powershell
python tests/openart_observation_test.py
```

Expected: 新函数和协议尚不存在，测试失败。

- [ ] **Step 2: 复用现有车身连通域输出几何信息**

把现有只返回中点的内部检测结果扩展为：

```python
(player_center, green_center, cyan_center, heading)
```

`heading` 使用绿色中心到青色中心的向量，青色半块代表车头；主轴明显时输出 `U/D/L/R`，横纵差不足以稳定区分时输出 `?`。保留原精确中心直方图算法，不新增第三种蓝色搜索。

- [ ] **Step 3: 增加指定 B 格黄色中心识别**

新增局部函数接收 `row,col` 和当前识别坐标系，只在该格内部 ROI 调用 `find_blobs()`。候选连通域必须由现有 `is_box_color()` 对中心及邻域复核，取像素数最大的合法 blob 质心，再转换为 `box_col_q/box_row_q`；结果必须仍落在请求格内。

- [ ] **Step 4: 增加观察请求和3帧发送**

`parse_map_uart_line()` 接受合法范围内的 `OBSERVE_REQ row,col`；新请求清除车身锚点、指定格和样本计数。只有同一帧同时得到小车中心、箱子中心时才发送样本；heading 可为 `?`，不影响中心样本计数。

- [ ] **Step 5: 增加独立调试开关**

在 `USER_SWITCHES` 增加：

```python
DEBUG_OBSERVATION_ENABLE = False
```

开启时遍历稳定字符地图中的全部 `B` 格，绘制黄色中心十字和绿色中心到青色中心的方向线，并按 `DEBUG_PRINT_PERIOD_MS` 打印：

```text
BOX_CENTER_GRID row=5 col=8 center=846,552
PLAYER_HEADING R valid=1
```

该开关不发送 `OBSERVE_SAMPLE`；关闭且无请求时不运行额外箱子 ROI 识别。

- [ ] **Step 6: 验证 ART1**

Run:

```powershell
python tests/openart_observation_test.py
python tests/openart_request_flow_test.py
python -m py_compile openmv/main_see.py
```

Expected: 全部通过，现有 `CENTER_REQ/CENTER_SAMPLE` 行为不变。

### Task 2: MCU UART1 观察请求与样本解析

**Files:**
- Modify: `project/user/inc/openart_uart.h`
- Modify: `project/user/src/openart_uart.c`
- Modify: `tests/openart_center_request_test.c`

- [ ] **Step 1: 增加失败测试**

覆盖发送 `OBSERVE_REQ 5,8\n`、严格接收顺序 `1/2/3`、忽略重复/乱序/越界坐标、接受 `U/D/L/R/?`、新请求清除旧样本，并确认普通中心请求队列互不串用。

- [ ] **Step 2: 定义最小公开接口**

```c
typedef struct
{
    uint16 car_col_q;
    uint16 car_row_q;
    uint16 box_col_q;
    uint16 box_row_q;
    char heading;
} openart_observation_sample_struct;

void openart_request_observation(uint8 box_row, uint8 box_col);
uint8 openart_get_observation_sample(openart_observation_sample_struct *sample);
```

- [ ] **Step 3: 实现独立3样本队列**

请求函数清空旧观察队列并使用固定缓冲拼出命令，不使用 `printf`。解析器只在当前请求活跃时接受样本，验证 `index=上次+1`、四个 q 坐标在 `16×12` 范围内、heading 属于 `UDLR?`；第3帧后结束请求。

- [ ] **Step 4: 运行协议回归**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
```

Expected: 原中心请求与新增观察请求全部 PASS。

### Task 3: executor 任意 offset 位置微调

**Files:**
- Modify: `project/user/inc/executor.h`
- Modify: `project/user/src/executor.c`
- Create: `tests/executor_position_correction_test.c`
- Create: `tests/run_executor_position_correction_test.ps1`

- [ ] **Step 1: 添加失败测试**

构造当前 pose `(2,-1)`、目标 `(4,3)`，验证 correction 模式产生正确方向的 `set_motion()`；连续满足 `PATH_ARRIVAL_THRESHOLD_CM` 和 `EXEC_ARRIVAL_STABLE_TICKS` 后停止并进入 `EXEC_STATE_DONE`；`executor_stop()` 和错误状态立即取消 correction。

- [ ] **Step 2: 增加公开入口**

```c
uint8 executor_start_position_correction(float target_x_cm,
                                         float target_y_cm);
```

入口只接受 executor 非运行状态，重置现有 X/Y path PID 和到点计数，保存绝对 pose 目标并进入 correction 子模式；不重置 pose、不改 yaw target。

- [ ] **Step 3: 在20ms executor 更新中复用现有位置环**

correction 子模式优先于 waypoint 读取，调用现有 `move_to_target()`、到点判定和稳定计数。完成时 `stop_motion()`、清除子模式并置 `DONE`。普通 waypoint、Step、ART push 和离线流程保持原分支。

- [ ] **Step 4: 运行 executor 回归**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_executor_position_correction_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
```

Expected: 新增 correction 案例与原推箱前暂停/继续案例全部 PASS。

### Task 4: 科目二箱子观察修正状态机

**Files:**
- Modify: `project/user/inc/subject2.h`
- Modify: `project/user/src/subject2.c`
- Modify: `tests/subject2_scan_test.c`

- [ ] **Step 1: 增加状态机失败测试**

覆盖：箱子导航到位后发 `OBSERVE_REQ`；目标导航仍发原 `CENTER_REQ`；3帧中心取中值；车头不一致不阻止；箱子中心偏移时启动 correction；复测通过后才发 ART2 分类请求；第一次复测不通过只再修一次；第二次失败或 `EXEC_ART_SYNC_TIMEOUT_MS` 超时进入中心错误并停车。

- [ ] **Step 2: 增加箱子观察状态**

在现有枚举中加入：

```c
SUBJECT2_SCAN_BOX_OBSERVE,
SUBJECT2_SCAN_BOX_ADJUST,
SUBJECT2_SCAN_BOX_VERIFY
```

屏幕短状态固定为 `VObs`、`VAdj`、`VChk`。不新增首页或 Execute 页字段；失败继续使用中心类错误码，状态文字为 `E:BoxCtr`。

- [ ] **Step 3: 计算 pose 与观察目标**

三帧分别对 `car_col_q/car_row_q/box_col_q/box_row_q` 取中值。要求最新地图的 `C` 位于 `current_observation.row/col`、请求格仍为 `B`、箱子中心仍在指定格。

计算：

```c
car_offset_x_cm = (car_col_q - (C_col * 100 + 50)) * GRID_SIZE_CM / 100;
car_offset_y_cm = -(car_row_q - (C_row * 100 + 50)) * GRID_SIZE_CM / 100;
box_offset_x_cm = (box_col_q - (B_col * 100 + 50)) * GRID_SIZE_CM / 100;
box_offset_y_cm = -(box_row_q - (B_row * 100 + 50)) * GRID_SIZE_CM / 100;
```

先 `drive_pose_reset(car_offset_x_cm, car_offset_y_cm, yaw)`，再以 `(box_offset_x_cm, box_offset_y_cm)` 启动 correction。因为观察格与箱子格名义中心相差恰好一格，二者应具有相同格内 offset。

- [ ] **Step 4: 实现一次微调和一次复测**

初次误差已在到点阈值内时跳过运动直接复测；否则等待 correction `DONE` 后重新请求。复测误差仍超限时只允许第二次 correction；再次复测仍失败则停车 `E:BoxCtr`。每轮请求从发送命令起用 `EXEC_ART_SYNC_TIMEOUT_MS` 计时。

- [ ] **Step 5: 保持固定车头语义**

记录三帧一致的 `heading`，不一致记 `?`。本版不调用 `set_target_yaw()`，不要求 heading 朝向箱子，也不以 heading 决定分类；只为 ART1 调试和未来主动旋转保留数据通路。

- [ ] **Step 6: 运行科目二回归**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
python tests/art2_protocol_test.py
```

Expected: 箱子观察走新流程，目标扫描、绑定、推箱确认和 ART2 协议原行为全部通过。

### Task 5: 集成验证

- [ ] **Step 1: 运行全部相关主机测试与差异检查**

```powershell
python tests/openart_observation_test.py
python tests/openart_request_flow_test.py
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_position_correction_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_vision_uart_test.ps1
git diff --check
```

- [ ] **Step 2: Keil 完整编译**

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: `0 Error(s), 0 Warning(s)`。

- [ ] **Step 3: ART1 独立调试**

临时设置 `DEBUG_OBSERVATION_ENABLE=True`，验证所有 `B` 格中心十字、绿色到青色方向线和每秒打印；验证四方向；完成后恢复 `False`。

- [ ] **Step 4: 上板联调**

让箱子图形偏离格中心约 `2~5cm`，确认屏幕依次显示 `VObs -> VAdj -> VChk -> VWait`，小车微调到实际观察位置后 ART2 才分类。遮挡箱子中心时应在超时后停车显示 `E:BoxCtr`；目标扫描与现有流程一致。
