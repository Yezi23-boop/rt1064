# 科目二分类推箱 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不破坏已完赛科目一的前提下，增加双 OpenART 分类识别、随机目标扫描、唯一类别绑定、指定箱子到指定目标求解，以及科目一返航后自动进入科目二的完整流程。

**Architecture:** OpenART #1 和现有 `art_replan` 继续负责地图、精确中心、发车和返航；新建 UART4 分类协议、科目二纯逻辑和科目二状态机。科目二每次只执行一个已绑定箱子任务，任务结束后由 MCU 比较最新 ART 地图并维护箱子类别身份。

**Tech Stack:** RT1064 C99、逐飞 UART/PIT 驱动、OpenART Plus MicroPython `sensor/tf/machine.UART`、PowerShell + GCC 主机测试、Keil MDK。

---

## 实施阶段与验收门

1. **阶段 A：ART2 通信与模型**：UART4 请求、双模型加载和三样本传输独立可测，不接运动。
2. **阶段 B：求解与分类纯逻辑**：指定配对 BFS、观察格选择、绑定校验、身份跟踪全部通过主机测试。
3. **阶段 C：科目二扫描**：能够从科目二地图依次扫描全部 `B/T`，但尚不推箱。
4. **阶段 D：科目二推箱**：分类表一致后执行指定配对任务，并正确处理 `Box OK` 与 `Push Retry`。
5. **阶段 E：FULL 全流程**：科目一返航后无需第二次 K3，自动发车进入科目二并最终返航。

每个阶段通过对应主机测试和 Keil 编译后再进入下一阶段。

## 文件结构

### 新建

- `project/user/inc/vision_uart.h`：ART2 UART4 协议公开接口。
- `project/user/src/vision_uart.c`：UART4 字节缓冲、行解析、模式/请求/ACK 发送。
- `project/user/inc/subject2_logic.h`：观察对象、绑定表、分类确认和身份追踪类型。
- `project/user/src/subject2_logic.c`：不持有硬件状态的科目二纯逻辑。
- `project/user/inc/subject2.h`：科目二状态机接口。
- `project/user/src/subject2.c`：扫描、中心校正、分类、指定推箱与地图确认。
- `project/user/inc/competition_flow.h`：三种比赛模式的纯状态转换接口。
- `project/user/src/competition_flow.c`：科目一/科目二/FULL 切换逻辑。
- `openmv/视觉/main.py`：ART2 双模型请求式分类程序。
- `tests/vision_uart_test.c`、`tests/run_vision_uart_test.ps1`。
- `tests/art2_protocol_test.py`。
- `tests/subject2_logic_test.c`、`tests/run_subject2_logic_test.ps1`。
- `tests/subject2_scan_test.c`、`tests/run_subject2_scan_test.ps1`。
- `tests/subject2_execution_test.c`、`tests/run_subject2_execution_test.ps1`。
- `tests/competition_flow_test.c`、`tests/run_competition_flow_test.ps1`。
- `tests/art_replan_subject2_handoff_test.c`、`tests/run_art_replan_subject2_handoff_test.ps1`。

### 修改

- `project/user/inc/solver.h`、`project/user/src/solver.c`：指定箱子到指定目标接口。
- `project/user/inc/art_replan.h`、`project/user/src/art_replan.c`：科目二发车交接、公共返航入口和返航完成事件。
- `project/user/inc/drive_config.h`：比赛模式和分类参数。
- `project/user/src/main.c`：初始化 ART2 UART。
- `project/user/src/app.c`：轮询 ART2 协议。
- `project/user/src/isr.c`：UART4 中断只向 `vision_uart` 推字节。
- `project/user/src/menu.c`：顶层科目调度、K3/K4 和状态文本衔接。
- `project/mdk/rt1064.uvprojx`：加入新增 C 文件。

模型二进制和现有标签文件属于用户现场资产。实施提交只暂存新脚本和源代码，不自动提交当前未跟踪的 `.tflite` 文件。

---

## 阶段 A：ART2 通信与模型

### Task 1: MCU UART4 分类协议驱动

**Files:**
- Create: `project/user/inc/vision_uart.h`
- Create: `project/user/src/vision_uart.c`
- Create: `tests/vision_uart_test.c`
- Create: `tests/run_vision_uart_test.ps1`

- [ ] **Step 1: 编写 UART 协议失败测试**

测试必须覆盖：模式 READY、递增请求号、旧请求样本丢弃、字段边界、ACK 和 CANCEL。

```c
vision_uart_init();
vision_uart_set_mode(VISION_MODE_BOX);
ASSERT_TX("VISION_MODE BOX\n");

feed_line("VISION_READY BOX");
ASSERT_TRUE(vision_uart_mode_ready(VISION_MODE_BOX));

request_id = vision_uart_request_classification();
ASSERT_EQ_U16(1u, request_id);
ASSERT_TX("VISION_REQ 1\n");

feed_line("VISION_SAMPLE 0 1 8 950"); /* 旧请求 */
feed_line("VISION_SAMPLE 1 1 8 932");
ASSERT_SAMPLE(1u, 1u, 8u, 932u);

feed_line("VISION_SAMPLE 1 2 10 900");  /* 非法 class */
feed_line("VISION_SAMPLE 1 3 8 1001"); /* 非法 confidence */
ASSERT_QUEUE_EMPTY();

vision_uart_ack(1u);
ASSERT_TX("VISION_ACK 1\n");
vision_uart_cancel();
ASSERT_TX("VISION_CANCEL\n");
```

- [ ] **Step 2: 运行测试确认失败**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_vision_uart_test.ps1
```

Expected: GCC 因 `vision_uart.h` 或接口不存在而失败。

- [ ] **Step 3: 定义最小公开接口**

```c
typedef enum
{
    VISION_MODE_NONE = 0,
    VISION_MODE_BOX,
    VISION_MODE_TARGET
} vision_mode_enum;

typedef struct
{
    uint16 request_id;
    uint16 sample_id;
    uint8 class_id;
    uint16 confidence_q;
} vision_sample_struct;

void vision_uart_init(void);
void vision_uart_push_byte(uint8 data);
void vision_uart_poll(void);
void vision_uart_set_mode(vision_mode_enum mode);
uint8 vision_uart_mode_ready(vision_mode_enum mode);
uint16 vision_uart_request_classification(void);
uint8 vision_uart_get_sample(vision_sample_struct *sample);
void vision_uart_ack(uint16 request_id);
void vision_uart_cancel(void);
```

- [ ] **Step 4: 实现最小环形缓冲和行解析**

实现要求：

```c
#define VISION_UART_INDEX          (UART_4)
#define VISION_RX_BUFFER_SIZE      (256u)
#define VISION_LINE_SIZE           (64u)
#define VISION_SAMPLE_QUEUE_SIZE   (8u)
```

- ISR 只调用 `vision_uart_push_byte()`。
- `vision_uart_poll()` 在主循环拆行。
- 只解析 `VISION_READY` 和 `VISION_SAMPLE`。
- `VISION_SAMPLE` 必须匹配当前请求号，且 `class_id <= 9`、`confidence_q <= 1000`。
- 新请求清空旧样本队列。
- 所有发送先写入固定长度 `tx_line`，再调用 `uart_write_string(UART_4, tx_line)`，不使用 `printf`。

- [ ] **Step 5: 运行协议测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_vision_uart_test.ps1
```

Expected: 所有 UART4 协议案例输出 `PASS`。

- [ ] **Step 6: 提交阶段性改动**

```powershell
git add project/user/inc/vision_uart.h project/user/src/vision_uart.c tests/vision_uart_test.c tests/run_vision_uart_test.ps1
git commit -m "feat: add ART2 vision UART protocol"
```

### Task 2: 接入 UART4 硬件调度

**Files:**
- Modify: `project/user/src/main.c`
- Modify: `project/user/src/app.c`
- Modify: `project/user/src/isr.c`
- Modify: `project/mdk/rt1064.uvprojx`

- [ ] **Step 1: 在 main 初始化 UART4 分类通道**

```c
#include "vision_uart.h"

vision_uart_init();
```

`vision_uart_init()` 内执行：

```c
uart_init(UART_4, 115200u, UART4_TX_C16, UART4_RX_C17);
uart_rx_interrupt(UART_4, 1u);
```

- [ ] **Step 2: 在主循环轮询分类协议**

在 `app_poll()` 中放在两个 UART 服务附近：

```c
openart_uart_poll();
vision_uart_poll();
```

- [ ] **Step 3: 让 LPUART4 中断只搬运 ART2 字节**

```c
void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        uint8 data = LPUART_ReadByte(LPUART4);
        vision_uart_push_byte(data);
    }
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);
}
```

删除该 ISR 中对 `flexio_camera_uart_handler()` 和 `gnss_uart_callback()` 的调用；不修改库文件。

- [ ] **Step 4: 把 vision_uart.c 加入 Keil User 组**

在 `rt1064.uvprojx` 的现有用户源文件组中添加：

```xml
<File>
  <FileName>vision_uart.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\user\src\vision_uart.c</FilePath>
</File>
```

- [ ] **Step 5: Keil 编译验证**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: `0 Error(s), 0 Warning(s)`。

- [ ] **Step 6: 提交硬件接入**

```powershell
git add project/user/src/main.c project/user/src/app.c project/user/src/isr.c project/mdk/rt1064.uvprojx
git commit -m "feat: connect ART2 on UART4"
```

### Task 3: OpenART #2 双模型请求式分类

**Files:**
- Create: `openmv/视觉/main.py`
- Create: `tests/art2_protocol_test.py`

- [ ] **Step 1: 编写可由 CPython 提取执行的协议测试**

仿照 `tests/openart_request_flow_test.py`，使用 AST 只提取常量和纯协议函数。覆盖：

```python
parse_uart_line("VISION_MODE BOX")
assert vision_mode == MODE_BOX
assert uart.tx[-1] == "VISION_READY BOX\n"

parse_uart_line("VISION_REQ 17")
assert request_active is True
assert request_id == 17

process_classification(uart, class_id=8, confidence_q=932)
assert uart.tx[-1] == "VISION_SAMPLE 17 1 8 932\n"

parse_uart_line("VISION_ACK 17")
assert request_active is False

parse_uart_line("VISION_CANCEL")
assert request_active is False
```

- [ ] **Step 2: 运行测试确认失败**

```powershell
python tests/art2_protocol_test.py
```

Expected: `openmv/视觉/main.py` 不存在或缺少协议函数。

- [ ] **Step 3: 实现摄像头、UART 和双模型初始化**

脚本常量必须直接使用已确认文件名：

```python
UART_INDEX = 12
UART_BAUD = 115200
CARTOON_MODEL = "/sd/model_in-uint8_out-uint8_channel_ptq.tflite"
NUMBER_MODEL = "/sd/numint8_in-int8_out-int8_channel_ptq.tflite"
MODE_NONE = 0
MODE_BOX = 1
MODE_TARGET = 2
```

摄像头设置沿用已训练模型的测试配置，不在首版重新调图像参数。启动时依次加载：

```python
cartoon_net = tf.load(CARTOON_MODEL, load_to_fb=True)
number_net = tf.load(NUMBER_MODEL, load_to_fb=True)
```

任一模型加载失败时打印错误并发送：

```text
VISION_ERROR MODEL_MEMORY
```

两模型加载成功后发送一次 `VISION_READY BOOT`，用于上板确认启动和内存状态。

- [ ] **Step 4: 实现请求期间推理**

```python
if request_active and vision_mode != MODE_NONE:
    img = sensor.snapshot()
    net = cartoon_net if vision_mode == MODE_BOX else number_net
    outputs = tf.classify(net, img)[0].output()
    class_id = outputs.index(max(outputs))
    confidence_q = int(outputs[class_id] * 1000 + 0.5)
    process_classification(uart, class_id, confidence_q)
else:
    sensor.snapshot()
```

ART2 每次推理都发送样本，不在 ART2 端做 0.75 门控。收到匹配 ACK 或 CANCEL 后停止推理。

- [ ] **Step 5: 运行 Python 验证**

```powershell
python tests/art2_protocol_test.py
python -m py_compile "openmv/视觉/main.py"
```

Expected: 协议测试输出 `art2-protocol PASS`，语法检查无输出且退出码为 0。

- [ ] **Step 6: OpenART Plus 上板内存验证**

将两个模型放到 SD 根目录并运行脚本。Expected:

```text
VISION_READY BOOT
```

且不出现 `VISION_ERROR MODEL_MEMORY`。如果双模型确实无法同时加载，停止本阶段，先把实测异常记录到设计文档，再改为单模型批次加载；不要静默引入第二套路径。

- [ ] **Step 7: 提交 ART2 脚本和测试**

```powershell
git add "openmv/视觉/main.py" tests/art2_protocol_test.py
git commit -m "feat: add request-driven ART2 classifier"
```

---

## 阶段 B：求解与分类纯逻辑

### Task 4: 指定箱子到指定目标 BFS

**Files:**
- Modify: `project/user/inc/solver.h`
- Modify: `project/user/src/solver.c`
- Modify: `tests/solver_navigation_test.c`

- [ ] **Step 1: 添加指定配对失败测试**

构造两个箱子、两个目标，验证指定接口不会选择更近但错误的目标：

```c
solved = solve_bound_box_path(&map.source,
                              map_cell_index(2u, 3u),
                              map_cell_index(7u, 9u),
                              &result);
ASSERT_TRUE(solved);
ASSERT_TRUE(result.task_count == 1u);
ASSERT_TRUE(result.waypoints[result.waypoint_count - 1u].task_end == 1u);
ASSERT_TRUE(replay_selected_box_ends_at(&result, 7u, 9u));
```

再覆盖：箱子坐标不存在、目标坐标不存在、指定任务无解。

- [ ] **Step 2: 运行现有 solver 测试确认新增案例失败**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_solver_navigation_test.ps1
```

Expected: `solve_bound_box_path` 未定义。

- [ ] **Step 3: 增加公开接口**

```c
uint8 solve_bound_box_path(const map_source_struct *source,
                           uint16 box_cell,
                           uint16 target_cell,
                           solve_result_struct *result);
```

- [ ] **Step 4: 复用现有单箱实现**

实现顺序固定为：

```c
clear_result(result);
map_load(source, &map, result);
find box_index matching box_cell;
find target_index matching target_cell;
solve_single_box(&map, box_index, target_index, single_path, &path_len);
apply_path_to_runtime(&map, box_index, target_index, single_path, path_len, result);
result->solved = 1u;
set_message(result, "Solved bound task");
```

不修改 `solve_map()` 的科目一贪心策略。

- [ ] **Step 5: 运行 solver 回归测试**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_solver_navigation_test.ps1
```

Expected: 原有导航/推箱案例和新增指定配对案例全部 `PASS`。

- [ ] **Step 6: 提交指定求解接口**

```powershell
git add project/user/inc/solver.h project/user/src/solver.c tests/solver_navigation_test.c
git commit -m "feat: solve a bound box-target task"
```

### Task 5: 观察选择、分类确认和身份追踪纯逻辑

**Files:**
- Create: `project/user/inc/subject2_logic.h`
- Create: `project/user/src/subject2_logic.c`
- Create: `tests/subject2_logic_test.c`
- Create: `tests/run_subject2_logic_test.ps1`

- [ ] **Step 1: 定义测试所需数据结构**

```c
#define SUBJECT2_CLASS_COUNT        (10u)
#define SUBJECT2_INVALID_CLASS      (0xFFu)
#define SUBJECT2_OBSERVATION_COUNT  (4u)

typedef struct
{
    uint16 cell;
    uint8 class_id;
    uint8 recognized;
    uint8 tried_observation_mask;
} subject2_object_struct;

typedef struct
{
    uint16 box_cell;
    uint16 target_cell;
    uint8 box_valid;
    uint8 target_valid;
    uint8 completed;
} subject2_binding_struct;

typedef struct
{
    uint8 object_index;
    uint8 observation_bit;
    uint8 row;
    uint8 col;
} subject2_observation_plan_struct;

typedef struct
{
    uint8 candidate_class;
    uint8 consecutive_count;
} subject2_classifier_struct;
```

- [ ] **Step 2: 编写失败测试**

测试至少包含：

```c
/* 最近可达观察格 */
ASSERT_TRUE(subject2_select_observation(&map.source,
                                        objects,
                                        object_count,
                                        &plan,
                                        &path));
ASSERT_EQ_CELL(expected_observe_cell, plan.row, plan.col);

/* 同类型对象等距时排除候选 */
ASSERT_FALSE(candidate_is_selected_when_two_boxes_are_one_cell_away());

/* 20 秒换位使用 tried mask，不重复选择失败方向 */
objects[i].tried_observation_mask |= plan.observation_bit;
ASSERT_TRUE(subject2_select_observation(&map.source,
                                        objects,
                                        object_count,
                                        &next_plan,
                                        &path));
ASSERT_TRUE(next_plan.observation_bit != plan.observation_bit);

/* 低置信度不累计，类别改变清零 */
subject2_classifier_push(&filter, 8u, 740u, 750u, 3u, &confirmed);
ASSERT_EQ_U8(0u, filter.consecutive_count);
subject2_classifier_push(&filter, 8u, 900u, 750u, 3u, &confirmed);
subject2_classifier_push(&filter, 4u, 900u, 750u, 3u, &confirmed);
ASSERT_EQ_U8(1u, filter.consecutive_count);

/* 类别唯一、集合相等 */
ASSERT_FALSE(subject2_bind_box(bindings, 8u, cell_b2));
ASSERT_TRUE(subject2_binding_sets_match(bindings));

/* 只有活动箱子移动时更新身份 */
ASSERT_EQ_U8(SUBJECT2_TRACK_MOVED,
    subject2_track_active_box(bindings, active_class, old_boxes, 3u, new_boxes, 3u));
ASSERT_EQ_U16(new_cell, bindings[active_class].box_cell);

/* 多个非活动箱子变化必须报错 */
ASSERT_EQ_U8(SUBJECT2_TRACK_AMBIGUOUS,
    subject2_track_active_box(bindings, active_class,
                              old_boxes, 3u,
                              bad_new_boxes, 3u));
```

- [ ] **Step 3: 运行测试确认失败**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
```

Expected: 新接口不存在。

- [ ] **Step 4: 实现对象收集和观察路径选择**

公开函数：

```c
uint8 subject2_collect_objects(const map_source_struct *source,
                               char symbol,
                               subject2_object_struct *objects,
                               uint8 *count);

uint8 subject2_select_observation(const map_source_struct *source,
                                  const subject2_object_struct *objects,
                                  uint8 object_count,
                                  subject2_observation_plan_struct *plan,
                                  solve_result_struct *path);
```

实现对每个未识别对象枚举 `up/down/left/right`：检查地图边界和字符、检查 `tried_observation_mask`、排除同类型等距候选、调用 `solve_navigation_path()`，最终选择 `action_count` 最小者。相同时保持对象数组顺序和方向顺序，保证测试可重复。

- [ ] **Step 5: 实现分类和绑定纯函数**

```c
void subject2_classifier_reset(subject2_classifier_struct *filter);
uint8 subject2_classifier_push(subject2_classifier_struct *filter,
                               uint8 class_id,
                               uint16 confidence_q,
                               uint16 threshold_q,
                               uint8 stable_samples,
                               uint8 *confirmed_class);
uint8 subject2_bind_box(subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
                        uint8 class_id, uint16 cell);
uint8 subject2_bind_target(subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
                           uint8 class_id, uint16 cell);
uint8 subject2_binding_sets_match(const subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT]);
```

低置信度把连续计数清零，避免不连续的高置信度帧被拼成“三连”。重复类别返回 0，不覆盖旧坐标。

- [ ] **Step 6: 实现活动箱子身份更新**

```c
typedef enum
{
    SUBJECT2_TRACK_UNCHANGED = 0,
    SUBJECT2_TRACK_MOVED,
    SUBJECT2_TRACK_AMBIGUOUS
} subject2_track_result_enum;

subject2_track_result_enum subject2_track_active_box(
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
    uint8 active_class,
    const uint16 *old_boxes,
    uint8 old_box_count,
    const uint16 *new_boxes,
    uint8 new_box_count);
```

同数量地图中，所有非活动箱子必须仍能在新集合中匹配。剩余唯一新坐标更新给活动类别；无法唯一匹配则返回 `AMBIGUOUS`。

- [ ] **Step 7: 运行纯逻辑测试**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
```

Expected: 全部案例 `PASS`。

- [ ] **Step 8: 加入 Keil 工程并编译**

将 `subject2_logic.c` 加到 `rt1064.uvprojx`，运行 Keil build，Expected: `0 Error(s), 0 Warning(s)`。

- [ ] **Step 9: 提交纯逻辑阶段**

```powershell
git add project/user/inc/subject2_logic.h project/user/src/subject2_logic.c tests/subject2_logic_test.c tests/run_subject2_logic_test.ps1 project/mdk/rt1064.uvprojx
git commit -m "feat: add subject2 planning and binding logic"
```

---

## 阶段 C：科目二扫描状态机

### Task 6: 扫描全部箱子和随机目标

**Files:**
- Create: `project/user/inc/subject2.h`
- Create: `project/user/src/subject2.c`
- Create: `tests/subject2_scan_test.c`
- Create: `tests/run_subject2_scan_test.ps1`
- Modify: `project/mdk/rt1064.uvprojx`

- [ ] **Step 1: 定义状态和上下文接口**

```c
typedef enum
{
    SUBJECT2_IDLE = 0,
    SUBJECT2_SCAN_BOX_MODE,
    SUBJECT2_SCAN_BOX_PLAN,
    SUBJECT2_SCAN_BOX_MOVE,
    SUBJECT2_SCAN_BOX_CENTER,
    SUBJECT2_SCAN_BOX_CLASSIFY,
    SUBJECT2_SCAN_TARGET_MODE,
    SUBJECT2_SCAN_TARGET_PLAN,
    SUBJECT2_SCAN_TARGET_MOVE,
    SUBJECT2_SCAN_TARGET_CENTER,
    SUBJECT2_SCAN_TARGET_CLASSIFY,
    SUBJECT2_VALIDATE_BINDINGS,
    SUBJECT2_SELECT_PUSH,
    SUBJECT2_EXECUTE_PUSH,
    SUBJECT2_CONFIRM_MAP,
    SUBJECT2_RETURN_REQUESTED,
    SUBJECT2_DONE,
    SUBJECT2_ERROR
} subject2_state_enum;

typedef struct
{
    solve_result_struct *result;
    map_source_struct *snapshot;
    char (*snapshot_rows)[MAP_COLS + 1];
    uint8 *snapshot_valid;
    uint32 *elapsed_ms;
    uint8 *start_row;
    uint8 *start_col;
    run_mode_enum run_mode;
} subject2_context_struct;

typedef struct
{
    uint8 redraw;
    uint8 enter_execute;
    uint8 return_requested;
    float return_pose_x_cm;
    float return_pose_y_cm;
    const char *run_state;
} subject2_update_struct;

void subject2_begin(const subject2_context_struct *context,
                    float initial_pose_x_cm,
                    float initial_pose_y_cm,
                    subject2_update_struct *update);
void subject2_tick(const subject2_context_struct *context,
                   subject2_update_struct *update);
void subject2_cancel(void);
subject2_state_enum subject2_get_state(void);
uint8 subject2_get_active_class(void);
```

- [ ] **Step 2: 编写扫描状态机失败测试**

使用 stub 模拟 `vision_uart`、`openart_uart`、executor 和时间。测试完整序列：

```text
begin -> VISION_MODE BOX -> READY
-> 最近观察路径 executor_start(art_sync=0)
-> executor DONE -> CENTER_REQ
-> 3帧中心 -> commit pose
-> VISION_REQ
-> 3个相同分类样本 -> ACK -> 绑定箱子
-> 扫完 B -> VISION_MODE TARGET
-> 扫完 T -> Bind
```

额外验证：

- 分类前必须先完成 ART1 中心提交。
- 低置信度和不同类别不能提前绑定。
- 20 秒后设置当前 `observation_bit` 并进入 `VRetry`。
- 四个候选全部失败后 `SUBJECT2_ERROR` 且调用 `stop_motion()`。
- 扫描期间 `B/T` 坐标变化立即报错。

- [ ] **Step 3: 运行测试确认失败**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: `subject2` 接口不存在。

- [ ] **Step 4: 实现 begin 和模型批次切换**

`subject2_begin()` 必须：

```c
copy initial map snapshot;
collect B objects and T objects;
clear all bindings and retry masks;
save initial pose offset;
vision_uart_set_mode(VISION_MODE_BOX);
state = SUBJECT2_SCAN_BOX_MODE;
```

模式未 READY 时保持停车和 `BScan/TScan`，不开始导航。

- [ ] **Step 5: 实现观察导航**

选出观察路径后：

```c
executor_start(result->waypoints, result->waypoint_count,
               car_row, car_col,
               current_pose_offset_x, current_pose_offset_y,
               0u, 0u);
```

若已经位于观察格、`waypoint_count == 0`，不调用 `executor_start()`，直接进入中心请求。

- [ ] **Step 6: 实现 ART1 中心请求和位置提交**

进入 `*_CENTER` 时只发送一次 `openart_request_player_center()`。收齐3个请求样本后依次调用：

```c
executor_apply_art_player_center(col_q, row_q, sample_index);
executor_commit_art_player_center(current_car_row, current_car_col);
```

只接受 `APPLIED` 或 `IGNORED`；`REJECTED/ABNORMAL/NONE` 停车并显示 `E:Ctr`。中心成功后才发送 `VISION_REQ`。

- [ ] **Step 7: 实现分类确认、ACK 和换位**

读取 `vision_uart_get_sample()`，调用 `subject2_classifier_push()`。确认后：

```c
vision_uart_ack(request_id);
bind class to current object cell;
mark object recognized;
return to *_PLAN;
```

当 `time_ms() - classify_start_ms >= SUBJECT2_VIEW_TIMEOUT_MS`：

```c
vision_uart_cancel();
object.tried_observation_mask |= current_plan.observation_bit;
state = current plan state;
update->run_state = "VRetry";
```

- [ ] **Step 8: 实现扫描完成后的集合校验**

`subject2_binding_sets_match()` 成功才进入 `SUBJECT2_SELECT_PUSH`。不一致时清除只存在于单侧集合的对象识别结果并重新扫描对应批次；第二次完整重扫仍不一致则 `E:Class`。

- [ ] **Step 9: 运行扫描测试与 Keil 编译**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: 主机测试全 PASS；Keil `0 Error(s), 0 Warning(s)`。

- [ ] **Step 10: 提交扫描阶段**

```powershell
git add project/user/inc/subject2.h project/user/src/subject2.c tests/subject2_scan_test.c tests/run_subject2_scan_test.ps1 project/mdk/rt1064.uvprojx
git commit -m "feat: scan and bind subject2 objects"
```

---

## 阶段 D：指定推箱与重规划

### Task 7: 选择最短绑定任务并执行

**Files:**
- Modify: `project/user/src/subject2.c`
- Create: `tests/subject2_execution_test.c`
- Create: `tests/run_subject2_execution_test.ps1`

- [ ] **Step 1: 编写任务选择失败测试**

准备三个有效绑定，其中一个无解，验证状态机调用 `solve_bound_box_path()` 尝试全部未完成类别，并选择 `action_count` 最小的可解任务。断言：

```c
ASSERT_EQ_U8(expected_class, subject2_get_active_class());
ASSERT_EQ_U8(1u, executor_start_art_sync);
ASSERT_EQ_U8(1u,
    result.waypoints[result.waypoint_count - 1u].task_end);
```

全部剩余绑定无解时应停止并显示 `E:Plan`。

- [ ] **Step 2: 运行测试确认失败**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_execution_test.ps1
```

Expected: 推箱状态尚未实现。

- [ ] **Step 3: 实现最短指定任务选择**

对 `class_id=0..9` 的有效未完成绑定调用：

```c
solve_bound_box_path(context->snapshot,
                     bindings[class_id].box_cell,
                     bindings[class_id].target_cell,
                     &candidate_result);
```

选择 `action_count` 最小者，复制到共享 `context->result`，以最新 `C` 和 ART 中心 offset 调用：

```c
executor_start(context->result->waypoints,
               context->result->waypoint_count,
               stats.car_row,
               stats.car_col,
               initial_pose_x_cm,
               initial_pose_y_cm,
               (RUN_MODE_STEP == context->run_mode) ? 1u : 0u,
               1u);
```

- [ ] **Step 4: 实现科目二推箱前中心校正**

当 `executor_art_pre_push_pending()` 为 1 时，科目二状态机接管请求：

```text
发送一次 CENTER_REQ
-> 收齐3帧
-> executor_apply_art_player_center()
-> executor_commit_art_player_center(latest C)
-> APPLIED/IGNORED 后 executor_continue_after_pre_push_center()
```

此阶段不调用 `art_replan_tick()`，避免它进入科目一 `solve_map()`。

- [ ] **Step 5: 编写并实现任务结束地图确认**

当 `executor_art_sync_pending()` 为 1：

- 等待 `EXEC_ART_STABLE_FRAMES` 个一致新地图帧。
- 记录执行前后的 `B/T` 数量和 `B` 坐标集合。
- `B/T` 各减少1：当前类别 `completed=1`，更新基线，回到 `SELECT_PUSH`。
- 数量不变：调用 `subject2_track_active_box()`；`UNCHANGED/MOVED` 都重算当前任务，`AMBIGUOUS` 进入 `E:Track`。
- 其他数量变化进入 `E:Track`。

成功和重试重启 executor 前，都再请求一次最新 ART1 中心并计算 pose offset，不能固定清零。

- [ ] **Step 6: 最后一箱完成后请求返航**

当全部绑定完成且稳定地图为 `B=0,T=0,C=1`：

```c
stop_motion();
map_source_snapshot(context->snapshot, context->snapshot_rows, stable_source);
*context->snapshot_valid = 1u;
state = SUBJECT2_RETURN_REQUESTED;
update->return_requested = 1u;
update->return_pose_x_cm = latest_pose_offset_x_cm;
update->return_pose_y_cm = latest_pose_offset_y_cm;
update->run_state = "S2Ret";
```

subject2 本身不复制返航算法。

- [ ] **Step 7: 运行执行测试**

覆盖 `Box OK`、箱子移动后的 Push Retry、箱子不动重算、多箱异常变化、最后一箱返航请求：

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_execution_test.ps1
```

Expected: 全部 `PASS`。

- [ ] **Step 8: Keil 编译和提交**

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
git add project/user/src/subject2.c tests/subject2_execution_test.c tests/run_subject2_execution_test.ps1
git commit -m "feat: execute bound subject2 push tasks"
```

---

## 阶段 E：发车、返航与 FULL 全流程

### Task 8: art_replan 提供科目二发车交接和公共返航事件

**Files:**
- Modify: `project/user/inc/art_replan.h`
- Modify: `project/user/src/art_replan.c`
- Create: `tests/art_replan_subject2_handoff_test.c`
- Create: `tests/run_art_replan_subject2_handoff_test.ps1`
- Modify: `tests/art_replan_requested_center_test.c`

- [ ] **Step 1: 添加发车交接测试**

测试调用：

```c
art_replan_begin_subject2(&update);
```

模拟现有 `W5S -> WCTR -> LCH -> WMAP -> ICtr`。中心和稳定地图完成后断言：

```c
ASSERT_EQ_U8(1u, update.subject2_map_ready);
ASSERT_EQ_U8(0u, executor_start_count);
ASSERT_NEAR(expected_dx, update.initial_pose_x_cm);
ASSERT_NEAR(expected_dy, update.initial_pose_y_cm);
```

证明科目二发车复用了现有发车流程，但没有调用科目一 `solve_map()`。

- [ ] **Step 2: 添加返航完成事件测试**

公开返航入口：

```c
uint8 art_replan_begin_return_home(const art_replan_context_struct *context,
                                   float initial_pose_x_cm,
                                   float initial_pose_y_cm,
                                   art_replan_update_struct *update);
```

返航验证成功后断言 `update.return_complete == 1u`，并保持现有 `executor_finish_done()` 行为。

- [ ] **Step 3: 扩展 update 结构**

```c
typedef struct
{
    uint8 redraw;
    uint8 enter_execute;
    uint8 reset_playback_step;
    uint8 subject2_map_ready;
    uint8 return_complete;
    float initial_pose_x_cm;
    float initial_pose_y_cm;
    art_replan_playback_enum playback;
    const char *run_state;
} art_replan_update_struct;
```

- [ ] **Step 4: 实现科目二初始分支**

新增内部 `art_launch_subject2` 标志。`art_replan_begin_initial()` 设为0；`art_replan_begin_subject2()` 设为1并进入同一个 `WAIT_LAUNCH`。

在 `art_replan_solve_snapshot_after_center()` 的 `ART_REPLAN_INITIAL` 分支中：

```c
if(art_launch_subject2)
{
    *context->start_row = stats.car_row;
    *context->start_col = stats.car_col;
    update->subject2_map_ready = 1u;
    update->initial_pose_x_cm = initial_pose_x_cm;
    update->initial_pose_y_cm = initial_pose_y_cm;
    art_replan_cancel();
    return;
}
```

不要更新为科目一求解结果，不启动 executor。

- [ ] **Step 5: 公开返航入口并上报完成**

公共入口只校验 home 有效、地图统计合法，然后调用现有 `art_replan_start_return()`。`art_replan_finish_return()` 在完成时设置 `update->return_complete=1u`。

- [ ] **Step 6: 运行 ART 回归测试**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_subject2_handoff_test.ps1
```

Expected: 科目一原测试和科目二交接测试均 PASS。

- [ ] **Step 7: 提交 ART 公共流程**

```powershell
git add project/user/inc/art_replan.h project/user/src/art_replan.c tests/art_replan_requested_center_test.c tests/art_replan_subject2_handoff_test.c tests/run_art_replan_subject2_handoff_test.ps1
git commit -m "feat: expose subject2 launch and return handoff"
```

### Task 9: 三种比赛模式纯状态转换

**Files:**
- Create: `project/user/inc/competition_flow.h`
- Create: `project/user/src/competition_flow.c`
- Create: `tests/competition_flow_test.c`
- Create: `tests/run_competition_flow_test.ps1`
- Modify: `project/mdk/rt1064.uvprojx`

- [ ] **Step 1: 编写三模式失败测试**

```c
competition_flow_start(COMPETITION_MODE_SUBJECT1_DEBUG);
ASSERT_EQ(COMPETITION_ACTION_START_SUBJECT1, take_action());
competition_flow_on_return_complete();
ASSERT_EQ(COMPETITION_ACTION_FINISH, take_action());

competition_flow_start(COMPETITION_MODE_SUBJECT2_DEBUG);
ASSERT_EQ(COMPETITION_ACTION_START_SUBJECT2, take_action());
competition_flow_on_return_complete();
ASSERT_EQ(COMPETITION_ACTION_FINISH, take_action());

competition_flow_start(COMPETITION_MODE_FULL);
ASSERT_EQ(COMPETITION_ACTION_START_SUBJECT1, take_action());
competition_flow_on_return_complete();
ASSERT_EQ(COMPETITION_ACTION_START_SUBJECT2, take_action());
competition_flow_on_return_complete();
ASSERT_EQ(COMPETITION_ACTION_FINISH, take_action());
```

- [ ] **Step 2: 实现最小状态机**

```c
typedef enum
{
    COMPETITION_ACTION_NONE = 0,
    COMPETITION_ACTION_START_SUBJECT1,
    COMPETITION_ACTION_START_SUBJECT2,
    COMPETITION_ACTION_FINISH
} competition_action_enum;

void competition_flow_start(uint8 mode);
void competition_flow_on_return_complete(void);
competition_action_enum competition_flow_take_action(void);
void competition_flow_cancel(void);
```

该模块不访问菜单、executor 或 UART，只产生一次性动作。

- [ ] **Step 3: 运行测试并加入 Keil**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_competition_flow_test.ps1
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: 流程测试 PASS；Keil 零错误零警告。

- [ ] **Step 4: 提交模式状态机**

```powershell
git add project/user/inc/competition_flow.h project/user/src/competition_flow.c tests/competition_flow_test.c tests/run_competition_flow_test.ps1 project/mdk/rt1064.uvprojx
git commit -m "feat: add competition subject flow"
```

### Task 10: menu 顶层调度和配置接入

**Files:**
- Modify: `project/user/inc/drive_config.h`
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: 增加已确认配置**

```c
#define COMPETITION_MODE_SUBJECT1_DEBUG      (1u)
#define COMPETITION_MODE_SUBJECT2_DEBUG      (2u)
#define COMPETITION_MODE_FULL                (3u)
#define COMPETITION_MODE                     (COMPETITION_MODE_SUBJECT2_DEBUG)

#define SUBJECT2_CLASS_CONFIDENCE_Q          (750u)
#define SUBJECT2_CLASS_STABLE_SAMPLES        (3u)
#define SUBJECT2_VIEW_TIMEOUT_MS             (20000u)
```

- [ ] **Step 2: K3 启动改为消费 competition action**

ART 来源 Run/Step 首次执行时：

```c
competition_flow_start(COMPETITION_MODE);
action = competition_flow_take_action();
if(action == COMPETITION_ACTION_START_SUBJECT1)
    art_replan_begin_initial(&update);
else if(action == COMPETITION_ACTION_START_SUBJECT2)
    art_replan_begin_subject2(&update);
```

离线地图和 `RUN_MODE_SOLVE` 保持原逻辑。

- [ ] **Step 3: menu_poll 分流 ART 与 subject2**

- 发车或返航期间调用 `art_replan_tick()`。
- `subject2_map_ready` 时调用 `subject2_begin()`。
- 科目二活动期间调用 `subject2_tick()`，并向 `run_state` 应用 `BScan/TScan/VPos/VCtr/VWait/VRetry/Bind/S2Push/S2Ret`。
- 科目二活动期间传给 `art_replan_tick()` 的 `art_source_enabled` 必须为0，避免科目一自动接管 `executor_art_sync_pending()`。
- `subject2_update.return_requested` 时，使用共享快照和 `return_pose_x_cm/return_pose_y_cm` 调用 `art_replan_begin_return_home()`。
- `art_update.return_complete` 时调用 `competition_flow_on_return_complete()`；FULL 第一次返航产生 `START_SUBJECT2` 并立即调用 `art_replan_begin_subject2()`，不等待 K3。

- [ ] **Step 4: K4 统一停止**

K4 短按和长按退出路径都执行：

```c
subject2_cancel();
vision_uart_cancel();
competition_flow_cancel();
art_replan_cancel();
executor_stop();
```

不得修改 K4 的现有菜单返回语义。

- [ ] **Step 5: Keil 编译和手动状态检查**

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: `0 Error(s), 0 Warning(s)`；Execute 页状态文本不新增额外诊断行。

- [ ] **Step 6: 提交顶层集成**

```powershell
git add project/user/inc/drive_config.h project/user/src/menu.c
git commit -m "feat: integrate subject2 competition flow"
```

---

## 最终回归与上板验收

### Task 11: 全量自动验证

**Files:**
- Modify only if verification exposes a defect in files introduced by this plan.

- [ ] **Step 1: 运行全部主机 C 测试**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_solver_navigation_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_vision_uart_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_execution_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_subject2_handoff_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_competition_flow_test.ps1
```

Expected: 每个脚本退出码为0并输出 PASS。

- [ ] **Step 2: 运行全部 OpenART Python 测试**

```powershell
python tests/openart_player_anchor_test.py
python tests/openart_precise_center_test.py
python tests/openart_request_flow_test.py
python tests/art2_protocol_test.py
python -m py_compile openmv/main_see.py
python -m py_compile "openmv/视觉/main.py"
```

Expected: 四个测试输出 PASS，两个语法检查退出码为0。

- [ ] **Step 3: Keil 最终编译**

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: `0 Error(s), 0 Warning(s)`。

- [ ] **Step 4: 科目二调试模式上板**

设置：

```c
#define COMPETITION_MODE (COMPETITION_MODE_SUBJECT2_DEBUG)
```

验收顺序：发车、获取科目二地图、最近邻扫描全部箱子、切换数字模式、扫描随机目标、集合校验、指定推箱、返航。记录 ART2 的 `request_id/class_id/confidence_q`，确认不存在旧请求串入。

- [ ] **Step 5: FULL 模式上板**

设置：

```c
#define COMPETITION_MODE (COMPETITION_MODE_FULL)
```

只按一次 K3。验收：科目一完赛返航、上位机自动切关、MCU 自动第二次发车、科目二完赛返航、最终 Done。

- [ ] **Step 6: 检查工作区和提交最终修复**

```powershell
git status --short
git diff --check
```

只提交本计划产生的必要修复；不要加入用户现有未跟踪模型文件或无关改动。

---

## 实施注意事项

- BFS、模型推理、UART 文本解析和屏幕绘制都不得放入 PIT 20ms ISR。
- `subject2.c` 的每次 `tick` 只推进一个非阻塞状态，不使用延时循环。
- 科目二执行阶段不能让现有 `art_replan_tick()` 自动处理段末同步，否则会退回科目一的任意配对求解。
- ART2 分类结果只有在 ART1 中心校正成功后才能绑定到地图坐标。
- 类别集合未完全一致前禁止启动任何推箱任务。
- 每次只推一个类别箱子，这是 Push Retry 时能够恢复身份的前提。
- 科目一现有 API 和行为尽量保持不变；新增入口不得改变离线地图路径。
