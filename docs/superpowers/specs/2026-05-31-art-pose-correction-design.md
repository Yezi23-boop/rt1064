# ART 格点校正设计

## 背景

当前 RT1064 端使用编码器和 IMU 做 20ms 本地位姿积分，`executor.c` 根据 `pose_x_cm`、`pose_y_cm` 判断 waypoint 是否到达。OpenART 端 `openart_plus_grid_recognizer_pure.py` 每 200ms 通过 UART 发送一帧 12x16 字符地图，其中小车所在格子用 `C` 表示。

现有 MCU 只把 OpenART 地图作为求解输入和显示来源，没有把地图中的 `C` 用于路径执行校正。因此可能出现 MCU 本地惯导认为已经到达目标格子，但屏幕/虚拟系统中的小车标记还没有到位，后续路径继续执行后与虚拟推箱子状态分叉。

目标是建立一套轻量、可调试、抗误触发的 ART 格点校正规则：本地惯导负责运动中的连续控制，ART 地图负责段末确认和长期漂移修正。

## 当前数据源

### ART 端输出

文件：`openmv/openart_plus_grid_recognizer_pure.py`

当前协议：

```text
MAP_BEGIN
12 行地图，每行 16 个字符
MAP_END
```

字符含义：

```text
# 墙
. 空地
B 箱子
T 目标点
C 小车
X 炸弹/障碍
```

发送周期：

```text
UART_MAP_SEND_PERIOD_MS = 200
```

输出稳定策略：

```text
STABILIZE_OUTPUT = True
STABLE_CHANGE_COUNT = 2
```

这意味着 ART 的 `C` 有天然延迟，不适合运动中实时闭环，但适合停车后的段末确认。

### MCU 端现状

文件：`project/user/src/openart_uart.c`

MCU 已经能接收完整地图帧，并保存为 `map_source_struct`：

```c
const map_source_struct *openart_map_get(void);
uint32 openart_uart_get_frame_count(void);
uint32 openart_last_rx_ms(void);
```

文件：`project/user/src/executor.c`

执行器当前流程：

```text
本地 pose 到点
-> reset_motion_segment()
-> 连续满足到点阈值
-> segment settle
-> 进入下一 waypoint
```

缺少步骤：

```text
segment settle
-> 等待/检查 ART 小车格子
-> 校正本地 pose
-> 再进入下一 waypoint
```

### 屏幕显示现状

当前屏幕已经有两类与 ART/地图相关的显示，但它们用途不同：

1. `ART Map` 页面显示 `openart_map_get()` 收到的 OpenART 原始地图，可以看到 ART 当前识别出来的 `C`。
2. `Run/Execute` 页面使用启动时的地图源作为底图，并调用 `draw_pose_map_from_start()`，小车高亮位置来自 MCU 本地 `pose_x_cm/pose_y_cm` 换算出的格子。

因此，Run/Execute 页面上看到的小车格子主要表示“MCU 认为自己在哪”，不等价于“ART 最新帧识别到 C 在哪”。后续校正调试需要同时显示：

```text
MCU pose row/col
ART C row/col
当前目标 waypoint row/col
```

这样才能直接看出三者是否一致。

## 设计原则

1. 运动过程中相信 MCU 本地惯导，不使用 ART 连续修正速度。
2. waypoint 段末停车后，以 ART 的 `C` 格子作为任务状态确认。
3. 第一版只校正 `pose_x_cm` 和 `pose_y_cm`，不校正 yaw。
4. ART 数据不新鲜、不唯一、不稳定时，不允许直接校正。
5. ART 与 MCU/目标格子冲突时，优先暂停而不是继续执行。
6. 发车前必须建立“人工确认后的基准帧”，避免人在场上或画面遮挡导致 ART 误识别时车辆自行启动。

## 坐标约定

MCU 内部以路径起点为物理原点：

```text
x_cm = (col - start_col) * GRID_SIZE_CM
y_cm = (start_row - row) * GRID_SIZE_CM
```

其中：

```text
+x 右移
+y 前进/上移
GRID_SIZE_CM = 20.0
```

当 ART 识别到 `C` 在 `(art_row, art_col)` 时，对应物理坐标为：

```text
art_x_cm = (art_col - start_col) * GRID_SIZE_CM
art_y_cm = (start_row - art_row) * GRID_SIZE_CM
```

第一版段末校正时，如果 ART 确认小车在当前目标 waypoint 格子，则将本地 pose 重置到 waypoint 格子中心：

```text
drive_pose_reset(target_x_cm, target_y_cm, current_pose_yaw)
```

## 启动安全门槛

用户提到的关键问题是：比赛发车可能需要人操作，人还没离开赛场时，车不能因为 ART 看到一个错误 `C` 就自己启动。因此 ART 校正必须分为“接收地图”和“允许执行”两个阶段。

### 上电接收阶段

MCU 可以持续接收 ART 地图，但此时不允许 executor 自动启动，也不允许把任意 `C` 当作可靠起点。

此阶段只做：

```text
接收地图
显示地图
统计 frame_count
检查是否存在唯一 C
```

### 待发车阶段

用户按键选择地图来源、求解路径后，进入待发车状态。此时仍然不立即动车，需要满足“起点确认”。

起点确认条件：

```text
1. 收到 ART 新帧
2. 地图中恰好只有一个 C
3. C 的 row/col 等于求解器识别出的 start_row/start_col
4. 连续 N 帧满足上述条件
5. 用户按下发车键后，再锁定当前 frame_count 作为 launch 基准
```

建议初值：

```text
ART_START_STABLE_FRAMES = 3
ART_FRAME_FRESH_MS = 300
```

这样可以挡住两类风险：

```text
人挡住屏幕导致单帧误识别
人还没离开时系统提前使用旧地图发车
```

### 发车定义

“第一次有效 ART 数据”不应该定义为上电后第一帧，也不应该定义为第一次看到 `C`。

推荐定义：

```text
用户明确发车后，第一组连续稳定、且 C 在规划起点格子的 ART 帧，才是本次任务的 ART 基准帧。
```

也就是说，只有在以下事件之后才允许 executor 进入 `RUNNING`：

```text
用户按下 Run/Resume
-> MCU 等待 ART 起点稳定
-> ART 起点连续确认成功
-> drive_pose_reset(0, 0, 0)
-> executor_start()
```

如果用户想跳过 ART 校验，需要单独提供 Debug/Bypass 选项，不能作为默认行为。

## 段末校正流程

每个 waypoint 到点后的推荐流程：

```text
EXEC_MOVING
-> MCU 本地 pose 到点
-> EXEC_LOCAL_ARRIVED
-> 停车 settle 150ms
-> EXEC_ART_WAIT
-> EXEC_ART_VALIDATE
-> 校正 pose
-> 进入下一 waypoint
```

详细规则：

1. 本地 pose 进入 `PATH_ARRIVAL_THRESHOLD_CM`，并连续满足 `EXEC_ARRIVAL_STABLE_TICKS`。
2. 调用 `reset_motion_segment()`，停止平移输出。
3. 停稳 `EXEC_SEGMENT_SETTLE_MS`，建议 150ms。
4. 等待 OpenART 新帧，最多 `ART_VERIFY_TIMEOUT_MS`，建议 600ms。
5. 从最新地图中查找 `C`。
6. 从最新地图中提取 `B` 箱子集合。
7. MCU 根据求解时地图和当前 waypoint 推演本段结束后的预期箱子集合。
8. 检查 `C` 是否唯一、帧是否新鲜、目标格是否匹配，并检查 ART 箱子集合是否等于预期箱子集合。
9. 成功则重置本地 pose 到目标格中心，并提交预期箱子状态。
10. 失败则暂停或进入错误状态。

## ART 数据有效性

一帧 ART 地图必须满足以下条件才可用于校正：

```text
frame_count 有变化，或该帧尚未被本次校正使用
time_ms() - openart_last_rx_ms() <= ART_FRAME_FRESH_MS
地图中恰好只有一个 C
C 不在墙、炸弹或边界异常区域
```

由于当前 `C` 与地图字符共用同一张地图，`C` 所在格子本身会覆盖原地图元素。第一版只要求唯一 `C`，并通过目标 waypoint 验证其合理性，不额外判断该格原始类型。

建议参数：

```text
ART_FRAME_FRESH_MS = 300
ART_VERIFY_TIMEOUT_MS = 600
ART_CONFIRM_FRAMES = 2
ART_START_STABLE_FRAMES = 3
```

段末校正建议至少确认 1 到 2 帧。起点确认建议 3 帧，因为启动前人的干扰风险更高。

## 冲突处理

### ART 与目标格、箱子集合一致

条件：

```text
art_row == target_row
art_col == target_col
art_boxes == expected_boxes_after_waypoint
```

处理：

```text
drive_pose_reset(target_x_cm, target_y_cm, current_pose_yaw)
进入下一 waypoint
```

### MCU 到点，但 ART 还在上一格或箱子集合不匹配

处理：

```text
保持停车
等待后续 ART 帧
如果超时仍不匹配，进入 ART_MISMATCH
```

不能直接进入下一段，因为虚拟系统可能还没移动到目标格，或箱子状态已经和规划状态分叉。

### ART 到目标格，但 MCU pose 仍有小误差

处理：

```text
如果 MCU 到目标距离小于半格，且 ART 连续确认目标格：
    认为到点成功
    reset pose 到目标格中心
```

这可以补偿编码器里程偏小、横移滑移或末端惯性导致的厘米级误差。

### ART 跳到非目标格

处理：

```text
连续几帧都不等于目标格：
    stop_motion()
    executor 进入 PAUSED 或 ERROR
    记录 target_cell、art_cell、mcu_pose、frame_count
```

第一版不建议自动改路径。因为这代表虚拟状态与 MCU 执行状态已经分叉，需要人确认。

### ART 无效或旧帧

处理：

```text
等待新帧
超时进入 ART_TIMEOUT
```

比赛默认不建议降级为纯 MCU 继续执行。纯 MCU 继续执行可以做成 Debug 选项。

## 状态机设计

建议在 executor 中保留现有外层状态，新增段末 ART 校正子状态。

第一版可以使用内部枚举：

```c
typedef enum
{
    EXEC_SEG_MOVING = 0,
    EXEC_SEG_SETTLE,
    EXEC_SEG_ART_WAIT,
    EXEC_SEG_ART_VALIDATE,
} executor_segment_state_enum;
```

也可以继续使用现有 `segment_settling` 变量，再补充：

```text
art_verify_active
art_verify_elapsed_ms
art_confirm_count
art_last_used_frame
```

更清晰的推荐是改成显式子状态，便于屏幕显示和调试。

错误类型建议扩展：

```c
EXEC_ERROR_ART_TIMEOUT
EXEC_ERROR_ART_NO_PLAYER
EXEC_ERROR_ART_MULTI_PLAYER
EXEC_ERROR_ART_MISMATCH
EXEC_ERROR_ART_START_UNSTABLE
```

## 模块边界

### openart_uart

保持负责 UART 收包和地图快照。

建议新增只读查询函数：

```c
uint8 openart_find_player_cell(uint8 *row, uint8 *col);
uint8 openart_count_player_cells(uint8 *first_row, uint8 *first_col);
uint8 openart_map_is_fresh(uint32 now_ms, uint32 max_age_ms);
```

不建议让 `openart_uart.c` 直接调用 `drive_pose_reset()` 或控制 executor。

### executor

负责在 waypoint 段末决定是否使用 ART 校正。

需要知道：

```text
当前 waypoint row/col
start_row/start_col
target_x_cm/target_y_cm
当前 pose yaw
ART player row/col
```

校正动作在 executor 内发生：

```text
drive_pose_reset(target_x_cm, target_y_cm, pose->yaw_deg)
```

### menu/app

负责用户启动门槛。

建议把“求解完成”和“真正发车”拆开：

```text
Solve Ready
-> Wait ART Start
-> Ready To Launch
-> Running
```

用户按发车后，如果 ART 起点不稳定，屏幕显示：

```text
WAIT ART START
```

而不是让车马上动。

## 调试输出

建议 VOFA 或屏幕增加以下字段：

```text
art_frame_count
art_age_ms
art_player_count
art_row
art_col
target_row
target_col
art_confirm_count
executor_error
```

发生错误时串口打印一行：

```text
ART_MISMATCH step=3 target=(5,7) art=(5,6) pose=(39.2,0.4) frame=128 age=40
```

这样可以快速判断是 MCU 惯导提前到点，还是 ART 识别延迟/错误。

## 参数初值

```c
#define ART_CORRECTION_ENABLE          (1)
#define ART_START_GUARD_ENABLE         (1)
#define ART_FRAME_FRESH_MS             (300u)
#define ART_VERIFY_TIMEOUT_MS          (600u)
#define ART_CONFIRM_FRAMES             (2u)
#define ART_START_STABLE_FRAMES        (3u)
#define ART_ALLOW_MCU_ONLY_FALLBACK    (0)
```

第一版不建议启用 MCU-only fallback。只有在 Debug 模式或现场确认 ART 完全不可用时，才允许跳过 ART 校正。

## 分阶段实现

### 阶段 1：只读 ART 小车格

1. 在 `openart_uart.c` 增加 `C` 查找函数。
2. 在现有 Run/Execute 或 Debug 显示基础上，额外显示 ART 的 `row/col`、数量和帧龄。
3. 不影响 executor 行为。

验收：

```text
手动移动虚拟小车，MCU 能稳定显示唯一 C 的 row/col。
Run/Execute 能区分 MCU pose 格子和 ART C 格子。
遮挡或误识别时，能显示 no/multi/unstable。
```

### 阶段 2：发车前起点确认

1. 用户选择地图并求解。
2. 发车键按下后先进入 ART 起点确认。
3. 连续多帧 `C == start_row/start_col` 才真正启动 executor。

验收：

```text
人挡屏幕或 C 不在起点时，小车不动。
C 稳定回到起点并按发车后，小车才开始执行。
```

### 阶段 3：段末 ART 校正

1. waypoint 到点并停稳后等待 ART 新帧。
2. ART 与目标格一致时 reset 本地 pose。
3. ART 不一致或超时时暂停并显示错误。

验收：

```text
每段结束后 pose 被拉回目标格中心。
ART 仍在上一格时，小车不会继续下一段。
```

### 阶段 4：可选连续坐标/独立协议

如果完整地图解析延迟或协议不够清晰，再在 ART 端追加轻量行：

```text
ART_CELL,seq,row,col,valid
```

第一版不需要这个协议，因为现有地图帧已经包含 `C`。

## 风险与处理

### ART 延迟

ART 每 200ms 发一帧，且有稳定滤波。运动中不使用 ART 修正，只在停车后等待新帧。

### 人员干扰

发车前必须等待用户确认和起点连续稳定帧。上电后第一帧不作为有效基准。

### ART 误识别

要求唯一 `C`、新鲜帧、连续确认。段末不一致时暂停，不自动继续。

### MCU 惯导误差

ART 确认目标格后，重置 MCU pose 到目标格中心，消除累计误差。

### yaw 不一致

第一版不使用 ART 校正 yaw。yaw 继续由 IMU 维护。后续如果 ART 能可靠识别朝向，只做 yaw 偏差报警，不立即自动 reset。

## 推荐结论

第一版采用“ART 格点裁判，MCU 连续控制”的架构：

```text
发车前：ART 起点连续确认，防止人员干扰误启动。
运动中：MCU 惯导独立闭环，保证实时性。
段末：停车等待 ART 确认目标格，确认后 reset pose。
冲突时：暂停报错，不强行继续。
```

这比运动中连续融合更适合当前推箱子任务，也能直接解决“MCU 显示到了，但虚拟系统没到”的状态分叉问题。
