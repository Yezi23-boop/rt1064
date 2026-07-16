# 科目三按需单次炸墙设计

## 1. 目标与规则

科目三复用当前科目二的观察、分类、绑定和指定推箱流程，只在现有规划器确认当前任务无路时，尝试用一颗炸弹炸开一次内部墙。第一版追求结构简单和上板可诊断，不提前分析全部任务，不在同一次阻塞中搜索多次爆炸序列，也不为了缩短已有可行路径主动炸墙。

已确认的比赛规则：

- `X` 是可推动炸弹，`#` 是墙。
- 炸弹被推入内部墙格时爆炸，随后炸弹自身消失。
- 爆炸以被推入的墙格为中心，删除 `3x3` 范围内的内部墙。
- 爆炸不删除小车、箱子、目标点或其他炸弹。
- 地图最外围一圈墙不能被炸，也不能作为爆炸中心。
- 一张地图可以有多颗炸弹；每颗炸弹爆炸后消失，因此每颗只能使用一次。
- `X` 不会与 `T` 重叠，协议不需要炸弹站在目标上的复合字符。

完整比赛流程改为：

```text
科目一 -> 返航
-> 科目二 -> 返航
-> 科目三 -> 返航
-> Done
```

同时增加独立的 `SUBJECT3_DEBUG`，用于只跑科目三。

## 2. 当前代码基础与约束

当前仓库已经具备以下基础：

- `openmv/main_see.py` 已识别红色炸弹并输出 `X`。
- `openart_uart.c` 已接受 `X`，无需修改 UART 文本协议。
- `solve_navigation_path()` 已把 `#`、`X`、`B` 视为不可通行。
- `solve_bound_box_path()` 已能检查指定箱子到指定目标是否可解。
- `subject2_select_observation()` 已负责选择当前最短可达观察位。
- `subject2_select_push_plan()` 已负责从有效绑定中选择当前可执行推箱任务。
- `map_stability_tracker_struct`、请求式中心采样和现有 pose offset 可用于爆炸后地图确认与重定位。

需要保持的边界：

- `solver.c::map_load()` 当前把 `X` 当墙，这对普通箱子求解是正确的，不能为了科目三改掉。
- executor 把大写动作视为黄色箱子推动，会触发 `OBSERVE_REQ`、车箱二维准备位和 `B/T` 段末确认。
- 炸弹不是黄色箱子，因此炸弹执行路径不能直接以普通大写推箱任务交给 executor。
- 当前 `competition_flow` 只有科目一和科目二，`FULL` 在科目二返航后立即结束。
- 当前 `subject2` 在观察规划或推箱规划失败时直接进入计划恢复；科目三需要在这两个失败点之前获得一次炸墙机会。

## 3. 方案选择

采用“现有规划失败后按需接管”，不采用旧版“分类前和分类后预扫描全部任务”。

科目二现有两个规划失败点分别是：

```text
subject2_select_observation() == 0
subject2_select_push_plan() == 0
```

科目三模式下，这两个位置不立刻报 `F:Plan`，而是发布一次炸墙请求并暂停 `subject2`。科目三枚举单次爆炸候选；如果某个候选能让原来的同一个规划函数成功，就执行代价最小的候选。若没有任何单次爆炸能恢复当前任务，再进入现有计划错误恢复。

这样有三个好处：

- 不重复编写“观察位是否可达”和“绑定任务是否可推”的判断。
- 不需要在分类完成处增加额外的全图预检查阶段。
- 炸墙只处理已经真实发生的阻塞，状态和错误原因更容易上板复现。

## 4. 模块结构

### 4.1 `subject3.c/.h`

新增科目三外层状态机，内部复用 `subject2`：

```text
S3_IDLE
-> S3_RUN_SUBJECT2
-> S3_SELECT_BLAST
-> S3_EXECUTE_BOMB
-> S3_CONFIRM_BLAST
-> S3_RUN_SUBJECT2
-> S3_RETURN_REQUESTED
-> S3_ERROR / S3_DONE
```

职责：

- 调用 `subject2_begin()` / `subject2_tick()`。
- 转发科目二的状态文本、执行页进入请求和返航请求。
- 消费 `subject2` 发布的当前规划阻塞事件。
- 选择一颗炸弹和一个内部墙格。
- 启动并监控炸弹 waypoint。
- 爆炸后确认真实 ART 地图并恢复 `subject2`。
- 按当前三级错误体系提供人工恢复入口。

`subject3` 不复制分类器、绑定表或推箱确认逻辑。

### 4.2 `subject2` 最小扩展

增加科目三专用的可选阻塞出口。普通科目二行为保持不变。

建议接口语义：

```c
typedef enum {
    SUBJECT2_BLOCK_NONE = 0,
    SUBJECT2_BLOCK_OBSERVE_BOX,
    SUBJECT2_BLOCK_OBSERVE_TARGET,
    SUBJECT2_BLOCK_PUSH
} subject2_block_reason_enum;
```

- `subject2_context_struct` 增加 `allow_blast_fallback`。
- 为 `0` 时，规划失败继续走当前恢复/致命错误路径。
- 为 `1` 时，规划失败进入内部 `WAIT_BLAST`，通过 update 发布阻塞原因。
- 增加“在模拟地图上重试当前阻塞规划”的查询接口。
- 增加“爆炸确认后用最新地图和 pose offset 恢复原规划状态”的接口。
- 观察阻塞恢复到原 `SCAN_BOX_PLAN` 或 `SCAN_TARGET_PLAN`。
- 推箱阻塞恢复到 `SELECT_PUSH`。

模拟地图检查必须调用现有 `subject2_select_observation()` 或 `subject2_select_push_plan()`，不能另写一套近似可达性判断。

这些接口必须满足无副作用契约：

- 进入等待时记录原阻塞状态和对象类型。
- 模拟查询只写调用者提供的临时 plan/result。
- 不修改正式 `current_observation`、`context->result`、分类数组、`tried_observation_mask`、active box/target、`retry_active_only` 或恢复计数。
- 爆炸成功后只替换真实地图快照和 pose offset，再恢复原规划状态。
- 无有效候选时先恢复原阻塞状态，再进入现有计划恢复入口。

### 4.3 `solver.c/.h`

新增单炸弹路径接口，不改变普通 `map_load()`：

```c
uint8 solve_bomb_path(const map_source_struct *source,
                      uint16 bomb_cell,
                      uint16 blast_wall_cell,
                      solve_result_struct *result,
                      uint16 *push_count,
                      uint16 *turn_count,
                      uint16 *final_car_cell);
```

炸弹 BFS 状态仍为：

```text
state = player_cell + selected_bomb_cell
```

搜索规则：

- 选中的 `bomb_cell` 必须是 `X`。
- `blast_wall_cell` 必须是非外圈 `#`。
- 其他 `X`、所有 `B` 和所有普通墙均为障碍。
- `.`、`T`、唯一 `C/+` 可供小车通行。
- 炸弹不能被推到 `T`，因为比赛地图不允许 `X/T` 重叠。
- 选中炸弹只能推，不能拉。
- 唯一允许炸弹进入的墙是最终爆炸墙。
- 终止条件是最后一次推动把炸弹推入该墙，输出包含这一步。

复用现有 BFS 静态缓冲和 waypoint 结构，不在 ISR 中求解。

### 4.4 科目三纯逻辑

候选模拟、确认和评分放在可主机测试的科目三逻辑模块中；只提供本次运行需要的函数，不建立通用搜索框架。

职责：

- 枚举地图中的 `X` 和非外圈 `#`。
- 构造完整的预期爆炸后地图。
- 统计 `X/B/T` 和检查外圈墙。
- 按固定规则比较两个单次爆炸候选。

## 5. 单次炸点选择

当 `subject2` 发布阻塞事件后，枚举每个“炸弹 + 内部墙”组合：

1. 调用 `solve_bomb_path()`；炸弹无法推入该墙则淘汰。
2. 在独立的 12x16 行缓存中构造完整的预期爆炸后地图。
3. 使用 `subject2` 当前阻塞规划查询接口，在模拟地图上重试原任务。
4. 原任务仍不可解则淘汰。
5. 保留有效候选并计算简单代价。

第一版代价固定为：

```text
cost = bomb_path.action_count + recovered_task_path.action_count
```

代价相同时依次比较：

1. 炸弹推动次数更少。
2. 炸弹路径转折更少。
3. 炸弹 cell 更小。
4. 爆炸墙 cell 更小。

不引入浮点时间权重，也不标定 ART 等待时间。每个候选都只包含一次爆炸，固定等待成本不会影响同轮候选排序。

如果没有有效候选，科目三让 `subject2` 回到现有计划恢复流程，最终仍使用 `COMPETITION_FATAL_PLAN`，不盲目推动炸弹。

预期爆炸后地图必须包含实际状态变化，不能只删除墙：

- 恢复旧 `C/+` 所覆盖的底层字符。
- 删除本次使用的炸弹 `X`。
- 按 `solve_bomb_path()` 返回的 `final_car_cell` 放置新的唯一 `C/+`。
- 删除爆炸中心 `3x3` 内的内部墙。
- 保持其他 `X/B/T` 和爆炸区外墙体不变。

模拟地图只用于判断候选是否值得执行以及计算代价；真实爆炸后绝不发布该模拟地图。

## 6. 炸弹路径如何交给 executor

炸弹 BFS 内部可以用“大写表示推弹”区分搜索动作，但写入最终 `solve_result_struct` 时统一转换为小写方向。真实底盘只需要走到对应格点，上位机根据虚拟小车位置自动完成炸弹推动。

这样可避免：

- 把红色炸弹误当黄色箱子发送 `OBSERVE_REQ`。
- 启动车箱二维安全准备位。
- 进入普通箱子的 `B/T` 减少确认。

第一步推弹对应的 waypoint 单独设置 `center_correct_before=1`，确保它不会与前面的普通直线段合并。炸弹执行时调用：

```text
executor_start(..., art_sync=1)
```

原因是当前 executor 只有 `art_sync_enabled` 时才处理 waypoint 前中心请求。炸弹结果中所有 `task_end=0`，因此不会触发普通箱子段末 ART 同步。

`subject3` 收到 `executor_art_pre_push_pending()` 后自己管理一次普通 waypoint 中心会话：使用现有 `art_center_batch_struct` 发送 `CENTER_REQ`、收集中值、取得请求样本配套地图、校验唯一 `C/+` 后提交 pose，并调用 `executor_continue_after_pre_push_center()`。由于当前及后续动作均为小写，`executor_get_pre_push_box_request()` 返回 0，不会进入箱子观察。menu 在 `subject3_active` 时不会调用 `art_replan_tick()`，因此不能把这个 pending 留给 art_replan 隐式处理。

Run 和 Step 模式继续使用 executor 现有语义。炸弹路径执行完成后，executor 进入 `DONE`，由 `subject3` 开始爆炸确认。

## 7. 爆炸确认与重定位

开始推弹前保存：

- 完整 ART 地图快照。
- 炸弹数量、箱子数量和目标数量。
- 爆炸中心及其 `3x3` 内原有内部墙集合。
- 外圈墙快照。

最终推动完成后停车，复用 `map_stability_tracker_struct` 等待 2 张连续一致的新地图。新增科目三专用常量：

```c
#define SUBJECT3_BLAST_STABLE_FRAMES (2u)
```

不复用当前值为 1 的 `EXEC_ART_STABLE_FRAMES`。确认条件全部满足才接受：

- 地图仍只有一个 `C/+`。
- `X` 数量恰好减少 1。
- 新地图中的 `X` 位置集合等于爆炸前集合删除本次选中炸弹，其他炸弹位置不变。
- 爆炸前位于预期 `3x3` 内的内部墙均不再是 `#`。
- 外圈墙与爆炸前一致。
- `B`、`T` 的位置集合不变。

上述条件只用于确认爆炸事件和地图基本完整性。MCU 预测的 `final_car_cell` 只作诊断，不作为拒绝 ART 地图的硬条件；爆炸区外其他内部格也不拿模拟地图覆盖。校验通过后，以这张 ART 新地图作为唯一正式状态。

稳定地图自带的 `PLAYER_CENTER_GRID` 有效且通过当前 `C/+` 格校验时，直接计算新的 pose offset；无有效中心时发送一次 `CENTER_REQ`，按当前中心批处理和超时策略取得配套地图。请求式中心的配套地图必须与已经接受的稳定爆炸地图一致；若不一致，则丢弃中心结果并回到 `S3_CONFIRM_BLAST` 重新等待 2 张一致新图，不能借中心请求绕过爆炸确认。确认成功后：

1. 用真实 ART 地图覆盖 subject2 快照。
2. 用最新中心相对当前 `C/+` 格中心的偏移更新 subject2 pose offset。
3. 丢弃爆炸前的炸弹路径、模拟地图和候选结果。
4. 恢复原观察规划或推箱选择状态。

不能把 MCU 模拟爆炸后的地图当成真实地图继续执行。

## 8. 比赛流程接入

保留当前数值，避免改变已有调试配置含义：

```c
#define COMPETITION_MODE_SUBJECT1_DEBUG (1u)
#define COMPETITION_MODE_SUBJECT2_DEBUG (2u)
#define COMPETITION_MODE_FULL           (3u)
#define COMPETITION_MODE_SUBJECT3_DEBUG (4u)
```

`competition_flow` 增加：

- `COMPETITION_STAGE_SUBJECT3`
- `COMPETITION_ACTION_START_SUBJECT3`

阶段切换：

```text
SUBJECT1_DEBUG: S1 -> Finish
SUBJECT2_DEBUG: S2 -> Finish
SUBJECT3_DEBUG: S3 -> Finish
FULL: S1 -> S2 -> S3 -> Finish
```

科目三和科目二使用相同的发车、等真实地图和返航入口。art_replan 不需要理解 S2/S3 业务差异，只需把当前 `art_launch_subject2` / `subject2_map_ready` 改成通用的“分类科目地图 ready”语义；menu 再根据 `competition_flow_get_subject()` 启动 subject2 或 subject3。

menu 增加 `subject3_active` 分支：

- `START_SUBJECT3` 进入科目三发车。
- S3 地图 ready 后调用 `subject3_begin()`。
- 主循环由 `subject3_tick()` 接管并转发界面 update。
- 科目三请求返航时继续调用现有 `art_replan_begin_return_home()`。
- 返航完成后通知 `competition_flow_on_return_complete()`。
- K4、fatal 锁存和人工恢复同时取消/处理 `subject3`。

## 9. 状态所有权与恢复

科目三包装 subject2 后，状态所有权固定如下：

| 当前阶段 | 自动恢复所有者 | K3 人工恢复 | 取消范围 |
|---|---|---|---|
| `S3_RUN_SUBJECT2` | subject2 | `subject3_manual_recover()` 委托 `subject2_manual_recover()` | subject3 同时取消 subject2/vision/executor |
| `S3_SELECT_BLAST` | subject3 | 重新读取最新 ART 地图并重新选点 | 清除候选和临时模拟图 |
| `S3_EXECUTE_BOMB` | subject3/executor | 停车读取稳定 ART 新图，禁止回放旧路径 | 停止 executor 并清中心会话 |
| `S3_CONFIRM_BLAST` / `S3Ctr` | subject3 | 重新等待地图或中心，不得另选炸点 | 保留已发生爆炸事实并清等待器 |

推弹执行阶段发生异常时，不区分当前停在普通移动、推弹中途还是最终推动附近。统一停车并读取 2 张一致的 ART 新图：

- 若新图已满足预期爆炸特征，转入 `S3_CONFIRM_BLAST` 完成确认。
- 若炸弹仍存在且地图结构有效，丢弃全部旧 waypoint，以该 ART 新图重新执行当前阻塞的一层单爆炸选点。
- 若 `X/B/T` 或墙体变化无法解释，进入 ART/MAP 错误。

不得从旧 waypoint 下标继续，也不得重新播放旧炸弹路径。最终推动一旦确认已经发生，只能恢复爆炸确认或中心阶段，不能再选择另一颗炸弹。每次确认成功后回到 subject2；后续出现新的独立阻塞时，可以从 ART 新地图中剩余的 `X` 再做一次单层贪心选择，不设置整场 `blast_used` 锁。

检测 fatal 前，subject3 先保存可恢复阶段和阻塞原因，再锁存当前已有的 fatal 分类。K4、长按退出、重新开始和返航交接统一取消 subject3、subject2、vision、art_replan 和 executor，不能留下嵌套业务状态。

## 10. 状态显示与错误

不新增屏幕页面，只复用 Execute 页状态字符串：

```text
S3Run   科目二观察/分类/推箱流程正常运行
S3Plan  正在枚举单次爆炸候选
S3Bomb  正在执行炸弹路径
S3Wait  等待爆炸后稳定地图
S3Ctr   爆炸后请求精确中心
```

错误沿用当前 fatal 分类：

- 无有效单次爆炸候选：`COMPETITION_FATAL_PLAN`。
- 炸弹 executor 失败或底盘不健康：`COMPETITION_FATAL_DRIVE`。
- 爆炸确认超时、地图不符合预期：`COMPETITION_FATAL_ART1` 或 `COMPETITION_FATAL_MAP`。
- 人工 K3 恢复时只重做当前阶段：重新选炸点、重新等待地图或重新请求中心，不跳过确认继续运行。

普通科目一、科目二的错误恢复行为不变。

## 11. 测试与验收

### Solver 主机测试

- 直线推弹进入内部墙。
- 小车绕到炸弹后方再推。
- 普通箱子和其他炸弹均阻挡路径。
- 非目标墙不可进入，外圈墙不可作为爆炸中心。
- 输出最终 car cell 正确，包含最后一次推动。
- 最终执行动作全部为小写，第一步推弹 waypoint 独立且带 `center_correct_before`。

### 科目三逻辑测试

- `3x3` 只删除内部 `#`，不删除未使用的 `B/T/X`。
- 完整模拟会删除选中 `X`、移动唯一 `C/+`，并保持其他对象不变。
- 外圈墙保持不变。
- 只保留能让原阻塞规划成功的候选。
- 按 action 总数、推弹数、转折和 cell 顺序确定性选优。
- 没有单次可行爆炸时返回失败。

### Subject2 接口测试

- 普通科目二规划失败仍进入原恢复流程。
- 科目三模式观察规划失败发布正确阻塞原因。
- 科目三模式推箱选择失败发布 `PUSH` 阻塞原因。
- 模拟地图重试调用原观察/推箱规划逻辑。
- 爆炸恢复后保留已识别类别、绑定和尝试记录，只刷新地图及 pose。

### 流程测试

- `SUBJECT3_DEBUG` 独立发车、分类、按需炸墙、推箱和返航。
- `FULL` 严格按 S1 -> S2 -> S3 -> Finish 切换。
- 当前任务可解时绝不使用炸弹。
- 推弹前只请求小车中心，不请求黄色箱子中心。
- 爆炸后 `X` 未减少、墙未消失、外圈变化或 `B/T` 变化均不能放行。
- 爆炸后其他 `X` 位置变化不能放行。
- 爆炸确认必须等待 2 张连续一致的新图。
- 请求式中心配套地图变化时回到爆炸确认，不能直接覆盖正式快照。
- 成功爆炸后允许后续独立阻塞使用剩余炸弹，但单次候选仍只模拟一次爆炸。
- 爆炸确认后使用真实新地图重新调用原阻塞规划。
- K4、fatal 和人工恢复不会留下后台 subject2/subject3/executor 状态。
- 推弹中断恢复必须丢弃旧路径，并根据稳定 ART 新图进入确认或重新选点。

### 完成验证

- 对应 GCC/Python 主机测试全部通过。
- `python -m py_compile openmv/main_see.py` 通过；OpenART 协议未变。
- Keil 编译为 `0 Error(s), 0 Warning(s)`。
- `git diff --check` 通过。

## 12. 第一版明确不做

- 不预先检查全部观察位和全部绑定任务。
- 不在同一次阻塞中搜索连续两次或更多爆炸序列。
- 不为了缩短已有可行路径使用炸弹。
- 不做浮点时间模型、ART 等待权重或全局最短比赛时间估计。
- 不识别炸弹精确中心，不做推弹过程视觉闭环。
- 不修改 OpenART UART 文本格式。
- 不改变普通 solver 中 `X` 作为障碍的语义。
