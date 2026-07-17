# 三级错误恢复实施计划

## 目标

基于当前 RT1064、OpenART #1、OpenART #2 状态机，把运行错误统一分为三级：

```text
第一类：精确视觉缺失，粗状态可信
-> 不进入 ERROR，直接降级继续

第二类：旧路径不可信，但新地图可能恢复
-> 短暂停车、获取新图、重新规划

第三类：控制基础不存在，或同一恢复键连续失败
-> 锁存停车，等待 K3 人工同步恢复
```

实现后，临时中心、yaw、车箱观察和地图抖动不再直接终止比赛；地图或对象冲突不会沿旧路径盲跑；真正 fatal 会保留 home、地图、分类和活动任务，允许现场修正后从当前上下文恢复。

## 设计依据

- [第一类可降级视觉错误设计](../specs/2026-07-16-art-recoverable-visual-errors-design.md)
- [第二类短暂停车同步恢复设计](../specs/2026-07-16-art-resync-recovery-errors-design.md)
- [第三类致命错误锁存与人工恢复设计](../specs/2026-07-16-fatal-error-handling-design.md)

## 当前仓库事实

### 现有终止行为

- `executor_set_error()` 会 `stop_motion()`、设置 `EXEC_STATE_ERROR` 并清除当前段状态。
- `subject2_fail()` 会取消 ART2 请求、调用 `executor_set_error()` 并进入 `SUBJECT2_ERROR`。
- `art_replan_center_timeout_error()` 和 `art_replan_return_error()` 会停车、取消 art_replan 并设置 executor 错误。
- Execute 页 K3 目前只处理 `EXEC_STATE_PAUSED`；`EXEC_STATE_ERROR` 下短按无动作。
- Execute 页 K4 和全局 K4 长按会调用 `subject2_cancel()`、`competition_flow_cancel()` 和 `executor_stop()`，属于完整人工退出路径。

### 已有恢复能力

- `ART_CENTER_TIMEOUT_FALLBACK_ENABLE` 默认开启，发车、初次中心、段末中心、转折中心和部分科目二中心已经存在降级分支。
- 科目一已有 `ART Retry`、`Host Sync`、`Push Retry`、稳定地图 tracker 和最新 B/T 基线。
- 科目二已有 `VSync`、`subject2_reconcile_object_lists()`、局部类别失效与重扫、动态同类 box-target 选择。
- 上位机 B/T 等量减少已经能够抢占旧 executor，并以最新地图重规划。
- 途经目标遮挡已有内部 `*` 归一化逻辑。
- OpenART 地图和中心按完整帧原子发布；请求样本带配套地图。

### 当前配置和帧率

- ART1 实测约 3FPS，约 333ms/帧。
- ART2 实测约 8FPS，约 125ms/帧。
- OpenART #1 使用 `STABLE_CHANGE_COUNT=3` 做逐格稳定滤波。
- MCU `EXEC_ART_STABLE_FRAMES=1`，恢复阶段无需再固定等待多张相同地图。
- `EXEC_ART_SYNC_TIMEOUT_MS=5000ms` 当前同时承担多种语义。
- `ART_BOX_OBSERVE_WAIT_MS=1500ms`，车箱观察需要 3 个样本并允许一次 5cm 恢复重试。

### 必须保留的上下文

- `subject2_cancel()` 会清空对象数组、分类结果、活动箱/目标和扫描状态，不能用于 fatal 锁存或 K3 恢复。
- `executor_stop()` 会清除 waypoint 指针并回到 IDLE，但不会清除外部保存的地图、home 和科目二对象数组。
- `competition_flow_cancel()` 会退出完整比赛阶段，不能用于 fatal 锁存。
- `art_replan_cancel()` 会清理当前重规划阶段和请求；fatal 原因与恢复入口必须在 cancel 之外保存。

### 当前能力边界

仓库目前没有完整的电机堵转、编码器断线或 IMU 离线诊断接口。本计划只对现有可观测的 executor 超时、非法输入、UART/地图、分类、规划和对象同步错误实施三级恢复；不在本次新增硬件故障检测算法。

## 总体实现约束

- 不新增通用 recovery `.c` 模块；科目一恢复留在 `art_replan.c`，科目二恢复留在 `subject2.c`。
- 不修改 OpenART UART 文本协议和两个 OpenART 脚本。
- 不修改 BFS、20cm 格距、PID、0.5cm 到点阈值和当前连续运动优化。
- 第一、第二类成功后 executor 不得遗留 `EXEC_STATE_ERROR`。
- 第二类只接受进入恢复状态之后发布的新完整地图。
- 第二类恢复期间不得继续旧 waypoint。
- `ART_CENTER_TIMEOUT_FALLBACK_ENABLE=1` 使用三级恢复；宏为 `0` 时保留严格调试行为。正式比赛配置必须为 `1`。

## 数据结构

### 科目一内部恢复上下文

在 `art_replan.c` 增加文件内私有结构，不新增公共模块：

```c
typedef enum {
    ART_RECOVERY_NONE = 0,
    ART_RECOVERY_CENTER_CONFLICT,
    ART_RECOVERY_BOX_CONFLICT,
    ART_RECOVERY_PREP_MOTION,
    ART_RECOVERY_MAP_TIMEOUT,
    ART_RECOVERY_PLAN,
    ART_RECOVERY_RETURN
} art_recovery_reason_enum;

typedef struct {
    art_recovery_reason_enum reason;
    uint8 count;
    uint32 start_frame;
    uint32 start_ms;
    art_replan_phase_enum resume_phase;
    uint8 baseline_box_count;
    uint8 baseline_target_count;
} art_recovery_context_struct;
```

恢复键由 `resume_phase + reason + B/T 确认基线` 组成。B/T 等量减少后自然形成新任务键，避免不同单箱任务误共享失败计数。

### 科目二内部恢复上下文

在 `subject2.c` 增加文件内私有结构：

```c
typedef enum {
    SUBJECT2_RECOVERY_NONE = 0,
    SUBJECT2_RECOVERY_CENTER_CONFLICT,
    SUBJECT2_RECOVERY_MAP,
    SUBJECT2_RECOVERY_TRACK,
    SUBJECT2_RECOVERY_OBSERVATION,
    SUBJECT2_RECOVERY_CLASS_SET,
    SUBJECT2_RECOVERY_PLAN,
    SUBJECT2_RECOVERY_CONFIRM
} subject2_recovery_reason_enum;

typedef struct {
    subject2_recovery_reason_enum reason;
    uint8 count;
    uint32 start_frame;
    uint32 start_ms;
    uint16 object_cell;
    uint16 target_cell;
    uint8 observation_bit;
} subject2_recovery_context_struct;
```

恢复键使用：

```text
reason + object_cell + target_cell + observation_bit
```

字段不适用时使用现有无效值。换对象、目标或观察方向后不能继承旧计数。

### 配置

在 `drive_config.h` 增加最少两个公共参数：

```c
#define ART_RECOVERY_TIMEOUT_MS      (3000u)
#define ART_RECOVERY_MAX_ATTEMPTS    (2u)
```

含义为最多执行两轮自动恢复；第三次相同恢复键失败时进入 fatal。运动动作本身继续使用现有 5000ms 超时。

## Task 1：建立恢复计数和 fatal 基础

**文件**

- `project/user/inc/executor.h`
- `project/user/inc/art_replan.h`
- `project/user/inc/subject2.h`
- `project/user/inc/drive_config.h`
- `project/user/src/art_replan.c`
- `project/user/src/subject2.c`
- `tests/art_replan_requested_center_test.c`
- `tests/subject2_scan_test.c`

### 1.1 先写失败测试

- 同一恢复键第一次、第二次失败不进入 executor ERROR。
- 第三次相同失败进入 fatal。
- 合法新地图、完成 waypoint、B/T 等量减少后计数清零。
- 换对象、目标或观察方向后使用新恢复键，旧计数不继承。
- 第一轮还在等待新图时重复 tick 不得重复增加计数。

### 1.2 实现私有恢复上下文

- 计数只在一轮恢复实际失败时增加，不能在每次主循环 tick 增加。
- 成功事件通过单一 helper 清零，避免不同分支遗漏。
- `subject2_begin()` 和新一轮 `art_replan_begin_initial()` 清空全部恢复上下文。
- K4 完整取消继续使用现有 cancel 路径。

### 1.3 fatal 入口

增加模块内部 fatal helper：

```text
art_replan_enter_fatal(...)
subject2_enter_fatal(...)
```

helper 负责取消当前视觉请求、设置明确 `F:*` 状态并调用 `executor_set_error()`；不得调用 `subject2_cancel()` 或 `competition_flow_cancel()`。

## Task 2：实现第一类科目一降级

**文件**

- `project/user/src/art_replan.c`
- `project/user/src/executor.c`
- `project/user/inc/executor.h`
- `tests/art_replan_requested_center_test.c`
- `tests/executor_pre_push_test.c`

### 2.1 中心和 yaw

- 发车中心保持固定 30cm fallback。
- `ICtr/RCtr` 无中心时继续使用地图 C 格中心和零 offset。
- `PCtr` 无中心但地图 C 仍位于 executor 预期格时，释放等待继续。
- 中心样本与合法地图冲突时，不把中心写入 pose；转入第二类 ART Sync。
- 发车 `YawFix` 超时改为锁定当前 MCU yaw、重建本地基准并继续，不进入 `E:Yaw`。

### 2.2 车箱观察缺失

新增只读 executor 查询，返回当前预期玩家格和请求箱子格。`art_replan` 使用最新合法地图检查：

```text
C 位于预期玩家格
B 位于请求箱子格
B/T 结构合法
```

三项满足但收不到 `OBSERVE_SAMPLE` 时，跳过精确 `BGap/BAlign/BNear`，调用现有 continue 接口继续格点推箱。

粗格检查必须使用 CENTER/OBSERVE 请求开始之后发布的新完整地图，不能用请求前的旧快照放行。请求开始时记录 frame count；fallback 地图的 frame count 必须更大。

已经收到可信样本且显示几何冲突时不得走本分支，交给第二类同步。

### 2.3 测试

- 发车 yaw 超时继续进入发车移动或 WMAP。
- PCtr 中心缺失且粗格匹配时继续同一 waypoint。
- 粗格不匹配时不放行旧 waypoint。
- BObs 缺失且 C/B 粗格匹配时继续推箱。
- BObs 可信冲突时进入 ART Sync。

## Task 3：实现第一类科目二降级

**文件**

- `project/user/src/subject2.c`
- `project/user/src/drive_control.c`（仅在现有接口不足时增加“锁定当前 yaw”薄接口）
- `project/user/inc/drive_control.h`
- `tests/subject2_scan_test.c`

### 3.1 中心

- `VCtr` 收不到中心时保留 MCU pose，跳过 `VAdj` 并进入 `VTurn`。
- `PCtr` 无中心但地图 C 位于预期格时继续当前 waypoint。
- `RCtr` 无中心继续使用地图 C 格中心。
- `CRef` 且字符地图合法时丢弃精确中心，根据阶段选择直接降级或第二类 `VSync`。
- `VAdj/CTim` 检查当前位置到目标的 X/Y 误差：均不超过 `SUBJECT2_FAST_CENTER_TOLERANCE_CM` 时取消修正并继续；否则进入第二类。

取消 standalone 位置修正使用 `executor_stop()` 回到 IDLE，再读取并保留当前 `drive_pose`；不能让已超时的位置修正继续在 PIT 中运行。

### 3.2 yaw

- `VTurn` 超时且 MCU yaw 误差不超过 2deg 时进入分类。
- `VTurn` 超时且误差超过 2deg 时标记当前观察方向失败，返回观察规划。
- `HYaw/VFix` 超时时锁定当前 MCU yaw、清除旧目标角并继续选择推箱。
- ART yaw 样本无效继续保持现有跳过行为。

2deg 作为文件内固定恢复判定，不增加现场配置；它只用于超时后的降级，不改变正常 `0.5deg + 100ms` 转向判定。

### 3.3 车箱观察

- `OBSERVE_SAMPLE` 缺失但最新地图 C/B 与预期一致时继续格点推箱。
- 可信样本显示车箱冲突时进入第二类 `VSync`。
- fallback 不能修改对象分类和活动目标。
- 和科目一相同，粗格 fallback 只能使用 OBSERVE 请求开始之后的新完整地图。

### 3.4 测试

- 覆盖 VCtr、CTim 小误差、VTurn 两种 yaw 误差、HYaw、VFix 和 BObs 两类结果。
- 所有第一类路径断言不进入 `EXEC_STATE_ERROR` 或 `SUBJECT2_ERROR`。

## Task 4：实现科目一第二类 ART Sync

**文件**

- `project/user/src/art_replan.c`
- `project/user/inc/art_replan.h`
- `tests/art_replan_requested_center_test.c`

### 4.1 恢复入口

统一 helper 完成：

```text
executor_stop()
-> 记录 reason/resume_phase/start_frame/start_ms
-> 丢弃半帧和旧 UART 数据
-> 重置 map_stability_tracker
-> 进入现有 INITIAL 或 SEGMENT 等图阶段
```

同一轮恢复只能初始化一次。

### 4.2 新地图解释

按以下顺序复用现有逻辑：

1. B/T 等量减少：走 `Host Sync`，更新基线并重规划。
2. 少一个 B 且可解释为途经目标：使用现有 `*` 归一化。
3. B/T 正常但 C 改变：以新 C 为起点重规划。
4. 结构无法解释：继续等下一张新图。
5. 3 秒没有合法图：本轮恢复失败并增加计数。

### 4.3 接入点

- 中心和地图冲突。
- 可信车箱几何冲突。
- `BGap/BAlign/BNear/BRetry` 动作超时或偏差过大。
- 返航中心冲突先重新进入 `RetCtr`，不立即 `RetErr`。

### 4.4 求解失败

第一轮新地图 `solve_map()` 失败继续 `ART Retry`；第二次相同恢复键失败后尝试最后一张新图；第三次进入 `F:Plan`。不能每次 `ART Retry` 重启时清零计数。

## Task 5：实现科目二第二类 VSync

**文件**

- `project/user/src/subject2.c`
- `project/user/src/subject2_logic.c`
- `project/user/inc/subject2_logic.h`
- `tests/subject2_scan_test.c`
- `tests/subject2_logic_test.c`

### 5.1 复用现有 VSync

扩展现有 `subject2_begin_scan_map_sync()`，允许携带 recovery reason 和恢复后的目标状态。它继续负责：

- 停止旧 executor。
- 取消当前 ART2 分类请求。
- 等进入状态后的新 ART1 地图。
- 调用 `subject2_reconcile_object_lists()`。
- 必要时进入局部 BScan/TScan。

不能清空已经确认的对象分类。

### 5.2 接入点

- `CRef`。
- `CTim` 且剩余误差超过 2cm。
- `Track` 对象协调歧义。
- `ATim` 确认地图超时。
- 可信 BObs 几何冲突。
- `VPos` 首次找不到观察路径。
- `Plan` 首次没有可解同类组合。
- `BSet` 类别数量不一致。
- `VMod` 等待 ART2 READY 超时。

### 5.3 分支行为

- `Track` 只让有歧义对象进入局部重扫。
- `ATim` 第一次只重启确认 tracker；第二次才完整 VSync。
- `VPos` 新地图仍不可解时换对象或观察方向；同恢复键第三次才 fatal。
- `Plan` 新地图后重新枚举所有同类 box-target；换具体组合后使用新恢复键。
- `BSet` 只失效数量异常类别，保持已经匹配类别。
- `VMod` 第一次失败重新发送模式并重置本轮 READY 计时；第二轮失败计入相同恢复键；第三次进入 `F:ART2`。它不需要等待 ART1 地图，但复用相同的恢复计数规则。

### 5.4 防循环测试

- 重复 tick 不增加 count。
- 相同坏地图帧不能重复触发恢复成功。
- 一张新图被拒绝后继续等下一帧，不重新初始化 3 秒计时。
- 局部重扫成功、B/T 减少或 waypoint 完成后计数清零。

## Task 6：实现第三类 fatal 和 K3 人工恢复

**文件**

- `project/user/inc/executor.h`
- `project/user/inc/art_replan.h`
- `project/user/inc/subject2.h`
- `project/user/src/art_replan.c`
- `project/user/src/subject2.c`
- `project/user/src/menu.c`
- `project/user/src/screen.c`
- `tests/competition_flow_test.c`
- `tests/menu_fatal_recovery_test.c`（新增）
- `tests/run_menu_fatal_recovery_test.ps1`（新增）
- `tests/art_replan_requested_center_test.c`
- `tests/subject2_scan_test.c`

### 6.1 fatal 原因

扩展错误显示所需枚举，至少覆盖：

```text
F:Drive
F:ART1
F:ART2
F:Map
F:Plan
F:Class
F:Track
F:Return
```

同时补齐现有 `EXEC_ERROR_SUBJECT2_PLAN` 的屏幕映射，避免 `E:?`。

### 6.2 锁存

fatal helper：

- 停止 executor 并取消当前视觉请求。
- 保留 art_replan home、最后地图 snapshot、subject2 对象数组和活动任务。
- 不调用 `subject2_cancel()` 或 `competition_flow_cancel()`。
- 设置模块 fatal 标志和恢复入口。
- 主循环继续解析 UART、刷新屏幕和扫描按键。

### 6.3 K3

为 Execute 页 `EXEC_STATE_ERROR + K3_SHORT` 增加人工恢复：

- 科目二活动时调用 `subject2_manual_recover()`，清除 fatal 锁存并进入 `VSync`。
- 科目一或返航时调用 `art_replan_manual_recover()`，从保存的 resume phase 进入 `ART Sync/RetCtr`。
- 两条路径都先 `executor_stop()`，不能 resume 旧 waypoint。
- 人工恢复清除自动恢复计数，但保留比赛上下文。

### 6.4 K4

Execute 页 K4、全局 K4 长按继续使用当前完整取消路径。新增 K3 逻辑不得改变 K4 优先级，也不得让 fatal 状态自动解除急停。

### 6.5 测试

- fatal 时电机停止且上下文保留。
- K3 从新地图同步开始，不恢复旧 waypoint。
- K4 清除比赛上下文并回到现有退出状态。
- `F:Drive/F:ART1/F:Map` 不触发自动返航。
- 科目二 fatal K3 后保留已识别分类和 active target。
- Execute 页 K3 在 ERROR 时调用人工恢复入口；PAUSED 时仍只执行原 `executor_resume()`。
- Execute 页 K4 和全局 K4 长按仍走完整取消，不受 K3 fatal 恢复影响。

## Task 7：显示与诊断收尾

**文件**

- `project/user/inc/screen.h`
- `project/user/src/menu.c`
- `project/user/src/screen.c`
- `docs/agents/file-map.md`（仅当新增公共接口职责需要说明时）

- Execute 页继续使用现有 `S:`，不增加新页面和大量调试字段。
- 第一类短暂显示 `CtrSkip`、`YawKeep`、`GridPush`。
- 第二类显示 `ART Sync`、`VSync`、`Replan`、`Rescan`、`Push Retry`。
- 第三类显示具体 `F:*`，并保留当前格、活动任务和地图。
- 不在 `screen.c` 直接读取业务模块；`menu.c` 继续构造 view。

## Task 8：验证

### 对应主机测试

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_executor_position_correction_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_competition_flow_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_menu_fatal_recovery_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
```

### 全量验证

```powershell
Get-ChildItem tests -Filter "run_*.ps1" | ForEach-Object {
    powershell -ExecutionPolicy Bypass -File $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
git diff --check
```

Keil 必须为 `0 Error(s), 0 Warning(s)`。本计划不下载固件。

### 上板验收

- ART1 中心缺失时第一类路径继续，地图冲突时第二类停车重算。
- ART1 断续恢复时不会在每个 tick 累加恢复次数。
- ART2 分类失败会换观察方向或局部重扫，不立即终止。
- 上位机 B/T 等量减少始终能抢占旧路径并重规划。
- 相同恢复键前两次自动恢复，第三次显示对应 `F:*`。
- fatal 下 K3 保留当前任务并从新图同步，K4 完整退出。
- 无合法地图或底盘执行错误时不会盲目返航。

## 实施顺序和检查点

1. Task 1：先建立恢复计数和 fatal 基础，所有现有行为暂不改变。
2. Task 2～3：只接入第一类，验证正常流程和超时降级。
3. Task 4～5：接入第二类，重点验证不沿旧路径和不无限循环。
4. Task 6：最后把剩余终止点收敛到第三类并接入 K3。
5. Task 7～8：统一显示、全量测试和 Keil 编译。

每个检查点独立保持主机测试通过，不允许一次性替换所有错误分支后再统一排错。

## 明确不做

- 不在本次优化 ART1/ART2 帧率、分类模型或样本数量。
- 不新增电机电流、堵转、编码器断线或 IMU 离线诊断。
- 不增加自动 SLAM、全局多箱最优搜索或新的返航算法。
- 不改变当前发车、路径连续运动和上位机完成优先规则。
- 不删除严格调试宏和现有离线地图模式。
