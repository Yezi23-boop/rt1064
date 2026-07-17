# ART 三类错误分级恢复实施计划

## 目标

基于当前仓库已有的 `art_replan`、`subject2`、`executor`、OpenART 原子地图和现有测试，建立三类错误处理：

```text
第一类：精确视觉缺失，粗地图和底盘仍可信
-> 直接降级继续

第二类：旧路径不可信，但新地图可能恢复
-> 短暂停车、同步、重规划或局部重扫

第三类：控制基础不存在，或同一问题连续恢复失败
-> 锁存停车，等待人工 K3 从新地图恢复
```

本计划只改变错误分级和恢复路径，不改变 BFS、20cm 格距、PID、地图字符语义和 OpenART 文本协议。

## 当前仓库事实

### 已有可复用能力

- `project/user/src/art_replan.c` 已有 `ART Retry`、`Host Sync`、`Push Retry`、`RCtr` 和返航状态。
- `project/user/src/subject2.c` 已有 `VSync`、对象协调、类别局部重扫、分类后退重试和推箱确认。
- `project/user/src/openart_uart.c` 已按 `MAP_BEGIN ... PLAYER_CENTER_GRID ... MAP_END` 原子发布完整地图。
- OpenART 已在端侧以 `STABLE_CHANGE_COUNT=3` 稳定字符地图；ART1 实测约3FPS，ART2实测约8FPS。
- `ART_CENTER_TIMEOUT_FALLBACK_ENABLE=1` 已使部分中心超时可以降级。
- `EXEC_ART_STABLE_FRAMES=1` 当前按一张已由OpenART稳定的合法地图接受。
- [executor.c](C:/Users/ye/Desktop/rt1064/project/user/src/executor.c) 已修复同方向近箱边界造成的额外停顿。

### 当前需要改变的行为

- [subject2.c](C:/Users/ye/Desktop/rt1064/project/user/src/subject2.c:163) 的 `subject2_fail()` 当前会直接进入 `SUBJECT2_ERROR`。
- [executor.c](C:/Users/ye/Desktop/rt1064/project/user/src/executor.c:687) 的 `executor_set_error()` 当前会立即 `stop_motion()` 并进入 `EXEC_STATE_ERROR`。
- 科目一的 `E:Ctr`、`E:BTim`、`E:BObs`、`RetErr` 仍会把可恢复问题变成永久停止。
- 科目二的 `E:CRef`、`E:CTim`、`E:Track`、`E:ATim`、`E:BObs`、`E:CPsh`、`E:Plan` 等仍大量直接调用 `subject2_fail()`。
- [screen.c](C:/Users/ye/Desktop/rt1064/project/user/src/screen.c:738) 没有 `EXEC_ERROR_SUBJECT2_PLAN` 的明确文字映射，可能显示 `E:?`。
- 当前 `EXEC_ART_SYNC_TIMEOUT_MS=5000ms` 同时承担中心、地图、位置修正和运动超时，后续需要按语义拆分或在状态内部使用更短恢复窗口。

## 不变量和安全边界

1. 只有第一类允许在旧路径上直接继续。
2. 第二类发现地图或对象冲突后必须停止旧 executor，不能沿旧 waypoint 盲跑。
3. 第三类硬件/IMU/执行器错误第一次发生即停车。
4. 精确中心无效时不能修改 ART 字符地图中的 `C/+`。
5. 精确车箱样本缺失可以降级；精确样本已经证明几何冲突时不能忽略。
6. B/T 等量减少以上位机虚拟推箱结果为准，优先更新基线和对象列表。
7. 地图、底盘或IMU不可信时不能自动返航。
8. 所有恢复成功都必须清除对应原因的恢复计数。

## 第一类共同安全门禁

所有“直接降级继续”的分支必须先通过同一个业务层检查，不能只因为超时就放行：

```text
can_fallback_to_grid()
```

检查内容：

- 当前地图帧必须是当前请求或当前状态之后的新完整帧。
- 地图必须恰好包含一个 `C/+`。
- 当前 B/C 和活动对象仍与任务上下文匹配。
- executor 不能已经处于 `EXEC_STATE_ERROR`。
- 底盘健康状态不能报告 IMU、编码器或电机硬错误。

门禁失败时必须进入第二类 `VSync/ART Sync`，不能继续旧任务。中心超时fallback、推箱前中心fallback和ART科目一推箱前fallback全部使用该门禁。

## 实施阶段

### 阶段0：建立当前基线

先不改行为，记录当前测试和编译基线：

- `tests/run_executor_pre_push_test.ps1`
- `tests/run_solver_navigation_test.ps1`
- `tests/run_subject2_scan_test.ps1`
- `tests/run_art_replan_requested_center_test.ps1`
- 全部 `tests/run_*.ps1`
- Keil `0 Error(s), 0 Warning(s)`
- `git diff --check`

新增测试必须先能复现“临时错误直接进入ERROR”的旧行为，再实现恢复。

### 阶段1：建立最小恢复原因和计数

不新建通用大型框架，分别在 `art_replan.c` 和 `subject2.c` 保存模块内恢复上下文：

```text
recovery_reason
recovery_count
recovery_start_frame
recovery_start_ms
recovery_phase
active_box/target 或 object_index（适用时）
```

恢复键必须由以下信息组成：

```text
业务阶段 + 错误原因 + 当前对象/活动箱子 + 当前目标 + 观察/推箱方向
```

规则：

- 第一次同原因：进入第一类或第二类恢复。
- 第二次同原因：更换观察方向、箱子、目标或执行局部重扫。
- 第三次完全相同：进入第三类。
- 取得合法新地图、完成 waypoint、B/T减少、成功分类或更换对象后清零。

不同箱子、不同目标和不同观察方向不能共用失败计数。

### 阶段2：实现第一类直接降级

#### 2.1 科目二中心

修改 `subject2.c`：

- `VCtr`：中心样本超时，保留 MCU encoder pose，跳过 `VAdj`，直接 `VTurn`。
- `PCtr`：地图 C 仍在预期 waypoint 时跳过视觉融合，释放当前路径；地图 C 已离开预期位置时转第二类 `VSync`。
- `ICtr/RCtr`：中心无效时使用最新合法地图 C 格中心，`offset=(0,0)`，继续求解。
- `CRef`：若地图结构合法，丢弃冲突的精确中心并按地图 C 继续；地图也非法时转第二类。
- `VAdj/CTim`：剩余误差不超过现有 `SUBJECT2_FAST_CENTER_TOLERANCE_CM=2.0f` 时取消修正并继续；超过2cm时转第二类。

不得因为中心缺失把本地 pose强制清零。中心缺失只表示精确格内offset不可用；直接放行前仍必须通过第一类共同安全门禁。

#### 2.2 yaw

修改 `subject2.c` 和 `art_replan.c`：

- 发车 yaw 样本或小角修正失败：锁定当前 MCU yaw，重建本地 yaw/pose 基准，继续发车。
- `VTurn` 超时：IMU误差不超过2deg时接受当前朝向并分类；超过2deg时标记当前观察方向失败，返回观察规划换方向。
- `HYaw/VFix` 超时：停止追踪旧目标角，锁定当前 MCU yaw并继续后续流程。

超过2deg不能强行读取图案；IMU本身失效直接进入第三类。

#### 2.3 精确车箱观察

修改 `subject2.c`、`art_replan.c` 和 `executor.c`：

- `OBSERVE_SAMPLE` 缺失，但新鲜地图中 C/B 仍在预期格：跳过 `BGap/BAlign/BNear` 精确准备，使用格点路径和 MCU pose继续推箱。
- 已收到可信车箱样本且证明车不在正确推箱侧、箱子离开请求格或几何方向冲突：转第二类，不得直接继续。
- 第一类缺失样本不再执行无意义的5cm恢复动作。
- 新增或复用 executor 的“释放推箱前等待、继续原 waypoint”接口，保证局部观察失败不会残留 `pre_push_center_waiting`。

#### 2.4 状态显示

第一类不进入 `E:*`，短暂显示：

```text
CtrSkip / YawKeep / GridPush
```

### 阶段3：实现第二类短暂停车同步

#### 3.1 统一入口

复用现有状态，不新建一套平行规划器：

- 科目一使用 `ART Sync`、`ART Retry`、`RCtr`、`Push Retry`。
- 科目二使用 `VSync`、`Replan`、`Rescan`、`Push Retry`。

进入第二类时：

```text
停止并取消旧 executor
记录进入时 frame_count
丢弃半帧和旧请求结果
只接受进入状态之后的新完整地图
```

ART1 已经端侧连续稳定3帧，MCU恢复阶段接受第一张结构合法的新地图，不再额外固定等待多张相同地图。

单次第二类恢复总窗口按ART1约3FPS设置为约3秒；超过后只升级恢复计数，不允许无限等待。

#### 3.2 新地图解释顺序

1. B/T等量减少：相信上位机完成事实，更新基线和对象列表，清除活动任务并重规划。
2. 少一个B、T未减少：检查途经目标遮挡条件，满足则内部规范化为 `*` 并重规划。
3. C移动但B/T结构正常：以新C为起点，有精确中心用offset，无中心使用0 offset，重新规划。
4. 对象类别/身份有歧义：保留能唯一恢复的对象，只局部重扫未识别对象。
5. B/T无法解释、C不唯一或地图字符非法：继续等待新图，不接受该快照。

#### 3.3 科目一入口

- `E:Ctr`、`E:BObs`、`E:BTim` 的可恢复情况转 `ART Sync`，新图后重新求解。
- B/T未变继续走已有 `Push Retry`。
- B/T已减少继续走已有 `Host Sync`。
- 返航中心冲突重新进入 `RetCtr`，不立即 `RetErr`。

#### 3.4 科目二入口

- `CRef`、大误差 `CTim`、`Track`、`ATim` 转 `VSync`。
- `VPos` 或 `Plan` 首次不可解，用新图重新尝试一次。
- `BSet` 只使数量异常类别失效，局部重扫。
- 可信车箱几何冲突停止旧推箱，`VSync` 后重新选择组合。

第二类恢复成功后必须重新启动 executor，不能恢复旧 waypoint 指针。

`VPos` 和 `Plan` 失败发生在需要 `subject2_context_struct`、活动箱子、活动目标和对象列表的调用路径中，不能直接调用没有上下文的通用 `subject2_fail()`。这些入口必须先保存上下文、停止executor、记录恢复帧号，再进入 `SUBJECT2_SCAN_MAP_SYNC`。

### 阶段4：实现第三类致命锁存和K3恢复

#### 4.1 致命条件

立即致命：

- executor输入或内部状态非法。
- IMU/编码器/电机控制基础不可用。
- K4急停。

升级致命：

- 同一恢复键第二类恢复失败两次后，第三次仍完全相同。
- 所有合法观察方向、箱子/目标组合或局部重扫都已失败。
- ART1/ART2持续不可用且无法取得任何合法数据。
- 返航两条允许出口均不可达。

#### 4.2 FATAL_WAIT

使用现有 `executor_set_error()` 完成最终安全停车，但增加比赛上下文锁存：

```text
停止电机
取消视觉请求
锁定当前yaw
保留最后合法地图、home、分类结果、活动任务和恢复键
进入 FATAL_WAIT
```

FATAL_WAIT 仍继续解析UART、刷新屏幕和扫描按键，但禁止自动恢复运动。

当前调用链必须显式接入该状态：

- `menu_poll()` 在 `subject2_active` 和 `art_replan` 分支之前优先检查 fatal latch。
- `subject2_tick()` 在 `SUBJECT2_ERROR` 时不能只返回，必须允许FATAL_WAIT处理按键和UART。
- Execute 页K3在 `EXEC_STATE_ERROR` 时不能调用普通 `executor_resume()`，而应触发业务层人工恢复入口。
- K3人工恢复必须进入 `VSync/ART Sync`，不能直接恢复旧waypoint。

显示改为具体原因：

```text
F:Drive / F:ART1 / F:ART2 / F:Map
F:Plan / F:Class / F:Track / F:Return
```

补齐 `screen.c` 中 `EXEC_ERROR_SUBJECT2_PLAN` 的文字映射，避免 `E:?`。

由于当前 `executor_error_enum` 只有通用错误，业务层必须增加独立的 `recovery_reason/fatal_reason`，由 `menu.c` 或业务 update 传入 Execute 页；不能只依赖 `executor_error_enum` 区分 `F:ART1`、`F:ART2`、`F:Map` 和 `F:Plan`。

executor一旦已经处于 `EXEC_STATE_ERROR`，业务层不能再次用泛化的 `EXEC_ERROR_SUBJECT2_CLASS` 覆盖原始错误。必须先读取并保存第一次executor错误，再决定是第二类恢复还是第三类锁存。

#### 4.3 K3人工恢复

修改 `menu.c`、`competition_flow.c` 及对应业务模块：

```text
FATAL_WAIT 按K3
-> 清除fatal锁存和当前恢复计数
-> 保留home、分类结果和最后合法地图
-> 进入 ART Sync 或 VSync
-> 只接受新地图
-> 重新规划并继续
```

K3不能直接恢复旧 waypoint，也不能立刻使用触发错误的旧地图开动。K4仍保持最高优先级，不能被自动恢复解除。

IMU、编码器和电机目前没有独立健康状态接口。实施时应增加由20ms控制链路更新、主循环读取的健康/故障锁存；不能仅凭 `yaw_error` 超时判断IMU已经失效，也不能在ISR中执行恢复状态机。

### 阶段5：错误显示、计时和配置整理

只增加确实需要的配置：

```c
RECOVERY_RESYNC_TIMEOUT_MS   // 第二类单次新图恢复窗口，建议约3000ms
RECOVERY_MAX_RETRIES         // 同恢复键自动恢复次数，建议2次后第三次升级
```

不再让所有状态共享5秒语义；中心、地图和真实运动超时分别在状态中使用合适窗口。ART1约3FPS、ART2约8FPS作为参数依据：

- 地图恢复窗口必须覆盖约9个ART1周期。
- 普通中心和观察样本不能用小于一个ART1帧周期的等待判断失败。
- ART2连续3帧分类约需375ms，分类失败等待不应无条件占用3秒。

正式运行前确认 `openmv/main_see.py` 使用 `MODE_RUN`，调试绘制和逐帧yaw诊断不能作为比赛默认路径。

## 预计修改文件

核心代码：

- `project/user/src/subject2.c`
- `project/user/src/art_replan.c`
- `project/user/src/executor.c`
- `project/user/src/menu.c`
- `project/user/src/competition_flow.c`
- `project/user/src/screen.c`
- `project/user/src/drive_control.c`（若现有底盘健康状态不足）

按需修改头文件：

- `project/user/inc/drive_config.h`
- `project/user/inc/executor.h`
- `project/user/inc/art_replan.h`
- `project/user/inc/subject2_logic.h`

OpenART协议和脚本本轮不改；只检查 `openmv/main_see.py` 正式运行开关和现有请求语义。

## 测试计划

### 第一类

- `VCtr` 无中心：不进入ERROR，直接VTurn。
- `PCtr` 无中心且C在预期格：继续waypoint。
- `ICtr/RCtr` 无中心：零offset求解并启动。
- `VAdj`超时且误差≤2cm：继续；误差>2cm：进入VSync。
- `VTurn`误差≤2deg：继续分类；误差>2deg：换观察方向。
- `HYaw/VFix`失败：锁定当前yaw并继续。
- `OBSERVE_SAMPLE`缺失且C/B粗格一致：GridPush继续。
- 精确车箱样本与地图冲突：不直接推，进入同步。
- 中心fallback在旧帧、多个C、活动B不匹配或executor错误时不得放行。
- executor原始错误不能被`subject2_fail()`覆盖。

### 第二类

- 新地图帧号必须大于进入恢复时帧号。
- B/T等量减少优先于旧活动任务判断。
- 途经目标遮挡正确规范化为`*`。
- C变化后旧waypoint不再执行。
- `Track`只局部重扫有歧义对象。
- 第一次、第二次恢复不进入ERROR。
- 恢复成功后对应计数清零。
- UART残留半帧不能被第二类恢复接受。
- 恢复期间电机保持停止，旧waypoint不能继续执行。
- `VPos/Plan/BObs/CPsh/E:Back`每个入口都有独立恢复测试。

### 第三类

- 控制/IMU硬错误第一次进入FATAL_WAIT。
- 同一恢复键第三次失败才进入FATAL_WAIT。
- 换对象、目标或观察方向不继承旧计数。
- FATAL_WAIT电机停止但UART、屏幕、按键继续工作。
- K3从新地图同步恢复，不继续旧waypoint。
- K4急停不能自动解除。
- `EXEC_ERROR_SUBJECT2_PLAN`有明确显示文字。
- F:Drive/F:ART1/F:Map状态不自动返航。
- FATAL_WAIT期间UART、屏幕和按键仍工作。
- `menu_poll()`集成测试覆盖FATAL_WAIT、K3恢复和K4锁存。
- `F:ART1/F:ART2`业务原因可以分别显示。

### 回归验证

```powershell
Get-ChildItem tests -Filter "run_*.ps1" | ForEach-Object {
    powershell -ExecutionPolicy Bypass -File $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
git diff --check
```

验收要求：C主机测试全部通过，Keil为 `0 Error(s), 0 Warning(s)`，不下载固件；OpenART脚本未修改时至少运行Python语法和协议测试。

## 实施检查点

1. 阶段0完成后确认基线测试结果。
2. 阶段1完成后确认恢复键不会跨对象误累计。
3. 阶段2完成后确认第一类不进入 `EXEC_STATE_ERROR`。
4. 阶段3完成后确认第二类不沿旧路径盲跑。
5. 阶段4完成后确认第三类锁存、K3恢复和K4优先级正确。
6. 阶段5完成后确认屏幕原因明确、计时参数与ART1/ART2帧率匹配。

## 现有错误入口分级映射

实施时必须逐个替换当前直接终止调用，不能只修改一个`subject2_fail()`函数：

| 当前入口 | 第一类直接降级 | 第二类同步恢复 | 第三类条件 |
|---|---|---|---|
| `VCtr/CTmo` | 新鲜合法地图且对象匹配时跳过VAdj | 门禁失败时VSync | 两次同步后仍无合法地图 |
| `PCtr/CPsh` | C仍在预期格时释放等待 | C移动或几何冲突时VSync/ART Sync | 同一箱/目标第三次失败 |
| `ICtr/RCtr/CRpl` | 地图C中心、offset=0 | 地图也无效时重新取图 | 连续无法取得合法C |
| `CRef` | 地图合法时丢弃精确中心 | 地图和中心均冲突时VSync | 同一恢复键第三次失败 |
| `CTim/CExe/CBsy` | 剩余误差≤2cm且executor健康时继续 | 误差大或状态不一致时VSync | 底盘健康错误或重复失败 |
| `VTurn/Yaw` | IMU误差≤2deg时接受当前朝向 | 超过2deg换观察方向 | IMU健康错误或所有方向失败 |
| `HYaw/VFix` | 锁定当前MCU yaw继续 | 当前yaw无法锁定时同步 | IMU失效 |
| `BObs/BTim` | 样本缺失且C/B粗格一致时GridPush | 精确样本冲突或粗格不一致时同步 | 同一推箱链第三次失败 |
| `E:Back` | 无精确视觉但地图/pose健康时跳过回退 | 回退结果与地图冲突时VSync | executor/底盘硬错误 |
| `E:VMod` | 不直接使用未READY的ART2 | 重发模式并重新初始化 | ART2多次不READY |
| `E:Map/ATim` | 无 | 重启VSync/确认tracker | 两次恢复仍无合法地图 |
| `E:VPos/Plan` | 无旧路径放行 | 保存上下文后新图重算一次 | 新图和所有备选仍不可解 |
| `E:BSet` | 无 | 只局部重扫异常类别 | 局部重扫仍不匹配 |
| `E:Track` | 无 | VSync后对象协调和局部重扫 | 对象持续无法恢复 |
| `RetErr` | 返航中心缺失可按第一类处理 | 返航地图/中心重新同步 | home无效、出口不可达或底盘错误 |

任何入口如果发现executor已经是`EXEC_STATE_ERROR`，先保存原始executor错误，再按健康状态决定第二类或第三类，不能覆盖成普通分类错误。
