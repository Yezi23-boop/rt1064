# ART 推箱前中心校正设计

> 当前实现：每段连续推箱只在第一个大写 waypoint 执行前校正一次。本文已按当前代码更新，不再描述“每个大写动作都校正”或“等待超时”。

## 目标

在 ART 地图来源下，车辆开始一段连续推箱动作前，先停车采集精确中心并修正 MCU pose。修正完成后连续执行该段推箱动作，避免在贴近箱子后反复停车识别。

普通小写移动、同一推箱段后续大写动作、离线地图和返航 BFS 不进入该流程。推箱前校正只修正 pose，不触发地图求解，也不更新 B/T 确认基线。

## 标记规则

求解器保留每个大写动作的独立 waypoint，并通过 `center_correct_before` 标记连续推箱段的起点：

- 当前 action 是大写，且它是路径首个 action，标记为 1。
- 当前 action 是大写，且前一个 action 是小写，标记为 1。
- 当前 action 是大写，且前一个 action 也是大写，标记为 0。
- 普通移动和返航 waypoint 固定为 0。

例如：

```text
rrRR -> r/r 不校正，第一个 R 前校正，第二个 R 不校正
dR   -> d 不校正，R 前校正
RRR  -> 第一个 R 前校正，后两个 R 不校正
```

## 执行流程

```text
准备执行连续推箱段的第一个 U/D/L/R
-> stop_motion()
-> 进入 PCtr
-> MCU 发送一次 CENTER_REQ
-> OpenART 每识别到一个有效精确中心就发送一个 CENTER_SAMPLE
-> MCU 收齐 3 个样本并对 col_q、row_q 分别取中值
-> 校验中心并按配置比例融合 pose
-> 放行当前大写 waypoint
-> Run 模式连续执行后续大写 waypoint
```

`PCtr` 不设置自动超时。无效图像帧不计入样本数；收不齐 3 个有效样本时保持停车和 `PCtr`，便于现场直接发现识别问题。K4 仍可人工停止。

## 中心识别与校验

OpenART 在收到 `CENTER_REQ` 后，只使用精确像素中心产生请求样本。每个有效图像帧最多发送一个样本，样本序号依次为 1、2、3。

MCU 收齐 3 个样本后取中值，并执行以下校验：

- 中心坐标位于 16x12 地图范围内。
- 中心所在格与参考 C 格的行、列差符合 `EXEC_ART_CENTER_ALLOW_NEIGHBOR_CELL`。
- 中心与 MCU pose 的偏差不超过 `EXEC_ART_CENTER_ABNORMAL_CM`。
- 可融合偏差不超过 `EXEC_ART_CENTER_FUSE_MAX_CM`。

偏差小于 `EXEC_ART_CENTER_IGNORE_CM` 时视为校正成功但不写 pose；其余合法偏差按 `EXEC_ART_CENTER_FUSE_ALPHA` 融合。

## 连续推箱

Run 模式下，大写 waypoint 到达后，如果下一 waypoint 仍是大写且当前点不是单箱任务结束点，则立即切换到下一目标：

- 不进入段间停稳。
- 不输出停车命令。
- 不重复进入 `PCtr`。
- 重置路径 PID 后，在同一 20ms 周期计算下一目标速度。

Step 模式仍逐 waypoint 暂停。最后一个推箱 waypoint 和单箱任务结束点仍停车等待 ART 地图确认。

## 状态与失败语义

- `PCtr`：等待推箱前 3 个有效中心样本。
- `E:Ctr`：样本收齐后，中心校验、融合提交或 executor 放行失败，底盘已停车。

持续识别不到中心不会自动进入 `E:Ctr`，而是保持 `PCtr`。这是当前为现场调试选择的行为。

## 保持不变

- ART 初次求解和单箱任务结束重规划继续使用配套中心初始化 executor pose。
- 单箱任务结束点继续根据 B/T 数量变化确认并重算。
- `EXEC_ART_CENTER_CORRECT_ENABLE` 仍只控制普通段末中心校正。
- 不修改 BFS 搜索、PID、里程计、20cm 格距、发车和返航流程。
- 运动过程中不做实时视觉闭环。

## 验证标准

- 连续推箱段只在第一个大写 waypoint 前进入一次 `PCtr`。
- MCU 每次校正只发送一次 `CENTER_REQ`。
- OpenART 只累计有效精确中心，依次发送 3 个 `CENTER_SAMPLE`。
- Run 模式连续大写动作之间不停车，Step 模式仍逐 waypoint 暂停。
- 最后一个推箱任务结束点仍停车等待 ART 确认。
- 离线地图和返航路径不进入 `PCtr`。
- Keil 编译为 `0 Error(s), 0 Warning(s)`。
