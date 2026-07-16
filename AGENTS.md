# AGENTS.md

本文件是仓库级协作入口，只保存长期有效的工程规则、架构边界和验证方式。
详细文件清单放在 `docs/agents/file-map.md`，阶段性设计放在 `docs/superpowers/`，不要把临时实现快照堆回本文件。

## 协作原则

- 尽量使用中文回答和编程。
- 编码前明确假设、歧义和取舍；需求不清时先说明，不要静默选择一种解释。
- 只实现用户要求的最小范围，不增加未要求的抽象、配置或功能。
- 修改现有代码时保持原有风格，不顺手重构相邻模块，不删除无关旧代码。
- 每一处修改都应能追溯到当前任务；只清理由本次修改产生的无用代码。
- Bug 修复先建立可复现测试，再做最小修复；完成前必须运行与改动范围匹配的验证。
- 工作树可能已有用户修改。不得回退、覆盖或格式化与当前任务无关的变更。

## CodeGraph

仓库根目录存在 `.codegraph/`。理解或定位代码时，先使用 CodeGraph，再使用 `rg` 或直接读文件：

```powershell
codegraph explore "要查找的符号、文件或数据流问题"
```

CodeGraph 用于定位当前符号、调用路径和动态分派；最终判断仍以当前磁盘源码、Keil 工程和测试结果为准。

## 比赛任务语义

本项目面向第 21 届智能车竞赛人工智能视觉组，采用虚实结合任务：

```text
真实车在物理场地运动
-> 全局摄像头识别真实车位置和方向
-> 上位机同步虚拟游戏中的车模
-> 虚拟游戏执行推箱子、目标消除等规则
-> 游戏画面投到车载屏幕
-> OpenART #1 识别地图、车和箱子几何状态
-> OpenART #2 识别科目二图片/数字类别
-> RT1064 规划并控制真实车继续运动
```

- 箱子推动、目标消失和炸墙等规则发生在虚拟游戏中，实车不接触物理箱子。
- 实车必须按照虚拟路径准确行驶；穿过虚拟墙会导致上位机不再正确刷新，严重时比赛结束。
- 调试“推箱不准”时，优先检查格点路径、里程计、yaw、视觉中心、速度和到点判定。
- 求解器中的小写 `u/d/l/r` 表示普通移动，大写 `U/D/L/R` 表示虚拟推箱动作边界；底盘都执行为对应格点运动。
- 当前多箱算法采用逐箱贪心拆解，不等价于完整多箱 Sokoban 全局搜索；修改或评价算法时不要忽略这个边界。

## 当前系统架构

系统由一个主控和两块 OpenART 组成：

```text
OpenART #1 地图/中心/观察几何
  openmv/main_see.py
          |
          | LPUART1 115200, B12/B13
          v
RT1064
  main -> app/menu -> competition_flow
                     |-> art_replan -> solver -> executor
                     `-> subject2 -> subject2_logic
                                           ^
                                           |
          LPUART4 115200, C16/C17          |
OpenART #2 图片/数字分类 -------------------'
  openmv/视觉/main.py
```

职责边界：

- OpenART #1：识别 16x12 地图、虚拟车中心、箱子中心和观察几何，不做路径规划。
- OpenART #2：按 MCU 请求切换图片/数字模式并返回分类样本，不决定箱子与目标绑定。
- RT1064：负责比赛流程、地图校验、BFS、分类绑定、执行器、里程计、yaw 和四轮闭环。
- `menu.c` 组织业务状态，`screen.c` 只绘制传入的数据，不直接读取业务模块。

## UART 分工

| 通道 | RT1064 引脚 | 对端 | 用途 |
|------|-------------|------|------|
| LPUART1 | TX B12 / RX B13 | OpenART #1 | 地图、中心请求、观察请求 |
| LPUART4 | TX C16 / RX C17 | OpenART #2 | 图片/数字分类请求和结果 |
| LPUART8 | 无线串口 | 上位机 | `printf`、VOFA 和调试输入 |

接线按 TX/RX 交叉：OpenART TX 接 RT1064 RX，OpenART RX 接 RT1064 TX，并共地。

### OpenART #1 协议摘要

周期地图帧：

```text
MAP_BEGIN
<12 行，每行 16 字符，字符集 #.TBXC>
PLAYER_CENTER_GRID <col_q>,<row_q> <valid>
MAP_END
```

- OpenART 是 `C/+` 的唯一生成者；每张合法地图必须恰好一个 `C/+`。
- `valid=1` 时 `PLAYER_CENTER_GRID` 必须位于该唯一 `C/+` 所在格；中心行缺失或 `valid=0` 不废弃地图，只表示本帧无精确中心。
- OpenART 精确中心连续识别失败前两帧可沿用最近结果；第 3 帧必须清除历史并发送 `valid=0`，重新识别后不得混用旧坐标。
- `CENTER_SAMPLE`、`OBSERVE_SAMPLE` 前必须各有一张新的规范地图；MCU 只校验和发布，不能依据中心再次移动 `C/+`。
- 指定箱子的精确中心仅服务推箱前准备位，不改字符地图中的 `B`。

RT1064 接受完整帧后回复：

```text
MAP_OK rows=12 cols=16
```

请求式数据：

```text
MCU -> CENTER_REQ
ART -> CENTER_SAMPLE <index>,<col_q>,<row_q>,<yaw_q>,<yaw_valid>

MCU -> OBSERVE_REQ <box_row>,<box_col>
ART -> OBSERVE_SAMPLE <index>,<player_col_q>,<player_row_q>,<box_col_q>,<box_row_q>
```

`yaw_q` 单位为 `0.01°`；`yaw_valid=0` 时中心样本仍然有效。除发车区校准和科目二全部观察完成、`HYaw` 稳定后的单次小角校准外，不得用该字段修改 MCU 姿态基准。

### OpenART #2 协议摘要

```text
MCU -> VISION_MODE BOX | VISION_MODE TARGET
ART -> VISION_READY BOX | VISION_READY TARGET
MCU -> VISION_REQ <request_id>
ART -> VISION_SAMPLE <request_id> <sample_id> <class_id> <confidence_q>
MCU -> VISION_ACK <request_id> | VISION_CANCEL
```

修改协议时必须同步修改 MCU、对应 OpenART 脚本和协议测试。

## 实时与并发边界

- **PIT_CH1，20ms**：编码器/IMU反馈 -> `executor_update_20ms()` -> 姿态、混控、四轮速度 PID 和 PWM 输出。
- **PIT_CH2，5ms**：菜单按键扫描 `menu_key_tick_5ms()`。
- **IMU INT2 GPIO 中断**：调用 `imu660rc_callback()` 更新驱动姿态缓存。
- **LPUART1/LPUART4 ISR**：只读取字节并推入模块缓冲，文本解析必须留在主循环。

硬规则：

- 不得在 ISR 中执行 BFS、屏幕绘制、`printf`、阻塞等待或协议解析。
- 主循环任务必须短轮询、可重复调用；长流程应拆成状态机。
- 主循环修改 executor/pose 等与 PIT_CH1 共享的复合状态时，必须使用现有临界区模式，不能只依赖 `volatile`。
- `screen.c`、无线输出和 OpenART ACK 延迟异常时，先检查主循环是否被同步计算或串口输出阻塞。

## 目录职责与搜索顺序

```text
project/user/     应用、比赛流程、控制和协议，优先修改
openmv/           两块 OpenART 的 MicroPython 脚本和模型
tests/            GCC 主机测试与 Python 协议/识别测试
docs/competition/ 比赛规则、硬件和算法背景
docs/agents/      Agent 导航与工作流文档
libraries/        平台库和设备驱动，确认问题在库内才修改
```

搜索顺序：

1. `project/user/`
2. `openmv/` 或 `tests/`
3. `libraries/zf_driver/`
4. `libraries/zf_device/`
5. `libraries/zf_common/`
6. `libraries/sdk/`

详细文件职责见 `docs/agents/file-map.md`。

## 核心模块入口

### 启动与调度

- `project/user/src/main.c`：硬件、两路视觉 UART、底盘和应用初始化。
- `project/user/src/app.c`：主循环短轮询。
- `project/user/src/isr.c`：PIT、UART 和 IMU GPIO 中断。
- `project/user/src/menu.c`：菜单、比赛启动和主循环业务协调。
- `project/user/src/competition_flow.c`：科目一、科目二和完整模式的阶段切换。

### 地图、规划与执行

- `project/user/src/openart_uart.c`：OpenART #1 地图、中心和观察协议。
- `project/user/src/art_replan.c`：发车、稳定地图、科目一重规划和返航。
- `project/user/src/solver.c`：单箱推箱 BFS、逐箱拆解和普通导航 BFS。
- `project/user/src/executor.c`：将 waypoint 转换为物理目标，在 PIT_CH1 中推进。
- `project/user/src/subject2.c`：科目二扫描、分类、绑定、推箱确认和返航请求状态机。
- `project/user/src/subject2_logic.c`：科目二纯逻辑算法，优先通过主机测试验证。

### 视觉分类

- `project/user/src/vision_uart.c`：OpenART #2 的 UART4 请求/响应协议。
- `openmv/main_see.py`：OpenART #1 正式地图与几何识别入口。
- `openmv/视觉/main.py`：OpenART #2 正式图片/数字分类入口。
- `openmv/main_model.py`：早期独立分类实验，不是当前比赛协议入口。

### 底盘控制

- `project/user/inc/drive_config.h`：比赛模式、网格、PID、视觉和执行参数。
- `project/user/inc/drive_test.h`：代码开关式底盘测试配置。
- `project/user/src/drive_control.c`：20ms 控制调度和运动接口。
- `project/user/src/drive_pose.c`：编码器与相对 yaw 位姿积分。
- `project/user/src/drive_imu.c`：yaw 归一化、锁定和姿态环。
- `project/user/src/drive_output.c`：麦轮混控、轮速 PID 和 PWM。
- `project/user/src/base_io.c`：电机、编码器和 IMU 硬件绑定。

## 构建与验证

### Keil MDK

```powershell
# 只编译，不下载
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"

# 下载到 Flash；只有用户明确要求时才执行
D:\Keil_v5\UV4\UV4.exe -f "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

下载成功后程序可能停在 `main()` 调试入口，需要手动复位或执行：

```powershell
uvx pyocd reset -t mimxrt1064
```

### Python 语法

```powershell
python -m py_compile openmv/main_see.py "openmv/视觉/main.py"
```

### 全量主机测试

```powershell
Get-ChildItem tests -Filter "run_*.ps1" | ForEach-Object {
    powershell -ExecutionPolicy Bypass -File $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Get-ChildItem tests -Filter "*_test.py" | ForEach-Object {
    python $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

修改 C 代码至少执行对应主机测试和 Keil 编译；修改 OpenART 脚本至少执行 Python 测试与 `py_compile`。完成前运行：

```powershell
git diff --check
```

## 调车与比赛配置安全

正式比赛或联调前检查：

- `project/user/inc/drive_test.h` 中所有 `DRIVE_*_TEST_ENABLE` 和 `DRIVE_WHEEL_JOG_ENABLE` 均为 `0`。
- `project/user/inc/drive_config.h` 中 `VISION_UART_BOARD_TEST_ENABLE` 为 `0`。
- `COMPETITION_MODE` 明确选择科目一调试、科目二调试或完整连续运行，不依赖默认值猜测。
- `openmv/main_see.py` 的 `WORK_MODE` 与用途一致；正式高速运行使用运行模式，标定时才使用调试模式。
- 修改里程计、PID、到点阈值或视觉融合参数时，一次只改一类变量，并记录实测距离/现象。
- 使用平移、方形或单轮测试后必须关闭测试开关，再编译比赛固件。

## OpenART 显示与识别约定

- `SHOW_RECTIFIED_VIEW` 只控制 IDE 显示，不得决定地图或中心是否下发。
- `USE_RECTIFIED_RECOGNITION=False` 是合法的原图高速识别路径，不代表中心无效。
- 正式高速模式应关闭不必要的调试绘制和打印。
- 不得用显示分支状态作为 `PLAYER_CENTER_GRID`、`CENTER_SAMPLE` 或观察样本的发送门控。
- 修改 `MAP_CORNERS`、透视或颜色阈值前，先阅读 `docs/competition/openart_grid_alignment_debug.md`。

## 菜单系统约定

菜单采用 `menu.c` 与 `screen.c` 分离：

- `menu.c` 从业务模块取得数据并构造 view。
- `screen.c` 只绘制 view，不访问 solver、executor、subject2 或 UART 内部状态。
- 新增页面、字段和刷新周期参考 `docs/agents/menu-extension-guide.md`。

## 添加新 C 文件

1. 在 `project/user/src/` 添加 `.c`，按需在 `project/user/inc/` 添加 `.h`。
2. 将 `.c` 同步加入 `project/mdk/rt1064.uvprojx`。
3. 添加与风险匹配的主机测试；纯逻辑优先在 GCC 主机测试中覆盖。
4. 运行对应测试和 Keil 编译，要求 `0 Error(s), 0 Warning(s)`。
5. 更新 `docs/agents/file-map.md`，不要把详细文件清单重复加入本文件。

## 文档优先级

- OpenART/UART/SD 卡：`docs/competition/openart_plus_rt1064_uart_experience.md`
- 网格透视与颜色：`docs/competition/openart_grid_alignment_debug.md`
- 姿态闭环与底盘：`docs/competition/姿态闭环框架.md`
- 菜单：`docs/competition/掉电保存菜单框架.md`、`docs/agents/menu-extension-guide.md`
- 推箱算法：`docs/competition/push_box_validation_cases.md`
- 当前文件导航：`docs/agents/file-map.md`
- OpenMV API：https://docs.openmv.io/library/index.html

文档与源码冲突时，以当前源码、Keil 工程和可复现测试为准，并在本次任务内同步修正文档。
