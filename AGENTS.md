# AGENTS.md

## 构建命令

```powershell
# Keil MDK 编译（只编译，不下载）
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"

# Keil MDK 下载到 Flash
D:\Keil_v5\UV4\UV4.exe -f "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"

# Python 语法验证
python -m py_compile openmv/main.py
```

**重要**：Keil 命令行下载成功后，程序可能停在调试态（`main()` 入口）。需要手动复位或执行 `uvx pyocd reset -t mimxrt1064` 让程序真正运行。

## 项目架构

**双机架构**：OpenART Plus（视觉识别） + RT1064（控制/求解）

```
OpenART Plus (MicroPython)          RT1064 (C)
┌─────────────────────┐            ┌─────────────────────────┐
│ 摄像头 → 16×12地图识别│   UART1   │ main.c → app.c          │
│       ↓              │  B12/B13  │       ↓                  │
│ send_map_uart() ────┼──────────→ │ openart_uart_push_byte() │
│   MAP_BEGIN          │  115200   │       ↓                  │
│   12行×16列          │           │ solver.c (BFS求解)       │
│   MAP_END            │           │       ↓                  │
└─────────────────────┘           │ drive_control.c (底盘)    │
                                   └─────────────────────────┘
```

## UART 通道分配

| 模块 | UART | 引脚 | 用途 |
|------|------|------|------|
| 无线串口 | LPUART8 | - | printf 调试输出（已重定向） |
| OpenART | LPUART1 | TX→B13, RX→B12 | 接收地图数据 |

printf 已重定向到无线串口（LPUART8），不与 OpenART 的 UART1 冲突。

## 目录结构与搜索顺序

```
project/user/     ← 用户代码（优先改这里）
libraries/        ← 平台库（不要改，除非 bug 明确在库内）
openmv/           ← OpenART MicroPython 脚本
docs/competition/ ← 竞赛规则、硬件接线、算法文档
docs/agents/      ← Agent 工作流文档
```

**搜索顺序**（找到问题所在再深入）：
1. `project/user/` — 应用行为
2. `libraries/zf_driver/` — 外设 API
3. `libraries/zf_device/` — 设备驱动（IMU、屏幕、相机）
4. `libraries/zf_common/` — 公共工具（debug、clock、FIFO）
5. `libraries/sdk/` — 芯片级寄存器、时钟树、启动

## 关键文件

### 入口与调度
- `project/user/src/main.c` — 系统入口
- `project/user/src/isr.c` — 中断处理（PIT 20ms 控制环、UART 收包）
- `project/user/src/app.c` — 应用层初始化和轮询

### 底盘控制
- `project/user/inc/drive_config.h` — 四轮麦轮配置、PID 参数、IMU 参数
- `project/user/src/drive_control.c` — 控制调度、20ms 姿态环 + 轮速 PID
- `project/user/src/base_io.c` — 硬件绑定层（编码器、PWM、IMU）

### 推箱子算法
- `project/user/src/solver.c` — BFS 求解器
- `project/user/inc/map_types.h` — 地图常量（MAP_ROWS=12, MAP_COLS=16）
- `project/user/src/maps.c` — 离线测试地图

### 通信
- `project/user/src/openart_uart.c` — OpenART UART 协议解析

## PIT 中断节拍

- **PIT_CH1**：20ms — 姿态环、混控、四轮速度 PID（`update_control_20ms()`）
- **PIT_CH2**：5ms — 菜单按键扫描（`menu_key_tick_5ms()`）

## OpenART UART 协议

```
MAP_BEGIN\n
################\n    ← 12行，每行16字符
#..............#\n    ← 字符集：#.TBXC
...
################\n
MAP_END\n
```

RT1064 收到完整帧后回复：`MAP_OK rows=12 cols=16\r\n`

## 添加新 C 文件的流程

1. 在 `project/user/src/` 添加 `.c`，在 `project/user/inc/` 添加 `.h`
2. **必须**同步更新 `project/mdk/rt1064.uvprojx`（Keil 工程文件）
3. 执行 Keil 编译验证：0 Error(s), 0 Warning(s)

## 菜单系统约定

菜单采用 `menu.c`（状态/按键）+ `screen.c`（显示）分离架构：
- `menu.c` 负责从业务模块取数据，传给 `screen.c`
- `screen.c` 只负责 IPS200 绘制，不访问业务状态
- 新增页面/变量参考：`docs/agents/menu-extension-guide.md`

## 文档优先原则

遇到以下问题时，**先查文档再改代码**：
- OpenART/UART/SD 卡问题 → `docs/competition/openart_plus_rt1064_uart_experience.md`
- 姿态闭环/底盘控制 → `docs/competition/姿态闭环框架.md`
- 菜单系统 → `docs/competition/掉电保存菜单框架.md` + `docs/agents/menu-extension-guide.md`
- 推箱子算法 → `docs/competition/push_box_validation_cases.md`
- OpenMV API → https://docs.openmv.io/library/index.html
