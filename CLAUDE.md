# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

第21届智能车竞赛人工智能视觉组项目。**双机架构**：OpenART Plus 负责视觉识别12×16地图，RT1064负责接收地图、推箱子求解、底盘运动控制。

## 构建命令

```bash
# Keil MDK 命令行编译
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"

# Python 语法验证
python -m py_compile openmv/openart_plus_grid_recognizer_pure.py
```

## 双机数据流

```
OpenART Plus (MicroPython)          RT1064 (C)
┌─────────────────────┐            ┌─────────────────────────┐
│ 摄像头 → 地图识别    │            │ main.c → app.c          │
│       ↓              │   UART1   │       ↓                  │
│ send_map_uart() ────┼──────────→ │ LPUART1_IRQHandler      │
│   MAP_BEGIN          │  B12/B13  │       ↓                  │
│   12行×16列          │  115200   │ openart_uart_push_byte() │
│   MAP_END            │           │       ↓                  │
│                      │           │ openart_uart_poll()      │
└─────────────────────┘           │       ↓                  │
                                   │ 求解器/菜单/屏幕         │
                                   └─────────────────────────┘
```

## UART 通道分配

| 模块 | UART | 引脚 | 用途 |
|------|------|------|------|
| 无线串口 | LPUART8 | - | printf 调试输出到电脑 |
| OpenART | LPUART1 | TX→B13, RX→B12 | 接收地图数据 |

**重要**：printf 已重定向到无线串口（LPUART8），不与 OpenART 的 UART1 冲突。

## 关键模块

### RT1064 启动流程
```
main.c: clock_init → debug_init → wireless_uart_init → openart_uart_init → control_init → app_init
app.c:  timebase_init → settings_init → screen_init → vofa_init → drive_test_init → menu_init
主循环: menu_poll → vofa_service → openart_uart_poll → drive_test_poll
```

### PIT 中断节拍（isr.c）
- **PIT_CH1**：20ms — 姿态环、混控、四轮速度PID（`update_control_20ms()`）
- **PIT_CH2**：5ms — 菜单按键扫描（`menu_key_tick_5ms()`）

### OpenART UART 协议
```
MAP_BEGIN\n
################\n    ← 12行，每行16字符
#..............#\n    ← 字符集：#.TBXC
...
################\n
MAP_END\n
```

RT1064 收到完整帧后回复：`MAP_OK rows=12 cols=16\r\n`

## 目录结构速查

- `project/user/src/` — 用户代码（main.c, app.c, isr.c, menu.c, drive_control.c, solver.c）
- `project/user/inc/` — 用户头文件（map_types.h 定义地图常量 MAP_ROWS=12, MAP_COLS=16）
- `openmv/` — OpenART MicroPython 脚本
- `libraries/` — 底层驱动库（**不要修改**，除非 bug 明确在库内）
- `docs/competition/` — 比赛规则、硬件接线、算法文档

## 环境约束

- Win11 环境，`rg`/`ripgrep` 可能不可用，优先用 `grep` 或 PowerShell `Select-String`
- Git commit message 使用中文
- **不要主动提交 Git**：完成修改后先汇报，等用户确认后再提交
- 添加新 `.c` 文件后必须同步更新 `project/mdk/rt1064.uvprojx`
