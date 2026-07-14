# OpenART Center Map Pairing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让请求得到的精确中心与 MCU 用于校正的地图 `C` 来自同一帧。

**Architecture:** OpenART 在每条有效 `CENTER_SAMPLE` 前发送一帧携带同一 `PLAYER_CENTER_GRID` 的完整地图。现有 MCU 地图接收器负责原子发布并归一 `C`，不扩展 UART 文本格式。

**Tech Stack:** OpenMV MicroPython、Python 源码协议测试。

---

### Task 1: 锁定发送顺序

**Files:**
- Modify: `tests/openart_request_flow_test.py`

- [x] 增加测试替身，记录 `send_map_uart()` 与 UART 写入顺序。
- [x] 断言有效中心先产生带中心地图，再产生同坐标 `CENTER_SAMPLE`。
- [x] 运行测试并确认当前实现失败。

### Task 2: 实现配套发送

**Files:**
- Modify: `openmv/main_see.py`

- [x] 给 `process_center_request()` 传入当前字符地图。
- [x] 有效中心时先调用 `send_map_uart(uart, char_matrix, center_grid)`，再写 `CENTER_SAMPLE`。
- [x] 返回本轮是否已发送配套地图；主循环据此更新 `last_uart_send_ms`。
- [x] 运行专项测试并确认通过。

### Task 3: 回归验证

**Files:**
- Test: `tests/openart_request_flow_test.py`
- Test: `tests/openart_player_anchor_test.py`
- Test: `tests/openart_precise_center_test.py`

- [x] 运行相关 Python 测试。
- [x] 运行 `python -m py_compile openmv/main_see.py`。
- [x] 运行 `git diff --check`。
