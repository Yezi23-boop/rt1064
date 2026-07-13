# UART4 板级自检设计

## 目标

通过编译期开关独立验证 RT1064 UART4 与 OpenART #2 的双向通信，不依赖 ART1、地图、发车或底盘运动。

## 配置

```c
#define VISION_UART_BOARD_TEST_ENABLE (1)
```

- `1`：上电自动执行 UART4 自检，禁止启动比赛流程。
- `0`：不运行自检，恢复正常比赛和首页 ART 帧数显示。

## 流程

```text
VISION_MODE BOX
-> VISION_READY BOX
-> VISION_REQ 1
-> 收到 3 条 VISION_SAMPLE
-> VISION_ACK 1
-> PASS
```

等待 READY 时每秒重发一次模式命令。发送请求后 5 秒内未收到 3 条样本则进入 FAIL。PASS/FAIL 保持到 MCU 复位。

## 首页显示

开启测试后，首页最后一行替换原 `ART:`：

```text
V4:MODE
V4:REQ
V4:S1 C8 Q932
V4:PASS C8 N3
V4:FAIL
```

屏幕层只绘制菜单层提供的测试状态，不直接访问 UART 驱动。

## 安全边界

- 测试期间不启动 competition flow、subject2 或 executor。
- 不发送任何底盘运动命令。
- 菜单和首页动态刷新继续运行。
- 不使用无线 printf 作为通过条件。
