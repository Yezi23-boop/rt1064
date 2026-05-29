# OpenART UART 地图通讯设计

## 背景

OpenART Plus 负责识别 12 行 x 16 列地图，RT1064 负责接收地图、求解和控制底盘。项目文档已验证 OpenART Plus 应使用 `UART(12)`，RT1064 侧使用 `UART_1`，接线为 OpenART TX 到 RT1064 B13、OpenART RX 到 RT1064 B12、共地，波特率 115200。

当前 `printf` 已重定向到无线串口输出，UART1 不再作为主要调试输出通道。UART1 的接收权交给 OpenART 通讯模块，避免 OpenART 发来的字节被 debug 接收 FIFO 消耗。

## 目标

- OpenART 定时主动发送识别稳定后的字符地图。
- RT1064 通过 UART1 中断接收字节，主循环解析完整地图帧。
- 通讯第一版优先可读、易调试、低风险，不做复杂握手。
- 接收成功后 RT1064 保存最新地图，并可回传 `MAP_OK rows=12 cols=16`。

## 非目标

- 第一版不做二进制压缩协议。
- 第一版不要求 RT1064 主动请求 OpenART 发图。
- 第一版不直接改求解器策略，只提供最新地图输入接口。
- 第一版不在 ISR 中解析字符串或运行 BFS。

## 协议

OpenART 每帧发送：

```text
MAP_BEGIN
################
#..............#
#..............#
#.....B........#
#..............#
#...C..........#
#..............#
#........T.....#
#..............#
#..............#
#..............#
################
MAP_END
```

帧格式要求：

- 起始行必须是 `MAP_BEGIN`。
- 中间必须恰好 12 行地图。
- 每行地图必须恰好 16 个字符。
- 结束行必须是 `MAP_END`。
- 行尾使用 `\r\n` 或 `\n` 均可，RT1064 解析时忽略 `\r`。
- 第一版合法地图字符为 `#`、`.`、`B`、`T`、`C`、`X`。

字符含义：

- `#`：墙。
- `.`：空地。
- `B`：箱子。
- `T`：目标点。
- `C`：小车。
- `X`：炸弹或不可通行危险格；第一版 RT1064 可以先按墙处理。

RT1064 收到合法帧后回传：

```text
MAP_OK rows=12 cols=16
```

收到非法帧时丢弃当前半帧，等待下一次 `MAP_BEGIN`。错误回包可以暂缓，只保留内部计数，避免调试输出刷屏。

## RT1064 架构

新增模块：

- `project/user/inc/openart_uart.h`
- `project/user/src/openart_uart.c`

模块职责：

- 初始化 UART1 参数和接收缓冲区。
- 在 `openart_uart_handler()` 中从 UART1 读取到来的字节，写入环形缓冲区。
- 在 `openart_uart_service()` 中按行解析协议。
- 保存最新一帧合法地图。
- 暴露查询接口给菜单、求解器或后续自动流程。

建议接口：

```c
void openart_uart_init(void);
void openart_uart_handler(void);
void openart_uart_service(void);
uint8 openart_uart_has_map(void);
const char (*openart_uart_get_map(void))[MAP_COLS + 1];
uint32 openart_uart_get_frame_count(void);
uint32 openart_uart_get_error_count(void);
```

主循环接入：

- `main.c` 初始化阶段调用 `openart_uart_init()`。
- `app_poll()` 中调用 `openart_uart_service()`。
- `LPUART1_IRQHandler()` 中把 UART1 RX 字节交给 `openart_uart_handler()`。

中断约束：

- ISR 只读 UART 字节并写入环形缓冲区。
- ISR 不做字符串比较、不打印、不调用求解器、不刷新屏幕。

## OpenART 架构

在当前地图识别脚本中增加 UART 输出开关和发送周期：

- `UART_ENABLE`
- `UART_ID = 12`
- `UART_BAUDRATE = 115200`
- `UART_SEND_PERIOD_MS`

发送内容使用稳定后的字符地图，避免单帧跳动直接传给 MCU。发送动作与调试打印分离，调试打印关闭时 UART 仍可发送。

OpenART 启动时先尝试加载 `cmm_load`，再初始化 `UART(12)`。如果 UART 初始化失败，应在 OpenMV IDE 打印明确错误，不影响视觉调试。

## 数据流

1. OpenART 完成一帧识别。
2. 视觉稳定器输出稳定字符地图。
3. OpenART 到达发送周期后通过 `UART(12)` 发送 `MAP_BEGIN`、12 行地图、`MAP_END`。
4. RT1064 UART1 中断收到字节，写入 OpenART 接收环形缓冲区。
5. RT1064 主循环从缓冲区取字节，按行构建协议状态机。
6. 收到合法完整帧后复制到最新地图缓存，递增成功帧计数。
7. 后续菜单或求解入口读取最新地图并转换为 `map_source_struct` 风格输入。

## 错误处理

- 行过长：丢弃当前行，并重置到等待 `MAP_BEGIN`。
- 地图行字符非法：丢弃当前帧。
- 地图行数不是 12：丢弃当前帧。
- `MAP_END` 前收到新的 `MAP_BEGIN`：重启新帧，便于从串口丢字节后快速恢复。
- 环形缓冲区满：丢弃新字节或清空半帧，并递增错误计数。

## 验证计划

OpenART 侧：

- 确认启动打印显示 UART(12)、115200 初始化成功。
- 在 OpenMV IDE 观察地图识别仍正常，发送周期不显著拉低 FPS。
- 临时接 USB-TTL 时可以看到完整文本帧。

RT1064 侧：

- 编译通过。
- 接入 OpenART 后，收到合法帧时无线串口或屏幕显示成功计数增加。
- 断开 OpenART 或发送残缺帧时，主循环不阻塞，底盘控制中断不受影响。
- 发送带 `X` 的地图时，RT1064 能保存字符，不误判为协议错误。

联调：

- 使用 12x16 固定测试帧验证 `MAP_OK rows=12 cols=16`。
- 使用当前视觉地图验证 `# . B T C X` 全部能通过协议。
- 观察长时间运行时成功帧计数持续增加，错误计数不持续增长。

## 已确认决策

- 采用 OpenART 定时主动发送，MCU 中断接收 + 主循环解析。
- UART1 专门接 OpenART RX。
- `printf` 继续主要用于无线串口调试。
- 第一版使用文本协议，不做二进制压缩。
