# OpenART Plus 到 RT1064 UART 最小通信经验

本文记录 OpenART Plus 与 RT1064 逐飞开发板完成最小 UART 通信验证时的关键经验。目标链路是：

```text
OpenART Plus -> UART 文本 -> RT1064 -> UART 回显 -> OpenART Plus
```

## 使用规则

遇到 OpenART Plus、RT1064、UART、SD 卡 `cmm_cfg/cmm_load`、REPL、串口终端这类类似问题时，先看本文和相关原文资料，再继续排查和改动。

优先顺序：

1. 先看本文里的最小闭环结论。
2. 再看逐飞公众号原文和官方例程。
3. 最后才动脚本、接线或固件。

## 验证结果

历史上的最小通信已经闭环，当前脚本在此基础上升级为最小地图帧协议 smoke test。当前这一版先只做 OpenART 侧发帧，RT1064 回包是后续接入项：

```text
OpenART Plus 发送: HELLO_RT1064\n
RT1064 回显: RX:HELLO_RT1064
OpenMV IDE 实际打印: b'RX:HELLO_RT1064\n\r\n'
```

如果后续 RT1064 接上并完成解析，协议预期回显：

```text
MAP_RX_BEGIN
MAP_OK rows=12 cols=16
```

## 最终接线

```text
OpenART Plus TX -> RT1064 B13
OpenART Plus RX -> RT1064 B12
OpenART Plus GND -> RT1064 GND
baudrate = 115200
```

RT1064 侧使用逐飞库默认调试串口：

```text
UART_1 TX = B12
UART_1 RX = B13
baudrate = 115200
```

注意这是交叉接法：OpenART 的 TX 接 RT1064 的 RX，OpenART 的 RX 接 RT1064 的 TX。

## OpenART Plus 关键结论

不要把 OpenART Plus 直接当成标准 OpenMV Cam 套用 `UART(1)`、`UART(2)` 或 `UART(3)`。

本机这块 OpenART Plus 固件实际表现如下：

```text
UART(1): 不存在
UART(2): 不存在
UART(3): 不存在
UART(12): 进入逐飞引脚映射层
```

逐飞 OpenART Plus 官方基础外设例程使用的是：

```python
from machine import UART

uart = UART(12, baudrate=115200)
```

因此本项目当前 OpenART Plus 用户串口脚本应使用 `UART(12)`。

## SD 卡必备文件

`UART(12)` 初始化依赖 OpenART Plus SD 卡中的逐飞引脚映射文件。如果缺少或版本不匹配，会报类似错误：

```text
ValueError: ERROR: cmm_cfg.csv not found pin information!
```

这次实际原因是 `E:\sd\cmm_cfg.csv` / `E:\sd\cmm_load.py` 是旧版文件，缺少 `UART(12)` 对应的引脚信息。

处理方式：

1. 从逐飞 `OpenART_Plus_Product` 资料里的 `SD卡必备文件` 获取新版：
   - `cmm_cfg.csv`
   - `cmm_load.py`
2. 放到 OpenART Plus 实际会读取的位置：
   - `E:\sd\cmm_cfg.csv`
   - `E:\sd\cmm_load.py`
3. 同时在 `E:\` 根目录也保留一份同版本官方文件：
   - `E:\cmm_cfg.csv`
   - `E:\cmm_load.py`
4. 原旧版文件可先备份：
   - `E:\sd\cmm_cfg.csv.bak_codex`
   - `E:\sd\cmm_load.py.bak_codex`

补充观察：这块固件实际执行日志会显示加载：

```text
Found and execute /sd/cmm_load.py!
loading /sd/cmm_cfg.csv
```

但复测发现，如果只保留 `E:\sd\` 下的文件，`UART(12)` 仍可能报：

```text
ERROR: cmm_cfg.csv not found pin information!
```

因此当前最小稳定配置不是“只保留 `E:\sd`”，而是：

```text
E:\cmm_cfg.csv
E:\cmm_load.py
E:\sd\cmm_cfg.csv
E:\sd\cmm_load.py
```

四个文件都使用 OpenART Plus 官方资料包中的同一版本。

## RT1064 关键结论

Keil / μVision 官方命令行参数已确认：

```text
UV4.exe -b <project.uvprojx>    只编译 Build，不下载
UV4.exe -f <project.uvprojx>    直接下载到 Flash
```

本项目常用形式：

```powershell
D:\Keil_v5\UV4\UV4.exe -b D:\rt1064\RT1064_Library\SeekFree_RT1064_Opensource_Library\project\mdk\rt1064.uvprojx
D:\Keil_v5\UV4\UV4.exe -f D:\rt1064\RT1064_Library\SeekFree_RT1064_Opensource_Library\project\mdk\rt1064.uvprojx
```

Keil 命令行下载成功不等于 RT1064 程序已经在运行。

本次 Keil 命令行下载成功日志：

```text
Erase Done. Programming Done. Verify OK.
Flash Load finished
```

但下载后 μVision 曾留在调试态，并停在 `main()` 入口。此时 RT1064 程序没有真正跑起来，OpenART Plus 发送数据也不会收到回显。

处理方式之一：

```powershell
uvx pyocd reset -t mimxrt1064
```

或者退出 Keil 调试会话后手动复位/重新上电，让 RT1064 从 Flash 启动运行。

## 当前最小 OpenART 脚本

路径：

```text
D:\rt1064\RT1064_Library\SeekFree_RT1064_Opensource_Library\openmv\openart_plus_uart_smoke.py
```

这个脚本是 OpenMV IDE 里临时运行的，不是烧录进板载 `main.py`。内容核心：

```python
from machine import UART
import time

try:
    import cmm_load
    cmm_load.load()
except Exception as exc:
    print("cmm_load skipped or failed:", repr(exc))

UART_ID = 12
BAUDRATE = 115200
FRAME_PERIOD_MS = 1000
MAP_FRAME = (
    "################",
    "#..............#",
    "#..............#",
    "#.....B........#",
    "#..............#",
    "#...C..........#",
    "#..............#",
    "#........T.....#",
    "#..............#",
    "#..............#",
    "#..............#",
    "################",
)

uart = UART(UART_ID, baudrate=BAUDRATE)

while True:
    uart.write("MAP_BEGIN\r\n")
    for row in MAP_FRAME:
        uart.write(row)
        uart.write("\r\n")
    uart.write("MAP_END\r\n")
    time.sleep_ms(FRAME_PERIOD_MS)
```

运行时，OpenMV IDE 终端会持续看到类似：

```text
OPENART_MAP_FRAME_READY UART(12) 115200
MAP_FRAME_SENT 1
MAP_FRAME_SENT 2
...
```

## 当前最小 RT1064 程序

路径：

```text
D:\rt1064\RT1064_Library\SeekFree_RT1064_Opensource_Library\project\user\src\main.c
```

行为：

1. 初始化系统时钟。
2. 初始化逐飞默认 debug UART。
3. 打印启动标识：

```text
RT1064_OPENART_UART_READY
UART_1 TX=B12 RX=B13 BAUD=115200
```

4. 循环读取 UART 接收缓冲区。
5. 收到任意数据后回显：

```text
RX:<received bytes>
```

## 推荐排错顺序

1. 先确认 RT1064 程序真的在运行，而不是只下载成功后停在 Keil 调试态。
2. 确认 OpenART Plus 使用 `UART(12)`，不要再从 `UART(1/2/3)` 开始猜。
3. 确认 `E:\sd\cmm_cfg.csv` 和 `E:\sd\cmm_load.py` 是 OpenART Plus 官方新版。
4. 确认波特率两边都是 `115200`。
5. 确认接线是交叉接法：

```text
OpenART TX -> RT1064 RX(B13)
OpenART RX -> RT1064 TX(B12)
GND -> GND
```

6. 如果 OpenART 初始化成功但无回显，再检查 RT1064 是否被 Keil 占用、是否停在断点/入口、是否需要复位运行。

## 后续升级方向（RT1064 接入后）

当前 OpenART 端会发送：

```text
MAP_BEGIN
16x12 grid payload
MAP_END
```

RT1064 端后续可以按行解析这张帧，并在收到完整 12 行后返回：

```text
MAP_OK rows=12 cols=16
```

后续再继续往下接：

1. 地图解析。
2. 车、箱子、目标、墙提取。
3. 分类阶段箱子-目标映射。
4. 单箱子子地图生成。
5. BFS 求解。
6. 转底盘目标点。
7. 执行后重新识别地图。
