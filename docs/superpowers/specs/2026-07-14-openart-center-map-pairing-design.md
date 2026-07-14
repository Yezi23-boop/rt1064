# OpenART 请求中心与地图 C 配套设计

## 目标

科目二请求精确中心时，确保 MCU 先收到包含该中心的完整地图帧，再收到对应的 `CENTER_SAMPLE`，避免最新中心与稳定字符地图中的滞后 `C` 混用并触发 `E:CPos`。

## 设计

- 保持 `CENTER_REQ` 和 `CENTER_SAMPLE index,col_q,row_q` 文本格式不变。
- OpenART 每得到一个有效请求中心，先发送 `MAP_BEGIN`、12行地图、同一中心的 `PLAYER_CENTER_GRID`、`MAP_END`，随后发送 `CENTER_SAMPLE`。
- MCU 现有 `openart_uart.c` 会在接受地图时用 `PLAYER_CENTER_GRID` 清理多个或滞后的 `C`，并把中心所在格发布为唯一 `C`。
- 若本轮已发送配套地图，则更新普通地图发送计时，避免循环末尾立即发送 `PLAYER_CENTER_GRID 0,0 0` 的周期地图覆盖它。
- 中心检测失败时不发送地图或样本，保留现有 `E:CTmo` 语义。

## 验证

- 协议测试验证配套地图早于每条 `CENTER_SAMPLE`。
- 第3条样本后请求关闭。
- 无有效中心时不发送任何配套数据。
- `python -m py_compile openmv/main_see.py` 和相关 Python 测试通过。
