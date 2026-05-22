from machine import UART
import time

try:
    import cmm_load
    cmm_load.load()
except Exception as exc:
    print("cmm_load skipped or failed:", repr(exc))

# 运行方式：在 OpenMV IDE 里打开并运行。
# 这是 PC 侧临时脚本，不写入板载 main.py。
UART_ID = 12
BAUDRATE = 115200
FRAME_PERIOD_MS = 1000
MAP_ROWS = 12
MAP_COLS = 16

# # = 墙, . = 空地, C = 车, B = 箱子, T = 目标
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


def validate_map_frame(frame):
    if len(frame) != MAP_ROWS:
        return False

    for row in frame:
        if len(row) != MAP_COLS:
            return False

    return True


def send_map_frame(uart, frame):
    uart.write("MAP_BEGIN\r\n")
    for row in frame:
        uart.write(row)
        uart.write("\r\n")
    uart.write("MAP_END\r\n")


if not validate_map_frame(MAP_FRAME):
    raise ValueError("MAP_FRAME must be 12x16")

uart = UART(UART_ID, baudrate=BAUDRATE)
print("OPENART_MAP_FRAME_READY UART(%d) %d" % (UART_ID, BAUDRATE))
print("MAP_FRAME_SIZE %d x %d" % (MAP_ROWS, MAP_COLS))

frame_seq = 0
while True:
    send_map_frame(uart, MAP_FRAME)
    frame_seq += 1
    print("MAP_FRAME_SENT %d" % frame_seq)
    time.sleep_ms(FRAME_PERIOD_MS)
