import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "openmv" / "视觉" / "main.py"
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

NAMES = {
    "MODE_NONE",
    "MODE_BOX",
    "MODE_TARGET",
    "UART_RX_LINE_MAX",
    "vision_mode",
    "request_active",
    "request_id",
    "sample_id",
    "uart_rx_line",
}
FUNCTIONS = {
    "reset_request",
    "parse_uart_line",
    "poll_uart_rx",
    "process_classification",
}

nodes = []
for node in TREE.body:
    if isinstance(node, ast.Assign):
        if any(isinstance(target, ast.Name) and target.id in NAMES for target in node.targets):
            nodes.append(node)
    elif isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
        nodes.append(node)

namespace = {}
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE_PATH), "exec"), namespace)


class FakeUart:
    def __init__(self):
        self.rx = bytearray()
        self.tx = []

    def any(self):
        return len(self.rx)

    def read(self, count):
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, text):
        self.tx.append(text)


uart = FakeUart()
namespace["parse_uart_line"]("VISION_MODE BOX", uart)
assert namespace["vision_mode"] == namespace["MODE_BOX"]
assert uart.tx[-1] == "VISION_READY BOX\n"

namespace["parse_uart_line"]("VISION_REQ 17", uart)
assert namespace["request_active"] is True
assert namespace["request_id"] == 17
assert namespace["sample_id"] == 0

namespace["process_classification"](uart, 8, 932)
namespace["process_classification"](uart, 8, 947)
assert uart.tx[-2:] == [
    "VISION_SAMPLE 17 1 8 932\n",
    "VISION_SAMPLE 17 2 8 947\n",
]

namespace["parse_uart_line"]("VISION_ACK 16", uart)
assert namespace["request_active"] is True
namespace["parse_uart_line"]("VISION_ACK 17", uart)
assert namespace["request_active"] is False

namespace["parse_uart_line"]("VISION_MODE TARGET", uart)
assert namespace["vision_mode"] == namespace["MODE_TARGET"]
assert uart.tx[-1] == "VISION_READY TARGET\n"

uart.rx.extend(b"VISION_REQ 18\r\nVISION_CANCEL\n")
namespace["poll_uart_rx"](uart)
assert namespace["request_id"] == 18
assert namespace["request_active"] is False

assert 'CARTOON_MODEL = "/sd/model_in-uint8_out-uint8_channel_ptq.tflite"' in SOURCE
assert 'NUMBER_MODEL = "/sd/numint8_in-int8_out-int8_channel_ptq.tflite"' in SOURCE
assert "tf.load(CARTOON_MODEL, load_to_fb=True)" in SOURCE
assert "tf.load(NUMBER_MODEL, load_to_fb=True)" in SOURCE
assert "VISION_READY BOOT\\n" in SOURCE

print("art2-protocol PASS")
