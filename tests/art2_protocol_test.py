import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "openmv" / "视觉" / "main.py"
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

NAMES = {
    "ZOOM",
    "CARTOON_MODEL",
    "NUMBER_MODEL",
    "CARTOON_LABELS",
    "NUMBER_LABELS",
    "CLASS_CONFIDENCE_Q_MIN",
    "CLASS_STABLE_FRAMES",
    "MODE_NONE",
    "MODE_BOX",
    "MODE_TARGET",
    "UART_RX_LINE_MAX",
    "DEBUG_REALTIME_ENABLE",
    "DEBUG_REALTIME_MODE",
    "vision_mode",
    "pending_mode",
    "active_net",
    "active_labels",
    "classification_candidate",
    "classification_stable_count",
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
    "debug_realtime_mode",
    "release_active_model",
    "activate_pending_mode",
    "reset_classification_stability",
    "classification_stability_push",
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


class FakeTf:
    def __init__(self):
        self.loads = []
        self.free_count = 0
        self.fail_path = None

    def load(self, path, load_to_fb=False):
        if path == self.fail_path:
            raise OSError("model load failed")
        self.loads.append((path, load_to_fb))
        return "net:" + path

    def free_from_fb(self):
        self.free_count += 1


class FakeGc:
    def collect(self):
        pass


def fake_open(path):
    return ["class_%d\n" % index for index in range(10)]


fake_tf = FakeTf()
namespace["tf"] = fake_tf
namespace["gc"] = FakeGc()
namespace["open"] = fake_open

uart = FakeUart()
namespace["parse_uart_line"]("VISION_MODE BOX", uart)
assert namespace["vision_mode"] == namespace["MODE_NONE"]
assert namespace["pending_mode"] == namespace["MODE_BOX"]
assert uart.tx == []
assert namespace["activate_pending_mode"](uart) is True
assert namespace["vision_mode"] == namespace["MODE_BOX"]
assert namespace["active_net"] == "net:/sd/cartoon_V3.tflite"
assert fake_tf.loads == [("/sd/cartoon_V3.tflite", True)]
assert uart.tx[-1] == "VISION_READY BOX\n"

namespace["parse_uart_line"]("VISION_MODE BOX", uart)
assert fake_tf.loads == [("/sd/cartoon_V3.tflite", True)]
assert uart.tx[-1] == "VISION_READY BOX\n"

namespace["parse_uart_line"]("VISION_REQ 17", uart)
assert namespace["request_active"] is True
assert namespace["request_id"] == 17
assert namespace["sample_id"] == 0

tx_count = len(uart.tx)
assert namespace["process_classification"](uart, 4, 749) is False
assert len(uart.tx) == tx_count
assert namespace["sample_id"] == 0
for _ in range(4):
    namespace["process_classification"](uart, 4, 750)
assert len(uart.tx) == tx_count
assert namespace["classification_stable_count"] == 4
assert namespace["process_classification"](uart, 4, 749) is False
assert namespace["classification_stable_count"] == 0
for _ in range(4):
    namespace["process_classification"](uart, 4, 750)
assert len(uart.tx) == tx_count
namespace["process_classification"](uart, 8, 932)
assert namespace["classification_stable_count"] == 1
assert len(uart.tx) == tx_count
for _ in range(4):
    sent = namespace["process_classification"](uart, 8, 932)
assert sent is True
assert uart.tx[-1] == "VISION_SAMPLE 17 1 8 932\n"
namespace["process_classification"](uart, 8, 932)
namespace["process_classification"](uart, 8, 947)
assert uart.tx[-2:] == [
    "VISION_SAMPLE 17 2 8 932\n",
    "VISION_SAMPLE 17 3 8 947\n",
]

namespace["parse_uart_line"]("VISION_ACK 16", uart)
assert namespace["request_active"] is True
namespace["parse_uart_line"]("VISION_ACK 17", uart)
assert namespace["request_active"] is False

namespace["parse_uart_line"]("VISION_MODE TARGET", uart)
assert namespace["vision_mode"] == namespace["MODE_BOX"]
assert namespace["pending_mode"] == namespace["MODE_TARGET"]
assert namespace["activate_pending_mode"](uart) is True
assert namespace["vision_mode"] == namespace["MODE_TARGET"]
assert namespace["active_net"] == "net:/sd/number_V1.tflite"
assert fake_tf.free_count == 1
assert fake_tf.loads[-1] == ("/sd/number_V1.tflite", True)
assert uart.tx[-1] == "VISION_READY TARGET\n"

uart.rx.extend(b"VISION_REQ 18\r\nVISION_CANCEL\n")
namespace["poll_uart_rx"](uart)
assert namespace["request_id"] == 18
assert namespace["request_active"] is False

assert namespace["ZOOM"] == 1.0
assert namespace["CLASS_CONFIDENCE_Q_MIN"] == 750
assert namespace["CLASS_STABLE_FRAMES"] == 5
assert namespace["DEBUG_REALTIME_ENABLE"] is False
assert namespace["debug_realtime_mode"]() == namespace["MODE_BOX"]
namespace["DEBUG_REALTIME_MODE"] = "TARGET"
assert namespace["debug_realtime_mode"]() == namespace["MODE_TARGET"]

fake_tf.fail_path = "/sd/cartoon_V3.tflite"
namespace["parse_uart_line"]("VISION_MODE BOX", uart)
ready_count = len(uart.tx)
assert namespace["activate_pending_mode"](uart) is False
assert len(uart.tx) == ready_count + 1
assert uart.tx[-1] == "VISION_ERROR ASSET\n"
assert namespace["vision_mode"] == namespace["MODE_NONE"]
assert namespace["active_net"] is None

assert 'CARTOON_MODEL = "/sd/cartoon_V3.tflite"' in SOURCE
assert 'NUMBER_MODEL = "/sd/number_V1.tflite"' in SOURCE
assert 'CARTOON_LABELS = "/sd/cartoon_labels.txt"' in SOURCE
assert 'NUMBER_LABELS = "/sd/number_labels.txt"' in SOURCE
assert "DEBUG_CLASSIFY_PRINT = True" in SOURCE
assert "sensor.set_windowing((offset_x, offset_y, crop_w, crop_h))" in SOURCE
assert SOURCE.index("sensor.set_vflip(True)") < SOURCE.index("set_zoom(ZOOM)")
assert SOURCE.index("set_zoom(ZOOM)") < SOURCE.index("sensor.set_hmirror(True)")
assert '"VISION_RESULT mode=%s class=%d label=%s mapped_digit=%d confidence=%.3f"' in SOURCE
assert '"VISION_REJECT mode=%s class=%d label=%s confidence=%.3f"' in SOURCE
assert '"VISION_PENDING mode=%s class=%d stable=%d/%d confidence=%.3f"' in SOURCE
assert "active_net = tf.load(model_path, load_to_fb=True)" in SOURCE
assert "tf.free_from_fb()" in SOURCE
assert "open(labels_path)" in SOURCE
assert "result = tf.classify(active_net, img)[0]" in SOURCE
assert "VISION_READY BOOT\\n" in SOURCE

print("art2-protocol PASS")
