import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OPENMV_SOURCE = (ROOT / "openmv" / "main_see.py").read_text(encoding="utf-8")
ART_REPLAN_SOURCE = (ROOT / "project" / "user" / "src" / "art_replan.c").read_text(
    encoding="utf-8"
)


def function_body(source, name, next_marker):
    start = source.index(name)
    end = source.index(next_marker, start)
    return source[start:end]


assert "def poll_map_uart_rx" in OPENMV_SOURCE
assert '"CENTER_REQ"' in OPENMV_SOURCE
assert '"CENTER_SAMPLE %d,%d,%d\\n"' in OPENMV_SOURCE
assert "CENTER_REQ_TIMEOUT" not in OPENMV_SOURCE

main_body = function_body(
    OPENMV_SOURCE,
    "def main():",
    "\n\nmain()",
)
assert "process_center_request(map_uart, precise_player_center" in main_body
assert "send_map_uart(map_uart, char_matrix, None)" in main_body
assert "\n            player_center_grid = player_center_to_grid_q" not in main_body
assert "DEBUG_PLAYER_CENTER_ENABLE =" in OPENMV_SOURCE
assert "if center_request_active or DEBUG_PLAYER_CENTER_ENABLE:" in main_body

request_branch = main_body.index("if center_request_active or DEBUG_PLAYER_CENTER_ENABLE:")
blob_call = main_body.index("blob_player_center = detect_player_center(")
precise_call = main_body.index("precise_player_center = detect_player_center_precise(")
request_process = main_body.index("process_center_request(map_uart, precise_player_center")
assert request_branch < blob_call < precise_call < request_process


tree = ast.parse(OPENMV_SOURCE)
selected_nodes = []
selected_names = {
    "UART_MAP_RX_ENABLE",
    "CENTER_SAMPLE_COUNT",
    "UART_RX_LINE_MAX",
    "center_request_active",
    "center_request_sample_count",
    "center_request_generation",
    "map_uart_rx_line",
}
selected_functions = {
    "parse_map_uart_line",
    "poll_map_uart_rx",
    "process_center_request",
}
for node in tree.body:
    if isinstance(node, (ast.Assign, ast.AnnAssign)):
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if any(isinstance(target, ast.Name) and target.id in selected_names for target in targets):
            selected_nodes.append(node)
    elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        if node.name in selected_functions:
            selected_nodes.append(node)

namespace = {}
exec(compile(ast.Module(body=selected_nodes, type_ignores=[]), "main_see.py", "exec"), namespace)
namespace["player_center_to_grid_q"] = lambda center, rectified, transform: center


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
uart.rx.extend(b"MAP_OK rows=12 cols=16\r\nCENTER_REQ\n")
namespace["poll_map_uart_rx"](uart)
assert namespace["center_request_active"] is True
first_generation = namespace["center_request_generation"]

namespace["process_center_request"](uart, None, True, None)
assert uart.tx == []
namespace["process_center_request"](uart, (568, 550), True, None)
namespace["process_center_request"](uart, (570, 550), True, None)
namespace["process_center_request"](uart, (572, 551), True, None)
namespace["process_center_request"](uart, (999, 999), True, None)
assert uart.tx == [
    "CENTER_SAMPLE 1,568,550\n",
    "CENTER_SAMPLE 2,570,550\n",
    "CENTER_SAMPLE 3,572,551\n",
]
assert namespace["center_request_active"] is False

namespace["parse_map_uart_line"]("CENTER_REQ")
assert namespace["center_request_generation"] == first_generation + 1
namespace["process_center_request"](uart, (580, 560), True, None)
assert uart.tx[-1] == "CENTER_SAMPLE 1,580,560\n"

begin_body = function_body(
    ART_REPLAN_SOURCE,
    "static void art_replan_begin(",
    "static void art_replan_restart_stability",
)
center_request_begin_body = function_body(
    ART_REPLAN_SOURCE,
    "static void art_replan_begin_center_request(",
    "static void art_replan_begin(",
)
assert "openart_request_player_center();" in center_request_begin_body
assert "art_replan_begin_center_request" in begin_body

pre_push_body = function_body(
    ART_REPLAN_SOURCE,
    "static void art_replan_tick_pre_push_center(",
    "static void art_replan_begin_wait_center",
)
assert "EXEC_ART_SYNC_TIMEOUT_MS" not in pre_push_body
assert "openart_get_requested_center_sample" in ART_REPLAN_SOURCE
assert "art_replan_get_pre_push_reference_cell" in ART_REPLAN_SOURCE
assert "art_replan_get_pre_push_reference_cell(context" in pre_push_body
assert "ART_REPLAN_INITIAL_CENTER" in ART_REPLAN_SOURCE
assert "ART_REPLAN_SEGMENT_CENTER" in ART_REPLAN_SOURCE
assert "art_replan_begin_center_request" in ART_REPLAN_SOURCE
assert "art_replan_calculate_pose_offset" in ART_REPLAN_SOURCE
assert "openart_get_player_center" not in ART_REPLAN_SOURCE

wait_center_body = function_body(
    ART_REPLAN_SOURCE,
    "static void art_replan_tick_wait_center(",
    "static void art_replan_tick_launch_move",
)
return_center_body = function_body(
    ART_REPLAN_SOURCE,
    "static void art_replan_tick_return_center(",
    "static void art_replan_tick_return_axis",
)
assert "EXEC_ART_SYNC_TIMEOUT_MS" not in wait_center_body
assert "EXEC_ART_SYNC_TIMEOUT_MS" not in return_center_body

print("openart-request-flow PASS")
