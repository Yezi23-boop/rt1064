import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OPENMV_SOURCE = (ROOT / "openmv" / "main_see.py").read_text(encoding="utf-8")
ART_REPLAN_SOURCE = (ROOT / "project" / "user" / "src" / "art_replan.c").read_text(
    encoding="utf-8"
)
ART_OBSERVATION_SOURCE = (
    ROOT / "project" / "user" / "src" / "art_observation.c"
).read_text(encoding="utf-8")
SUBJECT2_SOURCE = (ROOT / "project" / "user" / "src" / "subject2.c").read_text(
    encoding="utf-8"
)
MENU_SOURCE = (ROOT / "project" / "user" / "src" / "menu.c").read_text(
    encoding="utf-8"
)


def function_body(source, name, next_marker):
    start = source.index(name)
    end = source.index(next_marker, start)
    return source[start:end]


assert "def poll_map_uart_rx" in OPENMV_SOURCE
assert '"CENTER_REQ"' in OPENMV_SOURCE
assert '"CENTER_SAMPLE %d,%d,%d,%d,%d\\n"' in OPENMV_SOURCE
assert "CENTER_REQ_TIMEOUT" not in OPENMV_SOURCE

main_body = function_body(
    OPENMV_SOURCE,
    "def main():",
    "\n\nmain()",
)
assert "center_map_sent = process_center_request(" in main_body
assert "canonical_char_matrix = build_canonical_player_map(" in main_body
assert "send_map_uart(map_uart, canonical_char_matrix" in main_body
assert "send_map_uart(map_uart, char_matrix" not in main_body
periodic_send_body = main_body[main_body.index("if (UART_MAP_SEND_ENABLE"):]
assert "not center_request_active" not in periodic_send_body
assert "not observation_request_active" not in periodic_send_body
assert "DEBUG_PLAYER_CENTER_ENABLE =" in OPENMV_SOURCE
assert "player_count != 1" in main_body
assert "resolve_player_center(" in main_body
assert "PLAYER_CENTER_LOST_FRAME_LIMIT" in OPENMV_SOURCE
assert "player_center_lost_frames < PLAYER_CENTER_LOST_FRAME_LIMIT" in main_body
assert "last_precise_player_grid = None" in main_body

center_request_body = function_body(
    OPENMV_SOURCE,
    "def process_center_request(",
    "def process_observation_request(",
)
assert "if player_center_grid is None:" in center_request_body
assert "send_map_uart(uart, canonical_char_matrix, None)" in center_request_body
paired_map_send = center_request_body.index(
    "send_map_uart(uart, canonical_char_matrix, player_center_grid)"
)
center_sample_send = center_request_body.index(
    'uart.write("CENTER_SAMPLE %d,%d,%d,%d,%d\\n"'
)
assert paired_map_send < center_sample_send
assert "center_map_sent = process_center_request(" in main_body
assert "if center_map_sent:" in main_body
assert "last_uart_send_ms = now_ms" in main_body

request_branch = main_body.index("need_precise_player_center =")
resolver_call = main_body.index("player_center_result = resolve_player_center(")
request_process = main_body.index("center_map_sent = process_center_request(")
assert request_branch < resolver_call < request_process


tree = ast.parse(OPENMV_SOURCE)
selected_nodes = []
selected_names = {
    "UART_MAP_RX_ENABLE",
    "GRID_COLS",
    "GRID_ROWS",
    "CENTER_SAMPLE_COUNT",
    "OBSERVATION_SAMPLE_COUNT",
    "UART_RX_LINE_MAX",
    "center_request_active",
    "center_request_sample_count",
    "center_request_generation",
    "observation_request_active",
    "observation_request_row",
    "observation_request_col",
    "observation_request_sample_count",
    "observation_request_generation",
    "map_uart_rx_line",
}
selected_functions = {
    "parse_map_uart_line",
    "poll_map_uart_rx",
    "process_center_request",
    "process_observation_request",
    "send_map_uart",
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
assert namespace["CENTER_SAMPLE_COUNT"] == 5
class FakeUart:
    def __init__(self):
        self.rx = bytearray()
        self.tx = []
        self.inject_after_map_end = None

    def any(self):
        return len(self.rx)

    def read(self, count):
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, text):
        self.tx.append(text)
        if text == "MAP_END\n" and self.inject_after_map_end is not None:
            self.rx.extend(self.inject_after_map_end)
            self.inject_after_map_end = None


uart = FakeUart()
char_matrix = [["." for _ in range(16)] for _ in range(12)]
char_matrix[5][5] = "C"
uart.rx.extend(b"MAP_OK rows=12 cols=16\r\nCENTER_REQ\n")
namespace["poll_map_uart_rx"](uart)
assert namespace["center_request_active"] is True
first_generation = namespace["center_request_generation"]

namespace["process_center_request"](uart, None, char_matrix, None)
assert uart.tx[-2] == "PLAYER_CENTER_GRID 0,0 0\n"
assert uart.tx[-1] == "MAP_END\n"
namespace["process_center_request"](uart, (568, 550), char_matrix, 176.19)
namespace["process_center_request"](uart, (570, 550), char_matrix, None)
namespace["process_center_request"](uart, (572, 551), char_matrix, 177.46)
namespace["process_center_request"](uart, (574, 552), char_matrix, 180.0)
namespace["process_center_request"](uart, (576, 553), char_matrix, 359.999)
namespace["process_center_request"](uart, (999, 999), char_matrix, 180.0)
expected_samples = [
    "CENTER_SAMPLE 1,568,550,17619,1\n",
    "CENTER_SAMPLE 2,570,550,0,0\n",
    "CENTER_SAMPLE 3,572,551,17746,1\n",
    "CENTER_SAMPLE 4,574,552,18000,1\n",
    "CENTER_SAMPLE 5,576,553,0,1\n",
]
sample_lines = [line for line in uart.tx if line.startswith("CENTER_SAMPLE ")]
assert sample_lines == expected_samples
for sample_line in expected_samples:
    sample_position = uart.tx.index(sample_line)
    assert uart.tx[sample_position - 2].startswith("PLAYER_CENTER_GRID ")
    assert uart.tx[sample_position - 1] == "MAP_END\n"
assert namespace["center_request_active"] is False

namespace["parse_map_uart_line"]("CENTER_REQ")
assert namespace["center_request_generation"] == first_generation + 1
namespace["process_center_request"](uart, (580, 560), char_matrix, 90.0)
assert uart.tx[-1] == "CENTER_SAMPLE 1,580,560,9000,1\n"

namespace["parse_map_uart_line"]("OBSERVE_REQ 5,6")
assert namespace["observation_request_active"] is True
assert namespace["center_request_active"] is False
namespace["parse_map_uart_line"]("CENTER_REQ")
assert namespace["center_request_active"] is True
assert namespace["observation_request_active"] is False

uart.tx = []
uart.inject_after_map_end = b"OBSERVE_REQ 5,6\n"
namespace["process_center_request"](uart, (580, 560), char_matrix, 90.0)
assert any(line == "MAP_END\n" for line in uart.tx)
assert not any(line.startswith("CENTER_SAMPLE ") for line in uart.tx)
assert namespace["center_request_active"] is False
assert namespace["observation_request_active"] is True

uart.tx = []
uart.inject_after_map_end = b"CENTER_REQ\n"
namespace["process_observation_request"](
    uart, char_matrix, (580, 560), (650, 550))
assert any(line == "MAP_END\n" for line in uart.tx)
assert not any(line.startswith("OBSERVE_SAMPLE ") for line in uart.tx)
assert namespace["observation_request_active"] is False
assert namespace["center_request_active"] is True

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
assert "EXEC_ART_SYNC_TIMEOUT_MS" in pre_push_body
assert "executor_continue_after_pre_push_center()" in pre_push_body
assert "executor_start_pre_push_alignment" in pre_push_body
assert "openart_get_requested_center_sample" in ART_OBSERVATION_SOURCE
assert "openart_get_requested_center_map()" in pre_push_body
assert "map_validate_player_center" in pre_push_body
pre_push_box_body = function_body(
    ART_REPLAN_SOURCE,
    "static void art_replan_tick_pre_push_box(",
    "static void art_replan_tick_pre_push_center(",
)
assert "art_box_observation_session_request" in begin_body
assert "openart_request_observation" in ART_OBSERVATION_SOURCE
assert "art_box_observation_session_collect" in pre_push_box_body
assert "openart_get_observation_sample" in ART_OBSERVATION_SOURCE
assert "executor_start_pre_push_box_preparation" in pre_push_box_body
assert '"GridPush"' in pre_push_box_body
assert '"E:BGeo"' not in pre_push_box_body
assert "executor_continue_after_pre_push_center()" in pre_push_box_body
assert "art_replan_fail_pre_push_box" in pre_push_box_body
assert '"E:BObs"' not in pre_push_box_body
assert '"E:BTim"' not in pre_push_box_body
assert "executor_get_pre_push_box_prefetch_request" not in ART_REPLAN_SOURCE
assert "subject2_tick_box_prefetch" not in SUBJECT2_SOURCE
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
assert "EXEC_ART_SYNC_TIMEOUT_MS" in wait_center_body
assert "RECOVERY_RESYNC_TIMEOUT_MS" in return_center_body

launch_move_body = function_body(
    ART_REPLAN_SOURCE,
    "static void art_replan_tick_launch_move(",
    "static uint16 art_u16_difference",
)
return_axis_body = function_body(
    ART_REPLAN_SOURCE,
    "static void art_replan_tick_return_axis(",
    "static uint8 art_replan_solve_return_gate",
)
assert "executor_start_position_correction_with_pose_reset" in ART_REPLAN_SOURCE
assert "executor_start_position_correction_with_pose_reset" in SUBJECT2_SOURCE
assert "set_motion(" not in launch_move_body
assert "set_motion(" not in return_axis_body

new_competition_reset = MENU_SOURCE.index("art_replan_reset_competition_yaw();")
competition_start = MENU_SOURCE.index("competition_flow_start(COMPETITION_MODE);")
subject2_yaw_capture = MENU_SOURCE.index(
    "competition_launch_yaw_deg = get_control_status()->target_yaw;"
)
subject2_context_build = MENU_SOURCE.index(
    "build_subject2_context(&subject2_context);", subject2_yaw_capture
)
assert new_competition_reset < competition_start
assert subject2_yaw_capture < subject2_context_build
assert "art_replan_get_launch_yaw_bias(&context->art_yaw_bias_deg)" in MENU_SOURCE
assert "SUBJECT2_POST_OBSERVE_YAW_SAMPLE" in SUBJECT2_SOURCE
assert "SUBJECT2_POST_OBSERVE_YAW_FIX" in SUBJECT2_SOURCE
assert 'update->run_state = "VYaw"' in SUBJECT2_SOURCE
assert 'update->run_state = "VFix"' in SUBJECT2_SOURCE

print("openart-request-flow PASS")
