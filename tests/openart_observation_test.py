import ast
from pathlib import Path

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "openmv" / "main_see.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

NAMES = {
    "GRID_COLS",
    "GRID_ROWS",
    "FRAME_SCALE",
    "USE_VGA",
    "OBSERVATION_SAMPLE_COUNT",
    "observation_request_active",
    "observation_request_row",
    "observation_request_col",
    "observation_request_sample_count",
    "observation_request_generation",
    "BOX_CENTER_MIN_COLOR_PIXELS",
    "BOX_YELLOW_BLOB_THRESHOLD",
    "BOX_BLOB_AREA_THRESHOLD",
    "BOX_BLOB_MARGIN",
    "BOX_CELL_MIN_HITS",
    "BOX_CELL_MAX_MEAN_OFFSET",
    "MIN_BOX_GOAL_SUM",
    "BOX_SAMPLE_OFFSETS",
}
FUNCTIONS = {
    "normalize_color",
    "is_box_candidate",
    "trimmed_histogram_bounds",
    "box_cell_has_coverage",
    "split_box_blob_centers",
    "deduplicate_box_centers",
    "detect_box_centers",
    "select_box_center",
    "detect_box_center",
    "parse_map_uart_line",
    "process_observation_request",
}
nodes = []
for node in TREE.body:
    if isinstance(node, ast.Assign):
        if any(isinstance(target, ast.Name) and target.id in NAMES
               for target in node.targets):
            nodes.append(node)
    elif isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
        nodes.append(node)

namespace = {}
exec(compile(ast.Module(body=nodes, type_ignores=[]), "main_see.py", "exec"), namespace)


class FakeUart:
    def __init__(self):
        self.tx = []

    def write(self, text):
        self.tx.append(text)


class ImageAdapter:
    def __init__(self, image, blobs=None):
        self.image = image
        self.blobs = [] if blobs is None else blobs

    def width(self):
        return self.image.width

    def height(self):
        return self.image.height

    def get_pixel(self, x, y):
        return self.image.getpixel((x, y))

    def find_blobs(self, thresholds, roi, pixels_threshold, area_threshold,
                   merge, margin):
        del thresholds, roi, pixels_threshold, area_threshold, merge, margin
        return self.blobs


class Blob:
    def __init__(self, x, y, width, height):
        self._x = x
        self._y = y
        self._width = width
        self._height = height

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._width

    def h(self):
        return self._height

image = Image.new("RGB", (320, 240), (0, 0, 255))
box_center = (175, 107)
for y in range(99, 116):
    for x in range(167, 184):
        if x < 170 or x > 180 or y < 102 or y > 112:
            image.putpixel((x, y), (255, 255, 0))
for y in range(103, 112):
    for x in range(171, 180):
        image.putpixel((x, y), (0, 220, 0))

grid_points = []
for row in range(12):
    for col in range(16):
        grid_points.append((col * 20 + 10, row * 20 + 10))

adapter = ImageAdapter(image, [Blob(167, 99, 17, 17)])
assert namespace["box_cell_has_coverage"](adapter, 170, 110)
assert not namespace["box_cell_has_coverage"](adapter, 190, 110)
detected = namespace["detect_box_center"](adapter, grid_points, 5, 8)
assert detected is not None
assert abs(detected[0] - box_center[0]) <= 1
assert abs(detected[1] - box_center[1]) <= 1

split_centers = namespace["split_box_blob_centers"](
    Blob(204, 121, 40, 23), 20, 20)
assert split_centers == [(214, 132), (234, 132)]

single_center = namespace["split_box_blob_centers"](
    Blob(41, 190, 18, 20), 20, 20)
assert single_center == [(50, 200)]

deduplicated = namespace["deduplicate_box_centers"](
    [(50, 198), (50, 202)], 20, 20)
assert deduplicated == [(50, 200)]
assert namespace["deduplicate_box_centers"](
    [(214, 132), (234, 132)], 20, 20) == [(214, 132), (234, 132)]

element_matrix = [["space" for _ in range(16)] for _ in range(12)]
element_matrix[6][10] = "box"
element_matrix[6][11] = "box"
adjacent_adapter = ImageAdapter(
    image, [Blob(204, 121, 40, 23)])
adjacent_centers = namespace["detect_box_centers"](
    adjacent_adapter, grid_points, element_matrix)
assert adjacent_centers == [(214, 132), (234, 132)]
assert namespace["select_box_center"](
    adjacent_centers, grid_points, 6, 10) == (214, 132)
assert namespace["select_box_center"](
    adjacent_centers, grid_points, 6, 11) == (234, 132)
assert namespace["select_box_center"](
    [(226, 130)], grid_points, 6, 10) == (226, 130)

element_matrix = [["space" for _ in range(16)] for _ in range(12)]
element_matrix[9][2] = "box"
element_matrix[10][2] = "box"
cross_cell_adapter = ImageAdapter(
    image, [Blob(41, 190, 18, 20)])
cross_cell_centers = namespace["detect_box_centers"](
    cross_cell_adapter, grid_points, element_matrix)
assert cross_cell_centers == [(50, 200)]
assert namespace["select_box_center"](
    cross_cell_centers, grid_points, 9, 2) == (50, 200)
assert namespace["select_box_center"](
    cross_cell_centers, grid_points, 10, 2) == (50, 200)

detect_source = ast.get_source_segment(SOURCE, next(
    node for node in TREE.body
    if isinstance(node, ast.FunctionDef) and node.name == "detect_box_centers"
))
assert ".find_blobs(" in detect_source
assert ".get_pixel(" not in detect_source

obscured = image.copy()
obscured.putpixel((174, 114), (0, 220, 0))
assert namespace["box_cell_has_coverage"](ImageAdapter(obscured), 170, 110)

namespace["parse_map_uart_line"]("OBSERVE_REQ 5,8")
assert namespace["observation_request_active"] is True
assert namespace["observation_request_row"] == 5
assert namespace["observation_request_col"] == 8
assert namespace["observation_request_sample_count"] == 0

uart = FakeUart()
process = namespace["process_observation_request"]
process(uart, None, (850, 550))
assert uart.tx == []
process(uart, (750, 550), None)
assert uart.tx == []
process(uart, (750, 550), (850, 550))
process(uart, (751, 550), (851, 550))
process(uart, (752, 551), (852, 551))
process(uart, (753, 551), (853, 551))
assert uart.tx == [
    "OBSERVE_SAMPLE 1,750,550,850,550\n",
    "OBSERVE_SAMPLE 2,751,550,851,550\n",
    "OBSERVE_SAMPLE 3,752,551,852,551\n",
]
assert namespace["observation_request_active"] is False

namespace["parse_map_uart_line"]("OBSERVE_REQ 12,8")
assert namespace["observation_request_active"] is False
namespace["parse_map_uart_line"]("OBSERVE_REQ 5,16")
assert namespace["observation_request_active"] is False

assert "DEBUG_OBSERVATION_ENABLE =" in SOURCE
assert "player_heading_from_halves" not in SOURCE
assert "PLAYER_HEADING" not in SOURCE
print("openart-observation PASS")
