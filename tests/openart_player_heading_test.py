import ast
import math
from pathlib import Path

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "openmv" / "main_see.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

CONSTANTS = {
    "GRID_COLS",
    "GRID_ROWS",
    "IMG_WIDTH",
    "IMG_HEIGHT",
    "GRID_LEFT_MARGIN",
    "GRID_RIGHT_MARGIN",
    "GRID_TOP_MARGIN",
    "GRID_BOTTOM_MARGIN",
    "MIN_PLAYER_SUM",
    "PLAYER_CENTER_SEARCH_RADIUS",
    "PLAYER_PRECISE_MIN_COLOR_PIXELS",
    "PLAYER_PRECISE_TRIM_PERCENT",
    "PLAYER_HEADING_CORE_RADIUS",
    "PLAYER_HEADING_BOUNDARY_MAX_GAP",
    "PLAYER_HEADING_BOUNDARY_NORMAL_BAND",
    "PLAYER_HEADING_BOUNDARY_MIN_POINTS",
    "PLAYER_HEADING_BOUNDARY_MIN_AXIS_RATIO",
    "PLAYER_HEADING_MIN_VECTOR_Q",
    "PLAYER_HEADING_MAX_VECTOR_Q",
    "PLAYER_HEADING_SAMPLE_COUNT",
    "PLAYER_HEADING_MIN_VALID_SAMPLES",
    "PLAYER_HEADING_OUTLIER_MAX_DEG",
}
FUNCTIONS = {
    "normalize_color",
    "is_player_green_half",
    "is_player_cyan_half",
    "trimmed_histogram_bounds",
    "histogram_core_mean_coordinate",
    "player_heading_centers_from_boundary",
    "detect_player_pose_precise",
    "image_center_to_grid_q",
    "shortest_heading_error_deg",
    "player_heading_to_yaw_deg",
    "filter_player_heading_samples",
}

nodes = []
found_functions = set()
for node in TREE.body:
    if isinstance(node, ast.Assign):
        if any(isinstance(target, ast.Name) and target.id in CONSTANTS
               for target in node.targets):
            nodes.append(node)
    elif isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
        nodes.append(node)
        found_functions.add(node.name)

assert FUNCTIONS.issubset(found_functions), "player heading helpers are missing"

namespace = {"FRAME_SCALE": 1, "math": math}
exec(compile(ast.Module(body=nodes, type_ignores=[]),
             "main_see.py", "exec"), namespace)


class ImageAdapter:
    def __init__(self, image):
        self.image = image

    def width(self):
        return self.image.width

    def height(self):
        return self.image.height

    def get_pixel(self, x, y):
        return self.image.getpixel((x, y))


image = Image.new("RGB", (64, 64), (0, 0, 255))
for y in range(32, 42):
    for x in range(25, 36):
        image.putpixel((x, y), (0, 255, 0))
for y in range(22, 32):
    for x in range(25, 36):
        image.putpixel((x, y), (0, 200, 200))

pose = namespace["detect_player_pose_precise"](
    ImageAdapter(image), (30, 31))
assert pose is not None
_, green_center, cyan_center = pose
yaw_deg = namespace["player_heading_to_yaw_deg"](
    green_center, cyan_center, True, None)
assert yaw_deg is not None
assert abs(yaw_deg - 180.0) < 0.01

# 蓝底高亮可能落入青色阈值。它位于车体搜索框一侧时，不应把正上车头拉偏。
for y in range(22, 30):
    image.putpixel((18, y), (0, 170, 200))
pose_with_cyan_noise = namespace["detect_player_pose_precise"](
    ImageAdapter(image), (30, 31))
assert pose_with_cyan_noise is not None
_, noisy_green_center, noisy_cyan_center = pose_with_cyan_noise
noisy_yaw_deg = namespace["player_heading_to_yaw_deg"](
    noisy_green_center, noisy_cyan_center, True, None)
assert noisy_yaw_deg is not None
assert abs(noisy_yaw_deg - 180.0) < 0.01

heading = namespace["player_heading_to_yaw_deg"]
assert abs(heading((100, 100), (100, 110), True, None) - 0.0) < 0.01
assert abs(heading((100, 100), (110, 100), True, None) - 90.0) < 0.01
assert abs(heading((100, 100), (100, 90), True, None) - 180.0) < 0.01
assert abs(heading((100, 100), (90, 100), True, None) - 270.0) < 0.01
assert heading((100, 100), (101, 100), True, None) is None
assert heading((100, 100), (120, 100), True, None) is None

filter_samples = namespace["filter_player_heading_samples"]
filtered = filter_samples([180.22, 180.30, 181.98, 179.56, 187.65])
assert filtered is not None
assert filtered[1] == 4
assert abs(filtered[0] - 180.52) < 0.02

wrapped = filter_samples([359.0, 0.0, 1.0, 2.0, 20.0])
assert wrapped is not None
assert wrapped[1] == 4
assert (wrapped[0] < 2.0 or wrapped[0] > 359.0)

assert filter_samples([180.0, 181.0, 179.0, 200.0, 220.0]) is None

assert filter_samples([180.0, 181.0, 179.0, 180.5]) is None


def build_marker_labels(yaw_deg, size=31, half_size=8):
    labels = bytearray(size * size)
    yaw_rad = yaw_deg * math.pi / 180.0
    heading_x = math.sin(yaw_rad)
    heading_y = math.cos(yaw_rad)
    side_x = math.cos(yaw_rad)
    side_y = -math.sin(yaw_rad)
    if abs(heading_x) < 0.000001:
        heading_x = 0.0
    if abs(heading_y) < 0.000001:
        heading_y = 0.0
    if abs(side_x) < 0.000001:
        side_x = 0.0
    if abs(side_y) < 0.000001:
        side_y = 0.0
    center = (size - 1) / 2.0
    green_points = []
    cyan_points = []
    for y in range(size):
        for x in range(size):
            dx = x - center
            dy = y - center
            forward = dx * heading_x + dy * heading_y
            side = dx * side_x + dy * side_y
            if abs(forward) > half_size or abs(side) > half_size:
                continue
            if forward >= 0.0:
                labels[y * size + x] = 2
                cyan_points.append((x, y))
            else:
                labels[y * size + x] = 1
                green_points.append((x, y))

    green = (sum(point[0] for point in green_points) / len(green_points),
             sum(point[1] for point in green_points) / len(green_points))
    cyan = (sum(point[0] for point in cyan_points) / len(cyan_points),
            sum(point[1] for point in cyan_points) / len(cyan_points))
    return labels, green, cyan


boundary_heading = namespace["player_heading_centers_from_boundary"]
for expected_yaw in (0.0, 5.0, 10.0, 90.0, 180.0, 270.0, 355.0):
    labels, green, cyan = build_marker_labels(expected_yaw)
    result = boundary_heading(labels, 31, 31, 0, 0, green, cyan)
    assert result is not None
    refined_green, refined_cyan = result
    refined_dx = refined_cyan[0] - refined_green[0]
    refined_dy = refined_cyan[1] - refined_green[1]
    refined_yaw = math.atan2(refined_dx, refined_dy) * 180.0 / math.pi
    if refined_yaw < 0.0:
        refined_yaw += 360.0
    assert abs(namespace["shortest_heading_error_deg"](
        expected_yaw, refined_yaw)) < 1.5

assert boundary_heading(bytearray(31 * 31), 31, 31, 0, 0,
                        (15.0, 20.0), (15.0, 10.0)) is None

assert 'uart.write("PLAYER_YAW' not in SOURCE

print("openart-player-heading PASS yaw=%.2f filtered=%.2f accepted=%d" %
      (yaw_deg, filtered[0], filtered[1]))
