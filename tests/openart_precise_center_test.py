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
    "MIN_PLAYER_SUM",
    "PLAYER_CENTER_SEARCH_RADIUS",
    "PLAYER_PRECISE_MIN_COLOR_PIXELS",
    "PLAYER_PRECISE_TRIM_PERCENT",
    "PLAYER_HEADING_CORE_RADIUS",
    "PLAYER_HEADING_BOUNDARY_MAX_GAP",
    "PLAYER_HEADING_BOUNDARY_NORMAL_BAND",
    "PLAYER_HEADING_BOUNDARY_MIN_POINTS",
    "PLAYER_HEADING_BOUNDARY_MIN_AXIS_RATIO",
}
FUNCTIONS = {
    "normalize_color",
    "is_player_green_half",
    "is_player_cyan_half",
    "trimmed_histogram_bounds",
    "histogram_core_mean_coordinate",
    "player_heading_centers_from_boundary",
    "detect_player_pose_precise",
    "detect_player_center_precise",
}
nodes = []
for node in TREE.body:
    if isinstance(node, ast.Assign):
        if any(isinstance(target, ast.Name) and target.id in CONSTANTS for target in node.targets):
            nodes.append(node)
    elif isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
        nodes.append(node)

namespace = {"FRAME_SCALE": 1, "math": math}
exec(compile(ast.Module(body=nodes, type_ignores=[]), "main_see.py", "exec"), namespace)


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
    for x in range(29, 40):
        image.putpixel((x, y), (0, 255, 0))
for y in range(22, 32):
    for x in range(22, 30):
        image.putpixel((x, y), (0, 160, 160))
for y in range(23, 31):
    for x in range(34, 41):
        image.putpixel((x, y), (0, 160, 160))
for y in range(24, 42):
    for x in range(42, 58):
        image.putpixel((x, y), (255, 255, 0))

center = namespace["detect_player_center_precise"](
    ImageAdapter(image), (30, 31)
)
assert center is not None
assert abs(center[0] - 31) <= 1
assert abs(center[1] - 31) <= 1

blank = Image.new("RGB", (64, 64), (0, 0, 255))
assert namespace["detect_player_center_precise"](
    ImageAdapter(blank), (30, 31)
) is None

print("openart-precise-center PASS")
