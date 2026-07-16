import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "openmv" / "main_see.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

CONSTANTS = {
    "GRID_COLS",
    "GRID_ROWS",
    "PLAYER_CENTER_SEARCH_RADIUS",
    "PLAYER_BLOB_PIXELS_THRESHOLD",
    "PLAYER_BLOB_AREA_THRESHOLD",
    "PLAYER_BLOB_MARGIN",
    "PLAYER_GREEN_BLOB_THRESHOLD",
    "PLAYER_CYAN_BLOB_THRESHOLD",
    "PLAYER_RECENT_C_FALLBACK_FRAMES",
}
FUNCTIONS = {
    "find_player_coarse_center",
    "select_recent_player_anchor",
    "detect_player_center",
    "resolve_player_center",
}
nodes = []
found_functions = set()
for node in TREE.body:
    if isinstance(node, ast.Assign):
        if any(isinstance(target, ast.Name) and target.id in CONSTANTS for target in node.targets):
            nodes.append(node)
    elif isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
        nodes.append(node)
        found_functions.add(node.name)

assert found_functions == FUNCTIONS, "shared player center resolver is missing"

namespace = {"FRAME_SCALE": 1}
exec(compile(ast.Module(body=nodes, type_ignores=[]), "main_see.py", "exec"), namespace)
namespace["blob_center_is_player"] = lambda blob, image, predicate: True
namespace["is_player_green_half"] = lambda *args: True
namespace["is_player_cyan_half"] = lambda *args: True


class Blob:
    def __init__(self, x, y, pixel_count):
        self.x = x
        self.y = y
        self.pixel_count = pixel_count

    def cx(self):
        return self.x

    def cy(self):
        return self.y

    def pixels(self):
        return self.pixel_count


class Image:
    def __init__(self):
        self.rois = []

    def width(self):
        return 320

    def height(self):
        return 240

    def find_blobs(self, thresholds, roi, **kwargs):
        self.rois.append(roi)
        if thresholds[0] == namespace["PLAYER_GREEN_BLOB_THRESHOLD"]:
            return [Blob(108, 100, 20)]
        return [Blob(112, 100, 18)]


empty_map = [["space" for _ in range(16)] for _ in range(12)]
points = [(col * 20 + 10, row * 20 + 10) for row in range(12) for col in range(16)]
image = Image()

center = namespace["detect_player_center"](
    image, points, empty_map, empty_map, (100, 100)
)
assert center == (110, 100)
assert len(image.rois) == 2

image = Image()
center = namespace["detect_player_center"](
    image, points, empty_map, empty_map, None
)
assert center is None
assert image.rois == []

assert namespace["select_recent_player_anchor"](
    None, (100, 100), 2
) is None
assert namespace["select_recent_player_anchor"](
    None, (100, 100), 3
) == (100, 100)
assert namespace["select_recent_player_anchor"](
    (120, 100), (100, 100), 3
) == (120, 100)

anchors = []
namespace["detect_player_center"] = (
    lambda img, points, raw, stable, anchor:
    anchors.append(anchor) or anchor
)
namespace["detect_player_center_precise"] = lambda img, anchor: anchor
namespace["image_center_to_grid_q"] = (
    lambda center, rectified, transform: (550, 550)
    if center is not None else None
)

result = namespace["resolve_player_center"](
    image, points, empty_map, empty_map,
    (101, 102), (5, 6), True, None)
assert anchors[-1] == (101, 102)
assert result == ((101, 102), (550, 550), (5, 5))

result = namespace["resolve_player_center"](
    image, points, empty_map, empty_map,
    None, (5, 6), True, None)
assert anchors[-1] == points[5 * 16 + 6]
assert result[2] == (5, 5)

coarse_map = [["space" for _ in range(16)] for _ in range(12)]
coarse_map[4][7] = "player"
result = namespace["resolve_player_center"](
    image, points, coarse_map, empty_map,
    None, None, True, None)
assert anchors[-1] == points[4 * 16 + 7]
assert result[2] == (5, 5)

namespace["detect_player_center"] = lambda *args: None
namespace["detect_player_center_precise"] = lambda *args: None
assert namespace["resolve_player_center"](
    image, points, empty_map, empty_map,
    None, None, True, None) is None

print("openart-player-anchor PASS")
