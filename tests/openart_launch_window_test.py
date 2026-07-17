import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "openmv" / "main_see.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

CONSTANTS = {
    "DARK_PIXEL_THRESHOLD",
    "GRID_ROWS",
    "GRID_COLS",
    "LAUNCH_PLAYER_WINDOW_ENABLE",
    "LAUNCH_PLAYER_ROW_MIN",
    "LAUNCH_PLAYER_ROW_MAX",
    "LAUNCH_PLAYER_COL_MIN",
    "LAUNCH_PLAYER_COL_MAX",
}

nodes = []
for node in TREE.body:
    if isinstance(node, ast.Assign):
        if any(isinstance(target, ast.Name) and target.id in CONSTANTS
               for target in node.targets):
            nodes.append(node)
    elif isinstance(node, ast.FunctionDef) and node.name == "classify_element":
        nodes.append(node)

namespace = {
    "get_average_pixel": lambda img, x, y: (0, 0, 255),
    "sample_player_color": lambda img, x, y, r, g, b: False,
    "normalize_color": lambda r, g, b: (r, g, b, r + g + b),
    "is_box_color": lambda rn, gn, bn, color_sum: False,
    "is_box_candidate": lambda rn, gn, bn, color_sum: False,
    "sample_special_color": lambda *args: False,
    "box_cell_has_coverage": lambda img, x, y: False,
    "is_goal_color": lambda rn, gn, bn, color_sum: False,
    "is_goal_candidate": lambda rn, gn, bn, color_sum: False,
    "is_bomb_color": lambda rn, gn, bn, color_sum: False,
    "is_bomb_candidate": lambda rn, gn, bn, color_sum: False,
    "is_space_color": lambda rn, gn, bn, color_sum: True,
    "confirm_space_color": lambda img, x, y, r, g, b: True,
}
exec(compile(ast.Module(body=nodes, type_ignores=[]),
             "main_see.py", "exec"), namespace)

classify = namespace["classify_element"]

for row in (5, 6):
    for col in (0, 1):
        assert classify(None, row, col, 0, 0) == "space"

namespace["is_box_color"] = lambda rn, gn, bn, color_sum: True
assert classify(None, 6, 1, 0, 0) == "box"

namespace["is_box_color"] = lambda rn, gn, bn, color_sum: False
namespace["sample_player_color"] = lambda img, x, y, r, g, b: True
assert classify(None, 5, 0, 0, 0) == "player"

namespace["sample_player_color"] = lambda img, x, y, r, g, b: False
assert classify(None, 4, 0, 0, 0) == "wall"

print("openart-launch-window PASS")
