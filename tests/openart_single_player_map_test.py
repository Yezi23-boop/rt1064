import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "openmv" / "main_see.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

CONSTANTS = {"GRID_COLS", "GRID_ROWS", "ELEMENT_CHAR"}
FUNCTIONS = {
    "count_player_cells",
    "update_non_player_background",
    "build_canonical_player_map",
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

assert found_functions == FUNCTIONS, "canonical player map helpers are missing"
canonical_function = next(
    node for node in TREE.body
    if isinstance(node, ast.FunctionDef) and
    node.name == "build_canonical_player_map")
assert not any(isinstance(node, ast.ListComp)
               for node in ast.walk(canonical_function)), \
    "OpenMV v1.18 cannot execute this nested list comprehension"

namespace = {}
exec(compile(ast.Module(body=nodes, type_ignores=[]),
             "main_see.py", "exec"), namespace)

GRID_ROWS = namespace["GRID_ROWS"]
GRID_COLS = namespace["GRID_COLS"]


def matrix(fill):
    return [[fill for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]


def car_count(char_matrix):
    return sum(1 for row in char_matrix for value in row
               if value in ("C", "+"))


count_player_cells = namespace["count_player_cells"]
update_background = namespace["update_non_player_background"]
build_canonical = namespace["build_canonical_player_map"]

stable = matrix("space")
stable[5][5] = "player"
background = matrix("space")
assert count_player_cells(stable) == 1
canonical = build_canonical(stable, background, None)
assert canonical is not None
assert car_count(canonical) == 1
assert canonical[5][5] == "C"

stable = matrix("space")
stable[5][5] = "player"
stable[5][6] = "player"
background = matrix("space")
canonical = build_canonical(stable, background, (568, 542))
assert canonical is not None
assert car_count(canonical) == 1
assert canonical[5][5] == "C"
assert canonical[5][6] == "."

stable = matrix("space")
stable[5][5] = "player"
background = matrix("space")
background[5][5] = "goal"
canonical = build_canonical(stable, background, (550, 550))
assert canonical[5][5] == "+"

stable = matrix("space")
stable[5][5] = "player"
stable[5][6] = "player"
background = matrix("space")
background[5][5] = "goal"
background[5][6] = "wall"
canonical = build_canonical(stable, background, (550, 550))
assert canonical[5][5] == "+"
assert canonical[5][6] == "#"

stable = matrix("space")
stable[5][5] = "space"
background = matrix("space")
background[5][5] = "goal"
update_background(stable, background)
assert background[5][5] == "space"

stable[5][5] = "wall"
background[5][5] = "space"
update_background(stable, background, (5, 5))
assert background[5][5] == "space"

stable = matrix("space")
stable[5][5] = "player"
stable[5][6] = "player"
background = matrix("space")
background[5][6] = ""
assert build_canonical(stable, background, (550, 550)) is None
assert build_canonical(stable, background, (-1, 550)) is None
assert build_canonical(stable, background, (GRID_COLS * 100, 550)) is None

stable = matrix("space")
background = matrix("space")
assert build_canonical(stable, background, None) is None

print("openart-single-player-map PASS")
