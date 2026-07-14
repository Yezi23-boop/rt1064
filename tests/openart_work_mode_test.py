import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "openmv" / "main_see.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)


def assigned_literal(statements, name):
    for statement in statements:
        if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
            continue
        target = statement.targets[0]
        if isinstance(target, ast.Name) and target.id == name:
            return ast.literal_eval(statement.value)
    raise AssertionError("missing assignment: %s" % name)


mode_if = None
for node in TREE.body:
    if not isinstance(node, ast.If):
        continue
    test = node.test
    if (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name) and
            test.left.id == "WORK_MODE"):
        mode_if = node
        break

assert mode_if is not None, "WORK_MODE configuration branch is missing"

run_expected = {
    "DEBUG_ENABLE": False,
    "DEBUG_DRAW_ROI": False,
    "DEBUG_DRAW_GRID_LINES": False,
    "DEBUG_DRAW_POINTS": False,
    "DEBUG_PLAYER_CENTER_ENABLE": False,
    "DEBUG_OBSERVATION_ENABLE": False,
    "SHOW_RECTIFIED_VIEW": True,
    "USE_RECTIFIED_RECOGNITION": True,
}
debug_expected = {
    "DEBUG_ENABLE": True,
    "DEBUG_DRAW_ROI": True,
    "DEBUG_DRAW_GRID_LINES": True,
    "DEBUG_DRAW_POINTS": True,
    "DEBUG_PLAYER_CENTER_ENABLE": False,
    "DEBUG_OBSERVATION_ENABLE": True,
    "SHOW_RECTIFIED_VIEW": True,
    "USE_RECTIFIED_RECOGNITION": True,
}

for name, expected in run_expected.items():
    assert assigned_literal(mode_if.body, name) is expected, name

for name, expected in debug_expected.items():
    assert assigned_literal(mode_if.orelse, name) is expected, name

print("openart work mode tests passed")
