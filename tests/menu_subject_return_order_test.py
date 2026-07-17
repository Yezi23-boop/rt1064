from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "project" / "user" / "src" / "menu.c").read_text(
    encoding="utf-8"
)


def function_body(name):
    start = SOURCE.index("static void " + name)
    brace = SOURCE.index("{", start)
    depth = 0
    for index in range(brace, len(SOURCE)):
        if SOURCE[index] == "{":
            depth += 1
        elif SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[brace + 1 : index]
    raise AssertionError("unterminated function: " + name)


body = function_body("begin_classification_return")
start_return = body.index("art_replan_begin_return_home")
assert body.index("subject3_cancel") < start_return
assert body.index("subject2_cancel") < start_return
assert "subject3_cancel" not in body[start_return:]
assert "subject2_cancel" not in body[start_return:]
print("menu-subject-return-order PASS")
