from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "project" / "user" / "inc" / "drive_config.h").read_text(
    encoding="utf-8"
)
MENU = (ROOT / "project" / "user" / "src" / "menu.c").read_text(
    encoding="utf-8"
)


assert "#define SCREEN_RENDER_AFTER_K3_ENABLE (0)" in CONFIG
assert "static uint8 screen_render_enabled" in MENU
assert "menu_disable_screen_render_after_k3();" in MENU
assert "competition_flow_start(COMPETITION_MODE);" in MENU
assert MENU.index("competition_flow_start(COMPETITION_MODE);") < MENU.index(
    "menu_disable_screen_render_after_k3();"
)
assert "if(0u == menu_screen_render_allowed())" in MENU
assert "screen_draw_nav_cursor(previous_cursor_row, cursor_row);" in MENU
assert MENU.index("menu_screen_render_allowed()") < MENU.index(
    "screen_draw_nav_cursor(previous_cursor_row, cursor_row);"
)
assert "menu_enable_screen_render();" in MENU
print("menu-screen-render PASS")
