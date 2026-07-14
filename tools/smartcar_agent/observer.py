from configparser import ConfigParser
import ctypes
from ctypes import wintypes
from datetime import datetime, timezone
import json
import os
from pathlib import Path

from PIL import ImageGrab


MAP_ROWS = 12
MAP_COLS = 16
WINDOW_TITLE_MARKERS = ("smartcar_vr", "智能视觉组虚拟游戏系统", "game window")


def parse_map_text(text: str) -> dict:
    lines = text.splitlines()
    if len(lines) != MAP_ROWS:
        raise ValueError(f"map must contain {MAP_ROWS} rows")
    if any(len(line) != MAP_COLS for line in lines):
        raise ValueError(f"each map row must contain {MAP_COLS} columns")

    grid = "".join(lines)
    return {
        "rows": MAP_ROWS,
        "cols": MAP_COLS,
        "grid": lines,
        "counts": {
            "walls": grid.count("#"),
            "boxes": grid.count("$"),
            "goals": grid.count("."),
            "completed": grid.count("*"),
        },
    }


def parse_camera_ini(text: str) -> dict:
    parser = ConfigParser()
    parser.read_string("[camera]\n" + text)
    return {key: int(value) for key, value in parser["camera"].items()}


def select_smartcar_window(candidates: list[dict]) -> dict | None:
    matches = []
    for candidate in candidates:
        title = candidate.get("title", "")
        rect = candidate.get("rect", [0, 0, 0, 0])
        if not candidate.get("visible") or len(rect) != 4:
            continue
        if rect[2] <= rect[0] or rect[3] <= rect[1]:
            continue
        if any(marker in title.lower() for marker in WINDOW_TITLE_MARKERS):
            matches.append(candidate)
    if not matches:
        return None

    def window_score(candidate):
        left, top, right, bottom = candidate["rect"]
        not_minimized = left > -10000 and top > -10000
        area = (right - left) * (bottom - top)
        return not_minimized, area

    return max(matches, key=window_score)


def enumerate_top_level_windows() -> list[dict]:
    if os.name != "nt":
        return []

    user32 = ctypes.windll.user32
    candidates = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def callback(handle, _lparam):
        title_length = user32.GetWindowTextLengthW(handle)
        if title_length <= 0:
            return True
        title_buffer = ctypes.create_unicode_buffer(title_length + 1)
        user32.GetWindowTextW(handle, title_buffer, len(title_buffer))
        rect = wintypes.RECT()
        user32.GetWindowRect(handle, ctypes.byref(rect))
        candidates.append(
            {
                "handle": int(handle),
                "title": title_buffer.value,
                "visible": bool(user32.IsWindowVisible(handle)),
                "rect": [rect.left, rect.top, rect.right, rect.bottom],
            }
        )
        return True

    user32.EnumWindows(callback, 0)
    return candidates


def capture_smartcar_window(
    output_path: Path,
    candidates: list[dict] | None = None,
    grabber=None,
) -> dict:
    output_path = Path(output_path).resolve()
    selected = select_smartcar_window(
        enumerate_top_level_windows() if candidates is None else candidates
    )
    state = {
        "window_found": selected is not None,
        "window_title": selected["title"] if selected else None,
        "window_rect": selected["rect"] if selected else None,
        "capture_path": str(output_path),
        "capture_updated": False,
        "capture_error": None,
    }
    if selected is None:
        output_path.unlink(missing_ok=True)
        return state

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    try:
        image = (grabber or ImageGrab.grab)(
            bbox=tuple(selected["rect"]),
            all_screens=True,
        )
        image.save(temporary_path, format="PNG")
        os.replace(temporary_path, output_path)
        state["capture_updated"] = True
    except (OSError, ValueError) as error:
        temporary_path.unlink(missing_ok=True)
        state["capture_error"] = str(error)
    return state


def _file_metadata(path: Path) -> dict:
    exists = path.is_file()
    metadata = {"path": str(path.resolve()), "exists": exists}
    if exists:
        stat = path.stat()
        metadata.update({"size": stat.st_size, "modified_at": stat.st_mtime})
    return metadata


def build_snapshot(smartcar_dir: Path, runtime_dir: Path, window: dict | None = None) -> dict:
    smartcar_dir = Path(smartcar_dir).resolve()
    runtime_dir = Path(runtime_dir).resolve()
    warnings = []

    exe_path = smartcar_dir / "SmartCar_VR_V1.7.exe"
    helper_path = smartcar_dir / "camera_opencv.exe"

    camera_ini = smartcar_dir / "camera.ini"
    camera_settings = {}
    if camera_ini.is_file():
        try:
            camera_settings = parse_camera_ini(camera_ini.read_text(encoding="utf-8-sig"))
        except (OSError, ValueError) as error:
            warnings.append(f"camera.ini 解析失败: {error}")
    else:
        warnings.append("未找到 camera.ini")

    maps = []
    map_dir = smartcar_dir / "map_file"
    if map_dir.is_dir():
        for map_path in map_dir.glob("*.txt"):
            try:
                map_data = parse_map_text(map_path.read_text(encoding="utf-8-sig"))
            except (OSError, UnicodeError, ValueError) as error:
                warnings.append(f"地图 {map_path.name} 解析失败: {error}")
                continue
            stat = map_path.stat()
            maps.append(
                {
                    "name": map_path.name,
                    "path": str(map_path.resolve()),
                    "mode": "pure_sokoban" if map_path.name.startswith("-") else "classified",
                    "modified_at": stat.st_mtime,
                    **map_data,
                }
            )
        maps.sort(key=lambda item: (-item["modified_at"], item["name"]))
    else:
        warnings.append("未找到 map_file 目录")

    debug_image = _file_metadata(smartcar_dir / "debug_save.jpg")
    if not debug_image["exists"]:
        warnings.append("尚未生成 debug_save.jpg")

    window_state = {
        "window_found": False,
        "window_title": None,
        "window_rect": None,
        "capture_path": str((runtime_dir / "latest_window.png").resolve()),
    }
    if window:
        window_state.update(window)

    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "smartcar_dir": str(smartcar_dir),
        "app": {
            "exe_path": str(exe_path.resolve()),
            "exe_exists": exe_path.is_file(),
            "camera_helper_path": str(helper_path.resolve()),
            "camera_helper_exists": helper_path.is_file(),
            **window_state,
        },
        "camera": {
            "config_path": str(camera_ini.resolve()),
            "settings": camera_settings,
            "debug_image": debug_image,
        },
        "maps": maps,
        "selected_map_candidate": maps[0]["name"] if maps else None,
        "artifacts": {
            "state_json": str((runtime_dir / "latest_state.json").resolve()),
            "window_image": str((runtime_dir / "latest_window.png").resolve()),
            "debug_image": debug_image["path"],
        },
        "warnings": warnings,
    }


def write_snapshot(snapshot: dict, output_path: Path) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary_path.write_text(
        json.dumps(snapshot, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    os.replace(temporary_path, output_path)
