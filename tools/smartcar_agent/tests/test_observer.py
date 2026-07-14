import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

from PIL import Image


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from observer import (
    build_snapshot,
    capture_smartcar_window,
    parse_camera_ini,
    parse_map_text,
    select_smartcar_window,
    write_snapshot,
)


VALID_MAP = """################
#-$.*----------#
#--------------#
#--------------#
#--------------#
#--------------#
#--------------#
#--------------#
#--------------#
#--------------#
#--------------#
################"""


class MapParserTests(unittest.TestCase):
    def test_parses_grid_and_object_counts(self):
        result = parse_map_text(VALID_MAP)

        self.assertEqual(12, result["rows"])
        self.assertEqual(16, result["cols"])
        self.assertEqual(
            {"walls": 52, "boxes": 1, "goals": 1, "completed": 1},
            result["counts"],
        )
        self.assertEqual("-$.*", result["grid"][1][1:5])

    def test_rejects_wrong_dimensions(self):
        with self.assertRaisesRegex(ValueError, "12 rows"):
            parse_map_text("################\n################")


class CameraIniParserTests(unittest.TestCase):
    def test_parses_integer_camera_settings(self):
        result = parse_camera_ini(
            "; camera.ini\n"
            "camera_index=2\n"
            "compress_quality=40\n"
            "h_range=20\n"
            "s_range=80\n"
            "v_range=80\n"
        )

        self.assertEqual(
            {
                "camera_index": 2,
                "compress_quality": 40,
                "h_range": 20,
                "s_range": 80,
                "v_range": 80,
            },
            result,
        )


class SnapshotTests(unittest.TestCase):
    def test_builds_agent_snapshot_from_vendor_directory(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            runtime_dir = root / "runtime"
            map_dir = root / "map_file"
            map_dir.mkdir()
            (root / "SmartCar_VR_V1.7.exe").write_bytes(b"MZ")
            (root / "camera_opencv.exe").write_bytes(b"MZ")
            (root / "camera.ini").write_text(
                "camera_index=2\ncompress_quality=40\nh_range=20\n"
                "s_range=80\nv_range=80\n",
                encoding="utf-8",
            )
            debug_image = root / "debug_save.jpg"
            debug_image.write_bytes(b"jpg")
            older_map = map_dir / "map1.txt"
            latest_map = map_dir / "-map2.txt"
            older_map.write_text(VALID_MAP, encoding="utf-8")
            latest_map.write_text(VALID_MAP, encoding="utf-8")
            os.utime(older_map, (100, 100))
            os.utime(latest_map, (200, 200))

            result = build_snapshot(root, runtime_dir)

            self.assertEqual(1, result["schema_version"])
            self.assertEqual(str(root.resolve()), result["smartcar_dir"])
            self.assertTrue(result["app"]["exe_exists"])
            self.assertFalse(result["app"]["window_found"])
            self.assertEqual(2, result["camera"]["settings"]["camera_index"])
            self.assertTrue(result["camera"]["debug_image"]["exists"])
            self.assertEqual("-map2.txt", result["selected_map_candidate"])
            self.assertEqual("pure_sokoban", result["maps"][0]["mode"])
            self.assertEqual(2, len(result["maps"]))
            self.assertEqual([], result["warnings"])

    def test_writes_snapshot_as_json(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "runtime" / "latest_state.json"
            snapshot = {"schema_version": 1, "value": "ready"}

            write_snapshot(snapshot, output)

            self.assertEqual(snapshot, json.loads(output.read_text(encoding="utf-8")))
            self.assertFalse(output.with_suffix(".json.tmp").exists())


class WindowSelectionTests(unittest.TestCase):
    def test_prefers_visible_smartcar_window(self):
        candidates = [
            {"handle": 1, "title": "摄像头属性", "visible": True, "rect": [0, 0, 400, 300]},
            {"handle": 2, "title": "SmartCar_VR_1.7", "visible": False, "rect": [0, 0, 800, 600]},
            {"handle": 3, "title": "SmartCar_VR_1.7 - Game Window", "visible": True, "rect": [10, 20, 1010, 720]},
        ]

        result = select_smartcar_window(candidates)

        self.assertEqual(3, result["handle"])
        self.assertEqual([10, 20, 1010, 720], result["rect"])

    def test_returns_none_when_product_window_is_absent(self):
        candidates = [
            {"handle": 1, "title": "OpenMV IDE", "visible": True, "rect": [0, 0, 800, 600]},
        ]

        self.assertIsNone(select_smartcar_window(candidates))

    def test_prefers_onscreen_game_window_over_minimized_window(self):
        candidates = [
            {
                "handle": 2,
                "title": "SmartCar_VR_1.7",
                "visible": True,
                "rect": [-32000, -32000, -31840, -31972],
            },
            {
                "handle": 3,
                "title": "Game Window",
                "visible": True,
                "rect": [100, 80, 1380, 800],
            },
        ]

        result = select_smartcar_window(candidates)

        self.assertEqual(3, result["handle"])
        self.assertEqual("Game Window", result["title"])

    def test_captures_selected_window_to_png(self):
        candidates = [
            {"handle": 3, "title": "SmartCar_VR_1.7", "visible": True, "rect": [10, 20, 330, 220]},
        ]
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "latest_window.png"

            result = capture_smartcar_window(
                output,
                candidates=candidates,
                grabber=lambda **_kwargs: Image.new("RGB", (320, 200), "black"),
            )

            self.assertTrue(result["window_found"])
            self.assertEqual("SmartCar_VR_1.7", result["window_title"])
            self.assertEqual([10, 20, 330, 220], result["window_rect"])
            with Image.open(output) as image:
                self.assertEqual((320, 200), image.size)

    def test_reports_missing_window_without_capture(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "latest_window.png"

            result = capture_smartcar_window(output, candidates=[])

            self.assertFalse(result["window_found"])
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
