import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from urllib.request import urlopen


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from server import ObserverRuntime, _refresh_safely, create_http_server


VALID_MAP = "\n".join(
    ["################"]
    + ["#--------------#"] * 10
    + ["################"]
)


class ObserverRuntimeTests(unittest.TestCase):
    def test_refresh_publishes_agent_state(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            runtime_dir = root / "runtime"
            (root / "map_file").mkdir()
            (root / "map_file" / "map1.txt").write_text(VALID_MAP, encoding="utf-8")
            (root / "camera.ini").write_text("camera_index=2\n", encoding="utf-8")
            (root / "debug_save.jpg").write_bytes(b"jpeg")
            (root / "SmartCar_VR_V1.7.exe").write_bytes(b"MZ")
            (root / "camera_opencv.exe").write_bytes(b"MZ")
            capture_calls = []

            def capture(output_path):
                capture_calls.append(output_path)
                return {
                    "window_found": False,
                    "window_title": None,
                    "window_rect": None,
                    "capture_path": str(output_path),
                    "capture_updated": False,
                    "capture_error": None,
                }

            runtime = ObserverRuntime(root, runtime_dir, capture_window=capture)
            snapshot = runtime.refresh_once()

            self.assertEqual([runtime_dir / "latest_window.png"], capture_calls)
            self.assertEqual(2, snapshot["camera"]["settings"]["camera_index"])
            published = json.loads(runtime.state_path.read_text(encoding="utf-8"))
            self.assertEqual(snapshot, published)

    def test_capture_failure_is_published_without_stopping_refresh(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            runtime_dir = root / "runtime"
            (root / "map_file").mkdir()
            (root / "map_file" / "map1.txt").write_text(VALID_MAP, encoding="utf-8")
            (root / "camera.ini").write_text("camera_index=2\n", encoding="utf-8")
            (root / "debug_save.jpg").write_bytes(b"jpeg")

            def failing_capture(_output_path):
                raise RuntimeError("temporary capture failure")

            runtime = ObserverRuntime(root, runtime_dir, capture_window=failing_capture)
            snapshot = runtime.refresh_once()

            self.assertFalse(snapshot["app"]["window_found"])
            self.assertEqual("temporary capture failure", snapshot["app"]["capture_error"])
            self.assertIn("窗口截图失败", snapshot["warnings"][-1])
            self.assertTrue(runtime.state_path.is_file())

    def test_background_refresh_recovers_after_transient_io_failure(self):
        class FlakyRuntime:
            def __init__(self):
                self.calls = 0

            def refresh_once(self):
                self.calls += 1
                if self.calls == 1:
                    raise PermissionError("state file is temporarily locked")
                return {"schema_version": 1}

        runtime = FlakyRuntime()

        self.assertFalse(_refresh_safely(runtime))
        self.assertTrue(_refresh_safely(runtime))
        self.assertEqual(2, runtime.calls)


class HttpServerTests(unittest.TestCase):
    def test_serves_state_debug_image_and_dashboard(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            runtime_dir = root / "runtime"
            web_dir = root / "web"
            web_dir.mkdir()
            (web_dir / "index.html").write_text("observer dashboard", encoding="utf-8")
            (root / "map_file").mkdir()
            (root / "map_file" / "map1.txt").write_text(VALID_MAP, encoding="utf-8")
            (root / "camera.ini").write_text("camera_index=2\n", encoding="utf-8")
            (root / "debug_save.jpg").write_bytes(b"jpeg-data")
            runtime = ObserverRuntime(
                root,
                runtime_dir,
                capture_window=lambda output: {
                    "window_found": False,
                    "capture_path": str(output),
                },
            )
            runtime.refresh_once()
            httpd = create_http_server("127.0.0.1", 0, runtime, web_dir)
            thread = threading.Thread(target=httpd.serve_forever, daemon=True)
            thread.start()
            base_url = f"http://127.0.0.1:{httpd.server_port}"
            try:
                with urlopen(base_url + "/api/state") as response:
                    state = json.load(response)
                    self.assertEqual(1, state["schema_version"])
                    self.assertEqual("no-store", response.headers["Cache-Control"])
                with urlopen(base_url + "/api/debug") as response:
                    self.assertEqual(b"jpeg-data", response.read())
                    self.assertEqual("image/jpeg", response.headers.get_content_type())
                with urlopen(base_url + "/") as response:
                    self.assertIn(b"observer dashboard", response.read())
            finally:
                httpd.shutdown()
                httpd.server_close()
                thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
