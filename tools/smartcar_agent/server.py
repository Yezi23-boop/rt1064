import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
from urllib.parse import urlparse

from observer import build_snapshot, capture_smartcar_window, write_snapshot


DEFAULT_SMARTCAR_DIR = Path(r"E:\上位机\调试版本")


class ObserverRuntime:
    def __init__(self, smartcar_dir: Path, runtime_dir: Path, capture_window=None):
        self.smartcar_dir = Path(smartcar_dir).resolve()
        self.runtime_dir = Path(runtime_dir).resolve()
        self.state_path = self.runtime_dir / "latest_state.json"
        self.window_path = self.runtime_dir / "latest_window.png"
        self.capture_window = capture_window or capture_smartcar_window

    def refresh_once(self) -> dict:
        capture_warning = None
        try:
            window = self.capture_window(self.window_path)
        except Exception as error:
            capture_warning = f"窗口截图失败: {error}"
            window = {
                "window_found": False,
                "window_title": None,
                "window_rect": None,
                "capture_path": str(self.window_path),
                "capture_updated": False,
                "capture_error": str(error),
            }
        snapshot = build_snapshot(self.smartcar_dir, self.runtime_dir, window=window)
        if capture_warning:
            snapshot["warnings"].append(capture_warning)
        write_snapshot(snapshot, self.state_path)
        return snapshot


def _handler_class(runtime: ObserverRuntime, web_dir: Path):
    class ObserverRequestHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(web_dir), **kwargs)

        def do_GET(self):
            route = urlparse(self.path).path
            if route == "/api/state":
                self._send_file(runtime.state_path, "application/json; charset=utf-8")
                return
            if route == "/api/window":
                self._send_file(runtime.window_path, "image/png")
                return
            if route == "/api/debug":
                self._send_file(runtime.smartcar_dir / "debug_save.jpg", "image/jpeg")
                return
            super().do_GET()

        def _send_file(self, path: Path, content_type: str):
            try:
                content = path.read_bytes()
            except OSError:
                self.send_error(404, "artifact not available")
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

    return ObserverRequestHandler


def create_http_server(
    bind: str,
    port: int,
    runtime: ObserverRuntime,
    web_dir: Path,
) -> ThreadingHTTPServer:
    return ThreadingHTTPServer((bind, port), _handler_class(runtime, Path(web_dir).resolve()))


def _refresh_safely(runtime: ObserverRuntime) -> bool:
    try:
        runtime.refresh_once()
    except Exception:
        return False
    return True


def _poll(runtime: ObserverRuntime, interval: float, stop_event: threading.Event):
    while not stop_event.wait(interval):
        _refresh_safely(runtime)


def main() -> None:
    tool_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="SmartCar VR Agent observer")
    parser.add_argument("--smartcar-dir", type=Path, default=DEFAULT_SMARTCAR_DIR)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args()

    runtime = ObserverRuntime(args.smartcar_dir, tool_dir / "runtime")
    _refresh_safely(runtime)
    stop_event = threading.Event()
    poller = threading.Thread(
        target=_poll,
        args=(runtime, max(args.interval, 0.1), stop_event),
        daemon=True,
    )
    poller.start()
    httpd = create_http_server("127.0.0.1", args.port, runtime, tool_dir / "web")
    print(f"SmartCar Agent Observer: http://127.0.0.1:{httpd.server_port}")
    print(f"Agent state: {runtime.state_path}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        httpd.server_close()
        poller.join(timeout=2)


if __name__ == "__main__":
    main()
