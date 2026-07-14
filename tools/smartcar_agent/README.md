# SmartCar Agent 观察器

这是一个只读的本机辅助程序，把 SmartCar VR 上位机旁边已有的数据整理成 Agent 和浏览器都能稳定读取的状态。

## 启动

默认上位机目录为 `E:\上位机\调试版本`：

```powershell
powershell -ExecutionPolicy Bypass -File tools/smartcar_agent/run.ps1
```

指定其他目录或端口：

```powershell
powershell -ExecutionPolicy Bypass -File tools/smartcar_agent/run.ps1 `
    -SmartCarDir "E:\上位机\调试版本" `
    -Port 8765
```

浏览器面板：<http://127.0.0.1:8765>

运行环境需要 Python 3 和 Pillow：

```powershell
python -m pip install Pillow
```

## Agent 读取入口

- `tools/smartcar_agent/runtime/latest_state.json`：统一状态，包含摄像头参数、地图、告警和窗口状态。
- `tools/smartcar_agent/runtime/latest_window.png`：SmartCar VR 窗口截图；找不到窗口时文件会被删除，避免读取旧画面。
- `http://127.0.0.1:8765/api/state`：与 JSON 文件相同的实时状态。
- `http://127.0.0.1:8765/api/window`：最新窗口截图。
- `http://127.0.0.1:8765/api/debug`：官方 `debug_save.jpg`。

Agent 调参时应先读取 `latest_state.json`，再按其中 `artifacts` 的绝对路径查看窗口图和定位调试图。`selected_map_candidate` 只是按修改时间选出的候选地图，不代表已读取到 Godot 内部当前选项。

## 当前边界

- 不修改 `SmartCar_VR_V1.7.exe`、`camera_opencv.exe`、`camera.ini` 或地图文件。
- 不连接 `127.0.0.1:8888/8889`，避免抢占官方程序的单客户端图像和定位连接。
- 不解密 Godot 的 `GDSCe` 脚本，不读取游戏进程内存。
- 窗口截图使用可见窗口区域；窗口最小化、移出屏幕或被遮挡时可能无法得到有效画面，状态中的 `capture_error` 会记录原因。
- 第一版不占用无线串口。需要关联 RT1064 的 VOFA 曲线时，可后续增加从现有日志文件导入，避免与调试串口工具争用 COM 口。

## 测试

```powershell
python -m unittest discover -s tools/smartcar_agent/tests -v
```
