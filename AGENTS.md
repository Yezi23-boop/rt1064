# 仓库协作规则

## 全局环境说明

- 在这个 Win11 环境里，`rg` / `ripgrep` 可能不可用，或者会因为权限问题失败，例如报错 `Access is denied`。
- 即使 `rg` 平时是首选搜索工具，也不要默认它在这里可用。
- 在这个环境中，优先使用 PowerShell 原生命令作为替代方案：
  - 用 `Get-ChildItem -Recurse -File` 查找文件。
  - 用 `Select-String` 搜索文件内容。
- 如果后续在某个具体会话中已经确认 `rg` 可以正常运行，再使用它；在确认之前不要假设它可用。
- 使用 `uv` 管理 Python 相关工具和验证命令。
- 默认编码是 UTF-8。

## 文字说明

- 尽量使用中文回答和编程说明。
- 默认使用 ffmpeg 处理音频。
- 发本地图片时，请用 Markdown 图片语法，并把 Windows 路径改成正斜杠。

## 项目文档优先

- 遇到 OpenART Plus、RT1064、UART、SD 卡 `cmm_cfg/cmm_load`、REPL、串口终端、Keil 下载运行状态这类类似问题时，先查项目文档，再继续排查或改代码。
- 当前 OpenART Plus 到 RT1064 UART 最小通信经验文档：
  - `docs/competition/openart_plus_rt1064_uart_experience.md`
- 如果问题涉及第 21 届智能视觉组整体规则、OpenART/RT1064 分工、16x12 地图识别、推箱子 BFS 或双 OpenART Plus 方案，优先结合逐飞公众号原文和本仓库 `docs/competition` 下的整理文档判断，不要只凭聊天记忆下结论。
