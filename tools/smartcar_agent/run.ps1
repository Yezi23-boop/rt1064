param(
    [string]$SmartCarDir = "E:\上位机\调试版本",
    [int]$Port = 8765
)

$ErrorActionPreference = "Stop"
python "$PSScriptRoot\server.py" --smartcar-dir $SmartCarDir --port $Port
