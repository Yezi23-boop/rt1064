$ErrorActionPreference = "Stop"

$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$output = Join-Path $env:TEMP "rt1064_drive_control_yaw_limit_test.exe"

try
{
    & gcc `
        -std=c99 `
        -Wall `
        -Wextra `
        -Werror `
        -I (Join-Path $root "tests\host") `
        -I (Join-Path $root "project\user\inc") `
        (Join-Path $root "tests\drive_control_yaw_limit_test.c") `
        -lm `
        -o $output

    if(0 -ne $LASTEXITCODE) { exit $LASTEXITCODE }
    & $output
    exit $LASTEXITCODE
}
finally
{
    Remove-Item -LiteralPath $output -Force -ErrorAction SilentlyContinue
}
