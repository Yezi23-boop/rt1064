$ErrorActionPreference = "Stop"

$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$outputs = @(
    (Join-Path $env:TEMP "rt1064_art_center_fallback_test.exe"),
    (Join-Path $env:TEMP "rt1064_art_center_strict_test.exe")
)

try
{
    for($fallback = 1; $fallback -ge 0; $fallback--)
    {
        $output = $outputs[1 - $fallback]
        & gcc `
            -std=c99 `
            -Wall `
            -Wextra `
            -Werror `
            "-DART_CENTER_TIMEOUT_FALLBACK_ENABLE=$fallback" `
            -I (Join-Path $root "tests\host") `
            -I (Join-Path $root "project\user\inc") `
            (Join-Path $root "project\user\src\art_replan.c") `
            (Join-Path $root "project\user\src\map_utils.c") `
            (Join-Path $root "tests\art_replan_requested_center_test.c") `
            -lm `
            -o $output

        if(0 -ne $LASTEXITCODE)
        {
            exit $LASTEXITCODE
        }

        Write-Output "ART_CENTER_TIMEOUT_FALLBACK_ENABLE=$fallback"
        & $output
        if(0 -ne $LASTEXITCODE)
        {
            exit $LASTEXITCODE
        }
    }
}
finally
{
    $outputs | ForEach-Object {
        Remove-Item -LiteralPath $_ -Force -ErrorAction SilentlyContinue
    }
}
