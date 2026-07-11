$ErrorActionPreference = "Stop"

$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$output = Join-Path $env:TEMP "rt1064_subject2_logic_test.exe"

try
{
    & gcc `
        -std=c99 `
        -Wall `
        -Wextra `
        -Werror `
        -I (Join-Path $root "tests\host") `
        -I (Join-Path $root "project\user\inc") `
        (Join-Path $root "project\user\src\solver.c") `
        (Join-Path $root "project\user\src\map_utils.c") `
        (Join-Path $root "project\user\src\subject2_logic.c") `
        (Join-Path $root "tests\subject2_logic_test.c") `
        -o $output

    if(0 -ne $LASTEXITCODE)
    {
        exit $LASTEXITCODE
    }

    & $output
    exit $LASTEXITCODE
}
finally
{
    Remove-Item -LiteralPath $output -Force -ErrorAction SilentlyContinue
}
