param(
    [switch]$Headless,
    [int]$Cycles = 2,
    [switch]$AutoClose
)

$ErrorActionPreference = 'Stop'
$EnvPython = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $EnvPython)) {
    throw 'Run setup.ps1 in this folder first to create the demo environment.'
}
$DemoArgs = @('--cycles', "$Cycles")
if ($Headless) { $DemoArgs += '--headless' }
elseif ($AutoClose) { $DemoArgs += '--auto-close' }
& $EnvPython (Join-Path $PSScriptRoot 'demo.py') @DemoArgs
exit $LASTEXITCODE
