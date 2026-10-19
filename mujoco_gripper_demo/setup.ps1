param(
    [string]$Python,
    [switch]$Offline
)

$ErrorActionPreference = 'Stop'
$DemoDir = $PSScriptRoot
$WorkspaceDir = Split-Path -Parent $DemoDir
$CacheDir = Join-Path $WorkspaceDir '.cache/uv'
$EnvPython = Join-Path $DemoDir '.venv/Scripts/python.exe'
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    throw 'uv is required. Install uv, then run this script again.'
}
if (-not $Python) {
    $LocalPython = Join-Path $WorkspaceDir '.python/cpython-3.10-windows-x86_64-none/python.exe'
    $Python = if (Test-Path -LiteralPath $LocalPython) { $LocalPython } else { '3.10' }
}
$UvOptions = @('--cache-dir', $CacheDir)
if ($Offline) { $UvOptions += '--offline' }
if (-not (Test-Path -LiteralPath $EnvPython)) {
    & uv @UvOptions venv --python $Python (Join-Path $DemoDir '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Failed to create the Python environment.' }
}
$Requirements = Join-Path $DemoDir 'requirements.lock'
if (-not (Test-Path -LiteralPath $Requirements)) {
    $Requirements = Join-Path $DemoDir 'requirements.txt'
}
& uv @UvOptions pip install --python $EnvPython --link-mode copy -r $Requirements
if ($LASTEXITCODE -ne 0) { throw 'Failed to install demo dependencies.' }
Write-Host 'Setup complete. Run the demo:'
Write-Host "powershell -ExecutionPolicy Bypass -File `"$(Join-Path $DemoDir 'run.ps1')`""
