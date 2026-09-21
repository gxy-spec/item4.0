param(
    [string]$Config = "E:\Research\CityInspection_GOC\configs\experiments\ci_e0_town10hd_zone_a.yaml"
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$packageRoot = "D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"
$launcher = Join-Path $packageRoot "CarlaAir.ps1"
$condaExe = "D:\program\minicoonda\Scripts\conda.exe"
$pythonScript = Join-Path $projectRoot "scripts\run_stage0_preview.py"

try {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $launcher `
        Town10HD --port 2000 --quality Low --res 960x540 --no-traffic `
        --package-root (Join-Path $packageRoot "WindowsNoEditor")
    if ($LASTEXITCODE -ne 0) { throw "CARLA-Air launcher failed: $LASTEXITCODE" }

    & $condaExe run --no-capture-output -n carlaAir python -u $pythonScript --config $Config
    if ($LASTEXITCODE -ne 0) { throw "CI-E0 preview failed: $LASTEXITCODE" }
}
finally {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $launcher --kill
}

