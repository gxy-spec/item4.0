param(
    [int]$Seed = 1001
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$packageRoot = "D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"
$launcher = Join-Path $packageRoot "CarlaAir.ps1"
$condaExe = "D:\program\minicoonda\Scripts\conda.exe"
$pythonScript = Join-Path $projectRoot "tools\survey_town10hd.py"
$outputRoot = "E:\CarlaAirData\CityInspection_GOC\e0_platform\scene_definition_v1"

try {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $launcher `
        Town10HD --port 2000 --quality Low --res 960x540 --no-traffic `
        --package-root (Join-Path $packageRoot "WindowsNoEditor")
    if ($LASTEXITCODE -ne 0) { throw "CARLA-Air launcher failed: $LASTEXITCODE" }

    & $condaExe run --no-capture-output -n carlaAir python -u $pythonScript `
        --output-root $outputRoot --seed $Seed
    if ($LASTEXITCODE -ne 0) { throw "Town10HD survey failed: $LASTEXITCODE" }
}
finally {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $launcher --kill
}

