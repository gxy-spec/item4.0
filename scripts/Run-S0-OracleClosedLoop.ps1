param(
    [ValidateSet("Positive", "Negative", "All")]
    [string]$Mode = "All"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$PackageRoot = "D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"
$Launcher = Join-Path $PackageRoot "CarlaAir.ps1"
$CondaExe = "D:\program\minicoonda\Scripts\conda.exe"
$Runner = Join-Path $ProjectRoot "scripts\run_s0_oracle_closed_loop.py"
$PositiveConfig = Join-Path $ProjectRoot "configs\experiments\s0_oracle_closed_loop.yaml"
$NegativeConfig = Join-Path $ProjectRoot "configs\experiments\s0_oracle_no_message_control.yaml"
$ReplayBuilder = Join-Path $ProjectRoot "scripts\build_synchronized_multiview_replay.py"

try {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher `
        Town10HD --port 2000 --quality Low --res 960x540 --no-traffic `
        --package-root (Join-Path $PackageRoot "WindowsNoEditor")
    if ($LASTEXITCODE -ne 0) { throw "CARLA-Air launcher failed: $LASTEXITCODE" }

    if ($Mode -in @("Positive", "All")) {
        Write-Host "S0 Oracle positive closed-loop acceptance" -ForegroundColor Cyan
        & $CondaExe run --no-capture-output -n carlaAir python -u $Runner --config $PositiveConfig
        if ($LASTEXITCODE -ne 0) { throw "S0 Oracle positive run failed: $LASTEXITCODE" }
        $PositiveRun = Get-ChildItem -LiteralPath "E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\formal" -Directory |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        & $CondaExe run --no-capture-output -n carlaAir python -u $ReplayBuilder --run-dir $PositiveRun.FullName
        if ($LASTEXITCODE -ne 0) { throw "S0 Oracle positive replay failed: $LASTEXITCODE" }
    }
    if ($Mode -in @("Negative", "All")) {
        Write-Host "S0 Oracle no-message negative control" -ForegroundColor Cyan
        & $CondaExe run --no-capture-output -n carlaAir python -u $Runner --config $NegativeConfig
        if ($LASTEXITCODE -ne 0) { throw "S0 Oracle negative control failed: $LASTEXITCODE" }
        $NegativeRun = Get-ChildItem -LiteralPath "E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\negative_control" -Directory |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        & $CondaExe run --no-capture-output -n carlaAir python -u $ReplayBuilder --run-dir $NegativeRun.FullName
        if ($LASTEXITCODE -ne 0) { throw "S0 Oracle negative replay failed: $LASTEXITCODE" }
    }
}
finally {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher --kill
}
