param([switch]$SkipObstacle)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$PackageRoot = "D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"
$Launcher = Join-Path $PackageRoot "CarlaAir.ps1"
$CondaExe = "D:\program\minicoonda\Scripts\conda.exe"
$Runner = Join-Path $ProjectRoot "scripts\run_stage1_dynamic.py"
$ReplayBuilder = Join-Path $ProjectRoot "scripts\build_synchronized_multiview_replay.py"
$UGVRGBDReplayBuilder = Join-Path $ProjectRoot "scripts\build_ugv_rgbd_safety_replay.py"
$Configs = @(
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_rgbd_safety_obstacle_smoke.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_rgbd_safety_clear_smoke.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_rgbd_safety_stale_smoke.yaml")
)
if ($SkipObstacle) { $Configs = $Configs | Select-Object -Skip 1 }

try {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher `
        Town10HD --port 2000 --quality Low --res 960x540 --no-traffic `
        --package-root (Join-Path $PackageRoot "WindowsNoEditor")
    if ($LASTEXITCODE -ne 0) { throw "CARLA-Air launcher failed: $LASTEXITCODE" }

    foreach ($Config in $Configs) {
        Write-Host "Running $([System.IO.Path]::GetFileName($Config))" -ForegroundColor Cyan
        & $CondaExe run --no-capture-output -n carlaAir python -u $Runner --config $Config
        if ($LASTEXITCODE -ne 0) { throw "UGV RGB-D safety smoke failed for $Config (exit $LASTEXITCODE)" }

        $OutputLine = Get-Content -LiteralPath $Config -Encoding UTF8 |
            Where-Object { $_ -match '^  root:' } | Select-Object -First 1
        if (-not $OutputLine) { throw "Output root is missing in $Config" }
        $OutputRoot = ($OutputLine -replace '^  root:\s*', '').Trim()
        $Run = Get-ChildItem -LiteralPath $OutputRoot -Directory |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if ($null -eq $Run) { throw "No run directory found under $OutputRoot" }
        & $CondaExe run --no-capture-output -n carlaAir python -u $ReplayBuilder --run-dir $Run.FullName
        if ($LASTEXITCODE -ne 0) { throw "Synchronized replay generation failed for $($Run.FullName)" }
        & $CondaExe run --no-capture-output -n carlaAir python -u $UGVRGBDReplayBuilder --run-dir $Run.FullName
        if ($LASTEXITCODE -ne 0) { throw "UGV RGB-D replay generation failed for $($Run.FullName)" }
    }
}
finally {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher --kill
}
