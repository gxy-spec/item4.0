param([string[]]$Cases = @())

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$PackageRoot = "D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"
$Launcher = Join-Path $PackageRoot "CarlaAir.ps1"
$CondaExe = "D:\program\minicoonda\Scripts\conda.exe"
$Runner = Join-Path $ProjectRoot "scripts\run_stage1_dynamic.py"
$ReplayBuilder = Join-Path $ProjectRoot "scripts\build_ugv_rgbd_safety_replay.py"
$Configs = @(
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_20260929_clear.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_20260929_static_recovery.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_20260929_crossing_vehicle.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_20260929_crossing_pedestrian.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_20260929_occlusion.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_20260929_depth_failure.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_realistic_pedestrian.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_realistic_crossing_vehicle.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_realistic_occlusion.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e1_ugv_safety_realistic_overtake.yaml")
)
if ($Cases.Count -gt 0) {
    $Configs = $Configs | Where-Object {
        $name = [System.IO.Path]::GetFileNameWithoutExtension($_)
        @($Cases | Where-Object { $name.Contains($_, [System.StringComparison]::OrdinalIgnoreCase) }).Count -gt 0
    }
    if ($Configs.Count -eq 0) { throw "No safety config matched Cases: $($Cases -join ', ')" }
}
$FailedConfigs = [System.Collections.Generic.List[string]]::new()

try {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher `
        Town10HD --port 2000 --quality Low --res 960x540 --no-traffic `
        --package-root (Join-Path $PackageRoot "WindowsNoEditor")
    if ($LASTEXITCODE -ne 0) { throw "CARLA-Air launcher failed: $LASTEXITCODE" }

    foreach ($Config in $Configs) {
        Write-Host "Running $([System.IO.Path]::GetFileName($Config))" -ForegroundColor Cyan
        & $CondaExe run --no-capture-output -n carlaAir python -u $Runner --config $Config
        $RunExit = $LASTEXITCODE

        $OutputLine = Get-Content -LiteralPath $Config -Encoding UTF8 |
            Where-Object { $_ -match '^  root:' } | Select-Object -First 1
        if (-not $OutputLine) { throw "Output root is missing in $Config" }
        $OutputRoot = ($OutputLine -replace '^  root:\s*', '').Trim()
        $Run = Get-ChildItem -LiteralPath $OutputRoot -Directory -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if ($null -eq $Run) {
            $FailedConfigs.Add("$Config (runner exit $RunExit; no run directory)")
            continue
        }

        if (Test-Path (Join-Path $Run.FullName "synchronization\ugv_rgbd_frame_index.csv")) {
            & $CondaExe run --no-capture-output -n carlaAir python -u $ReplayBuilder --run-dir $Run.FullName
            if ($LASTEXITCODE -ne 0) { $FailedConfigs.Add("$Config (UGV RGB-D replay generation failed)") }
        }
        if ($RunExit -ne 0) { $FailedConfigs.Add("$Config (runner exit $RunExit)") }
    }
}
finally {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher --kill
}

if ($FailedConfigs.Count -gt 0) {
    Write-Host "One or more cases need review:" -ForegroundColor Yellow
    $FailedConfigs | ForEach-Object { Write-Host " - $_" -ForegroundColor Yellow }
    exit 1
}
Write-Host "All configured UGV RGB-D safety cases finished successfully." -ForegroundColor Green
