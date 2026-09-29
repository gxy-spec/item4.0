param()

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$PackageRoot = "D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"
$Launcher = Join-Path $PackageRoot "CarlaAir.ps1"
$CondaExe = "D:\program\minicoonda\Scripts\conda.exe"
$Runner = Join-Path $ProjectRoot "scripts\run_stage1_dynamic.py"
$Configs = @(
    (Join-Path $ProjectRoot "configs\experiments\ci_e7_persistent_target_communication_20261002_smoke.yaml"),
    (Join-Path $ProjectRoot "configs\experiments\ci_e7_persistent_target_communication_20261002_formal.yaml")
)

function Start-CarlaAir {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher `
        Town10HD --port 2000 --quality Low --res 960x540 --no-traffic `
        --package-root (Join-Path $PackageRoot "WindowsNoEditor")
    if ($LASTEXITCODE -ne 0) { throw "CARLA-Air launcher failed: $LASTEXITCODE" }
}

$ListeningPorts = @(Get-NetTCPConnection -State Listen -LocalPort 2000,41451 -ErrorAction SilentlyContinue)
if ($ListeningPorts.Count -gt 0) {
    throw "CARLA/AirSim ports 2000 or 41451 are already in use. Stop the existing simulation before this run."
}

try {
    for ($RunIndex = 0; $RunIndex -lt $Configs.Count; $RunIndex++) {
        if ($RunIndex -gt 0) {
            & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher --kill
            Start-Sleep -Seconds 2
        }
        Start-CarlaAir
        $Config = $Configs[$RunIndex]
        Write-Host "Running $([System.IO.Path]::GetFileName($Config))" -ForegroundColor Cyan
        $RunStartedAt = Get-Date
        & $CondaExe run --no-capture-output -n carlaAir python -u $Runner --config $Config
        $RunExit = $LASTEXITCODE

        $OutputLine = Get-Content -LiteralPath $Config -Encoding UTF8 |
            Where-Object { $_ -match '^  root:' } | Select-Object -First 1
        if (-not $OutputLine) { throw "Output root is missing in $Config" }
        $OutputRoot = ($OutputLine -replace '^  root:\s*', '').Trim()
        $Run = Get-ChildItem -LiteralPath $OutputRoot -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.LastWriteTime -ge $RunStartedAt.AddSeconds(-2) } |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if ($null -eq $Run) { throw "Runner produced no output directory for $Config (exit $RunExit)" }

        $ReportPath = Join-Path $Run.FullName "validation_report.json"
        if (-not (Test-Path -LiteralPath $ReportPath)) { throw "Validation report is missing: $ReportPath" }
        $Report = Get-Content -LiteralPath $ReportPath -Raw | ConvertFrom-Json
        if ($Report.status -ne "PASS") {
            throw "Acceptance failed for $($Run.FullName); preserve and inspect validation_report.json before any new acquisition."
        }
        if ($RunExit -ne 0) { throw "Runner exited with $RunExit after creating $($Run.FullName)" }
        Write-Host "Accepted run: $($Run.FullName)" -ForegroundColor Green
    }
}
finally {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher --kill
}

Write-Host "E7 persistent UAV/UGV target communication acceptance completed." -ForegroundColor Green
