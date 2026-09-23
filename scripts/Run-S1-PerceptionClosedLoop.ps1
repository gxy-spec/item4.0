param(
    [ValidateSet("Smoke", "Formal", "Negative", "All")]
    [string]$Mode = "All"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$PackageRoot = "D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"
$Launcher = Join-Path $PackageRoot "CarlaAir.ps1"
$CondaExe = "D:\program\minicoonda\Scripts\conda.exe"
$Runner = Join-Path $ProjectRoot "scripts\run_s1_perception_closed_loop.py"
$ReplayBuilder = Join-Path $ProjectRoot "scripts\build_synchronized_multiview_replay.py"
$SummaryBuilder = Join-Path $ProjectRoot "scripts\build_s1_summary.py"
$SmokeConfig = Join-Path $ProjectRoot "configs\experiments\s1_perception_closed_loop_smoke.yaml"
$FormalConfig = Join-Path $ProjectRoot "configs\experiments\s1_perception_closed_loop.yaml"
$NegativeConfig = Join-Path $ProjectRoot "configs\experiments\s1_perception_no_message_control.yaml"
$OutputBase = "E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop"

function Invoke-S1Run {
    param(
        [Parameter(Mandatory = $true)][string]$Label,
        [Parameter(Mandatory = $true)][string]$Config,
        [Parameter(Mandatory = $true)][string]$OutputRoot
    )
    Write-Host $Label -ForegroundColor Cyan
    $attempt = 0
    do {
        $attempt += 1
        & $CondaExe run --no-capture-output -n carlaAir python -u $Runner --config $Config | Out-Host
        $runnerExit = $LASTEXITCODE
        if ($runnerExit -eq 0) { break }
        $latestAttempt = Get-ChildItem -LiteralPath $OutputRoot -Directory -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        $retryable = $false
        if ($null -ne $latestAttempt) {
            $reportPath = Join-Path $latestAttempt.FullName "validation_report.json"
            if (Test-Path -LiteralPath $reportPath) {
                $attemptReport = Get-Content -LiteralPath $reportPath -Raw | ConvertFrom-Json
                $retryable = $attemptReport.status -eq "ERROR"
            }
        }
        if (-not $retryable -or $attempt -ge 2) { throw "$Label failed: $runnerExit" }
        Write-Warning "$Label encountered a retryable simulator error; retrying once after warm-up."
        Start-Sleep -Seconds 5
    } while ($attempt -lt 2)
    $run = Get-ChildItem -LiteralPath $OutputRoot -Directory |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($null -eq $run) { throw "$Label produced no run directory" }
    & $CondaExe run --no-capture-output -n carlaAir python -u $ReplayBuilder --run-dir $run.FullName | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$Label replay failed: $LASTEXITCODE" }
    & $CondaExe run --no-capture-output -n carlaAir python -u $SummaryBuilder --run-dir $run.FullName | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$Label summary failed: $LASTEXITCODE" }
    return $run
}

try {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher `
        Town10HD --port 2000 --quality Low --res 960x540 --no-traffic `
        --package-root (Join-Path $PackageRoot "WindowsNoEditor")
    if ($LASTEXITCODE -ne 0) { throw "CARLA-Air launcher failed: $LASTEXITCODE" }

    if ($Mode -in @("Smoke", "All")) {
        $null = Invoke-S1Run -Label "S1 perception 60-second smoke" -Config $SmokeConfig `
            -OutputRoot (Join-Path $OutputBase "smoke")
    }
    if ($Mode -in @("Formal", "All")) {
        $formal = Invoke-S1Run -Label "S1 perception 150-second formal" -Config $FormalConfig `
            -OutputRoot (Join-Path $OutputBase "formal")
        Set-Content -LiteralPath (Join-Path $OutputBase "LATEST.txt") -Value $formal.FullName -Encoding UTF8
    }
    if ($Mode -in @("Negative", "All")) {
        $null = Invoke-S1Run -Label "S1 perception no-message negative control" -Config $NegativeConfig `
            -OutputRoot (Join-Path $OutputBase "negative_control")
    }
}
finally {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher --kill
}
