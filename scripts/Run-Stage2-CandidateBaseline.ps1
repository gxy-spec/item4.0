param(
    [string]$InputRun = "E:\CarlaAirData\CityInspection_GOC\e1_physical_fix\smoke\CI_E1_PHYSICAL_FIX_T10_ZA_S1001_20260922T012328Z",
    [int]$MaxFrames = 0
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Python = "D:\program\minicoonda\envs\carlaAir\python.exe"
$Arguments = @(
    (Join-Path $PSScriptRoot "run_stage2_candidate_baseline.py"),
    "--input-run", $InputRun
)
if ($MaxFrames -gt 0) {
    $Arguments += @("--max-frames", $MaxFrames)
}
Set-Location $ProjectRoot
& $Python @Arguments
exit $LASTEXITCODE
