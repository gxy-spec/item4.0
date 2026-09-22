[CmdletBinding()]
param(
    [switch]$Apply
)

$ErrorActionPreference = "Stop"

$dataRoot = [System.IO.Path]::GetFullPath("E:\CarlaAirData\CityInspection_GOC").TrimEnd("\")
$repoRoot = [System.IO.Path]::GetFullPath((Split-Path -Parent $PSScriptRoot)).TrimEnd("\")

# This is an intentionally explicit allow-list. Never add LATEST targets or the
# accepted formal runs to it merely because a newer directory happens to exist.
$obsoleteData = @(
    "E:\CarlaAirData\CityInspection_GOC\e0_platform\_debug_attempts",
    "E:\CarlaAirData\CityInspection_GOC\e1_dynamic_sensor_acceptance\formal\CI_E1_T10_ZA_S1001_20260921T103917Z",
    "E:\CarlaAirData\CityInspection_GOC\e1_dynamic_sensor_acceptance\smoke\CI_E1_SMOKE_T10_ZA_S1001_20260921T103822Z",
    "E:\CarlaAirData\CityInspection_GOC\e1_physical_fix\smoke\CI_E1_PHYSICAL_FIX_T10_ZA_S1001_20260922T011942Z",
    "E:\CarlaAirData\CityInspection_GOC\e1_physical_fix\smoke\CI_E1_PHYSICAL_FIX_T10_ZA_S1001_20260922T012149Z",
    "E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\formal\S0_ORACLE_T10_ZA_S1001_20260922T061048Z",
    "E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\formal\S0_ORACLE_T10_ZA_S1001_20260922T061339Z",
    "E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\formal\S0_ORACLE_T10_ZA_S1001_20260922T062125Z",
    "E:\CarlaAirData\CityInspection_GOC\e2_candidate_baseline\CI_E2_UAV_CANDIDATE_T10_ZA_S1001_20260922T021911Z",
    "E:\CarlaAirData\CityInspection_GOC\e2_candidate_baseline\CI_E2_UAV_CANDIDATE_T10_ZA_S1001_20260922T022010Z",
    "E:\CarlaAirData\CityInspection_GOC\e2_candidate_baseline\CI_E2_UAV_CANDIDATE_T10_ZA_S1001_20260922T022209Z",
    "E:\CarlaAirData\CityInspection_GOC\e2_candidate_baseline\CI_E2_UAV_CANDIDATE_T10_ZA_S1001_20260922T022453Z"
)

function Assert-ChildPath {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Root
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path).TrimEnd("\")
    $rootPrefix = $Root + "\"
    if (-not $fullPath.StartsWith($rootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing path outside approved root: $fullPath"
    }
    if ($fullPath.Equals($Root, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to operate on root itself: $Root"
    }
    return $fullPath
}

$targets = New-Object System.Collections.Generic.List[string]
foreach ($path in $obsoleteData) {
    $targets.Add((Assert-ChildPath -Path $path -Root $dataRoot))
}

$cacheCandidates = @()
$pytestCache = Join-Path $repoRoot ".pytest_cache"
if (Test-Path -LiteralPath $pytestCache -PathType Container) {
    $cacheCandidates += $pytestCache
}
$cacheCandidates += Get-ChildItem -LiteralPath $repoRoot -Directory -Recurse -Filter "__pycache__" |
    Select-Object -ExpandProperty FullName
foreach ($path in $cacheCandidates) {
    $targets.Add((Assert-ChildPath -Path $path -Root $repoRoot))
}

$existing = @($targets | Where-Object { Test-Path -LiteralPath $_ -PathType Container } | Select-Object -Unique)
$totalBytes = 0L
foreach ($path in $existing) {
    $totalBytes += [long]((Get-ChildItem -LiteralPath $path -File -Recurse | Measure-Object -Property Length -Sum).Sum)
}

Write-Host ("Obsolete targets found: {0}" -f $existing.Count)
Write-Host ("Recoverable size: {0:N2} MB" -f ($totalBytes / 1MB))
foreach ($path in $existing) {
    Write-Host ("  {0}" -f $path)
}

if (-not $Apply) {
    Write-Host "Dry run only. Re-run with -Apply after reviewing the allow-list above."
    exit 0
}

foreach ($path in $existing) {
    Remove-Item -LiteralPath $path -Recurse -Force
}

Write-Host ("Removed {0} directories and recovered approximately {1:N2} MB." -f $existing.Count, ($totalBytes / 1MB))
