$ErrorActionPreference = "Stop"
$runner = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "Run-Stage1-Dynamic.ps1"

Write-Host "CI-E1 step 1/2: five-second dynamic smoke test" -ForegroundColor Cyan
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $runner -Mode Smoke
if ($LASTEXITCODE -ne 0) { throw "CI-E1 smoke test failed; formal run was not started." }

Write-Host "CI-E1 step 2/2: twenty-second formal capture" -ForegroundColor Cyan
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $runner -Mode Formal
if ($LASTEXITCODE -ne 0) { throw "CI-E1 formal run failed." }

