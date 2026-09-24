$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$PackageRoot = "D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"
$Launcher = Join-Path $PackageRoot "CarlaAir.ps1"
$CondaExe = "D:\program\minicoonda\Scripts\conda.exe"
$BatchRunner = Join-Path $ProjectRoot "scripts\run_s1_multiseed_batch.py"
$Design = Join-Path $ProjectRoot "configs\experiments\s1_multiseed_3x2_batch.yaml"
$exitCode = 0

try {
    & $CondaExe run --no-capture-output -n carlaAir python -u $BatchRunner `
        --design $Design --manage-simulator --launcher $Launcher --package-root $PackageRoot
    $exitCode = $LASTEXITCODE
}
catch {
    Write-Error $_
    if ($exitCode -eq 0) { $exitCode = 1 }
}
finally {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Launcher --kill
}

exit $exitCode
