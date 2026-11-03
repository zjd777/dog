param(
    [ValidateSet('Build', 'Rebuild', 'Flash', 'Probe', 'Clean')]
    [string]$Action = 'Build'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$make = 'C:\Users\MECHREVO\Desktop\make\mingw64\bin\make.exe'
$gccBin = 'C:\Users\MECHREVO\Desktop\make\gcc\bin'
$openocdBin = 'C:\Users\MECHREVO\Desktop\make\OpenOCD-20250710-0.12.0\bin'
$env:PATH = "$gccBin;$openocdBin;$(Split-Path $make);$env:PATH"

try {
    Set-Location -LiteralPath $projectRoot
    if ($Action -eq 'Clean') {
        $buildPath = [IO.Path]::GetFullPath((Join-Path $projectRoot 'build'))
        $expectedPath = [IO.Path]::GetFullPath($projectRoot) + '\build'
        if ($buildPath -ne $expectedPath) { throw 'Refusing to clean an unexpected path.' }
        if (Test-Path -LiteralPath $buildPath) {
            Remove-Item -LiteralPath $buildPath -Recurse -Force
        }
        Write-Host 'Removed build output.'
        exit 0
    }
    if (-not (Test-Path -LiteralPath $make)) { throw "GNU Make not found: $make" }
    if ($Action -eq 'Rebuild') {
        & $make clean
        if ($LASTEXITCODE -ne 0) { throw 'Clean failed.' }
    }
    $target = 'all'
    if ($Action -eq 'Flash') { $target = 'flash' }
    if ($Action -eq 'Probe') { $target = 'probe' }
    & $make -j8 $target
    if ($LASTEXITCODE -ne 0) { throw "make $target failed. See the output above." }
} catch {
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 1
}
exit 0
