param(
    [ValidateSet('Build', 'Rebuild', 'Flash', 'Probe')]
    [string]$Action = 'Build'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$keil = 'D:\Users\MECHREVO\AppData\Local\Keil_v5\UV4\UV4.exe'
$openocd = 'C:\Users\MECHREVO\Desktop\make\OpenOCD-20250710-0.12.0\bin\openocd.exe'
$openocdScripts = 'C:\Users\MECHREVO\Desktop\make\OpenOCD-20250710-0.12.0\share\openocd\scripts'
$project = Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_IMU.uvprojx'
$log = Join-Path $projectRoot 'MDK-ARM\vscode-build.log'
$image = Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_IMU\CtrBoard-H7_IMU.axf'
$targetConfig = Join-Path $PSScriptRoot 'stlink-h723.cfg'

try {
    if ($Action -ne 'Probe') {
        if (-not (Test-Path -LiteralPath $keil)) {
            throw "Keil uVision not found: $keil"
        }
        $buildFlag = '-b'
        if ($Action -eq 'Rebuild') { $buildFlag = '-r' }
        Write-Host "Building complete Keil target CtrBoard-H7_IMU ($buildFlag)..."
        Set-Content -LiteralPath $log -Value '' -Encoding UTF8
        # UV4 is a GUI executable: wait explicitly and forward its build log.
        $buildArgs = @($buildFlag, ('"' + $project + '"'), '-t', 'CtrBoard-H7_IMU', '-o', ('"' + $log + '"'))
        $buildProcess = Start-Process -FilePath $keil -ArgumentList $buildArgs -WorkingDirectory (Split-Path $project) -WindowStyle Hidden -Wait -PassThru
        $buildText = Get-Content -LiteralPath $log -Raw
        Write-Host $buildText
        # uVision returns 1 for a successful build with warnings.
        if ($buildProcess.ExitCode -notin @(0, 1) -or $buildText -notmatch ' - 0 Error\(s\)' -or -not (Test-Path -LiteralPath $image)) {
            throw "Build failed (uVision exit code $($buildProcess.ExitCode)). Download cancelled. See $log"
        }
        Write-Host "Build succeeded: $image"
    }

    if ($Action -in @('Flash', 'Probe')) {
        if (-not (Test-Path -LiteralPath $openocd)) {
            throw "OpenOCD not found: $openocd"
        }
        $openocdArgs = @('-s', $openocdScripts, '-f', $targetConfig)
        if ($Action -eq 'Probe') {
            $openocdArgs += @('-c', 'init; shutdown')
        } else {
            # Flash only after this invocation completed a successful build.
            $imageForTcl = $image.Replace('\', '/')
            $openocdArgs += @('-c', "program {$imageForTcl} verify reset exit")
        }
        & $openocd @openocdArgs
        if ($LASTEXITCODE -ne 0) {
            throw 'ST-Link connection/programming failed. Check USB detection, board power, GND, SWDIO and SWCLK. See OpenOCD output above.'
        }
    }
} catch {
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 1
}
exit 0
