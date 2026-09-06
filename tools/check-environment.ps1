[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$checks = [ordered]@{
    'WSL Ubuntu-22.04' = $false
    'ESP-IDF export.sh' = Test-Path -LiteralPath (Join-Path $env:USERPROFILE '.platformio\packages\framework-espidf\export.sh')
    'PlatformIO Python' = Test-Path -LiteralPath (Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe')
    'Windows esptool' = Test-Path -LiteralPath (Join-Path $env:USERPROFILE '.platformio\packages\tool-esptoolpy\esptool.py')
    'micro-ROS rcl.h' = Test-Path -LiteralPath (Join-Path $projectRoot 'components\micro_ros_espidf_component\include\rcl\rcl\rcl.h')
    'robot_microros.h' = Test-Path -LiteralPath (Join-Path $projectRoot 'components\robot_microros\include\robot_microros.h')
    'WSL compile database' = Test-Path -LiteralPath (Join-Path $projectRoot 'build-wsl\compile_commands.json')
}

$wslDistributions = @(& wsl.exe -l -q 2>$null) -replace "`0", ''
$checks['WSL Ubuntu-22.04'] = 'Ubuntu-22.04' -in $wslDistributions

$checks.GetEnumerator() | ForEach-Object {
    [pscustomobject]@{
        Check = $_.Key
        Result = if ($_.Value) { 'OK' } else { 'MISSING' }
    }
} | Format-Table -AutoSize

$ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
Write-Host ('Serial ports: ' + $(if ($ports.Count) { $ports -join ', ' } else { '(none detected)' }))

if ($checks.Values -contains $false) {
    exit 1
}
