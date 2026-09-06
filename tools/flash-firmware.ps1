[CmdletBinding()]
param(
    [string]$Port,
    [int]$Baud = 460800
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildRoot = Join-Path $projectRoot 'build-wsl'
$flashManifestPath = Join-Path $buildRoot 'flasher_args.json'
$pythonPath = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'
$esptoolPath = Join-Path $env:USERPROFILE '.platformio\packages\tool-esptoolpy\esptool.py'

foreach ($requiredPath in @($flashManifestPath, $pythonPath, $esptoolPath)) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "Required file not found: $requiredPath"
    }
}

$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if (-not $Port) {
    if ($availablePorts.Count -eq 1) {
        $Port = $availablePorts[0]
    } elseif ($availablePorts.Count -eq 0) {
        throw 'No serial port is available. Connect the ESP32-S3 and retry.'
    } else {
        throw "Multiple serial ports detected ($($availablePorts -join ', ')). Run this script with -Port COMx."
    }
}

if ($Port -notin $availablePorts) {
    $shownPorts = if ($availablePorts.Count) { $availablePorts -join ', ' } else { '(none)' }
    throw "Serial port $Port is not available. Detected ports: $shownPorts"
}

$manifest = Get-Content -LiteralPath $flashManifestPath -Raw | ConvertFrom-Json
$arguments = @(
    $esptoolPath,
    '--chip', [string]$manifest.extra_esptool_args.chip,
    '--port', $Port,
    '--baud', [string]$Baud,
    '--before', ([string]$manifest.extra_esptool_args.before).Replace('-', '_'),
    '--after', ([string]$manifest.extra_esptool_args.after).Replace('-', '_'),
    'write_flash',
    '--flash_mode', [string]$manifest.flash_settings.flash_mode,
    '--flash_size', [string]$manifest.flash_settings.flash_size,
    '--flash_freq', [string]$manifest.flash_settings.flash_freq
)

$flashEntries = foreach ($property in $manifest.flash_files.PSObject.Properties) {
    [pscustomobject]@{
        Offset = $property.Name
        NumericOffset = [Convert]::ToInt32($property.Name.Substring(2), 16)
        File = Join-Path $buildRoot ([string]$property.Value)
    }
}

foreach ($entry in ($flashEntries | Sort-Object NumericOffset)) {
    if (-not (Test-Path -LiteralPath $entry.File)) {
        throw "Firmware image not found: $($entry.File)"
    }
    $arguments += @($entry.Offset, $entry.File)
}

Write-Host "Flashing ESP32-S3 through $Port. Close SSCOM first if it owns the port."
& $pythonPath @arguments

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
