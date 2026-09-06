[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$idfWindowsPath = Join-Path $env:USERPROFILE '.platformio\packages\framework-espidf'

function Convert-ToWslPath {
    param([Parameter(Mandatory)][string]$WindowsPath)

    $fullPath = [System.IO.Path]::GetFullPath($WindowsPath)
    if ($fullPath -notmatch '^([A-Za-z]):\\(.*)$') {
        throw "Only local drive paths can be converted to WSL paths: $fullPath"
    }

    $drive = $Matches[1].ToLowerInvariant()
    $tail = $Matches[2].Replace('\', '/')
    return "/mnt/$drive/$tail"
}

if (-not (Test-Path -LiteralPath (Join-Path $idfWindowsPath 'export.sh'))) {
    throw "ESP-IDF was not found at $idfWindowsPath"
}

$projectWslPath = Convert-ToWslPath $projectRoot
$idfWslPath = Convert-ToWslPath $idfWindowsPath
$wslHome = (& wsl.exe -d Ubuntu-22.04 -- bash -lc 'printf %s "$HOME"').Trim()

if ($LASTEXITCODE -ne 0 -or -not $projectWslPath -or -not $idfWslPath -or -not $wslHome) {
    throw 'Unable to resolve the WSL build environment.'
}

$buildCommand = @"
set -e
export IDF_PATH='$idfWslPath'
export IDF_TOOLS_PATH='$wslHome/.espressif'
source '$idfWslPath/export.sh' >/dev/null
cd '$projectWslPath'
idf.py -B build-wsl -DIDF_TARGET=esp32s3 -DSDKCONFIG=sdkconfig.esp32-s3-devkitc-1 build
"@

Write-Host "Building ESP32-S3 firmware with ESP-IDF in WSL..."
& wsl.exe -d Ubuntu-22.04 -- bash -lc $buildCommand

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Host "Build complete: $projectRoot\build-wsl\esp32s3_freertos_basics.bin"
