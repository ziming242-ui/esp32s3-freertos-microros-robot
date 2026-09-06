[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$componentPath = Join-Path $projectRoot 'components\micro_ros_espidf_component'
$patchPath = Join-Path $projectRoot 'patches\micro_ros_espidf_component\0001-optional-log4cxx-ignore.patch'

if (-not (Test-Path -LiteralPath (Join-Path $componentPath '.git'))) {
    & git -C $projectRoot submodule update --init --recursive
    if ($LASTEXITCODE -ne 0) {
        throw 'Unable to initialize the micro-ROS ESP-IDF submodule.'
    }
}

& git -C $componentPath apply --check $patchPath 2>$null
if ($LASTEXITCODE -eq 0) {
    & git -C $componentPath apply $patchPath
    if ($LASTEXITCODE -ne 0) {
        throw 'Unable to apply the micro-ROS compatibility patch.'
    }
    Write-Host 'Applied micro-ROS compatibility patch.'
}
else {
    & git -C $componentPath apply --reverse --check $patchPath 2>$null
    if ($LASTEXITCODE -ne 0) {
        throw 'The micro-ROS component is neither patchable nor already patched.'
    }
    Write-Host 'micro-ROS compatibility patch is already applied.'
}

Write-Host 'Dependencies are ready.'
