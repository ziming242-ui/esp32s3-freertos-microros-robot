[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot '.host-build'
$preferredCompiler = 'C:\msys64\ucrt64\bin\gcc.exe'
$compiler = if (Test-Path -LiteralPath $preferredCompiler) {
    $preferredCompiler
} else {
    (Get-Command gcc -ErrorAction Stop).Source
}

New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$commonOptions = @(
    '-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-O0', '-g3'
)

$driveInclude = Join-Path $projectRoot 'components\robot_drive\include'
$driveExecutable = Join-Path $buildDirectory 'robot_drive_core_tests.exe'
$driveArguments = @(
    $commonOptions
    "-I$driveInclude"
    (Join-Path $projectRoot 'components\robot_drive\robot_drive_core.c')
    (Join-Path $projectRoot 'components\robot_drive\robot_drive_kinematics.c')
    (Join-Path $projectRoot 'components\robot_drive\test\test_robot_drive_core.c')
    '-lm'
    '-o'
    $driveExecutable
)
& $compiler @driveArguments
if ($LASTEXITCODE -ne 0) {
    throw 'Module 02 host compilation failed.'
}
& $driveExecutable
if ($LASTEXITCODE -ne 0) {
    throw 'Module 02 host tests failed.'
}

$imuInclude = Join-Path $projectRoot 'components\robot_imu\include'
$imuExecutable = Join-Path $buildDirectory 'robot_imu_policy_tests.exe'
$imuArguments = @(
    $commonOptions
    "-I$imuInclude"
    (Join-Path $projectRoot 'components\robot_imu\robot_imu_policy.c')
    (Join-Path $projectRoot 'components\robot_imu\test\test_robot_imu_policy.c')
    '-lm'
    '-o'
    $imuExecutable
)
& $compiler @imuArguments
if ($LASTEXITCODE -ne 0) {
    throw 'Module 03 host compilation failed.'
}
& $imuExecutable
if ($LASTEXITCODE -ne 0) {
    throw 'Module 03 host tests failed.'
}

Write-Host 'Module 02/03 host core tests passed. No hardware was exercised.'
