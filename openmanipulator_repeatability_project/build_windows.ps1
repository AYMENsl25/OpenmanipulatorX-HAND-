$ErrorActionPreference = 'Stop'
$projectDir = $PSScriptRoot
$workspaceDir = Split-Path $projectDir -Parent
$pythonExe = Join-Path $workspaceDir 'venv\Scripts\python.exe'
$releaseStamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
# PyInstaller COLLECT removes an existing output directory before rebuilding.
# Always use a new release directory to preserve calibration and live logs.
$releaseDir = Join-Path $workspaceDir "dist\releases\$releaseStamp"
& $pythonExe -m PyInstaller --noconfirm --distpath $releaseDir --workpath (Join-Path $workspaceDir 'build\desktop') (Join-Path $projectDir 'OpenManipulator.spec')
if ($LASTEXITCODE -ne 0) { throw 'Executable build failed' }
$appDir = Join-Path $releaseDir 'ISU-XR-OpenManipulator-Smooth'
$configDir = Join-Path $appDir 'openmanipulator_repeatability_project\config'
$calibrationDir = Join-Path $appDir 'data\vision_calibration'
New-Item -ItemType Directory -Force $configDir, $calibrationDir | Out-Null
Copy-Item -LiteralPath (Join-Path $projectDir 'config\experiment_config.json') -Destination $configDir
Copy-Item -LiteralPath (Join-Path $workspaceDir 'scan_cube_xyz_config.json') -Destination $appDir
foreach ($profile in @('center.json', 'left.json', 'right.json', 'moving_camera.json')) {
    $sourceProfile = Join-Path $workspaceDir "data\vision_calibration\$profile"
    if (Test-Path -LiteralPath $sourceProfile) { Copy-Item -LiteralPath $sourceProfile -Destination $calibrationDir }
}
Copy-Item -LiteralPath (Join-Path $projectDir 'WINDOWS_APP.md') -Destination (Join-Path $appDir 'README.md')
$firmwareDir = Join-Path $appDir 'firmware\OpenManipulatorXYZController'
New-Item -ItemType Directory -Force $firmwareDir | Out-Null
Copy-Item -LiteralPath (Join-Path $projectDir 'opencr_firmware\OpenManipulatorXYZController\OpenManipulatorXYZController.ino') -Destination $firmwareDir
Write-Output "Ready: $appDir\ISU-XR-OpenManipulator-Smooth.exe"
