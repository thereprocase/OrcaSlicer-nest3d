# Real silent installers with a unique test registry key and temporary install dirs.
# Intended for disposable Windows CI runners; never touches Orca's registry/config.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
. (Join-Path $repo 'scripts\windows\preserve_printer_trust.ps1')
$identity = 'ReproTrustSmoke-' + [Guid]::NewGuid().ToString('N')
$root = Join-Path ([IO.Path]::GetTempPath()) $identity
$oldInstall = Join-Path $root 'old installation'
$newInstall = Join-Path $root 'new installation'
$fixtures = @(Read-Certificates (Join-Path $repo 'resources\cert\printer.cer'))
# Use a public certificate from the other vendor bundle as the local test addition.
$extra = @(Read-Certificates (Join-Path $repo 'resources\cert\slicer_base64.cer'))[0]
if ($fixtures.Hash -contains $extra.Hash) { throw 'Local test fixture must differ from printer stock.' }
$principal = [Security.Principal.WindowsPrincipal]::new([Security.Principal.WindowsIdentity]::GetCurrent())
if (!$principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) { throw 'Use an elevated disposable CI runner.' }

function Build-TestInstaller([string]$Name, [bool]$Preserve, $Stock) {
    $source = Join-Path $root $Name
    $build = Join-Path $source 'build'
    $null = [IO.Directory]::CreateDirectory((Join-Path $source 'resources\cert'))
    Save-CertificateText (Join-Path $source 'resources\cert\printer.cer') (($Stock | ForEach-Object { $_.Pem }) -join '')
    $module = (Join-Path $repo 'cmake\repro\PreservePrinterTrust.cmake').Replace('\', '/')
    $extension = if ($Preserve) { "include(`"$module`")" } else { '' }
    $cmake = @'
cmake_minimum_required(VERSION 3.28)
project(ReproTrustInstallerTest NONE)
install(FILES resources/cert/printer.cer DESTINATION resources/cert)
set(CPACK_PACKAGE_NAME "@IDENTITY@")
set(CPACK_PACKAGE_VENDOR "ReproTrustInstallerTest")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_FILE_NAME "trust-test")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "@IDENTITY@")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "@IDENTITY@")
set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
set(CPACK_GENERATOR NSIS)
@EXTENSION@
include(CPack)
'@
    [IO.File]::WriteAllText((Join-Path $source 'CMakeLists.txt'), $cmake.Replace('@IDENTITY@', $identity).Replace('@EXTENSION@', $extension))
    & cmake -S $source -B $build | Out-Host
    if ($LASTEXITCODE -ne 0) { throw 'Test package configuration failed.' }
    & cpack --config (Join-Path $build 'CPackConfig.cmake') -B $build -G NSIS | Out-Host
    if ($LASTEXITCODE -ne 0) { throw 'Test NSIS compilation failed.' }
    return Join-Path $build 'trust-test.exe'
}
function Run-Installer([string]$Exe, [string]$Directory) {
    # NSIS requires /D last and consumes the remainder as a path, without quotes.
    $arguments = if ($Directory) { "/S /D=$Directory" } else { '/S' }
    $process = Start-Process -FilePath $Exe -ArgumentList $arguments -PassThru
    if (!$process.WaitForExit(90000)) { $process.Kill(); throw 'Silent installer timed out.' }
    if ($process.ExitCode -ne 0) { throw "Installer failed: $($process.ExitCode)" }
}
function Check-NewTrust {
    $actual = @((Read-Certificates (Join-Path $newInstall 'resources\cert\printer.cer')).Hash | Sort-Object)
    $expected = @($fixtures[1].Hash, $fixtures[2].Hash, $extra.Hash | Sort-Object)
    if (($actual -join ',') -cne ($expected -join ',')) { throw 'Installer did not preserve exactly new stock plus local trust.' }
}

try {
    $oldExe = Build-TestInstaller 'old package' $false @($fixtures[0], $fixtures[1])
    $newExe = Build-TestInstaller 'new package' $true @($fixtures[1], $fixtures[2])
    Run-Installer $oldExe $oldInstall
    [IO.File]::AppendAllText((Join-Path $oldInstall 'resources\cert\printer.cer'), $extra.Pem)
    Run-Installer $newExe $newInstall
    Check-NewTrust
    if (Test-Path (Join-Path $oldInstall 'resources\cert\printer.cer')) { throw 'Old uninstall did not run.' }
    Write-Host 'PASS real silent legacy upgrade across installation directories'
    Run-Installer $newExe $newInstall
    Check-NewTrust
    Write-Host 'PASS real silent repeat upgrade without duplicate trust'
    # Revoking the local addition must survive the next upgrade.
    Copy-Item (Join-Path $newInstall 'resources\cert\printer-stock.cer') (Join-Path $newInstall 'resources\cert\printer.cer') -Force
    Run-Installer $newExe $newInstall
    $actual = @(Read-Certificates (Join-Path $newInstall 'resources\cert\printer.cer'))
    if ($actual.Hash -contains $extra.Hash -or $actual.Count -ne 2) { throw 'Removed local certificate was resurrected.' }
    Write-Host 'PASS local certificate removal persists across upgrades'
    Run-Installer (Join-Path $newInstall 'Uninstall.exe') ''
    # NSIS uninstall may spawn its temporary copy; wait for file removal, bounded.
    for ($i = 0; $i -lt 100 -and (Test-Path (Join-Path $newInstall 'resources\cert\printer.cer')); $i++) { Start-Sleep -Milliseconds 100 }
    if (Test-Path (Join-Path $newInstall 'resources\cert\printer-stock.cer')) { throw 'Plain uninstall left stock baseline.' }
    if (Test-Path (Join-Path $newInstall 'resources\cert\printer.cer')) { throw 'Plain uninstall left printer trust.' }
    Write-Host 'PASS real plain uninstall removes installed trust and baseline'
} finally {
    # Keep failed CI fixtures for inspection; they contain only public vendor certs.
    foreach ($path in @($oldInstall, $newInstall)) {
        $uninstaller = Join-Path $path 'Uninstall.exe'
        if (Test-Path $uninstaller) { Start-Process $uninstaller -ArgumentList '/S' -Wait }
    }
}
