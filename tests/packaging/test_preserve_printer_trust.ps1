# Run with Windows PowerShell 5.1; no printer, network, registry or real install changes.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
. (Join-Path $repo 'scripts\windows\preserve_printer_trust.ps1')
$root = Join-Path ([IO.Path]::GetTempPath()) ('orca-trust-test-' + [Guid]::NewGuid().ToString('N'))
$old = Join-Path $root 'old install'
$new = Join-Path $root 'new install'
$state = Join-Path $root 'installer state'
$legacy = Join-Path $root 'legacy.sha256'
$oldCert = Join-Path $old 'resources\cert'
$newCert = Join-Path $new 'resources\cert'
foreach ($dir in @($oldCert, $newCert, $state)) { $null = [IO.Directory]::CreateDirectory($dir) }
$fixtures = @(Read-Certificates (Join-Path $repo 'resources\cert\printer.cer'))
if ($fixtures.Count -lt 5) { throw 'Tests need five distinct public certificate fixtures.' }
$a, $b, $c, $d, $e = $fixtures[0..4]
$passed = 0

function Assert-True($Condition, [string]$Message) {
    if (!$Condition) { throw $Message }
}
function Assert-Hashes([string]$Path, $Expected) {
    $actual = @((Read-Certificates $Path) | ForEach-Object { $_.Hash } | Sort-Object)
    Assert-True (($actual -join ',') -ceq (($Expected | Sort-Object) -join ',')) "Unexpected trust set in $Path"
}
function Install-Stock($Certificates) {
    $text = ($Certificates | ForEach-Object { $_.Pem }) -join ''
    Save-CertificateText (Join-Path $newCert 'printer.cer') $text
    Save-CertificateText (Join-Path $newCert 'printer-stock.cer') $text
}
function Assert-Fails($Action, [string]$Message) {
    $failed = $false
    try { & $Action } catch { $failed = $true }
    Assert-True $failed $Message
}
function Pass([string]$Name) { $script:passed++; Write-Host "PASS $Name" }

try {
    Save-CertificateText $legacy ($a.Hash + "`n" + $b.Hash + "`n")
    Save-CertificateText (Join-Path $oldCert 'printer.cer') ($a.Pem + $b.Pem + $c.Pem)
    Capture-PrinterTrust $old $state $legacy
    Install-Stock @($b, $d)
    Restore-PrinterTrust $new $state
    Assert-Hashes (Join-Path $newCert 'printer.cer') @($b.Hash, $c.Hash, $d.Hash)
    Pass 'First migration preserves local trust and removes retired vendor trust'

    # The old baseline, not the incoming stock, identifies vendor cert D later removed.
    Capture-PrinterTrust $new $state $legacy
    Install-Stock @($b, $e)
    Restore-PrinterTrust $new $state
    Assert-Hashes (Join-Path $newCert 'printer.cer') @($b.Hash, $c.Hash, $e.Hash)
    Pass 'Subsequent upgrade removes vendor certificates introduced after the legacy baseline'

    Capture-PrinterTrust $new $state $legacy
    Install-Stock @($b, $e)
    Restore-PrinterTrust $new $state
    Assert-Hashes (Join-Path $newCert 'printer.cer') @($b.Hash, $c.Hash, $e.Hash)
    Pass 'Same-version repair is idempotent'

    Install-Stock @($a, $b)
    Restore-PrinterTrust $new $state
    Assert-Hashes (Join-Path $newCert 'printer.cer') @($a.Hash, $b.Hash, $c.Hash)
    Pass 'Downgrade to an opted-in build keeps local additions'

    Save-CertificateText (Join-Path $oldCert 'printer.cer') ($a.Pem + ($c.Pem -replace "`r`n", "`n") + $c.Pem)
    Capture-PrinterTrust $old $state $legacy
    Install-Stock @($a, $c)
    $before = [Convert]::ToBase64String([IO.File]::ReadAllBytes((Join-Path $newCert 'printer.cer')))
    Restore-PrinterTrust $new $state
    Assert-True ($before -ceq [Convert]::ToBase64String([IO.File]::ReadAllBytes((Join-Path $newCert 'printer.cer')))) 'Duplicate cert changed the shipped bundle.'
    Pass 'Formatting differences and duplicates do not duplicate trust'

    Save-CertificateText (Join-Path $oldCert 'printer.cer') ($a.Pem + $b.Pem)
    Capture-PrinterTrust $old $state $legacy
    Install-Stock @($b, $d)
    $before = [IO.File]::ReadAllText((Join-Path $newCert 'printer.cer'))
    Restore-PrinterTrust $new $state
    Assert-True ($before -ceq [IO.File]::ReadAllText((Join-Path $newCert 'printer.cer'))) 'Stock-only migration changed bytes.'
    Pass 'No local additions leaves the shipped file byte-identical'

    foreach ($bad in @('-----BEGIN PRIVATE KEY-----', '-----BEGIN CERTIFICATE-----AAAA-----END CERTIFICATE-----', ($a.Pem + 'trailing junk'), $a.Pem.Substring(0, 100))) {
        Save-CertificateText (Join-Path $oldCert 'printer.cer') $bad
        Assert-Fails { Capture-PrinterTrust $old $state $legacy } 'Invalid capture was accepted.'
    }
    Pass 'Private key markers, invalid DER, junk and truncation abort capture'

    Save-CertificateText (Join-Path $oldCert 'printer.cer') ($a.Pem + $c.Pem)
    Save-CertificateText (Join-Path $oldCert 'printer-stock.cer') 'corrupt'
    Assert-Fails { Capture-PrinterTrust $old $state $legacy } 'Corrupt old baseline was accepted.'
    [IO.File]::Delete((Join-Path $oldCert 'printer-stock.cer'))
    Pass 'Malformed old baseline aborts before uninstall'

    Save-CertificateText (Join-Path $state 'printer-local.cer') 'corrupt'
    Install-Stock @($a, $b)
    $before = [IO.File]::ReadAllText((Join-Path $newCert 'printer.cer'))
    Assert-Fails { Restore-PrinterTrust $new $state } 'Corrupt captured state was accepted.'
    Assert-True ($before -ceq [IO.File]::ReadAllText((Join-Path $newCert 'printer.cer'))) 'Failed restore touched stock.'
    Pass 'Malformed captured state leaves newly installed stock intact'

    Capture-PrinterTrust $old $state $legacy
    $locked = [IO.File]::Open((Join-Path $newCert 'printer.cer'), 'Open', 'Read', 'None')
    try { Assert-Fails { Restore-PrinterTrust $new $state } 'Locked destination did not abort.' }
    finally { $locked.Dispose() }
    Assert-True ($before -ceq [IO.File]::ReadAllText((Join-Path $newCert 'printer.cer'))) 'Locked restore changed stock.'
    Assert-True (@(Get-ChildItem $newCert -Filter '*.tmp').Count -eq 0) 'Temporary output remained.'
    Pass 'Locked destination preserves stock and removes temporary output'

    Save-CertificateText (Join-Path $oldCert 'printer.cer') (' ' * 1048577)
    Assert-Fails { Capture-PrinterTrust $old $state $legacy } 'Oversized input was accepted.'
    Pass 'Oversized input is rejected'
    Write-Host "$passed migration tests passed."
} finally { Remove-Item -LiteralPath $root -Recurse -Force }
