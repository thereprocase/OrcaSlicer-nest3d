# Installer-only public certificate migration. Never read Orca user configuration.
param(
    [ValidateSet('Capture', 'Restore')][string]$Mode,
    [string]$Installation,
    [string]$StateDirectory,
    [string]$LegacyFingerprints
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Read-Certificates([string]$Path, [switch]$AllowEmpty) {
    $bytes = [IO.File]::ReadAllBytes($Path)
    if ($bytes.Length -gt 1048576) { throw 'Certificate bundle exceeds the migration size limit.' }
    $text = [Text.Encoding]::ASCII.GetString($bytes)
    $pattern = '-----BEGIN CERTIFICATE-----\s*([A-Za-z0-9+/=\s]+?)\s*-----END CERTIFICATE-----'
    $matches = [regex]::Matches($text, $pattern)
    # Reject keys, PKCS12, comments and malformed/truncated PEM instead of copying them.
    if ([regex]::Replace($text, $pattern, '').Trim().Length -ne 0) {
        throw 'Bundle contains data other than PEM public certificates.'
    }
    if ((!$AllowEmpty -and $matches.Count -eq 0) -or $matches.Count -gt 128) {
        throw 'Unexpected number of certificates in bundle.'
    }
    $seen = @{}
    foreach ($match in $matches) {
        $der = [Convert]::FromBase64String($match.Groups[1].Value -replace '\s', '')
        $cert = [Security.Cryptography.X509Certificates.X509Certificate2]::new($der)
        try {
            if ($cert.HasPrivateKey -or [Convert]::ToBase64String($cert.RawData) -cne [Convert]::ToBase64String($der)) {
                throw 'Bundle entry is not a single public X.509 certificate.'
            }
            $sha = [Security.Cryptography.SHA256]::Create()
            try { $hash = [BitConverter]::ToString($sha.ComputeHash($der)).Replace('-', '') }
            finally { $sha.Dispose() }
            if (!$seen.ContainsKey($hash)) {
                $seen[$hash] = $true
                [pscustomobject]@{ Hash = $hash; Pem = "-----BEGIN CERTIFICATE-----`r`n$([Convert]::ToBase64String($der, [Base64FormattingOptions]::InsertLineBreaks))`r`n-----END CERTIFICATE-----`r`n" }
            }
        } finally { $cert.Dispose() }
    }
}

function Save-CertificateText([string]$Path, [string]$Text) {
    [IO.File]::WriteAllText($Path, $Text, [Text.Encoding]::ASCII)
}

function Capture-PrinterTrust([string]$Root, [string]$State, [string]$Legacy) {
    $bundle = Join-Path $Root 'resources\cert\printer.cer'
    $baseline = Join-Path $Root 'resources\cert\printer-stock.cer'
    $certificates = @(Read-Certificates $bundle)
    $stock = @{}
    foreach ($line in [IO.File]::ReadAllLines($Legacy)) {
        if ($line -notmatch '^[0-9A-F]{64}$') { throw 'Invalid packaged stock fingerprint.' }
        $stock[$line] = $true
    }
    if (Test-Path -LiteralPath $baseline) {
        foreach ($cert in @(Read-Certificates $baseline)) { $stock[$cert.Hash] = $true }
    }
    $local = @($certificates | Where-Object { !$stock.ContainsKey($_.Hash) })
    $null = [IO.Directory]::CreateDirectory($State)
    Save-CertificateText (Join-Path $State 'printer-local.cer') (($local | ForEach-Object { $_.Pem }) -join '')
    Write-Host "Preserving $($local.Count) locally installed public printer certificate(s)."
    foreach ($cert in $local) { Write-Host "Local printer certificate SHA256: $($cert.Hash)" }
}

function Restore-PrinterTrust([string]$Root, [string]$State) {
    $directory = Join-Path $Root 'resources\cert'
    $bundle = Join-Path $directory 'printer.cer'
    $baseline = Join-Path $directory 'printer-stock.cer'
    $localFile = Join-Path $State 'printer-local.cer'
    $stock = @(Read-Certificates $baseline)
    $local = @(Read-Certificates $localFile -AllowEmpty)
    $seen = @{}
    foreach ($cert in $stock) { $seen[$cert.Hash] = $true }
    $additions = @($local | Where-Object { !$seen.ContainsKey($_.Hash) })
    if ($additions.Count -eq 0) { return } # Keep the shipped bundle byte-identical.
    $text = [IO.File]::ReadAllText($baseline, [Text.Encoding]::ASCII).TrimEnd() + "`r`n"
    $text += ($additions | ForEach-Object { $_.Pem }) -join ''
    $temporary = Join-Path $directory ('printer-migrate-' + [Guid]::NewGuid().ToString('N') + '.tmp')
    try {
        Save-CertificateText $temporary $text
        $verified = @(Read-Certificates $temporary)
        if ($verified.Count -ne $stock.Count + $additions.Count) { throw 'Merged certificate count did not match.' }
        # Atomic replacement: a failed write leaves the freshly installed stock bundle intact.
        [IO.File]::Replace($temporary, $bundle, $null)
    } finally {
        if (Test-Path -LiteralPath $temporary) { [IO.File]::Delete($temporary) }
    }
    Write-Host "Restored $($additions.Count) local public printer certificate(s)."
}

# Dot-sourcing exposes the same implementation to the isolated regression tests.
if ($MyInvocation.InvocationName -ne '.') {
    try {
        if (Get-Process -Name 'orca-slicer','OrcaSlicer' -ErrorAction SilentlyContinue) {
            throw 'Close Orca before installing the update.'
        }
        if ($Mode -eq 'Capture') { Capture-PrinterTrust $Installation $StateDirectory $LegacyFingerprints }
        elseif ($Mode -eq 'Restore') { Restore-PrinterTrust $Installation $StateDirectory }
        else { throw 'A migration mode is required.' }
    } catch {
        Write-Error $_
        exit 1
    }
}
