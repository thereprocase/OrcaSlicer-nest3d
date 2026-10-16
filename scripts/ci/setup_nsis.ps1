# Official NSIS portable distribution. No Chocolatey feed or machine-wide install.
# Hash provenance: https://github.com/NSIS-Dev/release-data/blob/main/data/versions.json
$ErrorActionPreference = 'Stop'
$version = '3.13'
$expected = 'ba63dffc4410ee89193e1cb5a41989991bd77c61068da17e3156d136b7b0b3d8'
$archive = Join-Path $env:RUNNER_TEMP "nsis-$version.zip"
$destination = Join-Path $env:RUNNER_TEMP 'repro-nsis'
$url = "https://downloads.sourceforge.net/project/nsis/NSIS%203/$version/nsis-$version.zip"
for ($attempt = 1; $attempt -le 3; $attempt++) {
    try {
        Invoke-WebRequest -Uri $url -OutFile $archive -TimeoutSec 120
        break
    } catch {
        if ($attempt -eq 3) { throw }
        Start-Sleep -Seconds (10 * $attempt)
    }
}
if ((Get-FileHash $archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
    throw 'NSIS archive SHA256 mismatch'
}
Expand-Archive -Path $archive -DestinationPath $destination -Force
$nsisDir = Join-Path $destination "nsis-$version"
$makensis = Join-Path $nsisDir 'makensis.exe'
& $makensis /VERSION
if ($LASTEXITCODE -ne 0) { throw 'NSIS compiler verification failed' }
$nsisDir | Out-File -Append -FilePath $env:GITHUB_PATH -Encoding utf8
