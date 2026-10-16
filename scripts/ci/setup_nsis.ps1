# Official NSIS portable distribution. No Chocolatey feed or machine-wide install.
# Hash provenance: https://github.com/NSIS-Dev/release-data/blob/main/data/versions.json
$ErrorActionPreference = 'Stop'
$version = '3.13'
$expected = 'ba63dffc4410ee89193e1cb5a41989991bd77c61068da17e3156d136b7b0b3d8'
$archive = Join-Path $env:RUNNER_TEMP "nsis-$version.zip"
$destination = Join-Path $env:RUNNER_TEMP 'repro-nsis'
# The redirect service can return an HTTP-200 landing page instead of the ZIP.
# Try explicit official mirrors, and accept only the pinned archive bytes.
$urls = @(
    "https://netix.dl.sourceforge.net/project/nsis/NSIS%203/$version/nsis-$version.zip",
    "https://phoenixnap.dl.sourceforge.net/project/nsis/NSIS%203/$version/nsis-$version.zip",
    "https://downloads.sourceforge.net/project/nsis/NSIS%203/$version/nsis-$version.zip"
)
$verified = $false
foreach ($url in $urls) {
    & curl.exe --fail --location --retry 2 --max-time 120 --output $archive $url
    if ($LASTEXITCODE -ne 0) { continue }
    $actual = (Get-FileHash $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -eq $expected) { $verified = $true; break }
    Write-Warning "Rejected NSIS response: $((Get-Item $archive).Length) bytes, SHA256 $actual from $url"
}
if (-not $verified) { throw 'No mirror supplied the pinned NSIS archive; refusing to execute it' }
Expand-Archive -Path $archive -DestinationPath $destination -Force
$nsisDir = Join-Path $destination "nsis-$version"
$makensis = Join-Path $nsisDir 'makensis.exe'
& $makensis /VERSION
if ($LASTEXITCODE -ne 0) { throw 'NSIS compiler verification failed' }
$nsisDir | Out-File -Append -FilePath $env:GITHUB_PATH -Encoding utf8
