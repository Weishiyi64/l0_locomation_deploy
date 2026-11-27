# download_foxglove_sdk.ps1
#
# Download Foxglove SDK precompiled libraries to 3rdparty/foxglove-sdk/<platform>/
# for offline builds.  Run this script once on a machine with internet access,
# then commit or copy the 3rdparty/foxglove-sdk/ directory to the target machine.
#
# Usage (PowerShell):
#   .\sample\foxglove_bridge\download_foxglove_sdk.ps1              # auto-detect platform
#   .\sample\foxglove_bridge\download_foxglove_sdk.ps1 win64        # force Windows x64
#   .\sample\foxglove_bridge\download_foxglove_sdk.ps1 x86_64       # Linux x86_64
#   .\sample\foxglove_bridge\download_foxglove_sdk.ps1 aarch64      # Linux aarch64

param(
    [ValidateSet("win64", "x86_64", "aarch64")]
    [string]$Platform = ""
)

$ErrorActionPreference = "Stop"

$Version = "0.27.0"
$BaseUrl = "https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v$Version"

# Auto-detect platform
if ($Platform -eq "") {
    if ($IsWindows -or $env:OS -eq "Windows_NT") {
        $Platform = "win64"
    } else {
        $arch = (uname -m).Trim()
        if ($arch -eq "x86_64") {
            $Platform = "x86_64"
        } elseif ($arch -eq "aarch64") {
            $Platform = "aarch64"
        } else {
            Write-Error "Cannot auto-detect platform for arch=$arch. Specify manually."
            exit 1
        }
    }
}

$targets = @{
    "win64"   = "foxglove-v${Version}-cpp-x86_64-pc-windows-msvc.zip"
    "x86_64"  = "foxglove-v${Version}-cpp-x86_64-unknown-linux-gnu.zip"
    "aarch64" = "foxglove-v${Version}-cpp-aarch64-unknown-linux-gnu.zip"
}

$zipName = $targets[$Platform]
if (-not $zipName) {
    Write-Error "Unknown platform: $Platform"
    exit 1
}

$url = "$BaseUrl/$zipName"
$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
# Fallback: if PSScriptRoot is not what we expect, try relative to CWD
if (-not (Test-Path (Join-Path $repoRoot "3rdparty"))) {
    $repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
}
$destDir = Join-Path (Join-Path (Join-Path $repoRoot "3rdparty") "foxglove-sdk") $Platform

Write-Host "Foxglove SDK v$Version — $Platform" -ForegroundColor Cyan
Write-Host "  URL:  $url"
Write-Host "  Dest: $destDir"
Write-Host ""

if (Test-Path $destDir) {
    Write-Host "  Directory already exists. Overwrite? [y/N] " -NoNewline
    $answer = Read-Host
    if ($answer -notin @("y", "Y", "yes")) {
        Write-Host "  Skipped."
        exit 0
    }
    Remove-Item -Recurse -Force $destDir
}

# Download
$tempZip = Join-Path ([System.IO.Path]::GetTempPath()) $zipName
Write-Host "  Downloading..." -NoNewline
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
(New-Object Net.WebClient).DownloadFile($url, $tempZip)
Write-Host " done ($(Get-Item $tempZip | ForEach-Object { [math]::Round($_.Length/1MB, 1) }) MB)"

# Extract
Write-Host "  Extracting..." -NoNewline
New-Item -ItemType Directory -Force -Path $destDir | Out-Null
Add-Type -AssemblyName System.IO.Compression.FileSystem
[System.IO.Compression.ZipFile]::ExtractToDirectory($tempZip, $destDir)
Remove-Item -Force $tempZip
Write-Host " done"

Write-Host ""
Write-Host "  Foxglove SDK installed to: $destDir" -ForegroundColor Green
Write-Host "  CMake will now use the local copy (no network required)."
Write-Host ""
