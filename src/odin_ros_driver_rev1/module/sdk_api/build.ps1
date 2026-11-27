# =============================================================================
# build.ps1 — Windows build entry point (counterpart of build.sh).
#
# Usage:
#   .\build.ps1                          # Release build, install to build\install
#   .\build.ps1 -BuildType Debug         # Debug build
#   .\build.ps1 -InstallPrefix "D:\odin" # Custom install prefix
#   .\build.ps1 -Clean                   # Remove build\ first
#
# What it does:
#   1. Locates Visual Studio (vswhere) and imports the x64 compiler
#      environment (vcvarsall.bat amd64) so cl.exe/nmake.exe are on PATH.
#   2. cmake configure + build + install (single-config Ninja generator).
#
# First-run note: when lib\win64\ has no prebuilt OpenSSL yet, the build
# falls back to compiling OpenSSL from 3rdparty\openssl (requires Perl, e.g.
# 'winget install StrawberryPerl.StrawberryPerl'; ~20-40 min, once). The
# resulting libs are written to lib\win64\ — commit them to git and every
# later build (yours or customers') takes the prebuilt path with zero
# extra dependencies.
# =============================================================================

[CmdletBinding()]
param(
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$BuildType = 'Release',

    [string]$InstallPrefix = '',

    [switch]$Clean
)

$ErrorActionPreference = 'Stop'

$RootDir = (Get-Variable -Name PSScriptRoot -ValueOnly)
if (-not $RootDir) { $RootDir = Split-Path -Parent $MyInvocation.MyCommand.Path }
$BuildDir = Join-Path $RootDir 'build'

if ($Clean -and (Test-Path $BuildDir)) {
    Write-Host "Removing $BuildDir ..."
    Remove-Item -Recurse -Force $BuildDir
}

if (-not $InstallPrefix) {
    $InstallPrefix = Join-Path $BuildDir 'install'
}
New-Item -ItemType Directory -Force -Path $InstallPrefix | Out-Null

# -----------------------------------------------------------------------------
# 1. Locate Visual Studio + import the x64 build environment.
#    Skipped when cl.exe is already on PATH (e.g. started from a VS developer
#    prompt); in that case the current environment is used as-is.
# -----------------------------------------------------------------------------
function Import-MsvcEnvironment {
    $cl = Get-Command cl.exe -ErrorAction SilentlyContinue
    if ($cl) {
        Write-Host "Using MSVC already on PATH: $($cl.Source)"
        return
    }

    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) {
        throw "vswhere.exe not found. Install Visual Studio 2022 or newer with the 'Desktop development with C++' workload."
    }

    $vsPath = & $vswhere -latest -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath
    if (-not $vsPath) {
        throw "Visual Studio with the C++ workload was not found. Install the 'Desktop development with C++' workload first."
    }

    $vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvarsall.bat'
    if (-not (Test-Path $vcvars)) {
        throw "vcvarsall.bat not found under $vsPath"
    }

    Write-Host "Importing MSVC environment: $vcvars amd64"
    # Run vcvarsall and re-export its environment variables into this session.
    $envOutput = & cmd /c " `"$vcvars`" amd64 && set" 2>$null
    foreach ($line in $envOutput) {
        if ($line -match '^([^=]+)=(.*)$') {
            Set-Item -Path "Env:$($Matches[1])" -Value $Matches[2]
        }
    }

    if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
        throw "Failed to import the MSVC environment from $vcvars"
    }
}

Import-MsvcEnvironment

# -----------------------------------------------------------------------------
# 2. Configure + build + install. Ninja (ships with VS and with CMake) keeps
#    the flow single-config, matching build.sh's Makefile behaviour.
# -----------------------------------------------------------------------------
Write-Host "Build type: $BuildType"
Write-Host "Install prefix: $InstallPrefix"

$configureArgs = @(
    '-S', $RootDir
    '-B', $BuildDir
    "-DCMAKE_BUILD_TYPE=$BuildType"
    "-DCMAKE_INSTALL_PREFIX=$InstallPrefix"
    '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
)
$ninja = Get-Command ninja.exe -ErrorAction SilentlyContinue
if ($ninja) {
    $configureArgs += '-G', 'Ninja'
}

& cmake @configureArgs
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

# Multi-config generators (Visual Studio, used when Ninja is unavailable) pick
# the configuration at build/install time; single-config Ninja takes it from
# CMAKE_BUILD_TYPE and ignores --config.
$buildArgs = @('--build', $BuildDir, '--parallel')
if (-not $ninja) { $buildArgs += '--config', $BuildType }
& cmake @buildArgs
if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

$installArgs = @('--install', $BuildDir, '--config', $BuildType)
& cmake @installArgs
if ($LASTEXITCODE -ne 0) { throw "cmake install failed" }

Write-Host ''
Write-Host '========================================'
Write-Host 'Build and install complete!'
Write-Host "Install directory: $InstallPrefix"
Write-Host '  - Headers: <prefix>\include\'
Write-Host '  - Library: <prefix>\lib\'
Write-Host '========================================'
