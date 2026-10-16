# Build and test one CMake preset with MSVC on Windows.
# Used by the GitLab Windows job; also runs locally from plain PowerShell.
#
# Usage: powershell -File tools/ci/windows-build.ps1 -Preset debug

param(
    [Parameter(Mandatory = $true)][string]$Preset
)

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')

# Native programs (cmake, ctest) don't throw on failure in PowerShell 5.1,
# so every call goes through this check.
function Invoke-Checked([string]$exe, [string[]]$arguments) {
    & $exe @arguments
    if ($LASTEXITCODE -ne 0) { throw "$exe $($arguments -join ' ') failed with exit code $LASTEXITCODE" }
}

# --- Load the MSVC x64 environment into this PowerShell session -------------
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { throw 'No Visual Studio installation with the C++ x64 tools was found.' }
$vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'
# Run vcvars in cmd, print the resulting environment, and copy it over.
cmd /c "`"$vcvars`" >nul && set" | ForEach-Object {
    if ($_ -match '^([^=]+)=(.*)$') { Set-Item -Path "env:$($Matches[1])" -Value $Matches[2] }
}
$env:CC = 'cl'
$env:CXX = 'cl'

# --- Ninja, pinned (GitLab's Windows image does not list it) ---------------
if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
    $ninjaDir = Join-Path $repoRoot 'build\_tools\ninja-1.13.2'
    $ninjaExe = Join-Path $ninjaDir 'ninja.exe'
    if (-not (Test-Path $ninjaExe)) {
        New-Item -ItemType Directory -Force $ninjaDir | Out-Null
        $zip = Join-Path $ninjaDir 'ninja-win.zip'
        Invoke-WebRequest -UseBasicParsing -OutFile $zip `
            -Uri 'https://github.com/ninja-build/ninja/releases/download/v1.13.2/ninja-win.zip'
        $hash = (Get-FileHash -Algorithm SHA256 $zip).Hash
        if ($hash -ne '07FC8261B42B20E71D1720B39068C2E14FFCEE6396B76FB7A795FB460B78DC65') {
            throw "Unexpected ninja-win.zip hash: $hash"
        }
        Expand-Archive -Path $zip -DestinationPath $ninjaDir -Force
    }
    $env:PATH = "$ninjaDir;$env:PATH"
}

Set-Location $repoRoot
Invoke-Checked cmake @('--version')
Invoke-Checked cmake @('--preset', $Preset)
Invoke-Checked cmake @('--build', '--preset', $Preset)
Invoke-Checked ctest @('--preset', $Preset)
