<#
.SYNOPSIS
  Install the DAO FreeCAD addon into the FreeCAD user Mod directory and (on a
  GPU-less / headless Windows box) deploy Mesa3D software OpenGL so FreeCAD's
  native 3D view renders.

.DESCRIPTION
  Run once after FreeCAD 1.0 is installed. Idempotent: re-running is safe.

  1. Links <repo>\freecad\DAO  ->  %APPDATA%\FreeCAD\Mod\DAO  (junction).
  2. If the system only exposes the Windows GDI generic OpenGL 1.1 driver
     (typical on Windows Server / RDP / VMs with no GPU), downloads the
     pal1000 Mesa3D build and deploys the llvmpipe desktop-GL driver
     system-wide. FreeCAD needs OpenGL >= 2.0; llvmpipe reports 4.x in software.

.NOTES
  Launch FreeCAD with GALLIUM_DRIVER=llvmpipe to force the software rasteriser.
#>
param(
    [string]$MesaVersion = "26.1.3",
    [switch]$SkipMesa
)
$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$AddonSrc = Join-Path $RepoRoot "freecad\DAO"
$ModDir   = Join-Path $env:APPDATA "FreeCAD\Mod"
$AddonDst = Join-Path $ModDir "DAO"

Write-Host "[DAO] addon source: $AddonSrc"
if (-not (Test-Path $AddonSrc)) { throw "addon source not found: $AddonSrc" }

New-Item -ItemType Directory -Force -Path $ModDir | Out-Null
if (Test-Path $AddonDst) {
    Write-Host "[DAO] Mod\DAO already present, leaving as-is"
} else {
    cmd /c mklink /J "`"$AddonDst`"" "`"$AddonSrc`"" | Out-Null
    Write-Host "[DAO] linked Mod\DAO -> $AddonSrc"
}

if ($SkipMesa) { Write-Host "[DAO] -SkipMesa set, done."; return }

# --- Mesa software OpenGL (only if no real GL is available) -------------------
$mesaInstalled = Test-Path (Join-Path $env:SystemRoot "System32\mesadrv.dll")
if ($mesaInstalled) {
    Write-Host "[DAO] Mesa software OpenGL already deployed system-wide."
    return
}

$work = Join-Path $env:TEMP "dao_mesa"
New-Item -ItemType Directory -Force -Path $work | Out-Null
$archive = Join-Path $work "mesa.7z"
$sevenZr = Join-Path $work "7zr.exe"
$mesaUrl = "https://github.com/pal1000/mesa-dist-win/releases/download/$MesaVersion/mesa3d-$MesaVersion-release-msvc.7z"

Write-Host "[DAO] downloading Mesa3D $MesaVersion ..."
Invoke-WebRequest -Uri $mesaUrl -OutFile $archive
Invoke-WebRequest -Uri "https://www.7-zip.org/a/7zr.exe" -OutFile $sevenZr

Write-Host "[DAO] extracting ..."
& $sevenZr x $archive "-o$work\mesa" -y | Out-Null

# Unattended system-wide deploy, choice 1 = core desktop OpenGL drivers.
Push-Location (Join-Path $work "mesa")
try {
    cmd /c "systemwidedeploy.cmd 1"
} finally {
    Pop-Location
}
Write-Host "[DAO] Mesa software OpenGL deployed. Launch FreeCAD with GALLIUM_DRIVER=llvmpipe."
