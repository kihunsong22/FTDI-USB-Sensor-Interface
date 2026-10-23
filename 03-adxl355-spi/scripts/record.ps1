# Record ADXL355 data with fixed vibration settings (FIFO, 4 kHz ODR, +/-8 g),
# then print a quick check of the file.
#
# Usage:
#   .\scripts\record.ps1 -Name idle-r1 -Seconds 60
#   .\scripts\record.ps1 -Name start-auto              # runs until Enter
#   .\scripts\record.ps1 -Name drive -Live             # with live display
#
# The quick check ends with engine rpm and order levels per steady segment
# (engine-metrics, 5 s minimum).
#
# Records with the collector (progress line with sensor temperature). -Live
# records with the live display (values, RMS, frequency peaks, vibration); it can
# still drop samples, so check the warning. Both store the sensor temperature.
#
# Without -Seconds, recording runs until Enter is pressed. Ctrl+C also saves the
# file but skips the quick check.
#
# Files are named <yyMMdd-HHmmss>_<Name>.h5 in OutDir (default: measurements\<yyMMdd>,
# which is gitignored).

param(
    [Parameter(Mandatory = $true)][ValidatePattern('^[A-Za-z0-9-]+$')][string]$Name,
    [int]$Seconds = 0,
    [string]$OutDir = (Join-Path $PSScriptRoot "..\measurements\$(Get-Date -Format yyMMdd)"),
    [ValidateSet(2, 4, 8)][int]$Range = 8,
    [switch]$Live,
    [int]$Cylinders = 4
)

$ErrorActionPreference = "Stop"
$bin = Join-Path $PSScriptRoot "..\target\x86_64-pc-windows-msvc\release"
$recorder = Join-Path $bin $(if ($Live) { "adxl355-spi-reader.exe" } else { "collector.exe" })
$analyzer = Join-Path $bin "analyzer.exe"
if (-not (Test-Path $recorder) -or -not (Test-Path $analyzer)) {
    Write-Error "Release binaries not found in $bin. Run: cargo build --release --features analysis"
}

New-Item -ItemType Directory -Force $OutDir | Out-Null
$OutDir = (Resolve-Path $OutDir).Path
$file = Join-Path $OutDir "$(Get-Date -Format yyMMdd-HHmmss)_$Name.h5"

$durationArgs = if ($Seconds -gt 0) { @("--duration", $Seconds) } else { @() }
if ($Live) {
    & $recorder --odr 4000 --range $Range @durationArgs --output $file
} else {
    & $recorder --mode fifo --rate 4000 --range $Range @durationArgs --output $file
}
if ($LASTEXITCODE -ne 0) {
    Write-Error "recording failed (exit code $LASTEXITCODE)"
}

Write-Host ""
Write-Host "Quick check: $file"
& $analyzer -i $file --statistics --vibration |
    Select-String -Pattern "^  Range:|Measured sample rate|WARNING|Clipped|^Axis|^X |^Y |^Z "
& (Join-Path $bin "engine-metrics.exe") --cylinders $Cylinders --steady-min 5 $file
