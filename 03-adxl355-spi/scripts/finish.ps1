# Write a full analyzer report next to every .h5 file in a folder, engine
# metrics for all files in metrics.csv, and SHA-256 hashes of the raw files in
# sha256.csv.
#
# Usage:
#   .\scripts\finish.ps1                                # measurements\<yyMMdd>
#   .\scripts\finish.ps1 -OutDir .\measurements\261018
#   .\scripts\finish.ps1 -Cylinders 3

param(
    [string]$OutDir = (Join-Path $PSScriptRoot "..\measurements\$(Get-Date -Format yyMMdd)"),
    [int]$Cylinders = 4
)

$ErrorActionPreference = "Stop"
$bin = Join-Path $PSScriptRoot "..\target\x86_64-pc-windows-msvc\release"
$analyzer = Join-Path $bin "analyzer.exe"
$metrics = Join-Path $bin "engine-metrics.exe"
$files = @(Get-ChildItem (Join-Path $OutDir "*.h5"))
if ($files.Count -eq 0) {
    Write-Error "No .h5 files in $OutDir"
}

foreach ($f in $files) {
    $report = $f.FullName -replace '\.h5$', '_report.txt'
    & $analyzer -i $f.FullName --all -o $report | Out-Null
    Write-Host "Report: $report"
}

& $metrics --cylinders $Cylinders -o (Join-Path $OutDir "metrics.csv") @($files | ForEach-Object { $_.FullName })

$hashes = Join-Path $OutDir "sha256.csv"
$files | Get-FileHash -Algorithm SHA256 | Select-Object Hash, Path |
    Export-Csv $hashes -NoTypeInformation -Encoding UTF8
Write-Host "Hashes: $hashes"
