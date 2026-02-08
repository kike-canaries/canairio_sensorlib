# Format all C/C++ files with clang-format using project .clang-format
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path "$root\.clang-format")) {
  Write-Error ".clang-format not found in $root"
  exit 1
}
$exts = @("*.cpp", "*.c", "*.h", "*.hpp", "*.cc", "*.cxx")
$count = 0
Get-ChildItem -Path $root -Recurse -Include $exts | Where-Object {
  $_.FullName -notlike "*\.pio\*" -and $_.FullName -notlike "*\.git\*"
} | ForEach-Object {
  & clang-format -i $_.FullName 2>$null
  if ($LASTEXITCODE -eq 0) {
    Write-Host "Formatted: $($_.FullName.Replace($root, '.'))"
    $script:count++
  }
}
Write-Host "Done. Formatted $count files."
