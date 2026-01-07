# Merge best results: 20C/200C from result/ + 50C/100C from test_maxIteration/
Write-Host "=== MERGING BEST HYBRID RESULTS ===" -ForegroundColor Cyan
Write-Host "  20C/200C: From result/ folder (maxEvaluation config)" -ForegroundColor Yellow
Write-Host "  50C/100C: From test_maxIteration/ (maxIteration=12/25)" -ForegroundColor Yellow
Write-Host ""

# Create output folder
$outputFolder = "result_best_hybrid"
if (Test-Path $outputFolder) {
    Remove-Item "$outputFolder\*.txt" -Force
    Write-Host "Cleared old results in $outputFolder" -ForegroundColor Gray
} else {
    New-Item -ItemType Directory -Path $outputFolder | Out-Null
}

$copied20 = 0
$copied50 = 0
$copied100 = 0
$copied200 = 0

# Copy 20C from result/
Write-Host "Copying 20C instances from result/..." -ForegroundColor White
Get-ChildItem "result/20.*.txt" | ForEach-Object {
    Copy-Item $_.FullName "$outputFolder\$($_.Name)"
    $copied20++
}
Write-Host "  ✓ Copied $copied20 files (20C)" -ForegroundColor Green

# Copy 50C from test_maxIteration/
Write-Host "Copying 50C instances from test_maxIteration/..." -ForegroundColor White
Get-ChildItem "test_maxIteration/50.*.txt" | ForEach-Object {
    Copy-Item $_.FullName "$outputFolder\$($_.Name)"
    $copied50++
}
Write-Host "  ✓ Copied $copied50 files (50C)" -ForegroundColor Green

# Copy 100C from test_maxIteration/
Write-Host "Copying 100C instances from test_maxIteration/..." -ForegroundColor White
Get-ChildItem "test_maxIteration/100.*.txt" | ForEach-Object {
    Copy-Item $_.FullName "$outputFolder\$($_.Name)"
    $copied100++
}
Write-Host "  ✓ Copied $copied100 files (100C)" -ForegroundColor Green

# Copy 200C from result/
Write-Host "Copying 200C instances from result/..." -ForegroundColor White
Get-ChildItem "result/200.*.txt" | ForEach-Object {
    Copy-Item $_.FullName "$outputFolder\$($_.Name)"
    $copied200++
}
Write-Host "  ✓ Copied $copied200 files (200C)" -ForegroundColor Green

$total = $copied20 + $copied50 + $copied100 + $copied200
Write-Host ""
Write-Host "=== MERGE COMPLETE ===" -ForegroundColor Cyan
Write-Host "Total files: $total" -ForegroundColor Green
Write-Host "  20C:  $copied20 files (from result/)" -ForegroundColor White
Write-Host "  50C:  $copied50 files (from test_maxIteration/)" -ForegroundColor White
Write-Host "  100C: $copied100 files (from test_maxIteration/)" -ForegroundColor White
Write-Host "  200C: $copied200 files (from result/)" -ForegroundColor White
Write-Host ""
Write-Host "Output folder: $outputFolder/" -ForegroundColor Yellow
