# Merge: 20C/200C from result/ + 50C/100C from result_50c_100c_new/
Write-Host "=== MERGING FINAL RESULTS ===" -ForegroundColor Cyan
Write-Host "  20C/200C: From result/ (maxEvaluation)" -ForegroundColor Yellow
Write-Host "  50C/100C: From result_50c_100c_new/ (maxIteration=12/25)" -ForegroundColor Yellow
Write-Host ""

# Create final output folder
$outputFolder = "result_final_combined"
if (Test-Path $outputFolder) {
    Remove-Item "$outputFolder\*.txt" -Force
    Write-Host "Cleared old results" -ForegroundColor Gray
} else {
    New-Item -ItemType Directory -Path $outputFolder | Out-Null
}

$copied = @{20=0; 50=0; 100=0; 200=0}

# Copy 20C from result/
Write-Host "Copying 20C from result/..." -ForegroundColor White
Get-ChildItem "result/20.*.txt" | ForEach-Object {
    Copy-Item $_.FullName "$outputFolder\$($_.Name)" -Force
    $copied[20]++
}

# Copy 50C from result_50c_100c_new/
Write-Host "Copying 50C from result_50c_100c_new/..." -ForegroundColor White
Get-ChildItem "result_50c_100c_new/50.*.txt" | ForEach-Object {
    Copy-Item $_.FullName "$outputFolder\$($_.Name)" -Force
    $copied[50]++
}

# Copy 100C from result_50c_100c_new/
Write-Host "Copying 100C from result_50c_100c_new/..." -ForegroundColor White
Get-ChildItem "result_50c_100c_new/100.*.txt" | ForEach-Object {
    Copy-Item $_.FullName "$outputFolder\$($_.Name)" -Force
    $copied[100]++
}

# Copy 200C from result/
Write-Host "Copying 200C from result/..." -ForegroundColor White
Get-ChildItem "result/200.*.txt" | ForEach-Object {
    Copy-Item $_.FullName "$outputFolder\$($_.Name)" -Force
    $copied[200]++
}

$total = $copied[20] + $copied[50] + $copied[100] + $copied[200]

Write-Host ""
Write-Host "=== MERGE COMPLETE ===" -ForegroundColor Cyan
Write-Host "Total files: $total/60" -ForegroundColor $(if($total -eq 60){"Green"}else{"Red"})
Write-Host "  20C:  $($copied[20]) files (from result/)" -ForegroundColor White
Write-Host "  50C:  $($copied[50]) files (from result_50c_100c_new/)" -ForegroundColor White
Write-Host "  100C: $($copied[100]) files (from result_50c_100c_new/)" -ForegroundColor White
Write-Host "  200C: $($copied[200]) files (from result/)" -ForegroundColor White
Write-Host ""
Write-Host "Output: $outputFolder/" -ForegroundColor Yellow
