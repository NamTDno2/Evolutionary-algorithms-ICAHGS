# Merge final results: Copy 20C/200C from result/, keep 50C/100C from result_final_hybrid/
Write-Host "=== MERGING FINAL RESULTS ===" -ForegroundColor Cyan
Write-Host ""

# Ensure result_final_hybrid directory exists
if (-not (Test-Path result_final_hybrid)) {
    New-Item -ItemType Directory -Path result_final_hybrid | Out-Null
}

# Copy 20C instances from result/ folder
Write-Host "Copying 20C instances from result/..." -ForegroundColor Yellow
$files20C = Get-ChildItem result\20.*.txt -ErrorAction SilentlyContinue
$count20C = 0
foreach ($file in $files20C) {
    Copy-Item $file.FullName result_final_hybrid\ -Force
    $count20C++
}
Write-Host "  Copied $count20C files (20C)" -ForegroundColor Green

# Copy 200C instances from result/ folder
Write-Host "Copying 200C instances from result/..." -ForegroundColor Yellow
$files200C = Get-ChildItem result\200.*.txt -ErrorAction SilentlyContinue
$count200C = 0
foreach ($file in $files200C) {
    Copy-Item $file.FullName result_final_hybrid\ -Force
    $count200C++
}
Write-Host "  Copied $count200C files (200C)" -ForegroundColor Green

# Count existing 50C/100C files
$files50C = Get-ChildItem result_final_hybrid\50.*.txt -ErrorAction SilentlyContinue
$files100C = Get-ChildItem result_final_hybrid\100.*.txt -ErrorAction SilentlyContinue

Write-Host ""
Write-Host "=== MERGE COMPLETED ===" -ForegroundColor Green
Write-Host "Final result_final_hybrid/ folder contains:" -ForegroundColor Cyan
Write-Host "  20C: $count20C files" -ForegroundColor White
Write-Host "  50C: $($files50C.Count) files" -ForegroundColor White
Write-Host "  100C: $($files100C.Count) files" -ForegroundColor White
Write-Host "  200C: $count200C files" -ForegroundColor White

$total = $count20C + $files50C.Count + $files100C.Count + $count200C
Write-Host "  TOTAL: $total/60 files" -ForegroundColor $(if($total -eq 60){"Green"}else{"Yellow"})

if ($total -eq 60) {
    Write-Host ""
    Write-Host "All 60 instances ready for comparison!" -ForegroundColor Green
    Write-Host "Run: python compare_final_hybrid_results.py" -ForegroundColor White
} else {
    Write-Host ""
    Write-Host "WARNING: Expected 60 files, got $total" -ForegroundColor Red
    Write-Host "Please check:" -ForegroundColor Yellow
    Write-Host "  - result/ should have 20C (12 files) and 200C (16 files)" -ForegroundColor White
    Write-Host "  - result_final_hybrid/ should have 50C (16 files) and 100C (16 files)" -ForegroundColor White
}
