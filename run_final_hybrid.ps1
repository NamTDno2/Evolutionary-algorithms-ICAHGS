# Run final hybrid configuration: maxIteration for 50C/100C
Write-Host "=== FINAL HYBRID CONFIGURATION ===" -ForegroundColor Cyan
Write-Host "Running 50C/100C with optimal maxIteration settings" -ForegroundColor Yellow
Write-Host ""
Write-Host "Configuration:" -ForegroundColor Cyan
Write-Host "  50C: maxIteration = 12 (43.8% win rate)" -ForegroundColor Green
Write-Host "  100C: maxIteration = 25 (12.5% win rate)" -ForegroundColor Green
Write-Host ""

# Clear old results for 50C/100C only
if (Test-Path result_final_hybrid) {
    Get-ChildItem result_final_hybrid\50.*.txt, result_final_hybrid\100.*.txt -ErrorAction SilentlyContinue | Remove-Item -Force
    Write-Host "Cleared old 50C/100C results" -ForegroundColor Green
}

# Test instances (50C and 100C only)
$instances = @(
    # 50C instances (16 total)
    "50.10.1", "50.10.2", "50.10.3", "50.10.4",
    "50.20.1", "50.20.2", "50.20.3", "50.20.4",
    "50.30.1", "50.30.2", "50.30.3", "50.30.4",
    "50.40.1", "50.40.2", "50.40.3", "50.40.4",
    # 100C instances (16 total)
    "100.10.1", "100.10.2", "100.10.3", "100.10.4",
    "100.20.1", "100.20.2", "100.20.3", "100.20.4",
    "100.30.1", "100.30.2", "100.30.3", "100.30.4",
    "100.40.1", "100.40.2", "100.40.3", "100.40.4"
)

$total = $instances.Count
$completed = 0

Write-Host "Running $total instances..." -ForegroundColor White
Write-Host "Estimated time:" -ForegroundColor Cyan
Write-Host "  50C (16 instances): ~2 minutes (12 iterations each)" -ForegroundColor White
Write-Host "  100C (16 instances): ~3 minutes (25 iterations each)" -ForegroundColor White
Write-Host "  TOTAL: ~5 minutes" -ForegroundColor Yellow
Write-Host ""

$startTime = Get-Date

foreach ($instance in $instances) {
    $completed++
    $filePath = "data\$instance.txt"
    $size = $instance.Split('.')[0]
    
    Write-Host "[$completed/$total] Running $instance..." -ForegroundColor Cyan -NoNewline
    
    $iterStart = Get-Date
    $output = & ".\build\main.exe" $filePath 2>&1 | Out-String
    $iterEnd = Get-Date
    $iterTime = ($iterEnd - $iterStart).TotalSeconds
    
    if ($output -match "Total evaluations:\s*(\d+)") {
        $evals = $matches[1]
        Write-Host " Done in $([math]::Round($iterTime, 1))s ($evals evals)" -ForegroundColor Green
    } else {
        Write-Host " Done in $([math]::Round($iterTime, 1))s" -ForegroundColor Green
    }
}

$endTime = Get-Date
$duration = ($endTime - $startTime).TotalMinutes

Write-Host ""
Write-Host "=== STEP 1 COMPLETED ===" -ForegroundColor Green
Write-Host "50C/100C completed in: $([math]::Round($duration, 2)) minutes" -ForegroundColor White
Write-Host ""
Write-Host "Next step: Copy 20C/200C results from result/ folder" -ForegroundColor Yellow
Write-Host "Run: .\merge_final_results.ps1" -ForegroundColor White
