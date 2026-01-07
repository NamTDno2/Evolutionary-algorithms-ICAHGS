# Test high iteration counts for 50C/100C
Write-Host "=== TESTING HIGH ITERATION COUNTS ===" -ForegroundColor Cyan
Write-Host "Configuration:" -ForegroundColor Yellow
Write-Host "  50C: maxIteration = 50 (vs baseline 12)" -ForegroundColor White
Write-Host "  100C: maxIteration = 100 (vs baseline 25)" -ForegroundColor White
Write-Host ""
Write-Host "Hypothesis: iterations = numCustomers may be optimal" -ForegroundColor Cyan
Write-Host ""

# Clear old results
if (Test-Path result_maxiter_high) {
    Remove-Item result_maxiter_high\*.txt -Force
    Write-Host "Cleared old results" -ForegroundColor Green
}

# Test instances
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
Write-Host "  50C (16 instances): ~7 minutes (50 iterations each)" -ForegroundColor White
Write-Host "  100C (16 instances): ~14 minutes (100 iterations each)" -ForegroundColor White
Write-Host "  TOTAL: ~21 minutes" -ForegroundColor Yellow
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
Write-Host "=== COMPLETED ===" -ForegroundColor Green
Write-Host "Total time: $([math]::Round($duration, 2)) minutes" -ForegroundColor White
Write-Host "Results saved to: result_maxiter_high/" -ForegroundColor Cyan
Write-Host ""
Write-Host "Run comparison:" -ForegroundColor Yellow
Write-Host "  python compare_maxiter_high_results.py" -ForegroundColor White
