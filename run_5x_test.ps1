# Test 5x buffer for 50C/100C
Write-Host "=== TESTING 5x BUFFER (105K/330K evaluations) ===" -ForegroundColor Cyan
Write-Host "Hypothesis: Fewer iterations may preserve diversity better" -ForegroundColor Yellow
Write-Host ""

# Clear old results
if (Test-Path result_5x) {
    Remove-Item result_5x\*.txt -Force
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
Write-Host "  50C (16 instances): ~5 minutes" -ForegroundColor White
Write-Host "  100C (16 instances): ~8 minutes" -ForegroundColor White
Write-Host "  TOTAL: ~13 minutes" -ForegroundColor Yellow
Write-Host ""

$startTime = Get-Date

foreach ($instance in $instances) {
    $completed++
    $filePath = "data\$instance.txt"
    
    Write-Host "[$completed/$total] Running $instance..." -ForegroundColor Cyan -NoNewline
    
    $output = & ".\build\main.exe" $filePath 2>&1 | Out-String
    
    if ($output -match "Total evaluations:\s*(\d+)") {
        $evals = $matches[1]
        Write-Host " Done ($evals evals)" -ForegroundColor Green
    } else {
        Write-Host " Done" -ForegroundColor Green
    }
}

$endTime = Get-Date
$duration = ($endTime - $startTime).TotalMinutes

Write-Host ""
Write-Host "=== COMPLETED ===" -ForegroundColor Green
Write-Host "Total time: $([math]::Round($duration, 2)) minutes" -ForegroundColor White
Write-Host "Results saved to: result_5x/" -ForegroundColor Cyan
Write-Host ""
Write-Host "Run comparison:" -ForegroundColor Yellow
Write-Host "  python compare_5x_results.py" -ForegroundColor White
