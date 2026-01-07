# Test maxIteration with 5 random 50C instances
# Test với 5 bộ 50C: 50.10.1, 50.20.1, 50.30.1, 50.40.1, 50.10.2

$testInstances = @(
    "50.10.1",
    "50.20.1", 
    "50.30.1",
    "50.40.1",
    "50.10.2"
)

Write-Host "=== TEST maxIteration với 5 bộ 50C ===" -ForegroundColor Cyan
Write-Host "Mỗi bộ chạy 15 iterations (config cũ)" -ForegroundColor Yellow
Write-Host ""

$totalStart = Get-Date

foreach ($instance in $testInstances) {
    $inputFile = "data/$instance.txt"
    
    Write-Host "Running: $instance" -ForegroundColor Green
    $start = Get-Date
    
    & ".\build\main.exe" $inputFile 2>&1 | Tee-Object -Variable output
    
    $end = Get-Date
    $elapsed = ($end - $start).TotalSeconds
    
    # Extract evaluations from output
    if ($output -match "Total evaluations: (\d+)") {
        $evals = $matches[1]
        Write-Host "  ✓ Completed in $([math]::Round($elapsed, 1))s - $evals evaluations" -ForegroundColor White
    } else {
        Write-Host "  ✓ Completed in $([math]::Round($elapsed, 1))s" -ForegroundColor White
    }
    Write-Host ""
}

$totalEnd = Get-Date
$totalElapsed = ($totalEnd - $totalStart).TotalMinutes

Write-Host "=== TEST COMPLETED ===" -ForegroundColor Green
Write-Host "Total time: $([math]::Round($totalElapsed, 1)) minutes" -ForegroundColor Yellow
Write-Host ""
Write-Host "Results saved to: test_maxIteration/" -ForegroundColor Cyan
Write-Host ""
Write-Host "To analyze evaluations:" -ForegroundColor White
Write-Host '  Get-ChildItem test_maxIteration/*.txt | ForEach-Object { $_.Name }' -ForegroundColor Gray
