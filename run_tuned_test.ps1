# Run 50C/100C with TUNED maxEvaluation values
# 50C: 21,000 evaluations (from 12 iterations average)
# 100C: 66,000 evaluations (from 25 iterations average)

$instances = @()

# 16 instances 50C
@(10, 20, 30, 40) | ForEach-Object {
    $density = $_
    1..4 | ForEach-Object {
        $instances += "50.$density.$_"
    }
}

# 16 instances 100C
@(10, 20, 30, 40) | ForEach-Object {
    $density = $_
    1..4 | ForEach-Object {
        $instances += "100.$density.$_"
    }
}

Write-Host "=== TESTING TUNED maxEvaluation ===" -ForegroundColor Cyan
Write-Host "50C: 21,000 evaluations (vs old 1,095,000 - reduced 52x!)" -ForegroundColor Yellow
Write-Host "100C: 66,000 evaluations (vs old 20,610,000 - reduced 314x!)" -ForegroundColor Yellow
Write-Host ""
Write-Host "Running 32 instances..." -ForegroundColor Green
Write-Host ""

$totalStart = Get-Date
$completed = 0

foreach ($instance in $instances) {
    $inputFile = "data/$instance.txt"
    
    if (!(Test-Path $inputFile)) {
        continue
    }
    
    $completed++
    $pct = [math]::Round($completed * 100 / 32, 0)
    Write-Host "[$completed/32 - $pct%] $instance" -ForegroundColor Green -NoNewline
    
    $start = Get-Date
    $output = & ".\build\main.exe" $inputFile 2>&1 | Out-String
    $end = Get-Date
    $elapsed = ($end - $start).TotalSeconds
    
    Write-Host " - $([math]::Round($elapsed, 1))s" -ForegroundColor White
    
    # Progress estimate every 8 instances
    if ($completed % 8 -eq 0) {
        $avgTimePerInstance = (($end - $totalStart).TotalSeconds) / $completed
        $remaining = 32 - $completed
        $estimatedRemaining = $remaining * $avgTimePerInstance
        Write-Host "  ETA: $([math]::Round($estimatedRemaining/60, 1)) minutes remaining" -ForegroundColor Cyan
        Write-Host ""
    }
}

$totalEnd = Get-Date
$totalElapsed = ($totalEnd - $totalStart).TotalMinutes

Write-Host ""
Write-Host "=== RUN COMPLETED ===" -ForegroundColor Green
Write-Host "Total time: $([math]::Round($totalElapsed, 1)) minutes" -ForegroundColor Yellow
Write-Host "Average: $([math]::Round($totalElapsed * 60 / 32, 1)) seconds per instance" -ForegroundColor White
Write-Host ""
Write-Host "Results saved to: result_tuned/" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next step: Compare with benchmark" -ForegroundColor Yellow
Write-Host '  python compare_tuned_results.py' -ForegroundColor Gray
