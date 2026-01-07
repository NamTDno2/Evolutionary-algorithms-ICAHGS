# Test all 50C and 100C instances with maxIteration
# Chạy TOÀN BỘ 32 instances (16x50C + 16x100C) để thu thập số evaluations

$instances = @()

# 16 instances 50C (density: 10, 20, 30, 40 - 4 instances each)
10..40 | ForEach-Object {
    $density = $_
    1..4 | ForEach-Object {
        $instances += "50.$density.$_"
    }
}

# 16 instances 100C (density: 10, 20, 30, 40 - 4 instances each)
10..40 | ForEach-Object {
    $density = $_
    1..4 | ForEach-Object {
        $instances += "100.$density.$_"
    }
}

Write-Host "=== TEST maxIteration với 32 instances (50C/100C) ===" -ForegroundColor Cyan
Write-Host "Total instances: $($instances.Count)" -ForegroundColor Yellow
Write-Host "50C: 16 instances" -ForegroundColor White
Write-Host "100C: 16 instances" -ForegroundColor White
Write-Host ""

# Create CSV file for results
$csvPath = "test_maxIteration\evaluation_data.csv"
"Instance,Size,Density,Iterations,Evaluations,Time_Seconds" | Out-File $csvPath

$totalStart = Get-Date
$completed = 0

foreach ($instance in $instances) {
    $inputFile = "data/$instance.txt"
    
    if (!(Test-Path $inputFile)) {
        Write-Host "WARNING: Skipping $instance (file not found)" -ForegroundColor Yellow
        continue
    }
    
    $completed++
    Write-Host "[$completed/$($instances.Count)] Running: $instance" -ForegroundColor Green
    $start = Get-Date
    
    # Run and capture output
    $output = & ".\build\main.exe" $inputFile 2>&1 | Out-String
    
    $end = Get-Date
    $elapsed = ($end - $start).TotalSeconds
    
    # Parse instance name: 50.30.1 -> Size=50, Density=30
    if ($instance -match "^(\d+)\.(\d+)\.(\d+)$") {
        $size = $matches[1]
        $density = $matches[2]
        $instanceNum = $matches[3]
    }
    
    # Extract data from output
    $iterations = 0
    $evals = 0
    
    if ($output -match "Max iterations: (\d+)") {
        $iterations = [int]$matches[1]
    }
    if ($output -match "Total evaluations: (\d+)") {
        $evals = [int]$matches[1]
    }
    
    # Save to CSV
    "$instance,$size,$density,$iterations,$evals,$([math]::Round($elapsed,2))" | Out-File $csvPath -Append
    
    Write-Host "  OK $iterations iterations - $evals evaluations ($([math]::Round($elapsed,1))s)" -ForegroundColor White
    
    # Progress estimate
    $avgTimePerInstance = (($end - $totalStart).TotalSeconds) / $completed
    $remaining = $instances.Count - $completed
    $estimatedRemaining = $remaining * $avgTimePerInstance
    
    if ($completed % 5 -eq 0) {
        Write-Host "  Progress: $completed/$($instances.Count) | ETA: $([math]::Round($estimatedRemaining/60, 1)) minutes" -ForegroundColor Cyan
    }
    
    Write-Host ""
}

$totalEnd = Get-Date
$totalElapsed = ($totalEnd - $totalStart).TotalMinutes

Write-Host "=== TEST COMPLETED ===" -ForegroundColor Green
Write-Host "Total time: $([math]::Round($totalElapsed, 1)) minutes" -ForegroundColor Yellow
Write-Host ""
Write-Host "Results saved to: $csvPath" -ForegroundColor Cyan
Write-Host ""

# Analyze results
Write-Host "=== ANALYSIS ===" -ForegroundColor Cyan
$data = Import-Csv $csvPath

$data50C = $data | Where-Object { $_.Size -eq "50" }
$data100C = $data | Where-Object { $_.Size -eq "100" }

if ($data50C) {
    $avg50C = ($data50C | Measure-Object -Property Evaluations -Average).Average
    $min50C = ($data50C | Measure-Object -Property Evaluations -Minimum).Minimum
    $max50C = ($data50C | Measure-Object -Property Evaluations -Maximum).Maximum
    
    Write-Host "`n50C Results (16 instances):" -ForegroundColor Green
    Write-Host "  Average evaluations: $([math]::Round($avg50C, 0))" -ForegroundColor White
    Write-Host "  Range: $min50C - $max50C" -ForegroundColor Gray
    Write-Host "  Current maxEvaluation: 1,095,000 (gấp $([math]::Round(1095000/$avg50C, 1))x!)" -ForegroundColor Red
    Write-Host "  Recommended maxEvaluation: $([math]::Round($avg50C, 0))" -ForegroundColor Yellow
}

if ($data100C) {
    $avg100C = ($data100C | Measure-Object -Property Evaluations -Average).Average
    $min100C = ($data100C | Measure-Object -Property Evaluations -Minimum).Minimum
    $max100C = ($data100C | Measure-Object -Property Evaluations -Maximum).Maximum
    
    Write-Host "`n100C Results (16 instances):" -ForegroundColor Green
    Write-Host "  Average evaluations: $([math]::Round($avg100C, 0))" -ForegroundColor White
    Write-Host "  Range: $min100C - $max100C" -ForegroundColor Gray
    Write-Host "  Current maxEvaluation: 20,610,000 (gấp $([math]::Round(20610000/$avg100C, 1))x!)" -ForegroundColor Red
    Write-Host "  Recommended maxEvaluation: $([math]::Round($avg100C, 0))" -ForegroundColor Yellow
}

Write-Host "`nNext step: Update main.cpp with these maxEvaluation values" -ForegroundColor Cyan
