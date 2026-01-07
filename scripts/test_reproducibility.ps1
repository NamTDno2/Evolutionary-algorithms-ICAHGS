# Test Reproducibility and Performance Statistics
# Runs each instance multiple times with fixed seed to measure:
# 1. Reproducibility (all runs should give same result)
# 2. Average performance metrics

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "REPRODUCIBILITY & PERFORMANCE TEST" -ForegroundColor Cyan
Write-Host "Fixed Seed = 42, Multiple Runs per Instance" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# Build first
Write-Host "Building with fixed seed..." -ForegroundColor Yellow
& "C:\msys64\ucrt64\bin\g++.exe" -fdiagnostics-color=always -g `
    -I"./src/header" ./src/*.cpp -o ./build/main.exe 2>&1 | Out-Null

if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Build failed!" -ForegroundColor Red
    exit 1
}

Write-Host "✅ Build successful!`n" -ForegroundColor Green

$testInstances = @("20.10.1", "50.10.1", "100.10.1", "200.10.1")
$numRuns = 5
$allResults = @()

foreach ($instance in $testInstances) {
    Write-Host "========================================" -ForegroundColor Yellow
    Write-Host "Testing: $instance ($numRuns runs)" -ForegroundColor Yellow
    Write-Host "========================================" -ForegroundColor Yellow
    
    $runResults = @()
    $times = @()
    $paretoSizes = @()
    $cts = @()
    $wts = @()
    
    for ($run = 1; $run -le $numRuns; $run++) {
        Write-Host "  Run $run/$numRuns..." -NoNewline
        
        $start = Get-Date
        $output = ./build/main.exe "data/$instance.txt" 2>&1 | Out-String
        $elapsed = ((Get-Date) - $start).TotalSeconds
        
        # Extract metrics
        $pareto = 0
        $unique = 0
        $ct = 0
        $wt = 0
        
        if ($output -match "Pareto front size: (\d+)") {
            $pareto = [int]$Matches[1]
        }
        if ($output -match "Unique solutions: (\d+)") {
            $unique = [int]$Matches[1]
        }
        if ($output -match "System Completion Time: ([\d.]+)") {
            $ct = [double]$Matches[1]
        }
        if ($output -match "Total Sample Waiting Time: ([\d.]+)") {
            $wt = [double]$Matches[1]
        }
        
        $times += $elapsed
        $paretoSizes += $pareto
        $cts += $ct
        $wts += $wt
        
        Write-Host " ${elapsed}s | CT=$ct | Pareto=$pareto" -ForegroundColor White
        
        $runResults += [PSCustomObject]@{
            Instance = $instance
            Run = $run
            Time = [math]::Round($elapsed, 2)
            Pareto = $pareto
            Unique = $unique
            CT = $ct
            WT = $wt
        }
    }
    
    # Calculate statistics
    $avgTime = ($times | Measure-Object -Average).Average
    $minTime = ($times | Measure-Object -Minimum).Minimum
    $maxTime = ($times | Measure-Object -Maximum).Maximum
    $stdDevTime = [math]::Sqrt((($times | ForEach-Object { [math]::Pow($_ - $avgTime, 2) }) | Measure-Object -Sum).Sum / $times.Count)
    
    $avgPareto = ($paretoSizes | Measure-Object -Average).Average
    $avgCT = ($cts | Measure-Object -Average).Average
    $avgWT = ($wts | Measure-Object -Average).Average
    
    # Check reproducibility (with fixed seed, all values should be same)
    $ctUnique = ($cts | Select-Object -Unique).Count
    $paretoUnique = ($paretoSizes | Select-Object -Unique).Count
    
    Write-Host "`n  Statistics:" -ForegroundColor Cyan
    Write-Host "    Time:   Avg=${avgTime:N2}s  Min=${minTime:N2}s  Max=${maxTime:N2}s  StdDev=${stdDevTime:N2}s" -ForegroundColor White
    Write-Host "    Pareto: Avg=${avgPareto:N1}  (Unique values: $paretoUnique)" -ForegroundColor White
    Write-Host "    CT:     Avg=${avgCT:N2}  (Unique values: $ctUnique)" -ForegroundColor White
    Write-Host "    WT:     Avg=${avgWT:N2}" -ForegroundColor White
    
    if ($ctUnique -eq 1 -and $paretoUnique -eq 1) {
        Write-Host "    ✅ REPRODUCIBLE: All runs produced identical results" -ForegroundColor Green
    } else {
        Write-Host "    ⚠️ VARIANCE: Results differ across runs" -ForegroundColor Yellow
    }
    
    $allResults += [PSCustomObject]@{
        Instance = $instance
        Customers = $instance.Split('.')[0]
        AvgTime = [math]::Round($avgTime, 2)
        StdDev = [math]::Round($stdDevTime, 2)
        MinTime = [math]::Round($minTime, 2)
        MaxTime = [math]::Round($maxTime, 2)
        AvgPareto = [math]::Round($avgPareto, 1)
        AvgCT = [math]::Round($avgCT, 2)
        AvgWT = [math]::Round($avgWT, 2)
        Reproducible = if ($ctUnique -eq 1 -and $paretoUnique -eq 1) { "Yes" } else { "No" }
    }
    
    Write-Host ""
}

# Summary table
Write-Host "`n========================================" -ForegroundColor Green
Write-Host "SUMMARY STATISTICS ($numRuns runs each)" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Green

$allResults | Format-Table -AutoSize

# Reproducibility check
$reproducibleCount = ($allResults | Where-Object { $_.Reproducible -eq "Yes" }).Count
$reproducibilityRate = ($reproducibleCount / $allResults.Count) * 100

Write-Host "`nReproducibility:" -ForegroundColor Cyan
Write-Host "  $reproducibleCount/$($allResults.Count) instances produced identical results ($reproducibilityRate%)" -ForegroundColor White

if ($reproducibilityRate -eq 100) {
    Write-Host "  ✅ Perfect reproducibility with fixed seed!" -ForegroundColor Green
} else {
    Write-Host "  ⚠️ Some variance detected (check if decoder/LS uses external randomness)" -ForegroundColor Yellow
}

Write-Host "`nPerformance Summary:" -ForegroundColor Cyan
foreach ($result in $allResults) {
    $variability = if ($result.StdDev -eq 0) { "0%" } else { "$([math]::Round(($result.StdDev / $result.AvgTime) * 100, 1))%" }
    Write-Host "  $($result.Customers)C: $($result.AvgTime)s ±$($result.StdDev)s (CV: $variability)" -ForegroundColor White
}

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "TEST COMPLETE!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
