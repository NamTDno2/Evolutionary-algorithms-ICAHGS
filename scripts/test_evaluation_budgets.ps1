# Baseline Evaluation Count Test
# Purpose: Run algorithm naturally to determine appropriate evaluation budgets
# Strategy: Run with generous iteration limits to see actual evaluation counts

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "BASELINE EVALUATION COUNT TEST" -ForegroundColor Cyan
Write-Host "Goal: Determine natural evaluation counts for each problem size" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# Build first
Write-Host "Building..." -ForegroundColor Yellow
& "C:\msys64\ucrt64\bin\g++.exe" -fdiagnostics-color=always -g `
    -I"./src/header" ./src/*.cpp -o ./build/main.exe 2>&1 | Out-Null

if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Build failed!" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Build successful!`n" -ForegroundColor Green

# Test instances (one per size)
$instances = @(
    @{ File = "20.10.1"; Size = "20C"; Iters = 100; Target = "~20-30s" }
    @{ File = "50.10.1"; Size = "50C"; Iters = 150; Target = "~200s" }
    @{ File = "100.10.1"; Size = "100C"; Iters = 200; Target = "~1000s" }
    @{ File = "200.10.1"; Size = "200C"; Iters = 250; Target = "~2000s" }
)

$results = @()

foreach ($instance in $instances) {
    Write-Host "========================================" -ForegroundColor Yellow
    Write-Host "Testing: $($instance.File) ($($instance.Size))" -ForegroundColor Yellow
    Write-Host "Iterations: $($instance.Iters) (benchmark-equivalent)" -ForegroundColor White
    Write-Host "Target time: $($instance.Target)" -ForegroundColor White
    Write-Host "========================================" -ForegroundColor Yellow
    
    $start = Get-Date
    
    # Run with generous evaluation limit (10M - won't be reached)
    # This lets algorithm run naturally based on iterations
    $output = ./build/main.exe "data/$($instance.File).txt" 200 2 10000000 2>&1 | Out-String
    
    $elapsed = ((Get-Date) - $start).TotalSeconds
    
    # Extract metrics
    $evalCount = 0
    $iterations = 0
    $pareto = 0
    $ct = 0
    $wt = 0
    
    if ($output -match "Total evaluations: (\d+)") {
        $evalCount = [int]$Matches[1]
    }
    if ($output -match "Iteration (\d+)") {
        $iterations = [int]$Matches[1]
    }
    if ($output -match "Pareto front size: (\d+)") {
        $pareto = [int]$Matches[1]
    }
    if ($output -match "System Completion Time: ([\d.]+)") {
        $ct = [double]$Matches[1]
    }
    if ($output -match "Total Sample Waiting Time: ([\d.]+)") {
        $wt = [double]$Matches[1]
    }
    
    Write-Host "`n  Results:" -ForegroundColor Cyan
    Write-Host "    Time: ${elapsed}s" -ForegroundColor White
    Write-Host "    Evaluations: $evalCount" -ForegroundColor Yellow
    Write-Host "    Iterations completed: $iterations" -ForegroundColor White
    Write-Host "    Pareto size: $pareto" -ForegroundColor White
    Write-Host "    Best CT: $ct" -ForegroundColor Green
    Write-Host "    Best WT: $wt" -ForegroundColor Green
    
    $results += [PSCustomObject]@{
        Size = $instance.Size
        ConfiguredIters = $instance.Iters
        ActualIters = $iterations
        Evaluations = $evalCount
        Time = [math]::Round($elapsed, 2)
        BestCT = $ct
        BestWT = $wt
        Pareto = $pareto
    }
    
    Write-Host ""
}

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "BASELINE EVALUATION COUNTS" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Green

$results | Format-Table Size, ConfiguredIters, ActualIters, Evaluations, Time -AutoSize

Write-Host "`nProposed Evaluation Budgets:" -ForegroundColor Cyan
Write-Host "(Based on actual counts from benchmark-equivalent iterations)`n" -ForegroundColor White

foreach ($result in $results) {
    # Round to nearest 10k for cleaner numbers
    $proposed = [math]::Round($result.Evaluations / 10000) * 10000
    Write-Host "  $($result.Size): $($result.Evaluations) evaluations → Propose: $proposed" -ForegroundColor Yellow
}

Write-Host "`nQuality Summary:" -ForegroundColor Cyan
$results | Format-Table Size, BestCT, BestWT, Pareto -AutoSize

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "RECOMMENDATION" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

Write-Host "`nUse these evaluation budgets in main.cpp:" -ForegroundColor White
foreach ($result in $results) {
    $proposed = [math]::Round($result.Evaluations / 10000) * 10000
    $sizeNum = $result.Size -replace "C", ""
    Write-Host "  if (numCustomers <= $sizeNum) maxEvaluations = $proposed;" -ForegroundColor Cyan
}

Write-Host "`n========================================`n" -ForegroundColor Green
