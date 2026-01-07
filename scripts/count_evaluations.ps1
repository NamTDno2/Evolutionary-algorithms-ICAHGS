# Count evaluation calls for each problem size
# This helps establish fair stopping criterion based on function evaluations

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "EVALUATION COUNT TEST" -ForegroundColor Cyan
Write-Host "Purpose: Count fitness evaluations per problem size" -ForegroundColor Cyan
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

$instances = @(
    @{ Name = "20.10.1"; Customers = 20; Iterations = 100 }
    @{ Name = "50.10.1"; Customers = 50; Iterations = 150 }
    @{ Name = "100.10.1"; Customers = 100; Iterations = 200 }
    @{ Name = "200.10.1"; Customers = 200; Iterations = 250 }
)

$results = @()

foreach ($instance in $instances) {
    Write-Host "========================================" -ForegroundColor Yellow
    Write-Host "Testing: $($instance.Name) ($($instance.Customers)C, $($instance.Iterations) iterations)" -ForegroundColor Yellow
    Write-Host "========================================" -ForegroundColor Yellow
    
    $start = Get-Date
    $output = ./build/main.exe "data/$($instance.Name).txt" 2>&1 | Out-String
    $elapsed = ((Get-Date) - $start).TotalSeconds
    
    # Extract evaluation count from output (need to add print in code)
    $evalCount = 0
    $pareto = 0
    $ct = 0
    
    if ($output -match "Total evaluations: (\d+)") {
        $evalCount = [int]$Matches[1]
    }
    if ($output -match "Pareto front size: (\d+)") {
        $pareto = [int]$Matches[1]
    }
    if ($output -match "System Completion Time: ([\d.]+)") {
        $ct = [double]$Matches[1]
    }
    
    Write-Host "  Time: ${elapsed}s" -ForegroundColor White
    Write-Host "  Evaluations: $evalCount" -ForegroundColor Cyan
    Write-Host "  Pareto size: $pareto" -ForegroundColor White
    Write-Host "  Best CT: $ct" -ForegroundColor White
    
    $results += [PSCustomObject]@{
        Instance = $instance.Name
        Customers = $instance.Customers
        Iterations = $instance.Iterations
        Evaluations = $evalCount
        Time = [math]::Round($elapsed, 2)
        Pareto = $pareto
        BestCT = $ct
        EvalPerIter = if ($instance.Iterations -gt 0) { [math]::Round($evalCount / $instance.Iterations, 1) } else { 0 }
    }
    
    Write-Host ""
}

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "EVALUATION COUNT SUMMARY" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Green

$results | Format-Table Instance, Customers, Iterations, Evaluations, EvalPerIter, Time -AutoSize

Write-Host "`nAnalysis:" -ForegroundColor Cyan
foreach ($result in $results) {
    Write-Host "  $($result.Customers)C: $($result.Evaluations) evaluations ($($result.EvalPerIter) per iteration)" -ForegroundColor White
}

Write-Host "`nProposed Evaluation Limits:" -ForegroundColor Cyan
Write-Host "  Based on these results, we can set fair stopping criterion" -ForegroundColor White
Write-Host "  using evaluation count instead of iterations or time." -ForegroundColor White

$eval20 = ($results | Where-Object { $_.Customers -eq 20 }).Evaluations
$eval50 = ($results | Where-Object { $_.Customers -eq 50 }).Evaluations
$eval100 = ($results | Where-Object { $_.Customers -eq 100 }).Evaluations
$eval200 = ($results | Where-Object { $_.Customers -eq 200 }).Evaluations

Write-Host "`n  20C:  $eval20 evaluations" -ForegroundColor Yellow
Write-Host "  50C:  $eval50 evaluations" -ForegroundColor Yellow
Write-Host "  100C: $eval100 evaluations" -ForegroundColor Yellow
Write-Host "  200C: $eval200 evaluations" -ForegroundColor Yellow

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "TEST COMPLETE!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
