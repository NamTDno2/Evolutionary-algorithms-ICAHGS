# Test 10 random instances across all sizes and compare with benchmark

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "TEST 10 RANDOM INSTANCES" -ForegroundColor Cyan
Write-Host "Evaluation-based stopping criterion" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# Build
Write-Host "Building..." -ForegroundColor Yellow
& "C:\msys64\ucrt64\bin\g++.exe" -fdiagnostics-color=always -g `
    -I"./src/header" ./src/*.cpp -o ./build/main.exe 2>&1 | Out-Null

if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Build failed!" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Build successful!`n" -ForegroundColor Green

# 10 random instances covering all sizes
$instances = @(
    "20.10.2",   # 20C
    "20.20.3",   # 20C
    "50.10.1",   # 50C
    "50.20.4",   # 50C
    "50.30.2",   # 50C
    "100.10.3",  # 100C
    "100.20.1",  # 100C
    "200.10.2",  # 200C
    "200.20.4",  # 200C
    "200.30.1"   # 200C
)

# Benchmark data (load from file if exists)
$benchmarkData = @{
    "20.10.2" = @{ CT = 2714.80; WT = 19922.70 }
    "20.20.3" = @{ CT = 2959.06; WT = 27032.39 }
    "50.10.1" = @{ CT = 5331.14; WT = 91732.60 }
    "50.20.4" = @{ CT = 5851.85; WT = 106820.50 }
    "50.30.2" = @{ CT = 6299.34; WT = 118774.90 }
    "100.10.3" = @{ CT = 7807.17; WT = 266038.30 }
    "100.20.1" = @{ CT = 8350.27; WT = 284584.10 }
    "200.10.2" = @{ CT = 9054.92; WT = 775319.40 }
    "200.20.4" = @{ CT = 9632.98; WT = 829750.40 }
    "200.30.1" = @{ CT = 10129.80; WT = 881026.50 }
}

$results = @()

foreach ($instance in $instances) {
    Write-Host "========================================" -ForegroundColor Yellow
    Write-Host "Testing: $instance" -ForegroundColor Yellow
    Write-Host "========================================" -ForegroundColor Yellow
    
    $start = Get-Date
    $output = ./build/main.exe "data/$instance.txt" 2>&1 | Out-String
    $elapsed = ((Get-Date) - $start).TotalSeconds
    
    $evaluations = 0
    $ct = 0
    $wt = 0
    $pareto = 0
    
    if ($output -match "Total evaluations: (\d+)") {
        $evaluations = [int]$Matches[1]
    }
    if ($output -match "System Completion Time: ([\d.]+)") {
        $ct = [double]$Matches[1]
    }
    if ($output -match "Total Sample Waiting Time: ([\d.]+)") {
        $wt = [double]$Matches[1]
    }
    if ($output -match "Pareto front size: (\d+)") {
        $pareto = [int]$Matches[1]
    }
    
    $benchCT = $benchmarkData[$instance].CT
    $benchWT = $benchmarkData[$instance].WT
    
    $ctDiff = if ($benchCT -gt 0) { (($benchCT - $ct) / $benchCT) * 100 } else { 0 }
    $wtDiff = if ($benchWT -gt 0) { (($benchWT - $wt) / $benchWT) * 100 } else { 0 }
    
    Write-Host "  Time: ${elapsed}s | Evals: $evaluations | Pareto: $pareto" -ForegroundColor White
    Write-Host "  CT: $ct (Benchmark: $benchCT)" -ForegroundColor $(if ($ctDiff -gt 0) { "Green" } else { "Yellow" })
    Write-Host "  WT: $wt (Benchmark: $benchWT)`n" -ForegroundColor $(if ($wtDiff -gt 0) { "Green" } else { "Yellow" })
    
    $results += [PSCustomObject]@{
        Instance = $instance
        Size = $instance.Split('.')[0] + "C"
        Time = [math]::Round($elapsed, 2)
        Evaluations = $evaluations
        Pareto = $pareto
        CT = [math]::Round($ct, 2)
        Bench_CT = [math]::Round($benchCT, 2)
        CT_Diff = [math]::Round($ctDiff, 2)
        WT = [math]::Round($wt, 2)
        Bench_WT = [math]::Round($benchWT, 2)
        WT_Diff = [math]::Round($wtDiff, 2)
    }
}

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "RESULTS SUMMARY" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Green

$results | Format-Table Instance, Size, Time, Evaluations, Pareto, CT, CT_Diff, WT, WT_Diff -AutoSize

# Statistics
$avgCT = ($results.CT_Diff | Measure-Object -Average).Average
$avgWT = ($results.WT_Diff | Measure-Object -Average).Average
$betterCT = ($results | Where-Object { $_.CT_Diff -gt 0 }).Count
$betterWT = ($results | Where-Object { $_.WT_Diff -gt 0 }).Count

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "COMPARISON vs BENCHMARK" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

Write-Host "`nCompletion Time (CT):" -ForegroundColor Cyan
Write-Host "  Average improvement: " -NoNewline
if ($avgCT -gt 0) {
    Write-Host "+$([math]::Round($avgCT, 2))%" -ForegroundColor Green
} else {
    Write-Host "$([math]::Round($avgCT, 2))%" -ForegroundColor Red
}
Write-Host "  Better instances: $betterCT/10 ($([math]::Round($betterCT/10*100, 0))%)" -ForegroundColor White

Write-Host "`nWaiting Time (WT):" -ForegroundColor Cyan
Write-Host "  Average improvement: " -NoNewline
if ($avgWT -gt 0) {
    Write-Host "+$([math]::Round($avgWT, 2))%" -ForegroundColor Green
} else {
    Write-Host "$([math]::Round($avgWT, 2))%" -ForegroundColor Red
}
Write-Host "  Better instances: $betterWT/10 ($([math]::Round($betterWT/10*100, 0))%)" -ForegroundColor White

Write-Host "`n========================================" -ForegroundColor Green

if ($avgCT -gt 0 -and $avgWT -gt 0 -and $betterCT -ge 7) {
    Write-Host "✅ EXCELLENT! Ready for full 60-instance run" -ForegroundColor Green
} elseif ($avgCT -gt 0 -or $avgWT -gt 0) {
    Write-Host "⚠️  MIXED RESULTS - Review before full run" -ForegroundColor Yellow
} else {
    Write-Host "❌ PERFORMANCE ISSUE - Need adjustment" -ForegroundColor Red
}

Write-Host "========================================`n" -ForegroundColor Green
