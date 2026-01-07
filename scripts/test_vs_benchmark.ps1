# Quick test and compare with benchmark
# Run 4 instances and compare CT/WT immediately

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "ICAHGS vs BENCHMARK COMPARISON" -ForegroundColor Cyan
Write-Host "Fixed Seed = 42" -ForegroundColor Cyan
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

$instances = @("20.10.1", "50.10.1", "100.10.1", "200.10.1")

# Benchmark data (from paper)
$benchmarkData = @{
    "20.10.1" = @{ CT = 2671.24; WT = 20379.69 }
    "50.10.1" = @{ CT = 5331.14; WT = 91732.60 }
    "100.10.1" = @{ CT = 7674.70; WT = 259900.32 }
    "200.10.1" = @{ CT = 8912.31; WT = 752473.72 }
}

$results = @()

foreach ($instance in $instances) {
    Write-Host "Testing: $instance..." -NoNewline -ForegroundColor Yellow
    
    $output = ./build/main.exe "data/$instance.txt" 2>&1 | Out-String
    
    $ct = 0
    $wt = 0
    
    if ($output -match "System Completion Time: ([\d.]+)") {
        $ct = [double]$Matches[1]
    }
    if ($output -match "Total Sample Waiting Time: ([\d.]+)") {
        $wt = [double]$Matches[1]
    }
    
    $benchCT = $benchmarkData[$instance].CT
    $benchWT = $benchmarkData[$instance].WT
    
    $ctDiff = (($benchCT - $ct) / $benchCT) * 100
    $wtDiff = (($benchWT - $wt) / $benchWT) * 100
    
    Write-Host " Done" -ForegroundColor Green
    
    $results += [PSCustomObject]@{
        Instance = $instance
        Size = $instance.Split('.')[0] + "C"
        ICAHGS_CT = [math]::Round($ct, 2)
        Bench_CT = [math]::Round($benchCT, 2)
        CT_Diff = [math]::Round($ctDiff, 2)
        ICAHGS_WT = [math]::Round($wt, 2)
        Bench_WT = [math]::Round($benchWT, 2)
        WT_Diff = [math]::Round($wtDiff, 2)
    }
}

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "COMPARISON RESULTS" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Green

Write-Host "COMPLETION TIME (CT):" -ForegroundColor Cyan
$results | Format-Table Instance, Size, ICAHGS_CT, Bench_CT, `
    @{Label="Improvement"; Expression={
        if ($_.CT_Diff -gt 0) { "+$($_.CT_Diff)%" } else { "$($_.CT_Diff)%" }
    }} -AutoSize

Write-Host "`nWAITING TIME (WT):" -ForegroundColor Cyan
$results | Format-Table Instance, Size, ICAHGS_WT, Bench_WT, `
    @{Label="Improvement"; Expression={
        if ($_.WT_Diff -gt 0) { "+$($_.WT_Diff)%" } else { "$($_.WT_Diff)%" }
    }} -AutoSize

# Summary
$avgCT = ($results | ForEach-Object { $_.CT_Diff } | Measure-Object -Average).Average
$avgWT = ($results | ForEach-Object { $_.WT_Diff } | Measure-Object -Average).Average

$betterCT = ($results | Where-Object { $_.CT_Diff -gt 0 }).Count
$betterWT = ($results | Where-Object { $_.WT_Diff -gt 0 }).Count

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "SUMMARY" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

Write-Host "`nCompletion Time (CT):" -ForegroundColor Cyan
Write-Host "  Average improvement: " -NoNewline
if ($avgCT -gt 0) {
    Write-Host "+$([math]::Round($avgCT, 2))%" -ForegroundColor Green
} else {
    Write-Host "$([math]::Round($avgCT, 2))%" -ForegroundColor Red
}
Write-Host "  Better instances: $betterCT/4 ($([math]::Round($betterCT/4*100, 0))%)" -ForegroundColor White

Write-Host "`nWaiting Time (WT):" -ForegroundColor Cyan
Write-Host "  Average improvement: " -NoNewline
if ($avgWT -gt 0) {
    Write-Host "+$([math]::Round($avgWT, 2))%" -ForegroundColor Green
} else {
    Write-Host "$([math]::Round($avgWT, 2))%" -ForegroundColor Red
}
Write-Host "  Better instances: $betterWT/4 ($([math]::Round($betterWT/4*100, 0))%)" -ForegroundColor White

Write-Host "`n========================================" -ForegroundColor Green

# Overall verdict
if ($avgCT -gt 0 -and $avgWT -gt 0) {
    Write-Host "✅ ICAHGS WINS on both objectives!" -ForegroundColor Green
} elseif ($avgCT -gt 0 -or $avgWT -gt 0) {
    Write-Host "⚖️  MIXED RESULTS" -ForegroundColor Yellow
} else {
    Write-Host "❌ Benchmark performs better" -ForegroundColor Red
}

Write-Host "========================================`n" -ForegroundColor Green
