# TEST: 50C/100C with 50% baseline (1 instance each - QUICK TEST)
# Config: 50C=1,095,000 evals | 100C=20,610,000 evals

Write-Host "`n=== QUICK TEST 50C/100C CONFIG 50% ===" -ForegroundColor Cyan
Write-Host "Running 2 instances (1x50C + 1x100C) to verify archive quality`n" -ForegroundColor Yellow

# Build
Write-Host "Building..." -ForegroundColor White
g++ -fdiagnostics-color=always -g -I./src/header ./src/*.cpp -o ./build/main.exe 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Build OK`n" -ForegroundColor Green

# Test instances
$testInstances = @(
    "50.10.1",
    "100.10.1"
)

$results = @()
$startTime = Get-Date

foreach ($instance in $testInstances) {
    Write-Host "Testing $instance..." -ForegroundColor White -NoNewline
    $instStart = Get-Date
    
    .\build\main.exe "data/$instance.txt" 2>&1 | Out-Null
    
    $elapsed = [math]::Round(((Get-Date) - $instStart).TotalSeconds, 1)
    
    if (Test-Path "result_50c_100c_50pct/$instance.txt") {
        $content = Get-Content "result_50c_100c_50pct/$instance.txt" -Raw
        if ($content -match "Last Update:(\d+)") {
            $archiveSize = $matches[1]
            Write-Host " ✅ $archiveSize solutions ($elapsed`s)" -ForegroundColor Green
            
            $results += [PSCustomObject]@{
                Instance = $instance
                ArchiveSize = $archiveSize
                Time = $elapsed
            }
        }
    } else {
        Write-Host " ❌ Failed" -ForegroundColor Red
    }
}

$totalTime = [math]::Round(((Get-Date) - $startTime).TotalMinutes, 1)

Write-Host "`n=== RESULTS ===" -ForegroundColor Cyan
$results | Format-Table -AutoSize

Write-Host "Comparison with result_50c_100c_doubled CSV:" -ForegroundColor Yellow
if (Test-Path result_50c_100c_doubled/results_summary.csv) {
    $csv = Import-Csv result_50c_100c_doubled/results_summary.csv
    foreach ($r in $results) {
        $csvRow = $csv | Where-Object { $_.Instance -eq $r.Instance }
        if ($csvRow) {
            Write-Host "  $($r.Instance): New=$($r.ArchiveSize) vs CSV=$($csvRow.ArchiveSize)" -ForegroundColor White
        }
    }
}

$avg = ($results | Measure-Object -Property ArchiveSize -Average).Average
Write-Host "`nAverage archive size: $([math]::Round($avg,0)) solutions" -ForegroundColor $(if($avg -gt 100){"Green"}elseif($avg -gt 10){"Yellow"}else{"Red"})
Write-Host "Total time: $totalTime minutes" -ForegroundColor Cyan

Write-Host "`nDecision:" -ForegroundColor Yellow
if ($avg -gt 100) {
    Write-Host "  ✅ GOOD! Archive size large enough, run full 32 instances" -ForegroundColor Green
} elseif ($avg -gt 10) {
    Write-Host "  ⚠️  MODERATE. Consider running full or investigate convergence" -ForegroundColor Yellow
} else {
    Write-Host "  ❌ BAD! Archive too small, need to fix convergence issue" -ForegroundColor Red
}
