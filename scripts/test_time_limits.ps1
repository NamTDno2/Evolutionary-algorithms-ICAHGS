# Test time-based stopping on representative instances
# Measures evaluation counts for each problem size

Write-Host "=== Time-Based Stopping Test ===" -ForegroundColor Cyan
Write-Host "Testing 3 instances (20C already done: 259,497 evals)" -ForegroundColor Yellow
Write-Host ""

$instances = @(
    @{file="data/50.10.1.txt"; time=200; name="50C"},
    @{file="data/100.10.1.txt"; time=1000; name="100C"},
    @{file="data/200.10.1.txt"; time=2000; name="200C"}
)

$results = @()

# Add 20C result
$results += [PSCustomObject]@{
    Size = "20C"
    TimeLimit = "20s"
    ActualTime = "20.0s"
    Iterations = "520"
    Evaluations = 259497
    EvalPerIter = 499
    ArchiveSize = "3"
}

foreach ($inst in $instances) {
    Write-Host "[$($inst.name)] Starting: $($inst.file)" -ForegroundColor Green
    Write-Host "  Time limit: $($inst.time)s (~$([math]::Round($inst.time/60, 1)) minutes)" -ForegroundColor Gray
    
    $startTime = Get-Date
    $output = & ".\build\main.exe" $inst.file 2>&1 | Out-String
    $endTime = Get-Date
    $elapsed = ($endTime - $startTime).TotalSeconds
    
    Write-Host "  Completed in $([math]::Round($elapsed, 1))s" -ForegroundColor Cyan
    
    # Extract metrics
    $evalCount = if ($output -match "Total evaluations:\s+(\d+)") { [int]$matches[1] } else { "N/A" }
    $iterations = if ($output -match "Total iterations:\s+(\d+)") { [int]$matches[1] } else { "N/A" }
    $archiveSize = if ($output -match "Archive size:\s+(\d+)") { $matches[1] } else { "N/A" }
    $actualTime = if ($output -match "Time limit reached:\s+([\d.]+)s") { "$($matches[1])s" } else { "N/A" }
    
    $results += [PSCustomObject]@{
        Size = $inst.name
        TimeLimit = "$($inst.time)s"
        ActualTime = $actualTime
        Iterations = $iterations
        Evaluations = $evalCount
        EvalPerIter = if ($iterations -ne "N/A" -and $evalCount -ne "N/A") { [math]::Round($evalCount / $iterations, 0) } else { "N/A" }
        ArchiveSize = $archiveSize
    }
    
    Write-Host "  Results: $evalCount evals, $iterations iters, archive=$archiveSize" -ForegroundColor White
    Write-Host ""
}

Write-Host "=== SUMMARY ===" -ForegroundColor Yellow
$results | Format-Table -AutoSize

Write-Host "`n=== Recommended Evaluation Limits ===" -ForegroundColor Cyan
foreach ($r in $results) {
    $rounded = [math]::Ceiling($r.Evaluations / 10000) * 10000
    Write-Host "  $($r.Size): $rounded evaluations" -ForegroundColor Green
}
