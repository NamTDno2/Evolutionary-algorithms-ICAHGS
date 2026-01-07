# Measure evaluation counts using benchmark time limits
# This will establish the baseline evaluation counts for each problem size
# Time limits from HNSGAII-TS paper: 20s (20C), 200s (50C), 1000s (100C), 2000s (200C)

Write-Host "=== Measuring Evaluation Counts with Time-Based Stopping ===" -ForegroundColor Cyan
Write-Host "Time limits: 20s (20C), 200s (50C), 1000s (100C), 2000s (200C)" -ForegroundColor Yellow
Write-Host ""

$instances = @(
    @{file="data/20.10.1.txt"; time=20; name="20C"},
    @{file="data/50.10.1.txt"; time=200; name="50C"},
    @{file="data/100.10.1.txt"; time=1000; name="100C"},
    @{file="data/200.10.1.txt"; time=2000; name="200C"}
)

$results = @()

foreach ($inst in $instances) {
    Write-Host "Running $($inst.name): $($inst.file) with time limit $($inst.time)s..." -ForegroundColor Green
    
    $output = & ".\build\main.exe" $inst.file 2>&1 | Out-String
    
    # Extract evaluation count from "Total evaluations:" line
    if ($output -match "Total evaluations:\s+(\d+)") {
        $evalCount = [int]$matches[1]
        
        # Extract actual time from output
        $actualTime = "N/A"
        if ($output -match "Time limit reached:\s+([\d.]+)s") {
            $actualTime = $matches[1]
        }
        
        # Extract archive size
        $archiveSize = "N/A"
        if ($output -match "Archive size:\s+(\d+)") {
            $archiveSize = $matches[1]
        }
        
        # Extract iterations
        $iterations = "N/A"
        if ($output -match "Total iterations:\s+(\d+)") {
            $iterations = $matches[1]
        }
        
        $results += [PSCustomObject]@{
            Size = $inst.name
            TimeLimit = "$($inst.time)s"
            ActualTime = $actualTime
            Iterations = $iterations
            Evaluations = $evalCount
            EvalPerIter = if ($iterations -ne "N/A") { [math]::Round($evalCount / [int]$iterations, 0) } else { "N/A" }
            ArchiveSize = $archiveSize
        }
        
        Write-Host "  Evaluations: $evalCount | Iterations: $iterations | Archive: $archiveSize" -ForegroundColor Cyan
    } else {
        Write-Host "  ERROR: Could not extract evaluation count" -ForegroundColor Red
    }
    
    Write-Host ""
}

Write-Host "=== SUMMARY: Evaluation Counts by Problem Size ===" -ForegroundColor Yellow
$results | Format-Table -AutoSize

Write-Host ""
Write-Host "=== Recommended maxEvaluations Configuration ===" -ForegroundColor Cyan
Write-Host "Based on time-limited runs matching benchmark paper:" -ForegroundColor White
foreach ($r in $results) {
    $rounded = [math]::Ceiling($r.Evaluations / 10000) * 10000
    Write-Host "  $($r.Size): $rounded evaluations (measured: $($r.Evaluations))" -ForegroundColor Green
}
Write-Host ""
Write-Host "Update main.cpp with these values for evaluation-based stopping criterion." -ForegroundColor Yellow
