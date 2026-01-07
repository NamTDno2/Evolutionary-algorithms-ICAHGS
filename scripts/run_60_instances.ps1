# Run ICAHGS on all 60 instances (20C to 200C) with evaluation-based stopping
# Evaluation limits: 20C=260K, 50C=2.19M, 100C=41.2M, 200C=75.2M

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "  RUNNING 60 INSTANCES" -ForegroundColor Green
Write-Host "  Evaluation-Based Stopping Criterion" -ForegroundColor Green
Write-Host "  Seed = 42 (Reproducible)" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Cyan

# Build first
Write-Host "Building project..." -ForegroundColor Yellow
& "C:\msys64\ucrt64\bin\g++.exe" -fdiagnostics-color=always -g `
    -I"./src/header" ./src/*.cpp -o ./build/main.exe 2>&1 | Out-Null

if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Build failed!" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Build successful!`n" -ForegroundColor Green

# Select instances from 20C to 200C (60 instances total)
$instances = Get-ChildItem -Path "data" -Filter "*.txt" | 
    Where-Object { $_.Name -match '^(20|50|100|200)\.\d+\.\d+\.txt$' } |
    Sort-Object { 
        $parts = $_.BaseName -split '\.'
        [int]$parts[0] * 10000 + [int]$parts[1] * 100 + [int]$parts[2]
    }

Write-Host "Found $($instances.Count) instances to run" -ForegroundColor Yellow

# Expected counts by size
$expectedCounts = @{
    "20" = 12   # 20.5.x (4) + 20.10.x (4) + 20.20.x (4)
    "50" = 16   # 50.10.x (4) + 50.20.x (4) + 50.30.x (4) + 50.40.x (4)
    "100" = 16  # 100.10.x (4) + 100.20.x (4) + 100.30.x (4) + 100.40.x (4)
    "200" = 16  # 200.10.x (4) + 200.20.x (4) + 200.30.x (4) + 200.40.x (4)
}

# Count by size
$countBySize = @{}
foreach ($inst in $instances) {
    $size = ($inst.BaseName -split '\.')[0]
    if (-not $countBySize.ContainsKey($size)) {
        $countBySize[$size] = 0
    }
    $countBySize[$size]++
}

Write-Host "Distribution:" -ForegroundColor Yellow
foreach ($size in @("20", "50", "100", "200")) {
    $count = $countBySize[$size]
    $expected = $expectedCounts[$size]
    $status = if ($count -eq $expected) { "OK" } else { "WARNING" }
    $color = if ($count -eq $expected) { "Green" } else { "Yellow" }
    Write-Host "  ${size}C: $count instances $status" -ForegroundColor $color
}
Write-Host ""

if ($instances.Count -ne 60) {
    Write-Host "WARNING: Expected 60 instances, found $($instances.Count)" -ForegroundColor Yellow
    $continue = Read-Host "Continue anyway? (y/n)"
    if ($continue -ne "y") {
        exit 0
    }
}

# Evaluation limits by size
$evalLimits = @{
    "20" = 260000
    "50" = 2190000
    "100" = 41220000
    "200" = 75200000
}

$totalInstances = $instances.Count
$currentInstance = 0
$successCount = 0
$failCount = 0
$startTime = Get-Date

# Results summary
$allResults = @()

foreach ($file in $instances) {
    $currentInstance++
    $instanceName = $file.BaseName
    $dataPath = $file.FullName
    $resultFile = "result\$instanceName.txt"
    
    # Get size and eval limit
    $size = ($instanceName -split '\.')[0]
    $evalLimit = $evalLimits[$size]
    $evalLimitStr = if ($evalLimit -ge 1000000) { 
        "{0:0.0}M" -f ($evalLimit / 1000000) 
    } else { 
        "{0}K" -f ($evalLimit / 1000) 
    }
    
    $instanceStart = Get-Date
    $sizeLabel = $size + "C"
    Write-Host "[$currentInstance/$totalInstances] $instanceName ($sizeLabel, $evalLimitStr evals)" -ForegroundColor Cyan
    
    try {
        # Run the algorithm (main.cpp automatically selects eval limit by size)
        $output = & "build\main.exe" $dataPath 2>&1 | Out-String
        $exitCode = $LASTEXITCODE
        
        # Check success: exit code 0 OR result file exists (handle stderr warnings)
        if ((Test-Path $resultFile)) {
            $elapsed = ((Get-Date) - $instanceStart).TotalSeconds
            $elapsedStr = "{0:0.1}s" -f $elapsed
            
            # Extract metrics from output
            $ct = 0
            $wt = 0
            $evals = 0
            $iterations = 0
            $archiveSize = 0
            
            if ($output -match "System Completion Time: ([\d.]+)") {
                $ct = [double]$Matches[1]
            }
            if ($output -match "Total Sample Waiting Time: ([\d.]+)") {
                $wt = [double]$Matches[1]
            }
            if ($output -match "Total evaluations: (\d+)") {
                $evals = [int]$Matches[1]
            }
            if ($output -match "Total iterations: (\d+)") {
                $iterations = [int]$Matches[1]
            }
            if ($output -match "Final archive size: (\d+)") {
                $archiveSize = [int]$Matches[1]
            }
            
            Write-Host "  OK CT=$ct, WT=$wt, Archive=$archiveSize, Evals=$evals, Time=$elapsedStr" -ForegroundColor Green
            
            $successCount++
            
            # Store result
            $result = New-Object PSObject -Property @{
                Instance = $instanceName
                Size = $size
                CT = $ct
                WT = $wt
                EvalLimit = $evalLimit
                Evaluations = $evals
                Iterations = $iterations
                ArchiveSize = $archiveSize
                Time = $elapsed
            }
            $allResults += $result
        }
        else {
            Write-Host "  FAILED (exit: $exitCode, result: $(Test-Path $resultFile))" -ForegroundColor Red
            $failCount++
        }
    }
    catch {
        Write-Host "  ERROR: $_" -ForegroundColor Red
        $failCount++
    }
    
    Write-Host ""
}

$totalTime = ((Get-Date) - $startTime).TotalSeconds
$totalTimeStr = "{0:0.0}" -f $totalTime

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "SUMMARY" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Total instances: $totalInstances" -ForegroundColor Yellow
Write-Host "Success: $successCount" -ForegroundColor Green
Write-Host "Failed: $failCount" -ForegroundColor $(if ($failCount -eq 0) { "Green" } else { "Red" })
Write-Host "Total time: ${totalTimeStr}s" -ForegroundColor Yellow
Write-Host ""

if ($allResults.Count -gt 0) {
    # Group by size and calculate stats
    Write-Host "Results by Size:" -ForegroundColor Cyan
    Write-Host ""
    
    foreach ($size in @("20", "50", "100", "200")) {
        $sizeResults = $allResults | Where-Object { $_.Size -eq $size }
        
        if ($sizeResults.Count -gt 0) {
            $avgCT = ($sizeResults | Measure-Object -Property CT -Average).Average
            $avgWT = ($sizeResults | Measure-Object -Property WT -Average).Average
            $avgArchive = ($sizeResults | Measure-Object -Property ArchiveSize -Average).Average
            $avgEvals = ($sizeResults | Measure-Object -Property Evaluations -Average).Average
            $avgTime = ($sizeResults | Measure-Object -Property Time -Average).Average
            
            $sizeLabel = $size + "C"
            $countLabel = $sizeResults.Count.ToString() + " instances"
            Write-Host "$sizeLabel ($countLabel):" -ForegroundColor Yellow
            
            $ctRound = [math]::Round($avgCT, 2)
            $wtRound = [math]::Round($avgWT, 2)
            $archiveRound = [math]::Round($avgArchive, 1)
            $evalsRound = [math]::Round($avgEvals, 0)
            $timeRound = [math]::Round($avgTime, 1)
            
            Write-Host "  Avg CT: $ctRound" -ForegroundColor White
            Write-Host "  Avg WT: $wtRound" -ForegroundColor White
            Write-Host "  Avg Archive: $archiveRound" -ForegroundColor White
            Write-Host "  Avg Evaluations: $evalsRound" -ForegroundColor White
            Write-Host "  Avg Time: $timeRound" -ForegroundColor White
            Write-Host ""
        }
    }
    
    # Export detailed results to CSV
    $csvPath = "results_60_instances.csv"
    $allResults | Export-Csv -Path $csvPath -NoTypeInformation -Encoding UTF8
    Write-Host "OK Detailed results exported to: $csvPath" -ForegroundColor Green
}

Write-Host "`nAll results saved to result/ directory" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan
