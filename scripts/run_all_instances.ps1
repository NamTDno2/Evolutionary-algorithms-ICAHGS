# Script to run ICAHGS on all instances

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  RUNNING ICAHGS ON ALL INSTANCES" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Get all data files and filter
$instances = Get-ChildItem -Path "data" -Filter "*.txt" | 
    Where-Object { $_.Name -match '^(20|50|100|200)\.\d+\.\d+\.txt$' } |
    Sort-Object { 
        $parts = $_.BaseName -split '\.'
        [int]$parts[0] * 10000 + [int]$parts[1] * 100 + [int]$parts[2]
    }

Write-Host "Found $($instances.Count) instances to run`n" -ForegroundColor Yellow

$totalInstances = $instances.Count
$currentInstance = 0
$successCount = 0
$failCount = 0
$startTime = Get-Date

foreach ($file in $instances) {
    $currentInstance++
    $instanceName = $file.BaseName
    $dataPath = $file.FullName
    $resultFile = "result\$instanceName.txt"
    
    $instanceStart = Get-Date
    Write-Host "[$currentInstance/$totalInstances] $instanceName" -ForegroundColor Cyan -NoNewline
    
    try {
        # Run the algorithm
        $null = & "build\main.exe" $dataPath 2>&1
        
        if ($LASTEXITCODE -eq 0 -and (Test-Path $resultFile)) {
            $elapsed = ((Get-Date) - $instanceStart).TotalSeconds
            $elapsedStr = "{0:0.0}s" -f $elapsed
            Write-Host " OK ($elapsedStr)" -ForegroundColor Green
            $successCount++
            
            # Count solutions
            $lines = Get-Content $resultFile
            if ($lines.Count -gt 5) {
                $numSol = $lines[5]
                Write-Host "  $numSol solutions" -ForegroundColor Gray
            }
        }
        else {
            Write-Host " FAILED" -ForegroundColor Red
            $failCount++
        }
    }
    catch {
        Write-Host " ERROR" -ForegroundColor Red
        $failCount++
    }
}

$totalTime = (Get-Date) - $startTime
$hours = [int]$totalTime.TotalHours
$minutes = [int]$totalTime.Minutes
$seconds = [int]$totalTime.Seconds

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  COMPLETE" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Total: $totalInstances" -ForegroundColor White
Write-Host "Success: $successCount" -ForegroundColor Green  
Write-Host "Failed: $failCount" -ForegroundColor Red
Write-Host "Time: ${hours}h ${minutes}m ${seconds}s" -ForegroundColor Yellow
Write-Host ""
