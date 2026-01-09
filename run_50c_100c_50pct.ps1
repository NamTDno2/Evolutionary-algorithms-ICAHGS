# Run 50C/100C with 50% baseline maxEvaluation
# Config: 50C=1,095,000 evals | 100C=20,610,000 evals

Write-Host "`n=== RUNNING 50C/100C WITH 50% BASELINE ===" -ForegroundColor Cyan
Write-Host "Config: 50C=1,095,000 evals | 100C=20,610,000 evals`n" -ForegroundColor Yellow

# Skip build - assume already built
Write-Host "✅ Using existing build`n" -ForegroundColor Green

# Create output folder
if (!(Test-Path result_50c_100c_50pct)) {
    New-Item -ItemType Directory -Path result_50c_100c_50pct | Out-Null
}

# Get 50C and 100C instances
$instances50 = Get-ChildItem "data/50.*.txt" | Sort-Object Name | ForEach-Object { $_.Name.Replace('.txt', '') }
$instances100 = Get-ChildItem "data/100.*.txt" | Sort-Object Name | ForEach-Object { $_.Name.Replace('.txt', '') }
$allInstances = $instances50 + $instances100

$total = $allInstances.Count
$completed = 0
$startTime = Get-Date

Write-Host "Processing $total instances (16 x 50C + 16 x 100C)...`n" -ForegroundColor Cyan

foreach ($instance in $allInstances) {
    $completed++
    $instanceStart = Get-Date
    
    Write-Host "[$completed/$total] Processing $instance..." -ForegroundColor White -NoNewline
    
    # Run algorithm
    .\build\main.exe "data/$instance.txt" 2>&1 | Out-Null
    
    $elapsed = [math]::Round(((Get-Date) - $instanceStart).TotalSeconds, 1)
    
    if (Test-Path "result_50c_100c_50pct/$instance.txt") {
        Write-Host " ✅ Done ($elapsed`s)" -ForegroundColor Green
    } else {
        Write-Host " ❌ Failed" -ForegroundColor Red
    }
}

$totalTime = [math]::Round(((Get-Date) - $startTime).TotalMinutes, 1)
$fileCount = (Get-ChildItem result_50c_100c_50pct/*.txt -ErrorAction SilentlyContinue).Count

Write-Host "`n=== COMPLETED ===" -ForegroundColor Green
Write-Host "Total time: $totalTime minutes" -ForegroundColor Cyan
Write-Host "Files created: $fileCount/$total" -ForegroundColor $(if($fileCount -eq $total){"Green"}else{"Red"})
Write-Host "Results saved to: result_50c_100c_50pct/`n" -ForegroundColor White
