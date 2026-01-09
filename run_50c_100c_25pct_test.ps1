# Test 50C/100C with maxEvaluation 25% baseline
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  TESTING 50C/100C WITH maxEvaluation" -ForegroundColor Cyan
Write-Host "  25% Baseline (547,500 / 10,305,000)" -ForegroundColor Yellow
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Build project
Write-Host "Building project..." -ForegroundColor Yellow
g++ -fdiagnostics-color=always -g -I./src/header ./src/*.cpp -o ./build/main.exe

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Build successful!" -ForegroundColor Green
Write-Host ""

# Create output directory
$outputDir = "result_50c_100c_25pct"
if (-not (Test-Path $outputDir)) {
    New-Item -ItemType Directory -Path $outputDir | Out-Null
}

# Get 50C and 100C instances
$instances50 = Get-ChildItem "data/50.*.txt" | Sort-Object Name
$instances100 = Get-ChildItem "data/100.*.txt" | Sort-Object Name
$allInstances = $instances50 + $instances100

$total = $allInstances.Count
$completed = 0

Write-Host "Processing $total instances (16 x 50C + 16 x 100C)..." -ForegroundColor Cyan
Write-Host "Expected time: ~3 minutes for 50C, ~30 minutes for 100C" -ForegroundColor Yellow
Write-Host ""

$startTime = Get-Date

foreach ($file in $allInstances) {
    $completed++
    $instance = $file.BaseName
    
    Write-Host "[$completed/$total] Processing $instance..." -ForegroundColor White -NoNewline
    
    $instanceStart = Get-Date
    & .\build\main.exe $file.FullName 2>&1 | Out-Null
    $instanceEnd = Get-Date
    $duration = ($instanceEnd - $instanceStart).TotalSeconds
    
    # Move result file
    if (Test-Path "result_final_hybrid/$instance.txt") {
        Move-Item "result_final_hybrid/$instance.txt" "$outputDir/$instance.txt" -Force
        Write-Host " ✅ Done ($([math]::Round($duration,1))s)" -ForegroundColor Green
    } else {
        Write-Host " ❌ Failed" -ForegroundColor Red
    }
}

$endTime = Get-Date
$totalDuration = ($endTime - $startTime).TotalMinutes

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "COMPLETED in $([math]::Round($totalDuration,1)) minutes" -ForegroundColor Green
Write-Host "Results saved to: $outputDir/" -ForegroundColor Yellow
Write-Host "Files created: $((Get-ChildItem "$outputDir/*.txt").Count)/$total" -ForegroundColor White
Write-Host "========================================" -ForegroundColor Cyan
