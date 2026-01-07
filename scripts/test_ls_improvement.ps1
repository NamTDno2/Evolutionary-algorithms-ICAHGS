# Test Local Search Improvement - Quality-Based (Front 1 Only)
# Compare execution time and solution quality

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "Testing Quality-Based Local Search" -ForegroundColor Cyan
Write-Host "Strategy: Only Front 1 solutions get LS" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# Build project first
Write-Host "Building project..." -ForegroundColor Yellow
& "C:\msys64\ucrt64\bin\g++.exe" -fdiagnostics-color=always -g `
    -I"./src/header" `
    ./src/*.cpp `
    -o ./build/main.exe

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit 1
}

Write-Host "Build successful!`n" -ForegroundColor Green

# Test instances
$instances = @(
    @{
        name = "20.10.1"
        file = "data/20.10.1.txt"
        customers = 20
    },
    @{
        name = "50.10.1"
        file = "data/50.10.1.txt"
        customers = 50
    }
)

$results = @()

foreach ($instance in $instances) {
    Write-Host "`n========================================" -ForegroundColor Yellow
    Write-Host "Testing Instance: $($instance.name)" -ForegroundColor Yellow
    Write-Host "Customers: $($instance.customers)" -ForegroundColor Yellow
    Write-Host "========================================`n" -ForegroundColor Yellow
    
    # Run algorithm
    $startTime = Get-Date
    
    $output = & ./build/main.exe $instance.file 2>&1 | Out-String
    
    $endTime = Get-Date
    $executionTime = ($endTime - $startTime).TotalSeconds
    
    # Extract metrics from output
    if ($output -match "Total execution time: ([\d.]+)") {
        $algoTime = [double]$Matches[1]
    } else {
        $algoTime = $executionTime
    }
    
    if ($output -match "Pareto front size: (\d+)") {
        $paretoSize = [int]$Matches[1]
    } else {
        $paretoSize = 0
    }
    
    if ($output -match "Unique solutions: (\d+)") {
        $uniqueSolutions = [int]$Matches[1]
    } else {
        $uniqueSolutions = 0
    }
    
    # Check result file exists
    $resultFile = "result/$($instance.name).txt"
    $resultExists = Test-Path $resultFile
    
    # Extract first solution objectives
    $firstCT = "N/A"
    $firstWT = "N/A"
    
    if ($resultExists) {
        $lines = Get-Content $resultFile
        # Find first objective line (after 6 header lines + 1 route line)
        for ($i = 7; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match "^([\d.]+)\s+([\d.]+)$") {
                $firstCT = [double]$Matches[1]
                $firstWT = [double]$Matches[2]
                break
            }
        }
    }
    
    $result = [PSCustomObject]@{
        Instance = $instance.name
        Customers = $instance.customers
        ExecutionTime = [math]::Round($algoTime, 2)
        ParetoSize = $paretoSize
        UniqueSolutions = $uniqueSolutions
        FirstCT = $firstCT
        FirstWT = $firstWT
        ResultFile = if ($resultExists) { "✅" } else { "❌" }
    }
    
    $results += $result
    
    # Display result
    Write-Host "`nResults:" -ForegroundColor Green
    Write-Host "  Execution Time: $($result.ExecutionTime) seconds" -ForegroundColor Cyan
    Write-Host "  Pareto Front Size: $($result.ParetoSize)" -ForegroundColor Cyan
    Write-Host "  Unique Solutions: $($result.UniqueSolutions)" -ForegroundColor Cyan
    Write-Host "  First Solution - CT: $($result.FirstCT), WT: $($result.FirstWT)" -ForegroundColor Cyan
    Write-Host "  Result File: $($result.ResultFile)" -ForegroundColor Cyan
}

# Summary comparison
Write-Host "`n`n========================================" -ForegroundColor Magenta
Write-Host "SUMMARY COMPARISON" -ForegroundColor Magenta
Write-Host "========================================`n" -ForegroundColor Magenta

$results | Format-Table -AutoSize

# Calculate speedup
if ($results.Count -eq 2) {
    Write-Host "`nAnalysis:" -ForegroundColor Yellow
    Write-Host "  20C Execution Time: $($results[0].ExecutionTime)s" -ForegroundColor White
    Write-Host "  50C Execution Time: $($results[1].ExecutionTime)s" -ForegroundColor White
    
    $ratio = [math]::Round($results[1].ExecutionTime / $results[0].ExecutionTime, 2)
    Write-Host "  Time Ratio (50C/20C): ${ratio}x" -ForegroundColor Cyan
    
    Write-Host "`n  20C Pareto Size: $($results[0].ParetoSize) solutions" -ForegroundColor White
    Write-Host "  50C Pareto Size: $($results[1].ParetoSize) solutions" -ForegroundColor White
    
    if ($results[0].FirstCT -ne "N/A" -and $results[1].FirstCT -ne "N/A") {
        Write-Host "`n  Solution Quality:" -ForegroundColor Yellow
        Write-Host "    20C: CT=$($results[0].FirstCT), WT=$($results[0].FirstWT)" -ForegroundColor White
        Write-Host "    50C: CT=$($results[1].FirstCT), WT=$($results[1].FirstWT)" -ForegroundColor White
    }
}

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "Testing Complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

# Calculate HV for comparison
Write-Host "`nCalculating Hypervolume..." -ForegroundColor Yellow

$hvScript = @"
import sys
import numpy as np

def normalize_front(front, ref_point):
    if len(front) == 0:
        return []
    
    front = np.array(front)
    min_vals = front.min(axis=0)
    max_vals = front.max(axis=0)
    
    # Normalize to [0, 1]
    ranges = max_vals - min_vals
    ranges[ranges == 0] = 1  # Avoid division by zero
    
    normalized = (front - min_vals) / ranges
    return normalized.tolist()

def calculate_hv_2d(front, ref_point):
    if len(front) == 0:
        return 0.0
    
    # Sort by first objective (ascending)
    sorted_front = sorted(front, key=lambda x: x[0])
    
    hv = 0.0
    prev_x = 0.0
    
    for point in sorted_front:
        if point[0] >= ref_point[0] or point[1] >= ref_point[1]:
            continue
        
        width = point[0] - prev_x
        height = ref_point[1] - point[1]
        hv += width * height
        prev_x = point[0]
    
    return hv

# Read result files
instances = ['20.10.1', '50.10.1']
ref_point = [1.1, 1.1]

for inst in instances:
    filename = f'result/{inst}.txt'
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
        
        front = []
        for i in range(7, len(lines)):
            line = lines[i].strip()
            if not line:
                continue
            
            parts = line.split()
            if len(parts) == 2:
                try:
                    ct = float(parts[0])
                    wt = float(parts[1])
                    front.append([ct, wt])
                except:
                    pass
        
        if len(front) > 0:
            normalized = normalize_front(front, ref_point)
            hv = calculate_hv_2d(normalized, ref_point)
            print(f'{inst}: HV = {hv:.4f} ({len(front)} solutions)')
        else:
            print(f'{inst}: No solutions found')
    except Exception as e:
        print(f'{inst}: Error - {e}')
"@

$hvScript | & python -

Write-Host "`nTest completed! Check result/ folder for output files." -ForegroundColor Green
