# Compare Local Search Strategies
# Test 1: Full LS (apply to all solutions)
# Test 2: Quality-Based LS (apply only to Front 1)

Write-Host "`n=============================================" -ForegroundColor Cyan
Write-Host "COMPARISON: Full LS vs Quality-Based LS" -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan

$instances = @(
    @{ name = "20.10.1"; file = "data/20.10.1.txt"; customers = 20 },
    @{ name = "50.10.1"; file = "data/50.10.1.txt"; customers = 50 }
)

$allResults = @()

# ========================================
# STRATEGY 1: FULL LOCAL SEARCH (Baseline)
# ========================================
Write-Host "`n`n========================================" -ForegroundColor Yellow
Write-Host "STRATEGY 1: FULL LOCAL SEARCH" -ForegroundColor Yellow
Write-Host "Apply LS to ALL solutions" -ForegroundColor Yellow
Write-Host "========================================`n" -ForegroundColor Yellow

# Backup current code
Copy-Item "src/ICAHGS.cpp" "src/ICAHGS.cpp.backup" -Force

# Modify code to apply LS to ALL solutions
$code = Get-Content "src/ICAHGS.cpp" -Raw

# Replace quality-based LS with full LS
$fullLSCode = @'
            // 4. FULL LOCAL SEARCH (Baseline)
            // Strategy: Apply LS to ALL solutions for comparison
            
            // Evaluate offspring first
            SolutionEvaluator tempEvaluator(instance);
            tempEvaluator.evaluate(offspringSol);
            
            // Determine LS iterations based on problem size
            int numCustomers = instance.getNumCustomers();
            int lsIterations = 0;
            
            if (numCustomers <= 20) {
                lsIterations = 20;
            } else if (numCustomers <= 50) {
                lsIterations = 30;
            } else if (numCustomers <= 100) {
                lsIterations = 40;
            } else {
                lsIterations = 30;
            }
            
            // Apply LS to ALL feasible solutions
            if (offspringSol.systemCompletionTime < INF) {
                offspringSol = localSearch.improve(offspringSol, lsIterations);
            }
'@

$pattern = '            // 4\. QUALITY-BASED LOCAL SEARCH[\s\S]*?if \(isInFront1 && offspringSol\.systemCompletionTime < INF\) \{[\s\S]*?\}'

$code = $code -replace $pattern, $fullLSCode

Set-Content "src/ICAHGS.cpp" $code

# Build
Write-Host "Building with FULL LS strategy..." -ForegroundColor Yellow
& "C:\msys64\ucrt64\bin\g++.exe" -fdiagnostics-color=always -g `
    -I"./src/header" ./src/*.cpp -o ./build/main.exe 2>&1 | Out-Null

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    Copy-Item "src/ICAHGS.cpp.backup" "src/ICAHGS.cpp" -Force
    exit 1
}

Write-Host "Build successful!`n" -ForegroundColor Green

foreach ($instance in $instances) {
    Write-Host "Testing $($instance.name) with FULL LS..." -ForegroundColor Cyan
    
    $startTime = Get-Date
    $output = & ./build/main.exe $instance.file 2>&1 | Out-String
    $endTime = Get-Date
    
    $executionTime = ($endTime - $startTime).TotalSeconds
    
    # Extract metrics
    $paretoSize = 0
    $uniqueSolutions = 0
    $firstCT = "N/A"
    $firstWT = "N/A"
    
    if ($output -match "Pareto front size: (\d+)") {
        $paretoSize = [int]$Matches[1]
    }
    if ($output -match "Unique solutions: (\d+)") {
        $uniqueSolutions = [int]$Matches[1]
    }
    
    # Read result file
    $resultFile = "result/$($instance.name).txt"
    if (Test-Path $resultFile) {
        $lines = Get-Content $resultFile
        for ($i = 7; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match "^([\d.]+)\s+([\d.]+)$") {
                $firstCT = [double]$Matches[1]
                $firstWT = [double]$Matches[2]
                break
            }
        }
    }
    
    $allResults += [PSCustomObject]@{
        Instance = $instance.name
        Strategy = "Full LS"
        ExecutionTime = [math]::Round($executionTime, 2)
        ParetoSize = $paretoSize
        UniqueSolutions = $uniqueSolutions
        FirstCT = $firstCT
        FirstWT = $firstWT
    }
    
    Write-Host "  Time: $([math]::Round($executionTime, 2))s | Pareto: $paretoSize | CT: $firstCT" -ForegroundColor White
}

# ========================================
# STRATEGY 2: QUALITY-BASED LOCAL SEARCH
# ========================================
Write-Host "`n`n========================================" -ForegroundColor Yellow
Write-Host "STRATEGY 2: QUALITY-BASED LOCAL SEARCH" -ForegroundColor Yellow
Write-Host "Apply LS only to Front 1 solutions" -ForegroundColor Yellow
Write-Host "========================================`n" -ForegroundColor Yellow

# Restore original quality-based code
Copy-Item "src/ICAHGS.cpp.backup" "src/ICAHGS.cpp" -Force

# Build
Write-Host "Building with QUALITY-BASED LS strategy..." -ForegroundColor Yellow
& "C:\msys64\ucrt64\bin\g++.exe" -fdiagnostics-color=always -g `
    -I"./src/header" ./src/*.cpp -o ./build/main.exe 2>&1 | Out-Null

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit 1
}

Write-Host "Build successful!`n" -ForegroundColor Green

foreach ($instance in $instances) {
    Write-Host "Testing $($instance.name) with QUALITY-BASED LS..." -ForegroundColor Cyan
    
    $startTime = Get-Date
    $output = & ./build/main.exe $instance.file 2>&1 | Out-String
    $endTime = Get-Date
    
    $executionTime = ($endTime - $startTime).TotalSeconds
    
    # Extract metrics
    $paretoSize = 0
    $uniqueSolutions = 0
    $firstCT = "N/A"
    $firstWT = "N/A"
    
    if ($output -match "Pareto front size: (\d+)") {
        $paretoSize = [int]$Matches[1]
    }
    if ($output -match "Unique solutions: (\d+)") {
        $uniqueSolutions = [int]$Matches[1]
    }
    
    # Read result file
    $resultFile = "result/$($instance.name).txt"
    if (Test-Path $resultFile) {
        $lines = Get-Content $resultFile
        for ($i = 7; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match "^([\d.]+)\s+([\d.]+)$") {
                $firstCT = [double]$Matches[1]
                $firstWT = [double]$Matches[2]
                break
            }
        }
    }
    
    $allResults += [PSCustomObject]@{
        Instance = $instance.name
        Strategy = "Quality-Based"
        ExecutionTime = [math]::Round($executionTime, 2)
        ParetoSize = $paretoSize
        UniqueSolutions = $uniqueSolutions
        FirstCT = $firstCT
        FirstWT = $firstWT
    }
    
    Write-Host "  Time: $([math]::Round($executionTime, 2))s | Pareto: $paretoSize | CT: $firstCT" -ForegroundColor White
}

# ========================================
# SUMMARY COMPARISON
# ========================================
Write-Host "`n`n=============================================" -ForegroundColor Magenta
Write-Host "COMPARISON RESULTS" -ForegroundColor Magenta
Write-Host "=============================================" -ForegroundColor Magenta

$allResults | Format-Table -AutoSize

# Detailed comparison per instance
foreach ($inst in $instances) {
    $fullLS = $allResults | Where-Object { $_.Instance -eq $inst.name -and $_.Strategy -eq "Full LS" }
    $qualityLS = $allResults | Where-Object { $_.Instance -eq $inst.name -and $_.Strategy -eq "Quality-Based" }
    
    Write-Host "`n$($inst.name) ($($inst.customers) customers):" -ForegroundColor Yellow
    Write-Host "=============================================" -ForegroundColor Yellow
    
    # Time comparison
    $timeImprovement = [math]::Round((($fullLS.ExecutionTime - $qualityLS.ExecutionTime) / $fullLS.ExecutionTime) * 100, 1)
    Write-Host "  Execution Time:" -ForegroundColor Cyan
    Write-Host "    Full LS:        $($fullLS.ExecutionTime)s" -ForegroundColor White
    Write-Host "    Quality-Based:  $($qualityLS.ExecutionTime)s" -ForegroundColor White
    if ($timeImprovement -gt 0) {
        Write-Host "    Speedup:        $timeImprovement% faster ✅" -ForegroundColor Green
    } else {
        Write-Host "    Speedup:        $([math]::Abs($timeImprovement))% slower ❌" -ForegroundColor Red
    }
    
    # Pareto size comparison
    Write-Host "`n  Pareto Front Size:" -ForegroundColor Cyan
    Write-Host "    Full LS:        $($fullLS.ParetoSize) solutions" -ForegroundColor White
    Write-Host "    Quality-Based:  $($qualityLS.ParetoSize) solutions" -ForegroundColor White
    $diff = $qualityLS.ParetoSize - $fullLS.ParetoSize
    if ($diff -gt 0) {
        Write-Host "    Difference:     +$diff (better) ✅" -ForegroundColor Green
    } elseif ($diff -lt 0) {
        Write-Host "    Difference:     $diff (worse) ⚠️" -ForegroundColor Yellow
    } else {
        Write-Host "    Difference:     Same" -ForegroundColor White
    }
    
    # Quality comparison
    if ($fullLS.FirstCT -ne "N/A" -and $qualityLS.FirstCT -ne "N/A") {
        Write-Host "`n  First Solution Quality:" -ForegroundColor Cyan
        Write-Host "    Full LS:        CT=$($fullLS.FirstCT), WT=$($fullLS.FirstWT)" -ForegroundColor White
        Write-Host "    Quality-Based:  CT=$($qualityLS.FirstCT), WT=$($qualityLS.FirstWT)" -ForegroundColor White
        
        $ctDiff = [math]::Round((($qualityLS.FirstCT - $fullLS.FirstCT) / $fullLS.FirstCT) * 100, 1)
        $wtDiff = [math]::Round((($qualityLS.FirstWT - $fullLS.FirstWT) / $fullLS.FirstWT) * 100, 1)
        
        Write-Host "    CT Change:      $ctDiff%" -ForegroundColor $(if ($ctDiff -lt 0) { "Green" } else { "Red" })
        Write-Host "    WT Change:      $wtDiff%" -ForegroundColor $(if ($wtDiff -lt 0) { "Green" } else { "Red" })
    }
}

# Calculate HV
Write-Host "`n`n=============================================" -ForegroundColor Magenta
Write-Host "HYPERVOLUME COMPARISON" -ForegroundColor Magenta
Write-Host "=============================================" -ForegroundColor Magenta

$hvScript = @"
import sys
import numpy as np

def normalize_front(front, ref_point):
    if len(front) == 0:
        return []
    front = np.array(front)
    min_vals = front.min(axis=0)
    max_vals = front.max(axis=0)
    ranges = max_vals - min_vals
    ranges[ranges == 0] = 1
    normalized = (front - min_vals) / ranges
    return normalized.tolist()

def calculate_hv_2d(front, ref_point):
    if len(front) == 0:
        return 0.0
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
            print(f'{inst}: No solutions')
    except Exception as e:
        print(f'{inst}: Error - {e}')
"@

$hvScript | & python -

# Cleanup
Remove-Item "src/ICAHGS.cpp.backup" -Force -ErrorAction SilentlyContinue

Write-Host "`n=============================================" -ForegroundColor Green
Write-Host "COMPARISON COMPLETE!" -ForegroundColor Green
Write-Host "=============================================" -ForegroundColor Green
