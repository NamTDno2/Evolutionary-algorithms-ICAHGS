# PowerShell script to run all datasets from 20.5.1 to 200.40.4
# Usage: .\run_all_datasets.ps1

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  AUTO-RUN ALL DATASETS (20.5.1 -> 200.40.4)" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# Clear previous results
if (Test-Path "results.csv") {
    Remove-Item "results.csv"
    Write-Host "`nCleared previous results.csv" -ForegroundColor Green
}

# Check if executable exists
if (-not (Test-Path "build\main.exe")) {
    Write-Host "ERROR: build\main.exe not found! Please build first." -ForegroundColor Red
    exit 1
}

# Define dataset patterns (customers.trucks.instance)
$datasets = @(
    # 20 customers
    "20.5.1", "20.5.2", "20.5.3", "20.5.4",
    "20.10.1", "20.10.2", "20.10.3", "20.10.4",
    "20.20.1", "20.20.2", "20.20.3", "20.20.4",
    
    # 50 customers
    "50.10.1", "50.10.2", "50.10.3", "50.10.4",
    "50.20.1", "50.20.2", "50.20.3", "50.20.4",
    "50.30.1", "50.30.2", "50.30.3", "50.30.4",
    "50.40.1", "50.40.2", "50.40.3", "50.40.4",
    
    # 100 customers
    "100.10.1", "100.10.2", "100.10.3", "100.10.4",
    "100.20.1", "100.20.2", "100.20.3", "100.20.4",
    "100.30.1", "100.30.2", "100.30.3", "100.30.4",
    "100.40.1", "100.40.2", "100.40.3", "100.40.4",
    
    # 200 customers
    "200.10.1", "200.10.2", "200.10.3", "200.10.4",
    "200.20.1", "200.20.2", "200.20.3", "200.20.4",
    "200.30.1", "200.30.2", "200.30.3", "200.30.4",
    "200.40.1", "200.40.2", "200.40.3", "200.40.4"
)

$totalDatasets = $datasets.Count
$completed = 0
$failed = 0
$startTime = Get-Date

Write-Host "`nTotal datasets to process: $totalDatasets`n" -ForegroundColor Yellow

foreach ($dataset in $datasets) {
    $dataFile = "data\$dataset.txt"
    
    # Check if file exists
    if (-not (Test-Path $dataFile)) {
        Write-Host "SKIP: $dataFile not found" -ForegroundColor Yellow
        $failed++
        continue
    }
    
    Write-Host "================================================" -ForegroundColor Cyan
    Write-Host "[$($completed + 1)/$totalDatasets] Running: $dataset" -ForegroundColor Cyan
    Write-Host "================================================" -ForegroundColor Cyan
    
    # Run the executable
    $output = & "build\main.exe" $dataFile 2>&1
    
    if ($LASTEXITCODE -eq 0) {
        # Extract summary from output
        $executionTime = ($output | Select-String "Total execution time: (\d+\.\d+)").Matches.Groups[1].Value
        $paretoSize = ($output | Select-String "Pareto front size: (\d+)").Matches.Groups[1].Value
        $uniqueSols = ($output | Select-String "Unique solutions: (\d+)").Matches.Groups[1].Value
        $feasibility = ($output | Select-String "Feasibility: (\w+)").Matches.Groups[1].Value
        
        if ($feasibility -eq "YES") {
            Write-Host "SUCCESS: $dataset | FEASIBLE" -ForegroundColor Green
        } else {
            Write-Host "SUCCESS: $dataset | INFEASIBLE" -ForegroundColor Yellow
        }
        Write-Host "   Execution: $executionTime s | Pareto: $paretoSize | Unique: $uniqueSols" -ForegroundColor Gray
        $completed++
    } else {
        Write-Host "FAILED: $dataset (Exit code: $LASTEXITCODE)" -ForegroundColor Red
        $failed++
    }
    
    Write-Host ""
}

$endTime = Get-Date
$totalTime = ($endTime - $startTime).TotalSeconds

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "           BATCH RUN COMPLETE" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Completed: $completed" -ForegroundColor Green
Write-Host "Failed:    $failed" -ForegroundColor Red
$roundedTime = [math]::Round($totalTime, 2)
Write-Host "Total time: $roundedTime seconds" -ForegroundColor Yellow
Write-Host "Results saved to: results.csv" -ForegroundColor Cyan
Write-Host ""

# Display first few lines of results
if (Test-Path "results.csv") {
    Write-Host "Preview of results.csv:" -ForegroundColor Yellow
    Get-Content "results.csv" -Head 10
    $totalLines = (Get-Content "results.csv" | Measure-Object -Line).Lines
    Write-Host "Total rows in CSV: $totalLines" -ForegroundColor Gray
}
