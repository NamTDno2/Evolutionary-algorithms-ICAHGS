# Script to run only 50C and 100C instances with doubled evaluations
# Test if increased budget improves performance

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  RUNNING 50C AND 100C INSTANCES" -ForegroundColor Cyan
Write-Host "  Doubled Evaluations Test" -ForegroundColor Cyan
Write-Host "  50C: 1,095,000 evals (was 547,500)" -ForegroundColor Yellow
Write-Host "  100C: 20,610,000 evals (was 10,305,000)" -ForegroundColor Yellow
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Build project
Write-Host "Building project..." -ForegroundColor Yellow
$buildArgs = @(
    "-fdiagnostics-color=always",
    "-g",
    "-I`"$PSScriptRoot/src/header`"",
    "$PSScriptRoot/src/*.cpp",
    "-o",
    "$PSScriptRoot/build/main.exe"
)
& "C:\msys64\ucrt64\bin\g++.exe" @buildArgs

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Build successful!" -ForegroundColor Green
Write-Host ""

# Create output directory
$outputDir = "result_50c_100c_doubled"
if (-not (Test-Path $outputDir)) {
    New-Item -ItemType Directory -Path $outputDir | Out-Null
}

# Get 50C and 100C instances
$instances = Get-ChildItem -Path "data" -Filter "*.txt" | Where-Object {
    $name = $_.BaseName
    $parts = $name.Split('.')
    if ($parts.Count -ge 3) {
        $size = [int]$parts[0]
        return ($size -eq 50 -or $size -eq 100)
    }
    return $false
} | Sort-Object Name

$total = $instances.Count
Write-Host "Found $total instances to run" -ForegroundColor Green
Write-Host ""

# Count by size
$count50 = ($instances | Where-Object { $_.BaseName.StartsWith("50.") }).Count
$count100 = ($instances | Where-Object { $_.BaseName.StartsWith("100.") }).Count
Write-Host "Distribution:" -ForegroundColor Cyan
Write-Host "  50C: $count50 instances" -ForegroundColor White
Write-Host "  100C: $count100 instances" -ForegroundColor White
Write-Host ""

# Run instances
$results = @()
$current = 0

foreach ($instance in $instances) {
    $current++
    $name = $instance.BaseName
    $parts = $name.Split('.')
    $size = [int]$parts[0]
    
    # Determine evaluation limit for display
    $evalLimit = if ($size -eq 50) { "2.2M" } else { "41.2M" }
    
    Write-Host "[$current/$total] $name (${size}C, $evalLimit evals)" -ForegroundColor Cyan
    
    # Run algorithm
    $startTime = Get-Date
    $output = & "$PSScriptRoot/build/main.exe" "$($instance.FullName)" 2>&1 | Out-String
    $endTime = Get-Date
    $duration = ($endTime - $startTime).TotalSeconds
    
    # Parse output
    if ($output -match "System Completion Time:\s+([\d.]+)") {
        $ct = [double]$Matches[1]
    } else { $ct = 0 }
    
    if ($output -match "Total Sample Waiting Time:\s+([\d.]+)") {
        $wt = [double]$Matches[1]
    } else { $wt = 0 }
    
    if ($output -match "Archive size:\s+(\d+)") {
        $archive = [int]$Matches[1]
    } else { $archive = 0 }
    
    if ($output -match "Total evaluations:\s+(\d+)") {
        $evals = [int]$Matches[1]
    } else { $evals = 0 }
    
    Write-Host "  OK CT=$ct, WT=$wt, Archive=$archive, Evals=$evals, Time=$([math]::Round($duration))s" -ForegroundColor Green
    
    # Store result
    $results += [PSCustomObject]@{
        Instance = $name
        Size = $size
        CT = $ct
        WT = $wt
        ArchiveSize = $archive
        Evaluations = $evals
        Time = $duration
    }
    
    Write-Host ""
}

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "SUMMARY" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Total instances: $total" -ForegroundColor White
Write-Host "Success: $total" -ForegroundColor Green
Write-Host "Total time: $([math]::Round(($results | Measure-Object -Property Time -Sum).Sum))s" -ForegroundColor White
Write-Host ""

# Summary by size
Write-Host "Results by Size:" -ForegroundColor Cyan
Write-Host ""

$results50 = $results | Where-Object { $_.Size -eq 50 }
$results100 = $results | Where-Object { $_.Size -eq 100 }

Write-Host "50C ($($results50.Count) instances):" -ForegroundColor Yellow
Write-Host "  Avg CT: $([math]::Round(($results50 | Measure-Object -Property CT -Average).Average, 2))" -ForegroundColor White
Write-Host "  Avg WT: $([math]::Round(($results50 | Measure-Object -Property WT -Average).Average, 2))" -ForegroundColor White
Write-Host "  Avg Archive: $([math]::Round(($results50 | Measure-Object -Property ArchiveSize -Average).Average, 1))" -ForegroundColor White
Write-Host "  Avg Evaluations: $([math]::Round(($results50 | Measure-Object -Property Evaluations -Average).Average, 0))" -ForegroundColor White
Write-Host "  Avg Time: $([math]::Round(($results50 | Measure-Object -Property Time -Average).Average, 1))" -ForegroundColor White
Write-Host ""

Write-Host "100C ($($results100.Count) instances):" -ForegroundColor Yellow
Write-Host "  Avg CT: $([math]::Round(($results100 | Measure-Object -Property CT -Average).Average, 2))" -ForegroundColor White
Write-Host "  Avg WT: $([math]::Round(($results100 | Measure-Object -Property WT -Average).Average, 2))" -ForegroundColor White
Write-Host "  Avg Archive: $([math]::Round(($results100 | Measure-Object -Property ArchiveSize -Average).Average, 1))" -ForegroundColor White
Write-Host "  Avg Evaluations: $([math]::Round(($results100 | Measure-Object -Property Evaluations -Average).Average, 0))" -ForegroundColor White
Write-Host "  Avg Time: $([math]::Round(($results100 | Measure-Object -Property Time -Average).Average, 1))" -ForegroundColor White
Write-Host ""

# Export to CSV
$csvPath = "$outputDir/results_summary.csv"
$results | Export-Csv -Path $csvPath -NoTypeInformation -Encoding UTF8
Write-Host "✅ Detailed results exported to: $csvPath" -ForegroundColor Green
Write-Host ""
Write-Host "All results saved to $outputDir/ directory" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
