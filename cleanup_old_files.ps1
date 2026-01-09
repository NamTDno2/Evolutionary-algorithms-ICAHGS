# Cleanup script - Remove all files except perfect results
# Keep only: result_final_perfect, result_final, result, perfect visualization files

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "CLEANING UP OLD FILES" -ForegroundColor Cyan
Write-Host "Keeping only perfect results files" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan

$kept = 0
$deleted = 0

# Files to KEEP (whitelist)
$keepFiles = @(
    # Perfect results
    'perfect_solution_comparison.png',
    'hv_comparison_perfect_vs_benchmark.html',
    'perfect_results_comparison.csv',
    'compare_perfect_results.py',
    'generate_hv_comparison_perfect_vs_benchmark.py',
    
    # Documentation
    'README',
    'PERFECT_RESULTS_README.md',
    '.gitignore',
    
    # Source code essentials
    'test.cpp',  # Keep in case needed for testing
    
    # This cleanup script itself
    'cleanup_old_files.ps1'
)

# Folders to KEEP
$keepFolders = @(
    '.git',
    'src',
    'include',
    'data',
    'config',
    'benchmark',
    'build',
    'scripts',
    'MultiObjectiveKy',
    'result_final_perfect',
    'result_final',
    'result'
)

Write-Host "`nPhase 1: Removing old Python scripts..." -ForegroundColor Yellow

# Delete old comparison scripts
Get-ChildItem -Path . -Filter "compare_*.py" | ForEach-Object {
    if ($keepFiles -notcontains $_.Name) {
        Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Force
        $deleted++
    } else {
        $kept++
    }
}

# Delete old generate scripts
Get-ChildItem -Path . -Filter "generate_*.py" | ForEach-Object {
    if ($keepFiles -notcontains $_.Name) {
        Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Force
        $deleted++
    } else {
        $kept++
    }
}

# Delete analysis scripts
Get-ChildItem -Path . -Filter "analyze_*.py" | ForEach-Object {
    Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
    Remove-Item $_.FullName -Force
    $deleted++
}

# Delete verify scripts
Get-ChildItem -Path . -Filter "verify_*.py" | ForEach-Object {
    Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
    Remove-Item $_.FullName -Force
    $deleted++
}

# Delete visualize scripts
Get-ChildItem -Path . -Filter "visualize_*.py" | ForEach-Object {
    Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
    Remove-Item $_.FullName -Force
    $deleted++
}

Write-Host "`nPhase 2: Removing PowerShell scripts..." -ForegroundColor Yellow

# Delete all .ps1 scripts except cleanup
Get-ChildItem -Path . -Filter "*.ps1" | ForEach-Object {
    if ($keepFiles -notcontains $_.Name) {
        Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Force
        $deleted++
    } else {
        $kept++
    }
}

Write-Host "`nPhase 3: Removing old HTML/CSV files..." -ForegroundColor Yellow

# Delete old HTML files
Get-ChildItem -Path . -Filter "*.html" | ForEach-Object {
    if ($keepFiles -notcontains $_.Name) {
        Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Force
        $deleted++
    } else {
        $kept++
    }
}

# Delete old CSV files
Get-ChildItem -Path . -Filter "*.csv" | ForEach-Object {
    if ($keepFiles -notcontains $_.Name) {
        Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Force
        $deleted++
    } else {
        $kept++
    }
}

Write-Host "`nPhase 4: Removing old PNG visualizations..." -ForegroundColor Yellow

# Delete old PNG files
Get-ChildItem -Path . -Filter "*.png" | ForEach-Object {
    if ($keepFiles -notcontains $_.Name) {
        Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Force
        $deleted++
    } else {
        $kept++
    }
}

Write-Host "`nPhase 5: Removing old test files..." -ForegroundColor Yellow

# Delete test cpp files
Get-ChildItem -Path . -Filter "test_*.cpp" | ForEach-Object {
    Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
    Remove-Item $_.FullName -Force
    $deleted++
}

# Delete test exe files
Get-ChildItem -Path . -Filter "test_*.exe" | ForEach-Object {
    Write-Host "  Deleting: $($_.Name)" -ForegroundColor Gray
    Remove-Item $_.FullName -Force
    $deleted++
}

# Delete compile_error.txt
if (Test-Path "compile_error.txt") {
    Write-Host "  Deleting: compile_error.txt" -ForegroundColor Gray
    Remove-Item "compile_error.txt" -Force
    $deleted++
}

Write-Host "`nPhase 6: Removing old result folders..." -ForegroundColor Yellow

# Delete old result folders
Get-ChildItem -Path . -Directory | ForEach-Object {
    if ($_.Name -match "^result" -and $keepFolders -notcontains $_.Name) {
        Write-Host "  Deleting folder: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Recurse -Force
        $deleted++
    } elseif ($_.Name -match "^old_") {
        Write-Host "  Deleting folder: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Recurse -Force
        $deleted++
    } elseif ($_.Name -eq "test_maxIteration") {
        Write-Host "  Deleting folder: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Recurse -Force
        $deleted++
    } elseif ($_.Name -eq "pareto_plots_new") {
        Write-Host "  Deleting folder: $($_.Name)" -ForegroundColor Gray
        Remove-Item $_.FullName -Recurse -Force
        $deleted++
    }
}

Write-Host "`n================================================" -ForegroundColor Green
Write-Host "CLEANUP COMPLETE!" -ForegroundColor Green
Write-Host "================================================" -ForegroundColor Green
Write-Host "Files deleted: $deleted" -ForegroundColor Yellow
Write-Host "Files kept: $kept" -ForegroundColor Green

Write-Host "`nKept folders:" -ForegroundColor Cyan
foreach ($folder in $keepFolders) {
    if (Test-Path $folder) {
        Write-Host "  ✓ $folder" -ForegroundColor Green
    }
}

Write-Host "`nKept files:" -ForegroundColor Cyan
foreach ($file in $keepFiles) {
    if (Test-Path $file) {
        Write-Host "  ✓ $file" -ForegroundColor Green
    }
}

Write-Host "`n✅ Workspace is now clean!" -ForegroundColor Green
Write-Host "Only perfect results and essential files remain." -ForegroundColor Green
