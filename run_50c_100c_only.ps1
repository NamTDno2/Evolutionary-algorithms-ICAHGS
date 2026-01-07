# Run ONLY 50C and 100C with maxIteration=12/25
Write-Host "=== RUNNING 50C/100C WITH maxIteration ===" -ForegroundColor Cyan
Write-Host ""

g++ -fdiagnostics-color=always -g -I./src/header ./src/*.cpp -o ./build/main.exe
if ($LASTEXITCODE -ne 0) { Write-Host "Build failed!" -ForegroundColor Red; exit 1 }
Write-Host "Build OK" -ForegroundColor Green

$tempFolder = "result_50c_100c_new"
if (!(Test-Path $tempFolder)) { New-Item -ItemType Directory -Path $tempFolder | Out-Null }

$instances50 = Get-ChildItem "benchmark/50.*.txt" | ForEach-Object { $_.Name.Replace('.txt', '') }
$instances100 = Get-ChildItem "benchmark/100.*.txt" | ForEach-Object { $_.Name.Replace('.txt', '') }
$allInstances = $instances50 + $instances100

$total = $allInstances.Count
$completed = 0

Write-Host "Processing $total instances..." -ForegroundColor Cyan

foreach ($instance in $allInstances) {
    $completed++
    Write-Host "[$completed/$total] $instance..." -ForegroundColor White
    .\build\main.exe "benchmark/$instance.txt" 2>&1 | Out-Null
    if (Test-Path "result_final_hybrid/$instance.txt") {
        Move-Item "result_final_hybrid/$instance.txt" "$tempFolder/$instance.txt" -Force
    }
}

Write-Host "Done! Files: $((Get-ChildItem "$tempFolder/*.txt").Count)" -ForegroundColor Green
