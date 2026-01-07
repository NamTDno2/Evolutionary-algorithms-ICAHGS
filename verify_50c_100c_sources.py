import os

def read_best_solution(filepath):
    """Read best solution from file"""
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        if len(lines) >= 8:
            ct_wt = lines[7].strip().split()
            if len(ct_wt) >= 2:
                return float(ct_wt[0]), float(ct_wt[1])
    except:
        pass
    return None, None

def compare_solutions(result_ct, result_wt, bench_ct, bench_wt):
    """Compare two solutions"""
    result_dominates = (result_ct <= bench_ct and result_wt <= bench_wt and 
                       (result_ct < bench_ct or result_wt < bench_wt))
    bench_dominates = (bench_ct <= result_ct and bench_wt <= result_wt and 
                      (bench_ct < result_ct or bench_wt < result_wt))
    
    if result_dominates:
        return 'WIN'
    elif bench_dominates:
        return 'LOSE'
    else:
        return 'TIE'

print("="*80)
print("COMPARING 50C/100C RESULTS FROM DIFFERENT FOLDERS")
print("="*80)

folders = [
    ('result/', 'maxEval=1.095M/20.6M'),
    ('test_maxIteration/', 'Last Iter=3-5'),
    ('result_maxiter_high/', 'maxIter=50/100'),
]

for size in [50, 100]:
    print(f"\n{'='*80}")
    print(f"{size}C COMPARISON")
    print('='*80)
    
    results_by_folder = {folder: {'win': 0, 'lose': 0, 'tie': 0} for folder, _ in folders}
    
    benchmark_files = sorted([f for f in os.listdir('benchmark') 
                            if f.startswith(f'{size}.') and f.endswith('.txt')])
    
    for bench_file in benchmark_files:
        instance_name = bench_file.replace('.txt', '')
        
        # Read benchmark
        bench_ct, bench_wt = read_best_solution(f'benchmark/{bench_file}')
        if bench_ct is None:
            continue
        
        print(f"\n{instance_name}:")
        
        for folder, config_name in folders:
            result_file = f'{folder}{bench_file}'
            if not os.path.exists(result_file):
                continue
            
            result_ct, result_wt = read_best_solution(result_file)
            if result_ct is None:
                continue
            
            status = compare_solutions(result_ct, result_wt, bench_ct, bench_wt)
            results_by_folder[folder][status.lower()] += 1
            
            symbol = '✅' if status == 'WIN' else '❌' if status == 'LOSE' else '⚠️'
            print(f"  {symbol} {folder:25s} ({config_name:20s}): {status:4s}")
    
    # Summary for this size
    print(f"\n{'-'*80}")
    print(f"SUMMARY FOR {size}C:")
    print('-'*80)
    for folder, config_name in folders:
        stats = results_by_folder[folder]
        total = stats['win'] + stats['lose'] + stats['tie']
        if total > 0:
            win_rate = (stats['win'] / total) * 100
            print(f"{folder:25s} ({config_name:20s}): "
                  f"{stats['win']:2d} wins / {total:2d} = {win_rate:5.1f}%")
