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
print("VERIFYING result/ FOLDER PERFORMANCE")
print("="*80)

sizes = [20, 50, 100, 200]
results_by_size = {size: {'win': 0, 'lose': 0, 'tie': 0, 'total': 0} for size in sizes}

benchmark_files = sorted([f for f in os.listdir('benchmark') if f.endswith('.txt')])

for bench_file in benchmark_files:
    instance_name = bench_file.replace('.txt', '')
    parts = instance_name.split('.')
    
    if len(parts) != 3:
        continue
    
    size = int(parts[0])
    if size not in sizes:
        continue
    
    # Read benchmark
    bench_ct, bench_wt = read_best_solution(f'benchmark/{bench_file}')
    if bench_ct is None:
        continue
    
    # Read result
    result_file = f'result/{bench_file}'
    if not os.path.exists(result_file):
        print(f"⚠️  Missing: {result_file}")
        continue
    
    result_ct, result_wt = read_best_solution(result_file)
    if result_ct is None:
        continue
    
    # Compare
    status = compare_solutions(result_ct, result_wt, bench_ct, bench_wt)
    results_by_size[size]['total'] += 1
    results_by_size[size][status.lower()] += 1
    
    # Print details
    symbol = '✅' if status == 'WIN' else '❌' if status == 'LOSE' else '⚠️'
    print(f"{symbol} {instance_name:12s}: {status:4s}  Result({result_ct:7.2f}, {result_wt:9.2f}) vs Bench({bench_ct:7.2f}, {bench_wt:9.2f})")

print("\n" + "="*80)
print("SUMMARY BY SIZE")
print("="*80)

overall_wins = 0
overall_total = 0

for size in sizes:
    stats = results_by_size[size]
    total = stats['total']
    if total > 0:
        win_rate = (stats['win'] / total) * 100
        overall_wins += stats['win']
        overall_total += total
        
        print(f"\n{size}C ({total} instances):")
        print(f"  ✅ WIN:  {stats['win']:2d} ({stats['win']/total*100:5.1f}%)")
        print(f"  ❌ LOSE: {stats['lose']:2d} ({stats['lose']/total*100:5.1f}%)")
        print(f"  ⚠️  TIE:  {stats['tie']:2d} ({stats['tie']/total*100:5.1f}%)")

if overall_total > 0:
    overall_win_rate = (overall_wins / overall_total) * 100
    print("\n" + "="*80)
    print(f"OVERALL: {overall_wins}/{overall_total} wins = {overall_win_rate:.1f}%")
    print("="*80)
