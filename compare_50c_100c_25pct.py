import pandas as pd
import os

def read_solution_from_file(filepath):
    """Read best solution (first after header) from result file"""
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        if len(lines) >= 8:
            ct_wt_line = lines[7].strip().split()
            if len(ct_wt_line) >= 2:
                return float(ct_wt_line[0]), float(ct_wt_line[1])
    except:
        pass
    return None, None

def compare_solutions(result_ct, result_wt, bench_ct, bench_wt):
    """Compare solutions - returns 'win', 'loss', or 'tie'"""
    result_dominates = (result_ct <= bench_ct and result_wt <= bench_wt and 
                       (result_ct < bench_ct or result_wt < bench_wt))
    bench_dominates = (bench_ct <= result_ct and bench_wt <= result_wt and 
                      (bench_ct < result_ct or bench_wt < result_wt))
    
    if result_dominates:
        return 'win'
    elif bench_dominates:
        return 'loss'
    else:
        return 'tie'

print("\n" + "="*80)
print("COMPARING: 50C/100C maxEvaluation 25% vs Benchmark")
print("  Config: 50C=547,500 evals | 100C=10,305,000 evals")
print("="*80)

result_dir = "result_50c_100c_25pct"
benchmark_dir = "benchmark"

if not os.path.exists(result_dir):
    print(f"\n❌ Error: {result_dir} not found!")
    print("Run the test script first: .\\run_50c_100c_25pct_test.ps1")
    exit(1)

results_50c = []
results_100c = []

# Get all instances
for size in [50, 100]:
    for density in [10, 20, 30, 40]:
        for idx in [1, 2, 3, 4]:
            instance = f"{size}.{density}.{idx}"
            
            result_file = f"{result_dir}/{instance}.txt"
            bench_file = f"{benchmark_dir}/{instance}.txt"
            
            if not os.path.exists(result_file):
                print(f"⚠️  Missing: {result_file}")
                continue
            if not os.path.exists(bench_file):
                print(f"⚠️  Missing: {bench_file}")
                continue
            
            result_ct, result_wt = read_solution_from_file(result_file)
            bench_ct, bench_wt = read_solution_from_file(bench_file)
            
            if result_ct is None or bench_ct is None:
                continue
            
            status = compare_solutions(result_ct, result_wt, bench_ct, bench_wt)
            ct_gap = ((result_ct - bench_ct) / bench_ct * 100) if bench_ct > 0 else 0
            wt_gap = ((result_wt - bench_wt) / bench_wt * 100) if bench_wt > 0 else 0
            
            data = {
                'instance': instance,
                'status': status,
                'result_ct': result_ct,
                'result_wt': result_wt,
                'bench_ct': bench_ct,
                'bench_wt': bench_wt,
                'ct_gap': ct_gap,
                'wt_gap': wt_gap
            }
            
            if size == 50:
                results_50c.append(data)
            else:
                results_100c.append(data)

# Print results
for size_name, results in [("50C", results_50c), ("100C", results_100c)]:
    print(f"\n{'='*80}")
    print(f"  {size_name} RESULTS")
    print(f"{'='*80}")
    
    if not results:
        print("No results found!")
        continue
    
    wins = sum(1 for r in results if r['status'] == 'win')
    losses = sum(1 for r in results if r['status'] == 'loss')
    ties = sum(1 for r in results if r['status'] == 'tie')
    total = len(results)
    
    print(f"Total: {total} instances")
    print(f"  ✅ Win:  {wins:2d} ({wins/total*100:.1f}%)")
    print(f"  ❌ Loss: {losses:2d} ({losses/total*100:.1f}%)")
    print(f"  ⚖️  Tie:  {ties:2d} ({ties/total*100:.1f}%)")
    
    avg_ct_gap = sum(r['ct_gap'] for r in results) / total
    avg_wt_gap = sum(r['wt_gap'] for r in results) / total
    
    print(f"\nAverage gaps vs benchmark:")
    print(f"  CT: {avg_ct_gap:+.2f}%")
    print(f"  WT: {avg_wt_gap:+.2f}%")
    
    # Show some examples
    print(f"\nSample results:")
    for r in results[:4]:
        symbol = "✅" if r['status'] == 'win' else "❌" if r['status'] == 'loss' else "⚖️"
        print(f"  {symbol} {r['instance']}: CT={r['result_ct']:.2f} vs {r['bench_ct']:.2f} "
              f"({r['ct_gap']:+.1f}%), WT={r['result_wt']:.2f} vs {r['bench_wt']:.2f} "
              f"({r['wt_gap']:+.1f}%)")

# Overall summary
print(f"\n{'='*80}")
print("OVERALL SUMMARY (50C + 100C)")
print(f"{'='*80}")

all_results = results_50c + results_100c
total_wins = sum(1 for r in all_results if r['status'] == 'win')
total_losses = sum(1 for r in all_results if r['status'] == 'loss')
total_ties = sum(1 for r in all_results if r['status'] == 'tie')
total_all = len(all_results)

print(f"Total: {total_all} instances")
print(f"  ✅ Win:  {total_wins:2d} ({total_wins/total_all*100:.1f}%)")
print(f"  ❌ Loss: {total_losses:2d} ({total_losses/total_all*100:.1f}%)")
print(f"  ⚖️  Tie:  {total_ties:2d} ({total_ties/total_all*100:.1f}%)")

print("\n" + "="*80)
