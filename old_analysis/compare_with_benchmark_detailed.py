import os
import sys
from pathlib import Path

def read_result_file(filepath):
    """Read result file and extract solutions"""
    solutions = []
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        # Skip header (first 7 lines)
        i = 7
        while i < len(lines):
            line = lines[i].strip()
            if not line:
                i += 1
                continue
            
            # Try to parse as objective values (CT WT)
            parts = line.split()
            if len(parts) == 2:
                try:
                    ct = float(parts[0])
                    wt = float(parts[1])
                    solutions.append((ct, wt))
                except:
                    pass
            i += 1
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
    
    return solutions

def dominates(sol1, sol2):
    """Check if sol1 dominates sol2 (minimization)"""
    ct1, wt1 = sol1
    ct2, wt2 = sol2
    
    better_in_one = (ct1 < ct2 or wt1 < wt2)
    not_worse = (ct1 <= ct2 and wt1 <= wt2)
    
    return better_in_one and not_worse

def compare_fronts(icahgs_solutions, benchmark_solutions):
    """Compare two Pareto fronts"""
    icahgs_dominates_benchmark = 0
    benchmark_dominates_icahgs = 0
    non_dominated = 0
    
    for i_sol in icahgs_solutions:
        dominated_by_benchmark = False
        dominates_benchmark = False
        
        for b_sol in benchmark_solutions:
            if dominates(b_sol, i_sol):
                dominated_by_benchmark = True
            if dominates(i_sol, b_sol):
                dominates_benchmark = True
        
        if dominated_by_benchmark and not dominates_benchmark:
            benchmark_dominates_icahgs += 1
        elif dominates_benchmark and not dominated_by_benchmark:
            icahgs_dominates_benchmark += 1
        else:
            non_dominated += 1
    
    return icahgs_dominates_benchmark, benchmark_dominates_icahgs, non_dominated

def calculate_improvement(icahgs_val, benchmark_val):
    """Calculate percentage improvement (negative = worse)"""
    if benchmark_val == 0:
        return 0
    return ((benchmark_val - icahgs_val) / benchmark_val) * 100

# Test instances
test_instances = [
    "20.10.1", "20.10.2", "20.10.3", "20.10.4",
    "50.10.1", "50.10.2", "50.10.3", "50.10.4",
    "100.10.1", "100.10.2", "100.10.3", "100.10.4",
    "200.10.1", "200.10.2", "200.10.3", "200.10.4"
]

print("=" * 80)
print("ICAHGS vs BENCHMARK COMPARISON")
print("=" * 80)
print()

results = []
missing_benchmark = []
missing_icahgs = []

for instance in test_instances:
    benchmark_file = f"benchmark/{instance}.txt"
    icahgs_file = f"result/{instance}.txt"
    
    if not os.path.exists(benchmark_file):
        missing_benchmark.append(instance)
        continue
    
    if not os.path.exists(icahgs_file):
        missing_icahgs.append(instance)
        continue
    
    # Read solutions
    benchmark_solutions = read_result_file(benchmark_file)
    icahgs_solutions = read_result_file(icahgs_file)
    
    if not benchmark_solutions or not icahgs_solutions:
        continue
    
    # Get best solutions (first solution in each front)
    benchmark_best = benchmark_solutions[0]
    icahgs_best = icahgs_solutions[0]
    
    # Calculate improvements
    ct_improvement = calculate_improvement(icahgs_best[0], benchmark_best[0])
    wt_improvement = calculate_improvement(icahgs_best[1], benchmark_best[1])
    
    # Compare fronts
    i_dom_b, b_dom_i, non_dom = compare_fronts(icahgs_solutions, benchmark_solutions)
    
    results.append({
        'instance': instance,
        'benchmark_ct': benchmark_best[0],
        'benchmark_wt': benchmark_best[1],
        'benchmark_size': len(benchmark_solutions),
        'icahgs_ct': icahgs_best[0],
        'icahgs_wt': icahgs_best[1],
        'icahgs_size': len(icahgs_solutions),
        'ct_improvement': ct_improvement,
        'wt_improvement': wt_improvement,
        'icahgs_dominates': i_dom_b,
        'benchmark_dominates': b_dom_i,
        'non_dominated': non_dom
    })

# Print detailed comparison
print(f"{'Instance':<12} {'Benchmark CT':<14} {'ICAHGS CT':<12} {'CT Improve':<12} "
      f"{'Benchmark WT':<14} {'ICAHGS WT':<12} {'WT Improve':<12} {'Dominance':<15}")
print("-" * 120)

for r in results:
    ct_symbol = "✅" if r['ct_improvement'] > 0 else ("❌" if r['ct_improvement'] < -5 else "≈")
    wt_symbol = "✅" if r['wt_improvement'] > 0 else ("❌" if r['wt_improvement'] < -5 else "≈")
    
    if r['icahgs_dominates'] > r['benchmark_dominates']:
        dom_result = f"ICAHGS+{r['icahgs_dominates']}"
    elif r['benchmark_dominates'] > r['icahgs_dominates']:
        dom_result = f"Bench+{r['benchmark_dominates']}"
    else:
        dom_result = "Equal"
    
    print(f"{r['instance']:<12} "
          f"{r['benchmark_ct']:<14.2f} {r['icahgs_ct']:<12.2f} "
          f"{r['ct_improvement']:>+6.1f}% {ct_symbol:<4} "
          f"{r['benchmark_wt']:<14.2f} {r['icahgs_wt']:<12.2f} "
          f"{r['wt_improvement']:>+6.1f}% {wt_symbol:<4} "
          f"{dom_result:<15}")

# Summary statistics
print("\n" + "=" * 80)
print("SUMMARY STATISTICS")
print("=" * 80)

if results:
    avg_ct_improvement = sum(r['ct_improvement'] for r in results) / len(results)
    avg_wt_improvement = sum(r['wt_improvement'] for r in results) / len(results)
    
    ct_better = sum(1 for r in results if r['ct_improvement'] > 0)
    ct_worse = sum(1 for r in results if r['ct_improvement'] < -5)
    ct_similar = len(results) - ct_better - ct_worse
    
    wt_better = sum(1 for r in results if r['wt_improvement'] > 0)
    wt_worse = sum(1 for r in results if r['wt_improvement'] < -5)
    wt_similar = len(results) - wt_better - wt_worse
    
    total_icahgs_dom = sum(r['icahgs_dominates'] for r in results)
    total_bench_dom = sum(r['benchmark_dominates'] for r in results)
    
    print(f"\nCompletion Time (CT):")
    print(f"  Average improvement: {avg_ct_improvement:+.2f}%")
    print(f"  Better: {ct_better}/{len(results)} ({ct_better/len(results)*100:.1f}%)")
    print(f"  Similar: {ct_similar}/{len(results)} ({ct_similar/len(results)*100:.1f}%)")
    print(f"  Worse: {ct_worse}/{len(results)} ({ct_worse/len(results)*100:.1f}%)")
    
    print(f"\nWaiting Time (WT):")
    print(f"  Average improvement: {avg_wt_improvement:+.2f}%")
    print(f"  Better: {wt_better}/{len(results)} ({wt_better/len(results)*100:.1f}%)")
    print(f"  Similar: {wt_similar}/{len(results)} ({wt_similar/len(results)*100:.1f}%)")
    print(f"  Worse: {wt_worse}/{len(results)} ({wt_worse/len(results)*100:.1f}%)")
    
    print(f"\nDomination Analysis:")
    print(f"  Total ICAHGS dominates Benchmark: {total_icahgs_dom} solutions")
    print(f"  Total Benchmark dominates ICAHGS: {total_bench_dom} solutions")
    
    if total_icahgs_dom > total_bench_dom:
        print(f"  Result: ICAHGS WINS by {total_icahgs_dom - total_bench_dom} solutions ✅")
    elif total_bench_dom > total_icahgs_dom:
        print(f"  Result: Benchmark WINS by {total_bench_dom - total_icahgs_dom} solutions ⚠️")
    else:
        print(f"  Result: TIE")
    
    print(f"\nPareto Front Size:")
    avg_bench_size = sum(r['benchmark_size'] for r in results) / len(results)
    avg_icahgs_size = sum(r['icahgs_size'] for r in results) / len(results)
    print(f"  Benchmark average: {avg_bench_size:.1f} solutions")
    print(f"  ICAHGS average: {avg_icahgs_size:.1f} solutions")

# Print missing files
if missing_benchmark:
    print(f"\n⚠️ Missing benchmark files: {len(missing_benchmark)}")
    print(f"   {', '.join(missing_benchmark)}")

if missing_icahgs:
    print(f"\n⚠️ Missing ICAHGS results: {len(missing_icahgs)}")
    print(f"   {', '.join(missing_icahgs)}")
    if missing_icahgs:
        instances_str = "', '".join(missing_icahgs[:5])
        print(f"\n💡 Run instances to generate results:")
        print(f"   foreach ($i in @('{instances_str}')) {{ ./build/main.exe data/$i.txt }}")

print("\n" + "=" * 80)
print("Comparison complete!")
print("=" * 80)
