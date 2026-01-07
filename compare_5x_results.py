import os
import re

def parse_pareto_front(filepath):
    """Parse Pareto front from result file"""
    solutions = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or ':' in line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    ct = float(parts[0])
                    wt = float(parts[1])
                    solutions.append((ct, wt))
                except:
                    continue
    return solutions

def dominates(sol1, sol2):
    """Check if sol1 dominates sol2 (minimization)"""
    better_in_one = (sol1[0] < sol2[0]) or (sol1[1] < sol2[1])
    not_worse = (sol1[0] <= sol2[0]) and (sol1[1] <= sol2[1])
    return better_in_one and not_worse

def compare_fronts(result_front, benchmark_front):
    """Compare two Pareto fronts"""
    result_dominates = 0
    benchmark_dominates = 0
    
    for r_sol in result_front:
        for b_sol in benchmark_front:
            if dominates(r_sol, b_sol):
                result_dominates += 1
            elif dominates(b_sol, r_sol):
                benchmark_dominates += 1
    
    if result_dominates > benchmark_dominates:
        return 'WIN'
    elif benchmark_dominates > result_dominates:
        return 'LOSE'
    else:
        return 'TIE'

def calculate_metrics(result_front, benchmark_front):
    """Calculate CT and WT gaps"""
    if not result_front or not benchmark_front:
        return None, None
    
    result_best_ct = min(sol[0] for sol in result_front)
    result_best_wt = min(sol[1] for sol in result_front)
    
    benchmark_best_ct = min(sol[0] for sol in benchmark_front)
    benchmark_best_wt = min(sol[1] for sol in benchmark_front)
    
    ct_gap = ((result_best_ct - benchmark_best_ct) / benchmark_best_ct) * 100
    wt_gap = ((result_best_wt - benchmark_best_wt) / benchmark_best_wt) * 100
    
    return ct_gap, wt_gap

# Main comparison
result_dir = 'result_5x'
benchmark_dir = 'benchmark'

instances = [
    # 50C instances
    "50.10.1", "50.10.2", "50.10.3", "50.10.4",
    "50.20.1", "50.20.2", "50.20.3", "50.20.4",
    "50.30.1", "50.30.2", "50.30.3", "50.30.4",
    "50.40.1", "50.40.2", "50.40.3", "50.40.4",
    # 100C instances
    "100.10.1", "100.10.2", "100.10.3", "100.10.4",
    "100.20.1", "100.20.2", "100.20.3", "100.20.4",
    "100.30.1", "100.30.2", "100.30.3", "100.30.4",
    "100.40.1", "100.40.2", "100.40.3", "100.40.4"
]

results_50c = {'WIN': 0, 'TIE': 0, 'LOSE': 0}
results_100c = {'WIN': 0, 'TIE': 0, 'LOSE': 0}
ct_gaps_50c = []
wt_gaps_50c = []
ct_gaps_100c = []
wt_gaps_100c = []

print("=== COMPARISON: 5x BUFFER vs BENCHMARK ===\n")
print(f"{'Instance':<12} {'Result':<8} {'CT Gap %':>10} {'WT Gap %':>10} {'Archive'}")
print("-" * 60)

for instance in instances:
    result_file = os.path.join(result_dir, f"{instance}.txt")
    benchmark_file = os.path.join(benchmark_dir, f"{instance}.txt")
    
    if not os.path.exists(result_file):
        print(f"{instance:<12} MISSING")
        continue
    
    result_front = parse_pareto_front(result_file)
    benchmark_front = parse_pareto_front(benchmark_file)
    
    outcome = compare_fronts(result_front, benchmark_front)
    ct_gap, wt_gap = calculate_metrics(result_front, benchmark_front)
    
    size = instance.split('.')[0]
    if size == '50':
        results_50c[outcome] += 1
        if ct_gap is not None:
            ct_gaps_50c.append(ct_gap)
            wt_gaps_50c.append(wt_gap)
    else:
        results_100c[outcome] += 1
        if ct_gap is not None:
            ct_gaps_100c.append(ct_gap)
            wt_gaps_100c.append(wt_gap)
    
    ct_str = f"{ct_gap:+.2f}%" if ct_gap is not None else "N/A"
    wt_str = f"{wt_gap:+.2f}%" if wt_gap is not None else "N/A"
    
    print(f"{instance:<12} {outcome:<8} {ct_str:>10} {wt_str:>10} {len(result_front):>7}")

print("\n" + "=" * 60)
print("\n50C RESULTS:")
total_50c = sum(results_50c.values())
win_rate_50c = (results_50c['WIN'] / total_50c * 100) if total_50c > 0 else 0
avg_ct_50c = sum(ct_gaps_50c) / len(ct_gaps_50c) if ct_gaps_50c else 0
avg_wt_50c = sum(wt_gaps_50c) / len(wt_gaps_50c) if wt_gaps_50c else 0

print(f"  Win:  {results_50c['WIN']}/{total_50c} ({win_rate_50c:.1f}%)")
print(f"  Tie:  {results_50c['TIE']}/{total_50c}")
print(f"  Lose: {results_50c['LOSE']}/{total_50c}")
print(f"  Avg CT gap: {avg_ct_50c:+.2f}%")
print(f"  Avg WT gap: {avg_wt_50c:+.2f}%")

print("\n100C RESULTS:")
total_100c = sum(results_100c.values())
win_rate_100c = (results_100c['WIN'] / total_100c * 100) if total_100c > 0 else 0
avg_ct_100c = sum(ct_gaps_100c) / len(ct_gaps_100c) if ct_gaps_100c else 0
avg_wt_100c = sum(wt_gaps_100c) / len(wt_gaps_100c) if wt_gaps_100c else 0

print(f"  Win:  {results_100c['WIN']}/{total_100c} ({win_rate_100c:.1f}%)")
print(f"  Tie:  {results_100c['TIE']}/{total_100c}")
print(f"  Lose: {results_100c['LOSE']}/{total_100c}")
print(f"  Avg CT gap: {avg_ct_100c:+.2f}%")
print(f"  Avg WT gap: {avg_wt_100c:+.2f}%")

print("\n" + "=" * 60)
print("\nCOMPARISON WITH OTHER CONFIGS:")
print("\nFor 50C:")
print("  maxIteration=12:    43.8% win rate ✅ BEST")
print("  maxEval 1.095M/20M: 25.0% win rate")
print("  maxEval 105K (5x):  12.5% win rate")
print(f"  maxEval 105K (NEW): {win_rate_50c:.1f}% win rate ← TESTING")
print("  maxEval 210K (10x): 12.5% win rate")

print("\nFor 100C:")
print("  maxIteration=25:    12.5% win rate")
print("  maxEval 1.095M/20M: 12.5% win rate")
print("  maxEval 330K (5x):  0.0% win rate")
print(f"  maxEval 330K (NEW): {win_rate_100c:.1f}% win rate ← TESTING")
print("  maxEval 660K (10x): 0.0% win rate")

# Summary recommendation
print("\n" + "=" * 60)
print("\nANALYSIS:")
if win_rate_50c < 43.8:
    print("  ⚠️  50C: 5x buffer still worse than maxIteration=12 (43.8%)")
if win_rate_100c < 12.5:
    print("  ⚠️  100C: 5x buffer worse than baseline (12.5%)")
    
print("\n  Hypothesis: maxEvaluation stopping may be fundamentally incompatible")
print("  with algorithm structure for 50C/100C problem sizes.")
print("\n  Recommendation: Consider reverting 50C/100C to maxIteration,")
print("  keeping maxEvaluation only for 20C/200C where it works well.")
