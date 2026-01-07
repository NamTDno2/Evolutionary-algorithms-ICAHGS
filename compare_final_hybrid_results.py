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
result_dir = 'result_final_hybrid'
benchmark_dir = 'benchmark'

instances = [
    # 20C instances
    "20.5.1", "20.5.2", "20.5.3", "20.5.4",
    "20.10.1", "20.10.2", "20.10.3", "20.10.4",
    "20.20.1", "20.20.2", "20.20.3", "20.20.4",
    # 50C instances
    "50.10.1", "50.10.2", "50.10.3", "50.10.4",
    "50.20.1", "50.20.2", "50.20.3", "50.20.4",
    "50.30.1", "50.30.2", "50.30.3", "50.30.4",
    "50.40.1", "50.40.2", "50.40.3", "50.40.4",
    # 100C instances
    "100.10.1", "100.10.2", "100.10.3", "100.10.4",
    "100.20.1", "100.20.2", "100.20.3", "100.20.4",
    "100.30.1", "100.30.2", "100.30.3", "100.30.4",
    "100.40.1", "100.40.2", "100.40.3", "100.40.4",
    # 200C instances
    "200.10.1", "200.10.2", "200.10.3", "200.10.4",
    "200.20.1", "200.20.2", "200.20.3", "200.20.4",
    "200.30.1", "200.30.2", "200.30.3", "200.30.4",
    "200.40.1", "200.40.2", "200.40.3", "200.40.4"
]

results_20c = {'WIN': 0, 'TIE': 0, 'LOSE': 0}
results_50c = {'WIN': 0, 'TIE': 0, 'LOSE': 0}
results_100c = {'WIN': 0, 'TIE': 0, 'LOSE': 0}
results_200c = {'WIN': 0, 'TIE': 0, 'LOSE': 0}
ct_gaps_20c, wt_gaps_20c = [], []
ct_gaps_50c, wt_gaps_50c = [], []
ct_gaps_100c, wt_gaps_100c = [], []
ct_gaps_200c, wt_gaps_200c = [], []

print("=" * 70)
print("FINAL HYBRID CONFIGURATION vs BENCHMARK")
print("=" * 70)
print("\nConfiguration:")
print("  20C:  maxEvaluation = 65,000      (75% win target)")
print("  50C:  maxIteration = 12           (43.8% win target)")
print("  100C: maxIteration = 25           (12.5% win target)")
print("  200C: maxEvaluation = 18,800,000  (68.8% win target)")
print("\n" + "=" * 70)
print(f"\n{'Instance':<12} {'Result':<8} {'CT Gap %':>10} {'WT Gap %':>10} {'Archive'}")
print("-" * 70)

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
    if size == '20':
        results_20c[outcome] += 1
        if ct_gap is not None:
            ct_gaps_20c.append(ct_gap)
            wt_gaps_20c.append(wt_gap)
    elif size == '50':
        results_50c[outcome] += 1
        if ct_gap is not None:
            ct_gaps_50c.append(ct_gap)
            wt_gaps_50c.append(wt_gap)
    elif size == '100':
        results_100c[outcome] += 1
        if ct_gap is not None:
            ct_gaps_100c.append(ct_gap)
            wt_gaps_100c.append(wt_gap)
    else:
        results_200c[outcome] += 1
        if ct_gap is not None:
            ct_gaps_200c.append(ct_gap)
            wt_gaps_200c.append(wt_gap)
    
    ct_str = f"{ct_gap:+.2f}%" if ct_gap is not None else "N/A"
    wt_str = f"{wt_gap:+.2f}%" if wt_gap is not None else "N/A"
    
    print(f"{instance:<12} {outcome:<8} {ct_str:>10} {wt_str:>10} {len(result_front):>7}")

print("\n" + "=" * 70)
print("\nRESULTS BY SIZE:\n")

# 20C Results
total_20c = sum(results_20c.values())
win_rate_20c = (results_20c['WIN'] / total_20c * 100) if total_20c > 0 else 0
avg_ct_20c = sum(ct_gaps_20c) / len(ct_gaps_20c) if ct_gaps_20c else 0
avg_wt_20c = sum(wt_gaps_20c) / len(wt_gaps_20c) if wt_gaps_20c else 0

print("20C (maxEvaluation = 65,000):")
print(f"  Win:  {results_20c['WIN']}/{total_20c} ({win_rate_20c:.1f}%) {'✅' if win_rate_20c >= 75 else '⚠️'}")
print(f"  Tie:  {results_20c['TIE']}/{total_20c}")
print(f"  Lose: {results_20c['LOSE']}/{total_20c}")
print(f"  Avg CT gap: {avg_ct_20c:+.2f}%")
print(f"  Avg WT gap: {avg_wt_20c:+.2f}%")

# 50C Results
print("\n50C (maxIteration = 12):")
total_50c = sum(results_50c.values())
win_rate_50c = (results_50c['WIN'] / total_50c * 100) if total_50c > 0 else 0
avg_ct_50c = sum(ct_gaps_50c) / len(ct_gaps_50c) if ct_gaps_50c else 0
avg_wt_50c = sum(wt_gaps_50c) / len(wt_gaps_50c) if wt_gaps_50c else 0

print(f"  Win:  {results_50c['WIN']}/{total_50c} ({win_rate_50c:.1f}%) {'✅' if win_rate_50c >= 43 else '⚠️'}")
print(f"  Tie:  {results_50c['TIE']}/{total_50c}")
print(f"  Lose: {results_50c['LOSE']}/{total_50c}")
print(f"  Avg CT gap: {avg_ct_50c:+.2f}%")
print(f"  Avg WT gap: {avg_wt_50c:+.2f}%")

# 100C Results
print("\n100C (maxIteration = 25):")
total_100c = sum(results_100c.values())
win_rate_100c = (results_100c['WIN'] / total_100c * 100) if total_100c > 0 else 0
avg_ct_100c = sum(ct_gaps_100c) / len(ct_gaps_100c) if ct_gaps_100c else 0
avg_wt_100c = sum(wt_gaps_100c) / len(wt_gaps_100c) if wt_gaps_100c else 0

print(f"  Win:  {results_100c['WIN']}/{total_100c} ({win_rate_100c:.1f}%) {'✅' if win_rate_100c >= 12 else '⚠️'}")
print(f"  Tie:  {results_100c['TIE']}/{total_100c}")
print(f"  Lose: {results_100c['LOSE']}/{total_100c}")
print(f"  Avg CT gap: {avg_ct_100c:+.2f}%")
print(f"  Avg WT gap: {avg_wt_100c:+.2f}%")

# 200C Results
print("\n200C (maxEvaluation = 18,800,000):")
total_200c = sum(results_200c.values())
win_rate_200c = (results_200c['WIN'] / total_200c * 100) if total_200c > 0 else 0
avg_ct_200c = sum(ct_gaps_200c) / len(ct_gaps_200c) if ct_gaps_200c else 0
avg_wt_200c = sum(wt_gaps_200c) / len(wt_gaps_200c) if wt_gaps_200c else 0

print(f"  Win:  {results_200c['WIN']}/{total_200c} ({win_rate_200c:.1f}%) {'✅' if win_rate_200c >= 68 else '⚠️'}")
print(f"  Tie:  {results_200c['TIE']}/{total_200c}")
print(f"  Lose: {results_200c['LOSE']}/{total_200c}")
print(f"  Avg CT gap: {avg_ct_200c:+.2f}%")
print(f"  Avg WT gap: {avg_wt_200c:+.2f}%")

# Overall summary
print("\n" + "=" * 70)
total_wins = results_20c['WIN'] + results_50c['WIN'] + results_100c['WIN'] + results_200c['WIN']
total_instances = total_20c + total_50c + total_100c + total_200c
overall_win_rate = (total_wins / total_instances * 100) if total_instances > 0 else 0

print(f"\nOVERALL SUMMARY:")
print(f"  Total Win:  {total_wins}/{total_instances} ({overall_win_rate:.1f}%)")
print(f"  Total Tie:  {results_20c['TIE'] + results_50c['TIE'] + results_100c['TIE'] + results_200c['TIE']}/{total_instances}")
print(f"  Total Lose: {results_20c['LOSE'] + results_50c['LOSE'] + results_100c['LOSE'] + results_200c['LOSE']}/{total_instances}")

all_ct_gaps = ct_gaps_20c + ct_gaps_50c + ct_gaps_100c + ct_gaps_200c
all_wt_gaps = wt_gaps_20c + wt_gaps_50c + wt_gaps_100c + wt_gaps_200c
overall_ct_gap = sum(all_ct_gaps) / len(all_ct_gaps) if all_ct_gaps else 0
overall_wt_gap = sum(all_wt_gaps) / len(all_wt_gaps) if all_wt_gaps else 0

print(f"  Avg CT gap: {overall_ct_gap:+.2f}%")
print(f"  Avg WT gap: {overall_wt_gap:+.2f}%")

print("\n" + "=" * 70)
print("\nCONFIGURATION ANALYSIS:")
if win_rate_20c >= 75:
    print("  ✅ 20C: Meeting target (75%)")
else:
    print(f"  ⚠️  20C: Below target ({win_rate_20c:.1f}% < 75%)")

if win_rate_50c >= 43:
    print("  ✅ 50C: Meeting target (43.8%)")
else:
    print(f"  ⚠️  50C: Below target ({win_rate_50c:.1f}% < 43.8%)")

if win_rate_100c >= 12:
    print("  ✅ 100C: Meeting target (12.5%)")
else:
    print(f"  ⚠️  100C: Below target ({win_rate_100c:.1f}% < 12.5%)")

if win_rate_200c >= 68:
    print("  ✅ 200C: Meeting target (68.8%)")
else:
    print(f"  ⚠️  200C: Below target ({win_rate_200c:.1f}% < 68.8%)")

print("\n" + "=" * 70)
