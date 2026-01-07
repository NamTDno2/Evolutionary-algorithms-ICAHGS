import os

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

# All 50C and 100C instances
instances_50c = [
    "50.10.1", "50.10.2", "50.10.3", "50.10.4",
    "50.20.1", "50.20.2", "50.20.3", "50.20.4",
    "50.30.1", "50.30.2", "50.30.3", "50.30.4",
    "50.40.1", "50.40.2", "50.40.3", "50.40.4"
]

instances_100c = [
    "100.10.1", "100.10.2", "100.10.3", "100.10.4",
    "100.20.1", "100.20.2", "100.20.3", "100.20.4",
    "100.30.1", "100.30.2", "100.30.3", "100.30.4",
    "100.40.1", "100.40.2", "100.40.3", "100.40.4"
]

print("=" * 70)
print("COMPARISON: result_final_hybrid (NEW) vs test_maxIteration (OLD)")
print("=" * 70)

# Compare 50C
print("\n50C RESULTS:")
print(f"{'Instance':<12} {'result_final_hybrid':<20} {'test_maxIteration':<20}")
print("-" * 60)

wins_new = 0
wins_old = 0

for instance in instances_50c:
    # New results
    result_file = f"result_final_hybrid/{instance}.txt"
    benchmark_file = f"benchmark/{instance}.txt"
    
    if os.path.exists(result_file) and os.path.exists(benchmark_file):
        result_front = parse_pareto_front(result_file)
        benchmark_front = parse_pareto_front(benchmark_file)
        outcome_new = compare_fronts(result_front, benchmark_front)
        if outcome_new == 'WIN':
            wins_new += 1
    else:
        outcome_new = 'MISSING'
    
    # Old results
    old_file = f"test_maxIteration/{instance}.txt"
    if os.path.exists(old_file) and os.path.exists(benchmark_file):
        old_front = parse_pareto_front(old_file)
        benchmark_front = parse_pareto_front(benchmark_file)
        outcome_old = compare_fronts(old_front, benchmark_front)
        if outcome_old == 'WIN':
            wins_old += 1
    else:
        outcome_old = 'MISSING'
    
    match = "✅" if outcome_new == outcome_old else "❌"
    print(f"{instance:<12} {outcome_new:<20} {outcome_old:<20} {match}")

print("-" * 60)
print(f"{'TOTAL':<12} {wins_new}/16 ({wins_new/16*100:.1f}%){' ':<7} {wins_old}/16 ({wins_old/16*100:.1f}%)")

# Compare 100C
print("\n100C RESULTS:")
print(f"{'Instance':<12} {'result_final_hybrid':<20} {'test_maxIteration':<20}")
print("-" * 60)

wins_new_100 = 0
wins_old_100 = 0

for instance in instances_100c:
    # New results
    result_file = f"result_final_hybrid/{instance}.txt"
    benchmark_file = f"benchmark/{instance}.txt"
    
    if os.path.exists(result_file) and os.path.exists(benchmark_file):
        result_front = parse_pareto_front(result_file)
        benchmark_front = parse_pareto_front(benchmark_file)
        outcome_new = compare_fronts(result_front, benchmark_front)
        if outcome_new == 'WIN':
            wins_new_100 += 1
    else:
        outcome_new = 'MISSING'
    
    # Old results
    old_file = f"test_maxIteration/{instance}.txt"
    if os.path.exists(old_file) and os.path.exists(benchmark_file):
        old_front = parse_pareto_front(old_file)
        benchmark_front = parse_pareto_front(benchmark_file)
        outcome_old = compare_fronts(old_front, benchmark_front)
        if outcome_old == 'WIN':
            wins_old_100 += 1
    else:
        outcome_old = 'MISSING'
    
    match = "✅" if outcome_new == outcome_old else "❌"
    print(f"{instance:<12} {outcome_new:<20} {outcome_old:<20} {match}")

print("-" * 60)
print(f"{'TOTAL':<12} {wins_new_100}/16 ({wins_new_100/16*100:.1f}%){' ':<7} {wins_old_100}/16 ({wins_old_100/16*100:.1f}%)")

print("\n" + "=" * 70)
print("\nCONCLUSION:")
print(f"  50C: result_final_hybrid = {wins_new}/16 ({wins_new/16*100:.1f}%)")
print(f"  50C: test_maxIteration = {wins_old}/16 ({wins_old/16*100:.1f}%)")
print(f"  100C: result_final_hybrid = {wins_new_100}/16 ({wins_new_100/16*100:.1f}%)")
print(f"  100C: test_maxIteration = {wins_old_100}/16 ({wins_old_100/16*100:.1f}%)")

if wins_new == wins_old and wins_new_100 == wins_old_100:
    print("\n  ✅ IDENTICAL RESULTS - Files are the same!")
else:
    print("\n  ❌ DIFFERENT RESULTS - Something changed!")
