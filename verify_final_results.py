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

# Test with specific instances
test_instances = {
    '50C': ['50.10.1', '50.10.2', '50.20.1', '50.20.2'],
    '100C': ['100.10.1', '100.10.2', '100.20.1', '100.20.2']
}

print("=== VERIFICATION: result_final_hybrid vs benchmark ===\n")

for size, instances in test_instances.items():
    print(f"{size}:")
    wins = 0
    for instance in instances:
        result_file = f"result_final_hybrid/{instance}.txt"
        benchmark_file = f"benchmark/{instance}.txt"
        
        if os.path.exists(result_file) and os.path.exists(benchmark_file):
            result_front = parse_pareto_front(result_file)
            benchmark_front = parse_pareto_front(benchmark_file)
            outcome = compare_fronts(result_front, benchmark_front)
            print(f"  {instance}: {outcome}")
            if outcome == 'WIN':
                wins += 1
        else:
            print(f"  {instance}: FILE MISSING")
    print(f"  Win rate: {wins}/{len(instances)} = {wins/len(instances)*100:.1f}%\n")

print("\nNow compare with test_maxIteration (known 43.8% win):")
print("\nUsing test_maxIteration folder:")
for size, instances in test_instances.items():
    if size == '50C':
        print(f"{size}:")
        wins = 0
        for instance in instances:
            result_file = f"test_maxIteration/{instance}.txt"
            benchmark_file = f"benchmark/{instance}.txt"
            
            if os.path.exists(result_file) and os.path.exists(benchmark_file):
                result_front = parse_pareto_front(result_file)
                benchmark_front = parse_pareto_front(benchmark_file)
                outcome = compare_fronts(result_front, benchmark_front)
                print(f"  {instance}: {outcome}")
                if outcome == 'WIN':
                    wins += 1
        print(f"  Win rate: {wins}/{len(instances)} = {wins/len(instances)*100:.1f}%\n")
