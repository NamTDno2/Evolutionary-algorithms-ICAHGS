import os
import numpy as np
from collections import defaultdict

def read_solutions_from_file(filepath):
    """Read solutions (CT, WT) from result file"""
    solutions = []
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            
        if len(lines) < 6:
            return solutions
            
        num_solutions = int(lines[5].strip())
        
        # Read solutions (each solution: route line + CT WT line)
        i = 6
        while i < len(lines) and len(solutions) < num_solutions:
            # Skip route line
            i += 1
            if i < len(lines):
                # Parse CT WT line
                parts = lines[i].strip().split()
                if len(parts) >= 2:
                    ct = float(parts[0])
                    wt = float(parts[1])
                    solutions.append([ct, wt])
            i += 1
            
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        
    return solutions

def calculate_hypervolume_2d(points, ref_point):
    """Calculate hypervolume for 2D points (CT, WT)"""
    if len(points) == 0:
        return 0.0
    
    # Convert to numpy array
    points = np.array(points)
    
    # Filter dominated points and points beyond reference
    pareto_front = []
    for p in points:
        if p[0] <= ref_point[0] and p[1] <= ref_point[1]:
            is_dominated = False
            for other in points:
                if (other[0] <= p[0] and other[1] <= p[1] and 
                    (other[0] < p[0] or other[1] < p[1])):
                    is_dominated = True
                    break
            if not is_dominated:
                pareto_front.append(p)
    
    if len(pareto_front) == 0:
        return 0.0
    
    # Sort by first objective
    pareto_front = sorted(pareto_front, key=lambda x: x[0])
    
    # Calculate hypervolume
    hv = 0.0
    prev_x = 0.0
    
    for i, point in enumerate(pareto_front):
        if i == 0:
            width = ref_point[0] - point[0]
            height = ref_point[1] - point[1]
            hv += width * height
            prev_x = point[0]
        else:
            width = pareto_front[i-1][0] - point[0]
            height = ref_point[1] - point[1]
            hv += width * height
    
    return hv

def calculate_igd(obtained_front, reference_front):
    """Calculate Inverted Generational Distance"""
    if len(obtained_front) == 0 or len(reference_front) == 0:
        return float('inf')
    
    obtained = np.array(obtained_front)
    reference = np.array(reference_front)
    
    distances = []
    for ref_point in reference:
        min_dist = float('inf')
        for obt_point in obtained:
            dist = np.sqrt(np.sum((ref_point - obt_point) ** 2))
            min_dist = min(min_dist, dist)
        distances.append(min_dist)
    
    return np.mean(distances)

def count_domination(front_a, front_b):
    """Count how many solutions in A dominate/dominated by B
    Returns: (A dominates B, Equal, B dominates A)"""
    a_dominates = 0
    equal = 0
    b_dominates = 0
    
    for a in front_a:
        best_relation = 0  # 0: no relation, 1: a dominates, -1: b dominates
        
        for b in front_b:
            # Check domination
            a_better = (a[0] <= b[0] and a[1] <= b[1] and 
                       (a[0] < b[0] or a[1] < b[1]))
            b_better = (b[0] <= a[0] and b[1] <= a[1] and 
                       (b[0] < a[0] or b[1] < a[1]))
            
            if a_better:
                best_relation = max(best_relation, 1)
            elif b_better:
                best_relation = min(best_relation, -1)
        
        if best_relation == 1:
            a_dominates += 1
        elif best_relation == -1:
            b_dominates += 1
        else:
            equal += 1
    
    return a_dominates, equal, b_dominates

# Get all instances
result_files = [f for f in os.listdir('result') if f.endswith('.txt')]
benchmark_files = [f for f in os.listdir('benchmark') if f.endswith('.txt')]

# Group by dataset size and grid
results_by_group = defaultdict(lambda: {'icahgs': [], 'benchmark': []})

for filename in result_files:
    if filename in benchmark_files:
        # Parse instance name (e.g., "20.5.1.txt" -> size=20, grid=5)
        parts = filename.replace('.txt', '').split('.')
        if len(parts) == 3:
            size = int(parts[0])
            grid = int(parts[1])
            
            # Read solutions
            icahgs_sols = read_solutions_from_file(f'result/{filename}')
            benchmark_sols = read_solutions_from_file(f'benchmark/{filename}')
            
            if icahgs_sols and benchmark_sols:
                results_by_group[(size, grid)]['icahgs'].append(icahgs_sols)
                results_by_group[(size, grid)]['benchmark'].append(benchmark_sols)

# Calculate metrics for each group
print("=" * 100)
print("PERFORMANCE COMPARISON: ICAHGS vs BENCHMARK")
print("=" * 100)
print()
print(f"{'Dataset':<10} {'GridSize':<10} {'HV(ICAHGS)':<15} {'HV(Bench)':<15} {'GapHV(%)':<12} {'+/=/- vs Bench':<15} {'IGD':<12}")
print("-" * 100)

summary_data = defaultdict(lambda: {
    'hv_values': [],
    'gap_values': [],
    'win': 0, 'tie': 0, 'lose': 0,
    'igd_values': []
})

for (size, grid), data in sorted(results_by_group.items()):
    icahgs_fronts = data['icahgs']
    benchmark_fronts = data['benchmark']
    
    # Combine all solutions for this group
    all_icahgs = []
    all_benchmark = []
    for front in icahgs_fronts:
        all_icahgs.extend(front)
    for front in benchmark_fronts:
        all_benchmark.extend(front)
    
    if not all_icahgs or not all_benchmark:
        continue
    
    # Calculate reference point (worst point + margin)
    all_points = all_icahgs + all_benchmark
    ref_point = [
        max(p[0] for p in all_points) * 1.1,
        max(p[1] for p in all_points) * 1.1
    ]
    
    # Calculate HV
    hv_icahgs = calculate_hypervolume_2d(all_icahgs, ref_point)
    hv_benchmark = calculate_hypervolume_2d(all_benchmark, ref_point)
    
    # Calculate Gap
    if hv_benchmark > 0:
        gap_hv = ((hv_icahgs - hv_benchmark) / hv_benchmark) * 100
    else:
        gap_hv = 0.0
    
    # Count domination
    dom_win, dom_tie, dom_lose = count_domination(all_icahgs, all_benchmark)
    
    # Calculate IGD
    igd = calculate_igd(all_icahgs, all_benchmark)
    
    # Store for summary
    summary_data[size]['hv_values'].append(hv_icahgs / hv_benchmark if hv_benchmark > 0 else 1.0)
    summary_data[size]['gap_values'].append(gap_hv)
    summary_data[size]['win'] += dom_win
    summary_data[size]['tie'] += dom_tie
    summary_data[size]['lose'] += dom_lose
    summary_data[size]['igd_values'].append(igd)
    
    print(f"{size:<10} {grid:<10} {hv_icahgs:<15.2f} {hv_benchmark:<15.2f} {gap_hv:<12.2f} "
          f"{dom_win}/{dom_tie}/{dom_lose:<15} {igd:<12.2f}")

print("-" * 100)
print("SUMMARY BY DATASET SIZE:")
print("-" * 100)

total_win = 0
total_tie = 0
total_lose = 0
all_gaps = []

for size in sorted(summary_data.keys()):
    data = summary_data[size]
    avg_hv_ratio = np.mean(data['hv_values'])
    avg_gap = np.mean(data['gap_values'])
    avg_igd = np.mean(data['igd_values'])
    
    total_win += data['win']
    total_tie += data['tie']
    total_lose += data['lose']
    all_gaps.append(avg_gap)
    
    print(f"{size:<10} Avg HV Ratio: {avg_hv_ratio:.4f}  Avg Gap: {avg_gap:>7.2f}%  "
          f"Dom: {data['win']}/{data['tie']}/{data['lose']}  Avg IGD: {avg_igd:.2f}")

print("-" * 100)
print(f"OVERALL:   Avg Gap: {np.mean(all_gaps):>7.2f}%  Total Dom: {total_win}/{total_tie}/{total_lose}")
print("=" * 100)
