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
        
        i = 6
        while i < len(lines) and len(solutions) < num_solutions:
            i += 1  # Skip route line
            if i < len(lines):
                parts = lines[i].strip().split()
                if len(parts) >= 2:
                    ct = float(parts[0])
                    wt = float(parts[1])
                    solutions.append([ct, wt])
            i += 1
            
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        
    return solutions

def normalize_front(front):
    """Normalize front to [0,1] range"""
    front = np.array(front)
    min_vals = front.min(axis=0)
    max_vals = front.max(axis=0)
    
    # Avoid division by zero
    ranges = max_vals - min_vals
    ranges[ranges == 0] = 1.0
    
    normalized = (front - min_vals) / ranges
    return normalized.tolist()

def calculate_hypervolume_2d(points):
    """Calculate normalized hypervolume for 2D points with ref point (1.1, 1.1)"""
    if len(points) == 0:
        return 0.0
    
    points = np.array(points)
    ref_point = np.array([1.1, 1.1])
    
    # Get Pareto front
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
    
    # Calculate HV
    hv = 0.0
    for i, point in enumerate(pareto_front):
        if i == 0:
            width = ref_point[0] - point[0]
            height = ref_point[1] - point[1]
            hv += width * height
        else:
            width = pareto_front[i-1][0] - point[0]
            height = ref_point[1] - point[1]
            hv += width * height
    
    return hv

def count_domination_instances(front_a_list, front_b_list):
    """Count instances where A wins/ties/loses against B"""
    wins = 0
    ties = 0
    loses = 0
    
    for front_a, front_b in zip(front_a_list, front_b_list):
        # Normalize together
        combined = front_a + front_b
        normalized = normalize_front(combined)
        norm_a = normalized[:len(front_a)]
        norm_b = normalized[len(front_a):]
        
        hv_a = calculate_hypervolume_2d(norm_a)
        hv_b = calculate_hypervolume_2d(norm_b)
        
        if hv_a > hv_b * 1.01:  # 1% threshold
            wins += 1
        elif hv_b > hv_a * 1.01:
            loses += 1
        else:
            ties += 1
    
    return wins, ties, loses

# Get all instances
result_files = sorted([f for f in os.listdir('result') if f.endswith('.txt')])
benchmark_files = set(os.listdir('benchmark'))

# Group by dataset size and grid
results_by_group = defaultdict(lambda: {'icahgs': [], 'benchmark': []})

for filename in result_files:
    if filename in benchmark_files:
        parts = filename.replace('.txt', '').split('.')
        if len(parts) == 3:
            size = int(parts[0])
            grid = int(parts[1])
            
            icahgs_sols = read_solutions_from_file(f'result/{filename}')
            benchmark_sols = read_solutions_from_file(f'benchmark/{filename}')
            
            if icahgs_sols and benchmark_sols:
                results_by_group[(size, grid)]['icahgs'].append(icahgs_sols)
                results_by_group[(size, grid)]['benchmark'].append(benchmark_sols)

# Calculate and print table
print("\n" + "=" * 120)
print("PERFORMANCE COMPARISON AMONG ALGORITHMS")
print("=" * 120)
print(f"{'Dataset':<10} {'GridSize':<10} {'HV':<10} {'GapHV(%)':<12} {'+/=/-':<10}")
print("-" * 120)

summary_by_size = defaultdict(lambda: {
    'hv_ratios': [],
    'gaps': [],
    'wins': 0,
    'ties': 0,
    'loses': 0
})

for (size, grid) in sorted(results_by_group.keys()):
    data = results_by_group[(size, grid)]
    icahgs_fronts = data['icahgs']
    benchmark_fronts = data['benchmark']
    
    # Calculate HV for each instance and average
    hv_ratios = []
    for icahgs_front, benchmark_front in zip(icahgs_fronts, benchmark_fronts):
        combined = icahgs_front + benchmark_front
        normalized = normalize_front(combined)
        
        norm_icahgs = normalized[:len(icahgs_front)]
        norm_benchmark = normalized[len(icahgs_front):]
        
        hv_icahgs = calculate_hypervolume_2d(norm_icahgs)
        hv_benchmark = calculate_hypervolume_2d(norm_benchmark)
        
        if hv_benchmark > 0:
            hv_ratios.append(hv_icahgs / hv_benchmark)
    
    avg_hv_ratio = np.mean(hv_ratios) if hv_ratios else 0.0
    gap_hv = (avg_hv_ratio - 1.0) * 100
    
    # Count domination
    wins, ties, loses = count_domination_instances(icahgs_fronts, benchmark_fronts)
    
    # Store for summary
    summary_by_size[size]['hv_ratios'].append(avg_hv_ratio)
    summary_by_size[size]['gaps'].append(gap_hv)
    summary_by_size[size]['wins'] += wins
    summary_by_size[size]['ties'] += ties
    summary_by_size[size]['loses'] += loses
    
    print(f"{size:<10} {grid:<10} {avg_hv_ratio:<10.2f} {gap_hv:<12.2f} {wins}/{ties}/{loses:<10}")

print("-" * 120)

# Print averages by size
for size in sorted(summary_by_size.keys()):
    data = summary_by_size[size]
    avg_hv = np.mean(data['hv_ratios'])
    avg_gap = np.mean(data['gaps'])
    
    print(f"Average {size}C {' ':<1} {avg_hv:<10.2f} {avg_gap:<12.2f} "
          f"{data['wins']}/{data['ties']}/{data['loses']}")

print("-" * 120)

# Overall summary
all_hv_ratios = []
all_gaps = []
total_wins = 0
total_ties = 0
total_loses = 0

for size in summary_by_size:
    data = summary_by_size[size]
    all_hv_ratios.extend(data['hv_ratios'])
    all_gaps.extend(data['gaps'])
    total_wins += data['wins']
    total_ties += data['ties']
    total_loses += data['loses']

overall_hv = np.mean(all_hv_ratios)
overall_gap = np.mean(all_gaps)

print(f"Summary    {' ':<10} {overall_hv:<10.2f} {overall_gap:<12.2f} {total_wins}/{total_ties}/{total_loses}")
print("=" * 120)
print()
