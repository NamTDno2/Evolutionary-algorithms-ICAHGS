import os
import numpy as np
import matplotlib.pyplot as plt
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

# Group by dataset size
results_by_size = defaultdict(lambda: {
    'instances': [],
    'hv_icahgs': [],
    'hv_benchmark': [],
    'gap_hv': [],
    'domination': {'win': 0, 'tie': 0, 'lose': 0}
})

print("=" * 100)
print("COMPARING 60 INSTANCES: ICAHGS (Ideal Point Method) vs BENCHMARK")
print("=" * 100)
print()

for filename in sorted(result_files):
    if filename in benchmark_files:
        # Parse instance name
        parts = filename.replace('.txt', '').split('.')
        if len(parts) == 3:
            size = int(parts[0])
            
            # Read solutions
            icahgs_sols = read_solutions_from_file(f'result/{filename}')
            benchmark_sols = read_solutions_from_file(f'benchmark/{filename}')
            
            if icahgs_sols and benchmark_sols:
                # Calculate reference point
                all_points = icahgs_sols + benchmark_sols
                ref_point = [
                    max(p[0] for p in all_points) * 1.1,
                    max(p[1] for p in all_points) * 1.1
                ]
                
                # Calculate HV
                hv_icahgs = calculate_hypervolume_2d(icahgs_sols, ref_point)
                hv_benchmark = calculate_hypervolume_2d(benchmark_sols, ref_point)
                
                # Calculate Gap
                if hv_benchmark > 0:
                    gap_hv = ((hv_icahgs - hv_benchmark) / hv_benchmark) * 100
                else:
                    gap_hv = 0.0
                
                # Count domination
                dom_win, dom_tie, dom_lose = count_domination(icahgs_sols, benchmark_sols)
                
                # Store results
                results_by_size[size]['instances'].append(filename)
                results_by_size[size]['hv_icahgs'].append(hv_icahgs)
                results_by_size[size]['hv_benchmark'].append(hv_benchmark)
                results_by_size[size]['gap_hv'].append(gap_hv)
                results_by_size[size]['domination']['win'] += dom_win
                results_by_size[size]['domination']['tie'] += dom_tie
                results_by_size[size]['domination']['lose'] += dom_lose

# Print summary by size
print("=" * 100)
print("SUMMARY BY DATASET SIZE:")
print("=" * 100)
print(f"{'Size':<10} {'Instances':<12} {'Avg HV Gap(%)':<15} {'Win/Tie/Lose':<20} {'Win Rate(%)':<15}")
print("-" * 100)

all_data = []
for size in sorted(results_by_size.keys()):
    data = results_by_size[size]
    n_inst = len(data['instances'])
    avg_gap = np.mean(data['gap_hv'])
    
    dom = data['domination']
    total_dom = dom['win'] + dom['tie'] + dom['lose']
    win_rate = (dom['win'] / total_dom * 100) if total_dom > 0 else 0
    
    print(f"{size}C      {n_inst:<12} {avg_gap:>12.2f}%  "
          f"{dom['win']}/{dom['tie']}/{dom['lose']:<20} {win_rate:>12.1f}%")
    
    all_data.append({
        'size': size,
        'n_instances': n_inst,
        'avg_gap': avg_gap,
        'win': dom['win'],
        'tie': dom['tie'],
        'lose': dom['lose'],
        'win_rate': win_rate
    })

# Overall statistics
total_win = sum(d['win'] for d in all_data)
total_tie = sum(d['tie'] for d in all_data)
total_lose = sum(d['lose'] for d in all_data)
total_dom = total_win + total_tie + total_lose
overall_win_rate = (total_win / total_dom * 100) if total_dom > 0 else 0
overall_gap = np.mean([d['avg_gap'] for d in all_data])

print("-" * 100)
print(f"{'OVERALL':<10} {sum(d['n_instances'] for d in all_data):<12} {overall_gap:>12.2f}%  "
      f"{total_win}/{total_tie}/{total_lose:<20} {overall_win_rate:>12.1f}%")
print("=" * 100)

# ==================== VISUALIZATION ====================

# 1. Domination Comparison Chart
fig1, ax1 = plt.subplots(figsize=(12, 7))

sizes = [d['size'] for d in all_data]
wins = [d['win'] for d in all_data]
ties = [d['tie'] for d in all_data]
loses = [d['lose'] for d in all_data]

x = np.arange(len(sizes))
width = 0.6

p1 = ax1.bar(x, wins, width, label='Our > Benchmark', color='#2ecc71', edgecolor='black', linewidth=1.5)
p2 = ax1.bar(x, ties, width, bottom=wins, label='Equal', color='#f39c12', edgecolor='black', linewidth=1.5)
p3 = ax1.bar(x, loses, width, bottom=np.array(wins)+np.array(ties), 
             label='Benchmark > Our', color='#e74c3c', edgecolor='black', linewidth=1.5)

ax1.set_xlabel('Dataset Size', fontsize=14, fontweight='bold')
ax1.set_ylabel('Number of Solutions', fontsize=14, fontweight='bold')
ax1.set_title('Domination Comparison: ICAHGS vs Benchmark\n(60 Instances Total)', 
              fontsize=16, fontweight='bold', pad=20)
ax1.set_xticks(x)
ax1.set_xticklabels([f'{s}C' for s in sizes], fontsize=12)
ax1.legend(fontsize=11, loc='upper left')
ax1.grid(axis='y', alpha=0.3, linestyle='--')

# Add value labels
for i, (w, t, l) in enumerate(zip(wins, ties, loses)):
    if w > 0:
        ax1.text(i, w/2, str(w), ha='center', va='center', fontsize=11, fontweight='bold', color='white')
    if t > 0:
        ax1.text(i, w + t/2, str(t), ha='center', va='center', fontsize=11, fontweight='bold', color='white')
    if l > 0:
        ax1.text(i, w + t + l/2, str(l), ha='center', va='center', fontsize=11, fontweight='bold', color='white')

plt.tight_layout()
plt.savefig('domination_comparison_60instances.png', dpi=300, bbox_inches='tight')
print("\n✅ Domination chart saved: domination_comparison_60instances.png")

# 2. Hypervolume Gap Chart
fig2, (ax2a, ax2b) = plt.subplots(1, 2, figsize=(16, 6))

# Left: HV Gap by size
gaps = [d['avg_gap'] for d in all_data]
colors = ['#2ecc71' if g > 0 else '#e74c3c' for g in gaps]
bars = ax2a.bar(range(len(sizes)), gaps, color=colors, edgecolor='black', linewidth=1.5, alpha=0.8)
ax2a.axhline(y=0, color='blue', linestyle='--', linewidth=2, label='Baseline (0%)')
ax2a.set_xticks(range(len(sizes)))
ax2a.set_xticklabels([f'{s}C' for s in sizes], fontsize=12)
ax2a.set_ylabel('Hypervolume Gap (%)', fontsize=14, fontweight='bold')
ax2a.set_title('Average HV Gap by Dataset Size', fontsize=15, fontweight='bold', pad=15)
ax2a.legend(fontsize=11)
ax2a.grid(axis='y', alpha=0.3, linestyle='--')

# Add values on bars
for i, (bar, val) in enumerate(zip(bars, gaps)):
    y_pos = val + (3 if val > 0 else -3)
    va = 'bottom' if val > 0 else 'top'
    ax2a.text(bar.get_x() + bar.get_width()/2, y_pos, f'{val:.1f}%', 
             ha='center', va=va, fontsize=11, fontweight='bold')

# Right: Win rate by size
win_rates = [d['win_rate'] for d in all_data]
bars2 = ax2b.bar(range(len(sizes)), win_rates, color='#3498db', edgecolor='black', linewidth=1.5, alpha=0.8)
ax2b.axhline(y=50, color='red', linestyle='--', linewidth=2, label='50% Threshold')
ax2b.set_xticks(range(len(sizes)))
ax2b.set_xticklabels([f'{s}C' for s in sizes], fontsize=12)
ax2b.set_ylabel('Win Rate (%)', fontsize=14, fontweight='bold')
ax2b.set_title('Win Rate by Dataset Size', fontsize=15, fontweight='bold', pad=15)
ax2b.set_ylim(0, 100)
ax2b.legend(fontsize=11)
ax2b.grid(axis='y', alpha=0.3, linestyle='--')

# Add values on bars
for bar, val in zip(bars2, win_rates):
    ax2b.text(bar.get_x() + bar.get_width()/2, val + 2, f'{val:.1f}%', 
             ha='center', va='bottom', fontsize=11, fontweight='bold')

plt.tight_layout()
plt.savefig('HV_60instances.png', dpi=300, bbox_inches='tight')
print("✅ Hypervolume chart saved: HV_60instances.png")

# 3. Print final summary
print("\n" + "=" * 100)
print("FINAL SUMMARY:")
print("=" * 100)
print(f"Total instances compared: 60")
print(f"Overall HV Gap: {overall_gap:+.2f}% ({'BETTER' if overall_gap > 0 else 'WORSE'} than benchmark)")
print(f"Overall Win Rate: {overall_win_rate:.1f}%")
print(f"Domination: {total_win} wins, {total_tie} ties, {total_lose} losses")
print("=" * 100)

plt.show()
