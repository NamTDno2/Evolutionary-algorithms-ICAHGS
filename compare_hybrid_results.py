import pandas as pd
import matplotlib.pyplot as plt
import os
from pathlib import Path

def read_pareto_front(file_path):
    """Read Pareto front from result file"""
    solutions = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        
        # Skip header (first 6 lines)
        i = 6
        while i < len(lines):
            # Route line (skip it)
            if i < len(lines) - 1:
                # Objective values
                parts = lines[i+1].strip().split()
                if len(parts) >= 2:
                    ct = float(parts[0])
                    wt = float(parts[1])
                    solutions.append((ct, wt))
                i += 2
            else:
                break
    
    return solutions

def dominates(sol1, sol2):
    """Check if sol1 dominates sol2 (minimization)"""
    ct1, wt1 = sol1
    ct2, wt2 = sol2
    return (ct1 <= ct2 and wt1 <= wt2) and (ct1 < ct2 or wt1 < wt2)

def compare_pareto_fronts(result_front, benchmark_front):
    """Compare two Pareto fronts"""
    result_dominates = 0
    benchmark_dominates = 0
    ties = 0
    
    for r_sol in result_front:
        for b_sol in benchmark_front:
            if dominates(r_sol, b_sol):
                result_dominates += 1
            elif dominates(b_sol, r_sol):
                benchmark_dominates += 1
    
    # Check for ties (non-dominated)
    for r_sol in result_front:
        is_dominated = False
        for b_sol in benchmark_front:
            if dominates(b_sol, r_sol):
                is_dominated = True
                break
        if not is_dominated:
            ties += 1
    
    return result_dominates, ties, benchmark_dominates

def calculate_hypervolume_2d(pareto_front, ref_point):
    """Calculate 2D hypervolume for a Pareto front"""
    if not pareto_front:
        return 0.0
    
    # Sort by first objective (CT)
    sorted_front = sorted(pareto_front, key=lambda x: x[0])
    
    # Calculate hypervolume
    hv = 0.0
    ref_ct, ref_wt = ref_point
    
    for i, (ct, wt) in enumerate(sorted_front):
        if ct >= ref_ct or wt >= ref_wt:
            continue
        
        width = ref_ct - ct
        
        if i < len(sorted_front) - 1:
            height = ref_wt - sorted_front[i+1][1]
        else:
            height = ref_wt - wt
        
        hv += width * height
        ref_ct = ct
    
    return hv

# Get all instances
instances = []
for file in os.listdir('benchmark'):
    if file.endswith('.txt'):
        instance_name = file.replace('.txt', '')
        size = int(instance_name.split('.')[0])
        instances.append((instance_name, size))

instances.sort()

results = []

print("\n=== COMPARING HYBRID RESULTS ===")
print("Using:")
print("  - 20C, 200C from result/ (25% evals)")
print("  - 50C, 100C from result_50c_100c_doubled/ (50% evals)")
print("\n")

for instance_name, size in instances:
    # Determine which folder to use
    if size in [50, 100]:
        result_file = f'result_50c_100c_doubled/{instance_name}.txt'
        eval_type = '50% evals'
    else:
        result_file = f'result/{instance_name}.txt'
        eval_type = '25% evals'
    
    benchmark_file = f'benchmark/{instance_name}.txt'
    
    # Check if result file exists
    if not os.path.exists(result_file):
        print(f"⚠️ Missing result: {instance_name}")
        continue
    
    # Read Pareto fronts
    result_front = read_pareto_front(result_file)
    benchmark_front = read_pareto_front(benchmark_file)
    
    # Compare fronts
    wins, ties, losses = compare_pareto_fronts(result_front, benchmark_front)
    
    # Calculate hypervolumes
    # Use reference point as max of both fronts
    all_solutions = result_front + benchmark_front
    ref_ct = max(sol[0] for sol in all_solutions) * 1.1
    ref_wt = max(sol[1] for sol in all_solutions) * 1.1
    ref_point = (ref_ct, ref_wt)
    
    hv_result = calculate_hypervolume_2d(result_front, ref_point)
    hv_benchmark = calculate_hypervolume_2d(benchmark_front, ref_point)
    hv_gap = ((hv_result - hv_benchmark) / hv_benchmark * 100) if hv_benchmark > 0 else 0
    
    results.append({
        'Instance': instance_name,
        'Size': size,
        'EvalType': eval_type,
        'Result_Solutions': len(result_front),
        'Benchmark_Solutions': len(benchmark_front),
        'Wins': wins,
        'Ties': ties,
        'Losses': losses,
        'HV_Result': hv_result,
        'HV_Benchmark': hv_benchmark,
        'HV_Gap_%': hv_gap
    })
    
    print(f"[{instance_name}] {eval_type:10s} | Archive: {len(result_front):4d} | W/T/L: {wins:4d}/{ties:2d}/{losses:4d} | HV: {hv_gap:+7.2f}%")

# Create DataFrame
df = pd.DataFrame(results)

# Save to CSV
df.to_csv('results_hybrid_comparison.csv', index=False)
print(f"\n✅ Results saved to: results_hybrid_comparison.csv")

# Create visualizations
fig, axes = plt.subplots(2, 2, figsize=(16, 12))
fig.suptitle('Hybrid Results Comparison vs Benchmark\n(20C/200C: 25% evals, 50C/100C: 50% evals)', 
             fontsize=16, fontweight='bold')

# Group by size
size_groups = df.groupby('Size')

# 1. Domination Comparison by Size
ax = axes[0, 0]
sizes = sorted(df['Size'].unique())
x = range(len(sizes))
width = 0.25

wins_by_size = [size_groups.get_group(s)['Wins'].sum() for s in sizes]
ties_by_size = [size_groups.get_group(s)['Ties'].sum() for s in sizes]
losses_by_size = [size_groups.get_group(s)['Losses'].sum() for s in sizes]

ax.bar([i - width for i in x], wins_by_size, width, label='Wins (Result dominates)', color='green', alpha=0.7)
ax.bar(x, ties_by_size, width, label='Ties', color='gray', alpha=0.7)
ax.bar([i + width for i in x], losses_by_size, width, label='Losses (Benchmark dominates)', color='red', alpha=0.7)

ax.set_xlabel('Instance Size', fontsize=12)
ax.set_ylabel('Domination Count', fontsize=12)
ax.set_title('Domination Comparison by Size', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes])
ax.legend()
ax.grid(True, alpha=0.3)

# Add values on bars
for i, (w, t, l) in enumerate(zip(wins_by_size, ties_by_size, losses_by_size)):
    ax.text(i - width, w, str(w), ha='center', va='bottom', fontsize=9)
    ax.text(i, t, str(t), ha='center', va='bottom', fontsize=9)
    ax.text(i + width, l, str(l), ha='center', va='bottom', fontsize=9)

# 2. Win Rate by Size
ax = axes[0, 1]
win_rates = []
for s in sizes:
    group = size_groups.get_group(s)
    total = group['Wins'].sum() + group['Ties'].sum() + group['Losses'].sum()
    win_rate = (group['Wins'].sum() / total * 100) if total > 0 else 0
    win_rates.append(win_rate)

colors = ['green' if wr >= 50 else 'orange' if wr >= 30 else 'red' for wr in win_rates]
bars = ax.bar(x, win_rates, color=colors, alpha=0.7)
ax.axhline(y=50, color='blue', linestyle='--', label='50% threshold')
ax.set_xlabel('Instance Size', fontsize=12)
ax.set_ylabel('Win Rate (%)', fontsize=12)
ax.set_title('Win Rate by Size', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes])
ax.legend()
ax.grid(True, alpha=0.3)

# Add percentage labels
for i, (bar, wr) in enumerate(zip(bars, win_rates)):
    ax.text(bar.get_x() + bar.get_width()/2, wr, f'{wr:.1f}%', 
            ha='center', va='bottom', fontsize=10, fontweight='bold')

# 3. HV Gap by Size
ax = axes[1, 0]
avg_hv_gap = [size_groups.get_group(s)['HV_Gap_%'].mean() for s in sizes]
colors = ['green' if hv >= 0 else 'red' for hv in avg_hv_gap]
bars = ax.bar(x, avg_hv_gap, color=colors, alpha=0.7)
ax.axhline(y=0, color='black', linestyle='-', linewidth=1)
ax.set_xlabel('Instance Size', fontsize=12)
ax.set_ylabel('Average HV Gap (%)', fontsize=12)
ax.set_title('Hypervolume Gap by Size', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes])
ax.grid(True, alpha=0.3)

# Add percentage labels
for bar, hv in zip(bars, avg_hv_gap):
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, height, f'{hv:+.1f}%', 
            ha='center', va='bottom' if height >= 0 else 'top', 
            fontsize=10, fontweight='bold')

# 4. Archive Size Comparison
ax = axes[1, 1]
result_archive = [size_groups.get_group(s)['Result_Solutions'].mean() for s in sizes]
benchmark_archive = [size_groups.get_group(s)['Benchmark_Solutions'].mean() for s in sizes]

ax.bar([i - width/2 for i in x], result_archive, width, 
       label='Result Archive', color='blue', alpha=0.7)
ax.bar([i + width/2 for i in x], benchmark_archive, width, 
       label='Benchmark Archive', color='orange', alpha=0.7)

ax.set_xlabel('Instance Size', fontsize=12)
ax.set_ylabel('Average Archive Size', fontsize=12)
ax.set_title('Archive Size Comparison', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes])
ax.legend()
ax.grid(True, alpha=0.3)

# Add values on bars
for i, (r, b) in enumerate(zip(result_archive, benchmark_archive)):
    ax.text(i - width/2, r, f'{r:.0f}', ha='center', va='bottom', fontsize=9)
    ax.text(i + width/2, b, f'{b:.0f}', ha='center', va='bottom', fontsize=9)

plt.tight_layout()
plt.savefig('comparison_hybrid_results.png', dpi=300, bbox_inches='tight')
print(f"✅ Visualization saved to: comparison_hybrid_results.png")

# Summary statistics
print("\n=== SUMMARY STATISTICS ===")
total_wins = df['Wins'].sum()
total_ties = df['Ties'].sum()
total_losses = df['Losses'].sum()
total = total_wins + total_ties + total_losses
win_rate = (total_wins / total * 100) if total > 0 else 0

print(f"\nOverall Domination:")
print(f"  Wins:   {total_wins:5d} ({total_wins/total*100:.1f}%)")
print(f"  Ties:   {total_ties:5d} ({total_ties/total*100:.1f}%)")
print(f"  Losses: {total_losses:5d} ({total_losses/total*100:.1f}%)")
print(f"  Win Rate: {win_rate:.1f}%")

print(f"\nAverage HV Gap: {df['HV_Gap_%'].mean():+.2f}%")

print("\nBy Size:")
for size in sizes:
    group = size_groups.get_group(size)
    w = group['Wins'].sum()
    t = group['Ties'].sum()
    l = group['Losses'].sum()
    total_size = w + t + l
    wr = (w / total_size * 100) if total_size > 0 else 0
    hv = group['HV_Gap_%'].mean()
    eval_type = group['EvalType'].iloc[0]
    
    print(f"  {size:3d}C ({eval_type:10s}): W/T/L={w:4d}/{t:2d}/{l:4d} | "
          f"Win Rate={wr:5.1f}% | HV Gap={hv:+7.2f}%")

plt.show()
