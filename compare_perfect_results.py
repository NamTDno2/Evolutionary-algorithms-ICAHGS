"""
Visualize Perfect Results vs Benchmark - Best Solution Comparison
Based on result_final_perfect folder
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def read_best_solution_from_file(filepath):
    """Read only the BEST solution (first solution after header) from file"""
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            
        # Skip header (6 lines) and route line, get first CT/WT
        if len(lines) >= 8:
            ct_wt_line = lines[7].strip().split()
            if len(ct_wt_line) >= 2:
                ct = float(ct_wt_line[0])
                wt = float(ct_wt_line[1])
                return ct, wt
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
    
    return None, None

def compare_solutions(result_ct, result_wt, bench_ct, bench_wt):
    """Compare two solutions and return domination status"""
    # Result dominates benchmark
    result_dominates = (result_ct <= bench_ct and result_wt <= bench_wt and 
                       (result_ct < bench_ct or result_wt < bench_wt))
    
    # Benchmark dominates result
    bench_dominates = (bench_ct <= result_ct and bench_wt <= result_wt and 
                      (bench_ct < result_ct or bench_wt < result_wt))
    
    if result_dominates:
        return 'win'
    elif bench_dominates:
        return 'loss'
    else:
        return 'tie'

# Get all benchmark instances
benchmark_files = sorted([f for f in os.listdir('benchmark') if f.endswith('.txt')])
result_folder = 'result_final_perfect'

results = []

print("\n" + "="*80)
print("COMPARING PERFECT RESULTS vs BENCHMARK")
print("  Source: result_final_perfect/")
print("  Config: 50% maxEvaluation, numEmpires=3")
print("="*80)

for bench_file in benchmark_files:
    instance_name = bench_file.replace('.txt', '')
    parts = instance_name.split('.')
    
    if len(parts) != 3:
        continue
    
    size = int(parts[0])
    
    # Read benchmark best solution
    bench_ct, bench_wt = read_best_solution_from_file(f'benchmark/{bench_file}')
    
    if bench_ct is None:
        continue
    
    # Get result solution
    result_file = f'{result_folder}/{bench_file}'
    if not os.path.exists(result_file):
        print(f"⚠️  Missing: {result_file}")
        continue
    
    result_ct, result_wt = read_best_solution_from_file(result_file)
    
    if result_ct is None:
        continue
    
    # Compare solutions
    status = compare_solutions(result_ct, result_wt, bench_ct, bench_wt)
    
    # Calculate gaps
    ct_gap = ((result_ct - bench_ct) / bench_ct * 100) if bench_ct > 0 else 0
    wt_gap = ((result_wt - bench_wt) / bench_wt * 100) if bench_wt > 0 else 0
    
    results.append({
        'Instance': instance_name,
        'Size': size,
        'Result_CT': result_ct,
        'Result_WT': result_wt,
        'Bench_CT': bench_ct,
        'Bench_WT': bench_wt,
        'CT_Gap_%': ct_gap,
        'WT_Gap_%': wt_gap,
        'Status': status
    })

# Create DataFrame
df = pd.DataFrame(results)

# Aggregate by size
size_groups = df.groupby('Size')
sizes = sorted(df['Size'].unique())

wins = []
ties = []
losses = []

for s in sizes:
    group = size_groups.get_group(s)
    w = len(group[group['Status'] == 'win'])
    t = len(group[group['Status'] == 'tie'])
    l = len(group[group['Status'] == 'loss'])
    wins.append(w)
    ties.append(t)
    losses.append(l)
    
    print(f"\n{s}C:")
    print(f"  Total: {len(group)} instances")
    print(f"  Win:   {w:2d} ({w/len(group)*100:5.1f}%)")
    print(f"  Tie:   {t:2d} ({t/len(group)*100:5.1f}%)")
    print(f"  Loss:  {l:2d} ({l/len(group)*100:5.1f}%)")
    print(f"  CT Gap:  {group['CT_Gap_%'].mean():+7.2f}%")
    print(f"  WT Gap:  {group['WT_Gap_%'].mean():+7.2f}%")

# Create visualization
fig, axes = plt.subplots(2, 2, figsize=(18, 14))
fig.suptitle('Perfect Results vs Benchmark - Best Solution Comparison', 
             fontsize=16, fontweight='bold', y=0.995)

x = np.arange(len(sizes))
width = 0.25

# 1. Win/Tie/Loss Distribution
ax = axes[0, 0]
bars1 = ax.bar(x - width, wins, width, label='Win', color='green', alpha=0.7, edgecolor='black')
bars2 = ax.bar(x, ties, width, label='Tie', color='gray', alpha=0.7, edgecolor='black')
bars3 = ax.bar(x + width, losses, width, label='Loss', color='red', alpha=0.7, edgecolor='black')

ax.set_xlabel('Instance Size', fontsize=12, fontweight='bold')
ax.set_ylabel('Count', fontsize=12, fontweight='bold')
ax.set_title('Win/Tie/Loss Distribution', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes])
ax.legend()
ax.grid(True, alpha=0.3, axis='y')

# Add values on bars
for i, (w, t, l) in enumerate(zip(wins, ties, losses)):
    if w > 0:
        ax.text(i - width, w, str(w), ha='center', va='bottom', fontweight='bold')
    if t > 0:
        ax.text(i, t, str(t), ha='center', va='bottom', fontweight='bold')
    if l > 0:
        ax.text(i + width, l, str(l), ha='center', va='bottom', fontweight='bold')

# 2. Win Rate
ax = axes[0, 1]
win_rates = [(w / (w + t + l) * 100) if (w + t + l) > 0 else 0 
             for w, t, l in zip(wins, ties, losses)]

colors = ['green' if wr >= 50 else 'orange' if wr >= 30 else 'red' for wr in win_rates]
bars = ax.bar(x, win_rates, color=colors, alpha=0.7, edgecolor='black')
ax.axhline(y=50, color='blue', linestyle='--', linewidth=2, label='50% threshold')
ax.axhline(y=30, color='orange', linestyle='--', linewidth=1.5, alpha=0.5, label='30% threshold')

ax.set_xlabel('Instance Size', fontsize=12, fontweight='bold')
ax.set_ylabel('Win Rate (%)', fontsize=12, fontweight='bold')
ax.set_title('Win Rate by Size', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes])
ax.legend()
ax.grid(True, alpha=0.3, axis='y')
ax.set_ylim(0, max(win_rates) * 1.3 if win_rates else 100)

for bar, wr in zip(bars, win_rates):
    ax.text(bar.get_x() + bar.get_width()/2, wr + 3, f'{wr:.1f}%', 
            ha='center', va='bottom', fontweight='bold')

# 3. CT Gap
ax = axes[1, 0]
ct_gaps = [size_groups.get_group(s)['CT_Gap_%'].mean() for s in sizes]
colors = ['green' if ct <= 0 else 'red' for ct in ct_gaps]
bars = ax.bar(x, ct_gaps, color=colors, alpha=0.7, edgecolor='black')
ax.axhline(y=0, color='black', linestyle='-', linewidth=1.5)

ax.set_xlabel('Instance Size', fontsize=12, fontweight='bold')
ax.set_ylabel('Average CT Gap (%)', fontsize=12, fontweight='bold')
ax.set_title('Completion Time Gap', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes])
ax.grid(True, alpha=0.3, axis='y')

for bar, ct in zip(bars, ct_gaps):
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, 
            height + (abs(height)*0.15 if height > 0 else -abs(height)*0.15), 
            f'{ct:+.1f}%', 
            ha='center', va='bottom' if height > 0 else 'top', 
            fontweight='bold')

# 4. WT Gap
ax = axes[1, 1]
wt_gaps = [size_groups.get_group(s)['WT_Gap_%'].mean() for s in sizes]
colors = ['green' if wt <= 0 else 'red' for wt in wt_gaps]
bars = ax.bar(x, wt_gaps, color=colors, alpha=0.7, edgecolor='black')
ax.axhline(y=0, color='black', linestyle='-', linewidth=1.5)

ax.set_xlabel('Instance Size', fontsize=12, fontweight='bold')
ax.set_ylabel('Average WT Gap (%)', fontsize=12, fontweight='bold')
ax.set_title('Waiting Time Gap', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes])
ax.grid(True, alpha=0.3, axis='y')

for bar, wt in zip(bars, wt_gaps):
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, 
            height + (abs(height)*0.15 if height > 0 else -abs(height)*0.15), 
            f'{wt:+.1f}%', 
            ha='center', va='bottom' if height > 0 else 'top', 
            fontweight='bold')

plt.tight_layout()
plt.savefig('perfect_solution_comparison.png', dpi=300, bbox_inches='tight')
print(f"\n✅ Visualization saved to: perfect_solution_comparison.png")

# Print summary
print("\n" + "="*80)
print("SUMMARY STATISTICS")
print("="*80)

total_wins = sum(wins)
total_ties = sum(ties)
total_losses = sum(losses)
total = total_wins + total_ties + total_losses
win_rate = (total_wins / total * 100) if total > 0 else 0

print(f"\nOverall Performance:")
print(f"  Wins:   {total_wins:3d}/{total} ({total_wins/total*100:5.1f}%)")
print(f"  Ties:   {total_ties:3d}/{total} ({total_ties/total*100:5.1f}%)")
print(f"  Losses: {total_losses:3d}/{total} ({total_losses/total*100:5.1f}%)")
print(f"  Win Rate: {win_rate:.1f}%")

print(f"\nAverage Gaps:")
print(f"  CT: {df['CT_Gap_%'].mean():+.2f}%")
print(f"  WT: {df['WT_Gap_%'].mean():+.2f}%")

print("\n" + "-"*80)
print(f"{'Size':<8} {'Instances':<10} {'W/T/L':<12} {'WinRate':<10} {'CT Gap':<10} {'WT Gap':<10}")
print("-"*80)

for i, s in enumerate(sizes):
    group = size_groups.get_group(s)
    count = len(group)
    w, t, l = wins[i], ties[i], losses[i]
    wr = win_rates[i]
    ct = ct_gaps[i]
    wt = wt_gaps[i]
    
    print(f"{s:3d}C     {count:<10} {w:2d}/{t:2d}/{l:2d}      "
          f"{wr:5.1f}%     {ct:+6.1f}%    {wt:+6.1f}%")

print("="*80)

# Save to CSV
csv_file = 'perfect_results_comparison.csv'
df.to_csv(csv_file, index=False)
print(f"\n✅ Detailed results saved to: {csv_file}")

plt.show()
