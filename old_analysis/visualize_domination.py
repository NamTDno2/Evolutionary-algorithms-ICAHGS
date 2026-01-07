import os
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

def read_solutions(filepath):
    """Read CT, WT from file"""
    solutions = []
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        if len(lines) < 7:
            return solutions
        
        num_sols = int(lines[5].strip())
        i = 6
        count = 0
        while i < len(lines) and count < num_sols:
            i += 1  # Skip route
            if i < len(lines):
                parts = lines[i].strip().split()
                if len(parts) >= 2:
                    solutions.append([float(parts[0]), float(parts[1])])
                    count += 1
            i += 1
    except:
        pass
    return solutions

def dominates(a, b):
    """Check if solution a dominates solution b"""
    return (a[0] <= b[0] and a[1] < b[1]) or (a[0] < b[0] and a[1] <= b[1])

def coverage_metric(A, B):
    """
    Calculate C(A, B) = |{b in B : exists a in A, a dominates b}| / |B|
    Returns the fraction of solutions in B that are dominated by at least one solution in A
    """
    if not B:
        return 0.0
    
    dominated_count = 0
    for b in B:
        for a in A:
            if dominates(a, b):
                dominated_count += 1
                break  # b is dominated, no need to check other a's
    
    return dominated_count / len(B)

# Read all files and calculate coverage metrics
result_files = sorted([f for f in os.listdir('result') if f.endswith('.txt')])
data_by_size = defaultdict(lambda: {
    'c_icahgs_bench': [],  # C(ICAHGS, Benchmark)
    'c_bench_icahgs': [],  # C(Benchmark, ICAHGS)
    'icahgs_dominates': 0,
    'benchmark_dominates': 0
})

for fname in result_files:
    if os.path.exists(f'benchmark/{fname}'):
        parts = fname.replace('.txt', '').split('.')
        if len(parts) == 3:
            size = int(parts[0])
            
            ic_sols = read_solutions(f'result/{fname}')
            bm_sols = read_solutions(f'benchmark/{fname}')
            
            if ic_sols and bm_sols:
                # Calculate coverage metrics
                c_ib = coverage_metric(ic_sols, bm_sols)  # How much ICAHGS dominates Benchmark
                c_bi = coverage_metric(bm_sols, ic_sols)  # How much Benchmark dominates ICAHGS
                
                data_by_size[size]['c_icahgs_bench'].append(c_ib)
                data_by_size[size]['c_bench_icahgs'].append(c_bi)
                
                # Count domination
                if c_ib > c_bi:
                    data_by_size[size]['icahgs_dominates'] += 1
                elif c_bi > c_ib:
                    data_by_size[size]['benchmark_dominates'] += 1

# Prepare data for visualization
sizes = [20, 50, 100, 200]
coverage_icahgs = []  # C(ICAHGS, Benchmark) - ICAHGS dominates Benchmark
coverage_bench = []   # C(Benchmark, ICAHGS) - Benchmark dominates ICAHGS
domination_icahgs = []
domination_bench = []
coverage_diff = []

for size in sizes:
    d = data_by_size[size]
    avg_c_ib = np.mean(d['c_icahgs_bench']) * 100  # Convert to percentage
    avg_c_bi = np.mean(d['c_bench_icahgs']) * 100
    
    coverage_icahgs.append(avg_c_ib)
    coverage_bench.append(avg_c_bi)
    domination_icahgs.append(d['icahgs_dominates'])
    domination_bench.append(d['benchmark_dominates'])
    coverage_diff.append(avg_c_ib - avg_c_bi)

# Create figure with 4 subplots (2x2)
fig = plt.figure(figsize=(18, 12))

# 1. Coverage Metric by Customer Size (top-left)
ax1 = plt.subplot(2, 2, 1)
x = np.arange(len(sizes))
width = 0.35

bars1 = ax1.bar(x - width/2, coverage_icahgs, width, label='C(ICAHGS, Benchmark)', 
                color='steelblue', alpha=0.8, edgecolor='black')
bars2 = ax1.bar(x + width/2, coverage_bench, width, label='C(Benchmark, ICAHGS)', 
                color='orange', alpha=0.8, edgecolor='black')

# Add horizontal line at 50%
ax1.axhline(y=50, color='red', linestyle='--', linewidth=1.5, alpha=0.7)

ax1.set_xlabel('Customer Size', fontweight='bold', fontsize=12)
ax1.set_ylabel('Coverage (%)', fontweight='bold', fontsize=12)
ax1.set_title('Coverage Metric by Customer Size\n(Higher = More Domination)', 
              fontweight='bold', fontsize=13, pad=15)
ax1.set_xticks(x)
ax1.set_xticklabels([f'{s}C' for s in sizes], fontsize=11)
ax1.legend(loc='upper right', fontsize=10)
ax1.grid(axis='y', alpha=0.3)
ax1.set_ylim(0, max(max(coverage_icahgs), max(coverage_bench)) * 1.15)

# Add value labels on bars
for bars in [bars1, bars2]:
    for bar in bars:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + 1.5,
                f'{height:.1f}%', ha='center', va='bottom', fontsize=10, fontweight='bold')

# 2. Domination Count by Customer Size (top-right)
ax2 = plt.subplot(2, 2, 2)

# Calculate ties for each size
ties_by_size = []
for size in sizes:
    d = data_by_size[size]
    
    # Count total instances for this size
    if size == 20:
        total_instances_size = 12  # 3 grids × 4 files
    else:
        total_instances_size = 16  # 4 grids × 4 files
    
    ties = total_instances_size - d['icahgs_dominates'] - d['benchmark_dominates']
    ties_by_size.append(ties)

# Create stacked bar chart
width = 0.6
bars3 = ax2.bar(x, domination_icahgs, width, label='ICAHGS Dominates', 
                color='green', alpha=0.8, edgecolor='black')
bars4 = ax2.bar(x, domination_bench, width, bottom=domination_icahgs,
                label='Benchmark Dominates', color='red', alpha=0.8, edgecolor='black')
bars5 = ax2.bar(x, ties_by_size, width, 
                bottom=[i+j for i,j in zip(domination_icahgs, domination_bench)],
                label='Ties', color='gray', alpha=0.8, edgecolor='black')

ax2.set_xlabel('Customer Size', fontweight='bold', fontsize=12)
ax2.set_ylabel('Number of Instances', fontweight='bold', fontsize=12)
ax2.set_title('Domination Count by Customer Size\n(Total: 20C=12, 50C/100C/200C=16)', 
              fontweight='bold', fontsize=13, pad=15)
ax2.set_xticks(x)
ax2.set_xticklabels([f'{s}C' for s in sizes], fontsize=11)
ax2.legend(loc='upper right', fontsize=10)
ax2.grid(axis='y', alpha=0.3)
ax2.set_ylim(0, 18)

# Add value labels on bars
for i, (ic, bm, tie) in enumerate(zip(domination_icahgs, domination_bench, ties_by_size)):
    # ICAHGS wins
    if ic > 0:
        ax2.text(x[i], ic/2, f'{int(ic)}', ha='center', va='center', 
                fontsize=10, fontweight='bold', color='white')
    # Benchmark wins
    if bm > 0:
        ax2.text(x[i], ic + bm/2, f'{int(bm)}', ha='center', va='center',
                fontsize=10, fontweight='bold', color='white')
    # Ties
    if tie > 0:
        ax2.text(x[i], ic + bm + tie/2, f'{int(tie)}', ha='center', va='center',
                fontsize=10, fontweight='bold', color='black')

# 3. Win Rate Distribution (Coverage Metric) (bottom-left)
ax3 = plt.subplot(2, 2, 3)
total_instances = 60
total_icahgs_wins = sum(domination_icahgs)
total_benchmark_wins = sum(domination_bench)
total_ties = total_instances - total_icahgs_wins - total_benchmark_wins

categories = ['ICAHGS', 'Benchmark', 'Tie']
counts = [total_icahgs_wins, total_benchmark_wins, total_ties]
colors = ['steelblue', 'orange', 'gray']
percentages = [(c/total_instances*100) for c in counts]

bars5 = ax3.bar(categories, counts, color=colors, alpha=0.8, edgecolor='black', width=0.6)

ax3.set_ylabel('Number of Instances', fontweight='bold', fontsize=12)
ax3.set_title(f'Win Rate Distribution (Coverage Metric)\nICAHGS: {total_icahgs_wins}/{total_instances} = {percentages[0]:.1f}%', 
              fontweight='bold', fontsize=13, pad=15)
ax3.grid(axis='y', alpha=0.3)
ax3.set_ylim(0, max(counts) * 1.25)
ax3.tick_params(axis='x', labelsize=11)

# Add value labels
for bar, count, pct in zip(bars5, counts, percentages):
    height = bar.get_height()
    ax3.text(bar.get_x() + bar.get_width()/2., height + 1,
            f'{count}\n({pct:.1f}%)', ha='center', va='bottom', fontsize=11, fontweight='bold')

# 4. Coverage Difference (bottom-right)
ax4 = plt.subplot(2, 2, 4)
colors4 = ['green' if d > 0 else 'red' if d < 0 else 'gray' for d in coverage_diff]
bars6 = ax4.bar(x, coverage_diff, color=colors4, alpha=0.8, edgecolor='black', width=0.6)

ax4.axhline(y=0, color='black', linestyle='-', linewidth=2)
ax4.set_xlabel('Customer Size', fontweight='bold', fontsize=12)
ax4.set_ylabel('Coverage Difference (%)', fontweight='bold', fontsize=12)
ax4.set_title('C(ICAHGS,Bench) - C(Bench,ICAHGS)\n(Positive = ICAHGS Better)', 
              fontweight='bold', fontsize=13, pad=15)
ax4.set_xticks(x)
ax4.set_xticklabels([f'{s}C' for s in sizes], fontsize=11)
ax4.grid(axis='y', alpha=0.3)

# Adjust y-axis limits to prevent label overlap
y_max = max(abs(min(coverage_diff)), max(coverage_diff))
ax4.set_ylim(-y_max * 1.2, y_max * 1.2)

# Add value labels
for bar, val in zip(bars6, coverage_diff):
    height = bar.get_height()
    y_pos = height + 3 if height > 0 else height - 3
    va = 'bottom' if height > 0 else 'top'
    ax4.text(bar.get_x() + bar.get_width()/2., y_pos,
            f'{val:+.1f}%', ha='center', va=va, fontsize=11, fontweight='bold')

plt.tight_layout(pad=3.0)
plt.savefig('domination_comparison.png', dpi=300, bbox_inches='tight')
print("✓ Domination comparison charts saved to: domination_comparison.png")
print("\nSummary:")
print(f"  Total instances: {total_instances}")
print(f"  ICAHGS dominates: {total_icahgs_wins} ({percentages[0]:.1f}%)")
print(f"  Benchmark dominates: {total_benchmark_wins} ({percentages[1]:.1f}%)")
print(f"  Ties: {total_ties} ({percentages[2]:.1f}%)")
print("\nCoverage by size:")
for i, size in enumerate(sizes):
    print(f"  {size}C: C(I,B)={coverage_icahgs[i]:.1f}%, C(B,I)={coverage_bench[i]:.1f}%, Diff={coverage_diff[i]:+.1f}%")

plt.show()
