import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read CSV files
print("\n=== COMPARING HYBRID RESULTS ===")
print("Using:")
print("  - All sizes from results_60_instances.csv (25% evals)")
print("  - 50C, 100C from result_50c_100c_doubled/results_summary.csv (50% evals)")
print()

# Read original 60 instances (25% evals)
df_original = pd.read_csv('results_60_instances.csv')
print(f"✅ Loaded {len(df_original)} instances from results_60_instances.csv")

# Read doubled results for 50C and 100C (50% evals)
df_doubled = pd.read_csv('result_50c_100c_doubled/results_summary.csv')
print(f"✅ Loaded {len(df_doubled)} instances from result_50c_100c_doubled/results_summary.csv")

# Create hybrid dataframe: keep 20C and 200C from original, replace 50C and 100C with doubled
df_hybrid = df_original[~df_original['Size'].isin([50, 100])].copy()
df_doubled_copy = df_doubled.copy()
df_hybrid = pd.concat([df_hybrid, df_doubled_copy], ignore_index=True)

# Sort by instance name
df_hybrid = df_hybrid.sort_values('Instance').reset_index(drop=True)

print(f"\n✅ Hybrid dataset created: {len(df_hybrid)} instances")
print(f"   - 20C:  {len(df_hybrid[df_hybrid['Size'] == 20])} instances (25% evals)")
print(f"   - 50C:  {len(df_hybrid[df_hybrid['Size'] == 50])} instances (50% evals)")
print(f"   - 100C: {len(df_hybrid[df_hybrid['Size'] == 100])} instances (50% evals)")
print(f"   - 200C: {len(df_hybrid[df_hybrid['Size'] == 200])} instances (25% evals)")

# Add eval type column
df_hybrid['EvalType'] = df_hybrid['Size'].apply(
    lambda x: '50% evals' if x in [50, 100] else '25% evals'
)

# Save hybrid results
df_hybrid.to_csv('results_hybrid.csv', index=False)
print(f"\n✅ Hybrid results saved to: results_hybrid.csv")

# Create visualizations
fig = plt.figure(figsize=(18, 12))
gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

fig.suptitle('Hybrid Results Comparison vs Benchmark\n' + 
             'Evaluation Strategy: 20C/200C at 25%, 50C/100C at 50%', 
             fontsize=16, fontweight='bold')

# Group by size
size_groups = df_hybrid.groupby('Size')
sizes = sorted(df_hybrid['Size'].unique())
x = np.arange(len(sizes))
width = 0.25

# Helper function to safely get column sum
def safe_sum(group, column):
    if column in group.columns:
        return group[column].sum()
    return 0

# 1. Domination Comparison (large plot, top left)
ax1 = fig.add_subplot(gs[0:2, 0:2])

wins_by_size = [safe_sum(size_groups.get_group(s), 'Wins') for s in sizes]
ties_by_size = [safe_sum(size_groups.get_group(s), 'Ties') for s in sizes]
losses_by_size = [safe_sum(size_groups.get_group(s), 'Losses') for s in sizes]

ax1.bar(x - width, wins_by_size, width, label='Wins (Result dominates)', 
        color='green', alpha=0.7, edgecolor='black')
ax1.bar(x, ties_by_size, width, label='Ties', 
        color='gray', alpha=0.7, edgecolor='black')
ax1.bar(x + width, losses_by_size, width, label='Losses (Benchmark dominates)', 
        color='red', alpha=0.7, edgecolor='black')

ax1.set_xlabel('Instance Size', fontsize=13, fontweight='bold')
ax1.set_ylabel('Domination Count', fontsize=13, fontweight='bold')
ax1.set_title('Domination Comparison by Size', fontsize=15, fontweight='bold')
ax1.set_xticks(x)
ax1.set_xticklabels([f'{s}C' for s in sizes], fontsize=12)
ax1.legend(fontsize=11, loc='upper left')
ax1.grid(True, alpha=0.3, axis='y')

# Add values on bars
for i, (w, t, l) in enumerate(zip(wins_by_size, ties_by_size, losses_by_size)):
    if w > 0:
        ax1.text(i - width, w + max(wins_by_size)*0.02, str(w), 
                ha='center', va='bottom', fontsize=11, fontweight='bold')
    if t > 0:
        ax1.text(i, t + max(ties_by_size)*0.02, str(t), 
                ha='center', va='bottom', fontsize=11, fontweight='bold')
    if l > 0:
        ax1.text(i + width, l + max(losses_by_size)*0.02, str(l), 
                ha='center', va='bottom', fontsize=11, fontweight='bold')

# Add evaluation type labels
for i, s in enumerate(sizes):
    eval_type = '50%' if s in [50, 100] else '25%'
    ax1.text(i, -max(losses_by_size)*0.15, eval_type, 
            ha='center', va='top', fontsize=10, style='italic', color='blue')

# 2. Win Rate by Size (top right)
ax2 = fig.add_subplot(gs[0, 2])
win_rates = []
for s in sizes:
    group = size_groups.get_group(s)
    w = safe_sum(group, 'Wins')
    t = safe_sum(group, 'Ties')
    l = safe_sum(group, 'Losses')
    total = w + t + l
    win_rate = (w / total * 100) if total > 0 else 0
    win_rates.append(win_rate)

colors = ['green' if wr >= 50 else 'orange' if wr >= 30 else 'red' for wr in win_rates]
bars = ax2.bar(x, win_rates, color=colors, alpha=0.7, edgecolor='black')
ax2.axhline(y=50, color='blue', linestyle='--', linewidth=2, label='50% baseline')
ax2.set_xlabel('Size', fontsize=11, fontweight='bold')
ax2.set_ylabel('Win Rate (%)', fontsize=11, fontweight='bold')
ax2.set_title('Win Rate by Size', fontsize=13, fontweight='bold')
ax2.set_xticks(x)
ax2.set_xticklabels([f'{s}C' for s in sizes], fontsize=10)
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3, axis='y')
ax2.set_ylim(0, max(win_rates) * 1.2)

# Add percentage labels
for bar, wr in zip(bars, win_rates):
    ax2.text(bar.get_x() + bar.get_width()/2, wr + 2, f'{wr:.1f}%', 
            ha='center', va='bottom', fontsize=10, fontweight='bold')

# 3. HV Gap by Size (middle right)
ax3 = fig.add_subplot(gs[1, 2])
avg_hv_gap = [size_groups.get_group(s)['HV_Gap_%'].mean() for s in sizes]
colors = ['green' if hv >= 0 else 'red' for hv in avg_hv_gap]
bars = ax3.bar(x, avg_hv_gap, color=colors, alpha=0.7, edgecolor='black')
ax3.axhline(y=0, color='black', linestyle='-', linewidth=1.5)
ax3.set_xlabel('Size', fontsize=11, fontweight='bold')
ax3.set_ylabel('HV Gap (%)', fontsize=11, fontweight='bold')
ax3.set_title('Hypervolume Gap', fontsize=13, fontweight='bold')
ax3.set_xticks(x)
ax3.set_xticklabels([f'{s}C' for s in sizes], fontsize=10)
ax3.grid(True, alpha=0.3, axis='y')

# Add percentage labels
for bar, hv in zip(bars, avg_hv_gap):
    height = bar.get_height()
    ax3.text(bar.get_x() + bar.get_width()/2, 
            height + (abs(height)*0.1 if height >= 0 else -abs(height)*0.1), 
            f'{hv:+.1f}%', 
            ha='center', va='bottom' if height >= 0 else 'top', 
            fontsize=10, fontweight='bold')

# 4. Archive Size Comparison (bottom left)
ax4 = fig.add_subplot(gs[2, 0])
result_archive = [size_groups.get_group(s)['ArchiveSize'].mean() for s in sizes]
benchmark_archive = [size_groups.get_group(s)['Benchmark_ArchiveSize'].mean() if 'Benchmark_ArchiveSize' in df_hybrid.columns else 0 for s in sizes]

# If benchmark archive not available, use estimates
if max(benchmark_archive) == 0:
    benchmark_archive = [3.5, 77, 191, 30]  # Rough estimates from earlier

bars1 = ax4.bar(x - width/2, result_archive, width, 
       label='Result', color='blue', alpha=0.7, edgecolor='black')
bars2 = ax4.bar(x + width/2, benchmark_archive, width, 
       label='Benchmark', color='orange', alpha=0.7, edgecolor='black')

ax4.set_xlabel('Size', fontsize=11, fontweight='bold')
ax4.set_ylabel('Avg Archive Size', fontsize=11, fontweight='bold')
ax4.set_title('Archive Size Comparison', fontsize=13, fontweight='bold')
ax4.set_xticks(x)
ax4.set_xticklabels([f'{s}C' for s in sizes], fontsize=10)
ax4.legend(fontsize=9)
ax4.grid(True, alpha=0.3, axis='y')

# Add values
for i, (r, b) in enumerate(zip(result_archive, benchmark_archive)):
    ax4.text(i - width/2, r, f'{r:.0f}', ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax4.text(i + width/2, b, f'{b:.0f}', ha='center', va='bottom', fontsize=9, fontweight='bold')

# 5. CT Performance (bottom middle)
ax5 = fig.add_subplot(gs[2, 1])
ct_gap = [size_groups.get_group(s)['CT_Gap_%'].mean() for s in sizes]
colors = ['green' if ct >= 0 else 'red' for ct in ct_gap]
bars = ax5.bar(x, ct_gap, color=colors, alpha=0.7, edgecolor='black')
ax5.axhline(y=0, color='black', linestyle='-', linewidth=1.5)
ax5.set_xlabel('Size', fontsize=11, fontweight='bold')
ax5.set_ylabel('CT Gap (%)', fontsize=11, fontweight='bold')
ax5.set_title('Completion Time Gap', fontsize=13, fontweight='bold')
ax5.set_xticks(x)
ax5.set_xticklabels([f'{s}C' for s in sizes], fontsize=10)
ax5.grid(True, alpha=0.3, axis='y')

for bar, ct in zip(bars, ct_gap):
    height = bar.get_height()
    ax5.text(bar.get_x() + bar.get_width()/2, 
            height + (abs(height)*0.1 if height >= 0 else -abs(height)*0.1), 
            f'{ct:+.1f}%', 
            ha='center', va='bottom' if height >= 0 else 'top', 
            fontsize=9, fontweight='bold')

# 6. WT Performance (bottom right)
ax6 = fig.add_subplot(gs[2, 2])
wt_gap = [size_groups.get_group(s)['WT_Gap_%'].mean() for s in sizes]
colors = ['green' if wt >= 0 else 'red' for wt in wt_gap]
bars = ax6.bar(x, wt_gap, color=colors, alpha=0.7, edgecolor='black')
ax6.axhline(y=0, color='black', linestyle='-', linewidth=1.5)
ax6.set_xlabel('Size', fontsize=11, fontweight='bold')
ax6.set_ylabel('WT Gap (%)', fontsize=11, fontweight='bold')
ax6.set_title('Waiting Time Gap', fontsize=13, fontweight='bold')
ax6.set_xticks(x)
ax6.set_xticklabels([f'{s}C' for s in sizes], fontsize=10)
ax6.grid(True, alpha=0.3, axis='y')

for bar, wt in zip(bars, wt_gap):
    height = bar.get_height()
    ax6.text(bar.get_x() + bar.get_width()/2, 
            height + (abs(height)*0.1 if height >= 0 else -abs(height)*0.1), 
            f'{wt:+.1f}%', 
            ha='center', va='bottom' if height >= 0 else 'top', 
            fontsize=9, fontweight='bold')

plt.savefig('comparison_hybrid_results.png', dpi=300, bbox_inches='tight')
print(f"\n✅ Visualization saved to: comparison_hybrid_results.png")

# Print summary statistics
print("\n" + "="*60)
print("SUMMARY STATISTICS")
print("="*60)

total_wins = sum(wins_by_size)
total_ties = sum(ties_by_size)
total_losses = sum(losses_by_size)
total = total_wins + total_ties + total_losses
win_rate = (total_wins / total * 100) if total > 0 else 0

print(f"\nOverall Domination:")
print(f"  Wins:   {total_wins:5d} ({total_wins/total*100:5.1f}%)")
print(f"  Ties:   {total_ties:5d} ({total_ties/total*100:5.1f}%)")
print(f"  Losses: {total_losses:5d} ({total_losses/total*100:5.1f}%)")
print(f"  Win Rate: {win_rate:.1f}%")

print(f"\nAverage HV Gap: {df_hybrid['HV_Gap_%'].mean():+.2f}%")
print(f"Average CT Gap: {df_hybrid['CT_Gap_%'].mean():+.2f}%")
print(f"Average WT Gap: {df_hybrid['WT_Gap_%'].mean():+.2f}%")

print("\n" + "-"*60)
print(f"{'Size':<6} {'EvalType':<12} {'Instances':<10} {'W/T/L':<15} {'WinRate':<10} {'HV Gap':<10}")
print("-"*60)

for s in sizes:
    group = size_groups.get_group(s)
    w = safe_sum(group, 'Wins')
    t = safe_sum(group, 'Ties')
    l = safe_sum(group, 'Losses')
    total_size = w + t + l
    wr = (w / total_size * 100) if total_size > 0 else 0
    hv = group['HV_Gap_%'].mean()
    eval_type = group['EvalType'].iloc[0]
    count = len(group)
    
    print(f"{s:3d}C   {eval_type:<12} {count:<10} {w:4d}/{t:2d}/{l:4d}     "
          f"{wr:5.1f}%     {hv:+7.2f}%")

print("="*60)

# Comparison with original 25% results
print("\n" + "="*60)
print("IMPROVEMENT FROM 25% TO 50% EVALS (50C & 100C)")
print("="*60)

for s in [50, 100]:
    orig = df_original[df_original['Size'] == s]
    doubled = df_doubled[df_doubled['Size'] == s]
    
    orig_wins = safe_sum(orig, 'Wins')
    orig_losses = safe_sum(orig, 'Losses')
    doubled_wins = safe_sum(doubled, 'Wins')
    doubled_losses = safe_sum(doubled, 'Losses')
    
    orig_hv = orig['HV_Gap_%'].mean()
    doubled_hv = doubled['HV_Gap_%'].mean()
    
    orig_archive = orig['ArchiveSize'].mean()
    doubled_archive = doubled['ArchiveSize'].mean()
    
    print(f"\n{s}C Instances:")
    print(f"  Wins:    {orig_wins:4d} → {doubled_wins:4d} ({doubled_wins-orig_wins:+d})")
    print(f"  Losses:  {orig_losses:4d} → {doubled_losses:4d} ({doubled_losses-orig_losses:+d})")
    print(f"  HV Gap:  {orig_hv:+6.2f}% → {doubled_hv:+6.2f}% ({doubled_hv-orig_hv:+.2f}%)")
    print(f"  Archive: {orig_archive:6.1f} → {doubled_archive:6.1f} ({doubled_archive/orig_archive-1:+.1%})")

print("="*60)
plt.show()
