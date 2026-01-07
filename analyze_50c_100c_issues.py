import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read CSV
df = pd.read_csv('results_60_instances.csv')

# Separate by size
df_20 = df[df['Size'] == 20]
df_50 = df[df['Size'] == 50]
df_100 = df[df['Size'] == 100]
df_200 = df[df['Size'] == 200]

# Create comprehensive analysis figure
fig = plt.figure(figsize=(18, 12))

# 1. Archive Size Distribution
ax1 = plt.subplot(3, 3, 1)
sizes = [20, 50, 100, 200]
avg_archives = [df_20['ArchiveSize'].mean(), df_50['ArchiveSize'].mean(), 
                df_100['ArchiveSize'].mean(), df_200['ArchiveSize'].mean()]
colors = ['green', 'red', 'orange', 'yellow']
bars1 = ax1.bar(range(len(sizes)), avg_archives, color=colors, edgecolor='black', linewidth=1.5)
ax1.set_xticks(range(len(sizes)))
ax1.set_xticklabels([f'{s}C' for s in sizes])
ax1.set_ylabel('Average Archive Size', fontweight='bold')
ax1.set_title('Archive Size by Dataset Size', fontweight='bold')
ax1.grid(axis='y', alpha=0.3)
for bar, val in zip(bars1, avg_archives):
    ax1.text(bar.get_x() + bar.get_width()/2, val + 5, f'{val:.1f}', 
             ha='center', va='bottom', fontweight='bold')

# 2. Execution Time per Evaluation
ax2 = plt.subplot(3, 3, 2)
time_per_eval = [
    (df_20['Time'].sum() / df_20['Evaluations'].sum()) * 1000,
    (df_50['Time'].sum() / df_50['Evaluations'].sum()) * 1000,
    (df_100['Time'].sum() / df_100['Evaluations'].sum()) * 1000,
    (df_200['Time'].sum() / df_200['Evaluations'].sum()) * 1000
]
bars2 = ax2.bar(range(len(sizes)), time_per_eval, color=colors, edgecolor='black', linewidth=1.5)
ax2.set_xticks(range(len(sizes)))
ax2.set_xticklabels([f'{s}C' for s in sizes])
ax2.set_ylabel('Time per 1000 Evals (seconds)', fontweight='bold')
ax2.set_title('Computational Cost per Evaluation', fontweight='bold')
ax2.grid(axis='y', alpha=0.3)
for bar, val in zip(bars2, time_per_eval):
    ax2.text(bar.get_x() + bar.get_width()/2, val + 0.05, f'{val:.2f}', 
             ha='center', va='bottom', fontweight='bold', fontsize=9)

# 3. Archive Size Distribution (Box Plot)
ax3 = plt.subplot(3, 3, 3)
archive_data = [df_20['ArchiveSize'].values, df_50['ArchiveSize'].values,
                df_100['ArchiveSize'].values, df_200['ArchiveSize'].values]
bp = ax3.boxplot(archive_data, labels=[f'{s}C' for s in sizes], patch_artist=True)
for patch, color in zip(bp['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.6)
ax3.set_ylabel('Archive Size', fontweight='bold')
ax3.set_title('Archive Size Variability', fontweight='bold')
ax3.grid(axis='y', alpha=0.3)

# 4. Evaluation Coverage Analysis
ax4 = plt.subplot(3, 3, 4)
eval_limits = [65000, 547500, 10305000, 18800000]
avg_evals = [df_20['Evaluations'].mean(), df_50['Evaluations'].mean(),
             df_100['Evaluations'].mean(), df_200['Evaluations'].mean()]
coverage = [(avg/limit)*100 for avg, limit in zip(avg_evals, eval_limits)]
bars4 = ax4.bar(range(len(sizes)), coverage, color=colors, edgecolor='black', linewidth=1.5)
ax4.axhline(y=100, color='blue', linestyle='--', linewidth=2, label='Target')
ax4.set_xticks(range(len(sizes)))
ax4.set_xticklabels([f'{s}C' for s in sizes])
ax4.set_ylabel('Evaluation Coverage (%)', fontweight='bold')
ax4.set_title('Actual vs Target Evaluations', fontweight='bold')
ax4.legend()
ax4.grid(axis='y', alpha=0.3)
for bar, val in zip(bars4, coverage):
    ax4.text(bar.get_x() + bar.get_width()/2, val + 2, f'{val:.1f}%', 
             ha='center', va='bottom', fontweight='bold')

# 5. Archive Size per Instance (50C and 100C)
ax5 = plt.subplot(3, 3, 5)
ax5.plot(df_50['Instance'], df_50['ArchiveSize'], 'ro-', label='50C', markersize=8, linewidth=2)
ax5.set_xlabel('Instance', fontweight='bold')
ax5.set_ylabel('Archive Size', fontweight='bold')
ax5.set_title('50C: Archive Size Pattern', fontweight='bold', color='red')
ax5.grid(True, alpha=0.3)
ax5.tick_params(axis='x', rotation=45)
plt.setp(ax5.xaxis.get_majorticklabels(), fontsize=7)

ax6 = plt.subplot(3, 3, 6)
ax6.plot(df_100['Instance'], df_100['ArchiveSize'], 'o-', color='orange', 
         markersize=8, linewidth=2, label='100C')
ax6.set_xlabel('Instance', fontweight='bold')
ax6.set_ylabel('Archive Size', fontweight='bold')
ax6.set_title('100C: Archive Size Pattern', fontweight='bold', color='orange')
ax6.grid(True, alpha=0.3)
ax6.tick_params(axis='x', rotation=45)
plt.setp(ax6.xaxis.get_majorticklabels(), fontsize=7)

# 7. Performance Summary Table
ax7 = plt.subplot(3, 3, 7)
ax7.axis('off')
summary_data = []
for size, data in [(20, df_20), (50, df_50), (100, df_100), (200, df_200)]:
    summary_data.append([
        f'{size}C',
        f'{data["ArchiveSize"].mean():.1f}',
        f'{data["Evaluations"].mean()/1000:.0f}K',
        f'{data["Time"].mean():.1f}s'
    ])

table = ax7.table(cellText=summary_data,
                 colLabels=['Size', 'Avg Archive', 'Avg Evals', 'Avg Time'],
                 cellLoc='center',
                 loc='center',
                 colWidths=[0.2, 0.3, 0.3, 0.3])
table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1, 2)
for i in range(4):
    table[(i+1, 0)].set_facecolor(colors[i])
    table[(i+1, 0)].set_alpha(0.6)
ax7.set_title('Performance Summary', fontweight='bold', pad=20)

# 8. Time Distribution
ax8 = plt.subplot(3, 3, 8)
time_data = [df_20['Time'].values, df_50['Time'].values,
             df_100['Time'].values, df_200['Time'].values]
bp2 = ax8.boxplot(time_data, labels=[f'{s}C' for s in sizes], patch_artist=True)
for patch, color in zip(bp2['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.6)
ax8.set_ylabel('Execution Time (seconds)', fontweight='bold')
ax8.set_title('Time Variability', fontweight='bold')
ax8.grid(axis='y', alpha=0.3)

# 9. Key Insights Text
ax9 = plt.subplot(3, 3, 9)
ax9.axis('off')
insights = f"""
KEY FINDINGS:

50C Issues (Worst: -50.17% HV):
• Avg Archive: {df_50['ArchiveSize'].mean():.1f} solutions
• High variability: 1-375 solutions
• Many instances: Archive = 1-7 (too small!)
• Not enough diversity

100C Issues (+9.81% HV but 8% win):
• Avg Archive: {df_100['ArchiveSize'].mean():.1f} solutions  
• High variability: 1-1331 solutions
• Many small archives (1-27)
• Good HV but poor domination

ROOT CAUSE:
✗ Evaluation limit too low (25% baseline)
✗ Insufficient convergence time
✗ Small archives = poor diversity
✗ 50C/100C need more iterations

SOLUTION:
✓ Increase evaluations to 50-100%
✓ Adjust archive management
✓ Better stopping criteria
"""
ax9.text(0.05, 0.95, insights, transform=ax9.transAxes,
         fontsize=9, verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

plt.tight_layout()
plt.savefig('analysis_50c_100c_problems.png', dpi=300, bbox_inches='tight')
print("✅ Saved: analysis_50c_100c_problems.png")

# Print detailed analysis
print("\n" + "="*80)
print("DETAILED ANALYSIS: WHY 50C AND 100C PERFORMED POORLY")
print("="*80)

print("\n50C ANALYSIS:")
print("-" * 80)
print(f"Average Archive Size: {df_50['ArchiveSize'].mean():.2f} (Range: {df_50['ArchiveSize'].min()}-{df_50['ArchiveSize'].max()})")
print(f"Instances with Archive ≤ 10: {len(df_50[df_50['ArchiveSize'] <= 10])}/16")
print(f"Average Evaluations: {df_50['Evaluations'].mean():.0f} (Target: 547,500)")
print(f"Average Time: {df_50['Time'].mean():.2f}s")
print("\nProblem: High archive variability indicates:")
print("  - Some instances converged poorly (Archive=1-7)")
print("  - Large archives (205-375) suggest non-dominated solutions accumulation")
print("  - Insufficient time to refine Pareto front")

print("\n100C ANALYSIS:")
print("-" * 80)
print(f"Average Archive Size: {df_100['ArchiveSize'].mean():.2f} (Range: {df_100['ArchiveSize'].min()}-{df_100['ArchiveSize'].max()})")
print(f"Instances with Archive ≤ 27: {len(df_100[df_100['ArchiveSize'] <= 27])}/16")
print(f"Average Evaluations: {df_100['Evaluations'].mean():.0f} (Target: 10,305,000)")
print(f"Average Time: {df_100['Time'].mean():.2f}s")
print("\nProblem: Good HV (+9.81%) but poor domination (8% win rate) suggests:")
print("  - Solutions cover good hypervolume region")
print("  - But individual solutions don't dominate benchmark")
print("  - Need more refinement iterations")

print("\n" + "="*80)
print("ROOT CAUSES:")
print("="*80)
print("1. EVALUATION BUDGET: Only 25% of baseline (65K-18.8M)")
print("   → Not enough iterations for convergence in complex instances")
print("\n2. ARCHIVE MANAGEMENT:")
print("   → Too many instances with very small archives (1-10 solutions)")
print("   → Indicates premature convergence or poor diversity")
print("\n3. COMPLEXITY SCALING:")
print("   → 50C and 100C have higher complexity per evaluation")
print("   → Fixed evaluation budget doesn't scale well")
print("\n4. IDEAL POINT METHOD:")
print("   → Selects best compromise but may sacrifice diversity")
print("   → Works well for 20C (simple) and 200C (many solutions)")
print("   → Struggles with 50C/100C mid-range complexity")

print("\n" + "="*80)
print("RECOMMENDATIONS:")
print("="*80)
print("1. Increase evaluation budget to 50% or 100% baseline")
print("2. Implement adaptive archive size management")
print("3. Use evaluation-per-complexity scaling")
print("4. Add diversity preservation mechanisms")
print("="*80)

plt.show()
