import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams

# Set up Vietnamese font support (if available)
rcParams['font.family'] = 'sans-serif'
rcParams['font.size'] = 10

# Load the two result files
print("Loading result files...")
user_results = pd.read_csv('results.csv')
benchmark_results = pd.read_csv('results_benchmark.csv')

print(f"User results: {len(user_results)} solutions from {user_results['Dataset'].nunique()} datasets")
print(f"Benchmark results: {len(benchmark_results)} solutions from {benchmark_results['Dataset'].nunique()} datasets")

# Find best solution for each dataset (minimum CompletionTime)
print("\nFinding best solutions for each dataset...")
user_best = user_results.loc[user_results.groupby('Dataset')['CompletionTime'].idxmin()]
benchmark_best = benchmark_results.loc[benchmark_results.groupby('Dataset')['CompletionTime'].idxmin()]

# Merge the results for comparison
comparison = pd.merge(
    user_best[['Dataset', 'CompletionTime', 'TotalWaitingTime', 'ExecutionTime', 'UniqueSolutions']],
    benchmark_best[['Dataset', 'CompletionTime', 'TotalWaitingTime']],
    on='Dataset',
    suffixes=('_User', '_Benchmark')
)

# Calculate differences
comparison['CT_Diff'] = comparison['CompletionTime_User'] - comparison['CompletionTime_Benchmark']
comparison['CT_Diff_Percent'] = (comparison['CT_Diff'] / comparison['CompletionTime_Benchmark']) * 100
comparison['WT_Diff'] = comparison['TotalWaitingTime_User'] - comparison['TotalWaitingTime_Benchmark']
comparison['WT_Diff_Percent'] = (comparison['WT_Diff'] / comparison['TotalWaitingTime_Benchmark']) * 100

# Extract dataset size (number of customers)
comparison['Size'] = comparison['Dataset'].str.split('.').str[0].astype(int)

# Sort by dataset name
comparison = comparison.sort_values('Dataset')

# Print summary statistics
print("\n" + "="*80)
print("COMPARISON SUMMARY")
print("="*80)
print(f"\nTotal datasets compared: {len(comparison)}")
print(f"\nCompletion Time (CT) Analysis:")
print(f"  User better (lower CT): {(comparison['CT_Diff'] < 0).sum()} datasets ({(comparison['CT_Diff'] < 0).sum()/len(comparison)*100:.1f}%)")
print(f"  Benchmark better: {(comparison['CT_Diff'] > 0).sum()} datasets ({(comparison['CT_Diff'] > 0).sum()/len(comparison)*100:.1f}%)")
print(f"  Average difference: {comparison['CT_Diff'].mean():.2f} ({comparison['CT_Diff_Percent'].mean():.2f}%)")
print(f"  Median difference: {comparison['CT_Diff'].median():.2f} ({comparison['CT_Diff_Percent'].median():.2f}%)")

print(f"\nTotal Waiting Time (WT) Analysis:")
print(f"  User better (lower WT): {(comparison['WT_Diff'] < 0).sum()} datasets ({(comparison['WT_Diff'] < 0).sum()/len(comparison)*100:.1f}%)")
print(f"  Benchmark better: {(comparison['WT_Diff'] > 0).sum()} datasets ({(comparison['WT_Diff'] > 0).sum()/len(comparison)*100:.1f}%)")
print(f"  Average difference: {comparison['WT_Diff'].mean():.2f} ({comparison['WT_Diff_Percent'].mean():.2f}%)")
print(f"  Median difference: {comparison['WT_Diff'].median():.2f} ({comparison['WT_Diff_Percent'].median():.2f}%)")

# Print detailed comparison by size
print("\n" + "="*80)
print("RESULTS BY DATASET SIZE")
print("="*80)
for size in sorted(comparison['Size'].unique()):
    size_data = comparison[comparison['Size'] == size]
    print(f"\n{size} customers ({len(size_data)} datasets):")
    print(f"  CT: User better in {(size_data['CT_Diff'] < 0).sum()}/{len(size_data)} datasets (avg diff: {size_data['CT_Diff_Percent'].mean():.2f}%)")
    print(f"  WT: User better in {(size_data['WT_Diff'] < 0).sum()}/{len(size_data)} datasets (avg diff: {size_data['WT_Diff_Percent'].mean():.2f}%)")
    print(f"  Avg execution time: {size_data['ExecutionTime'].mean():.2f}s")

# Save detailed comparison to CSV
comparison.to_csv('comparison_detail.csv', index=False)
print(f"\nDetailed comparison saved to: comparison_detail.csv")

# Create visualizations
print("\nCreating visualizations...")
fig = plt.figure(figsize=(16, 12))

# 1. Completion Time Comparison (Bar chart)
ax1 = plt.subplot(3, 2, 1)
x = np.arange(len(comparison))
width = 0.35
ax1.bar(x - width/2, comparison['CompletionTime_User'], width, label='User', alpha=0.8, color='#2E86AB')
ax1.bar(x + width/2, comparison['CompletionTime_Benchmark'], width, label='Benchmark', alpha=0.8, color='#A23B72')
ax1.set_xlabel('Dataset Index')
ax1.set_ylabel('Completion Time')
ax1.set_title('Completion Time: User vs Benchmark')
ax1.legend()
ax1.grid(axis='y', alpha=0.3)

# 2. Completion Time Difference (%)
ax2 = plt.subplot(3, 2, 2)
colors = ['green' if x < 0 else 'red' for x in comparison['CT_Diff_Percent']]
ax2.bar(range(len(comparison)), comparison['CT_Diff_Percent'], color=colors, alpha=0.7)
ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
ax2.set_xlabel('Dataset Index')
ax2.set_ylabel('Difference (%)')
ax2.set_title('Completion Time Difference (User - Benchmark)\nGreen: User better | Red: Benchmark better')
ax2.grid(axis='y', alpha=0.3)

# 3. Waiting Time Comparison (Bar chart)
ax3 = plt.subplot(3, 2, 3)
ax3.bar(x - width/2, comparison['TotalWaitingTime_User'], width, label='User', alpha=0.8, color='#2E86AB')
ax3.bar(x + width/2, comparison['TotalWaitingTime_Benchmark'], width, label='Benchmark', alpha=0.8, color='#A23B72')
ax3.set_xlabel('Dataset Index')
ax3.set_ylabel('Total Waiting Time')
ax3.set_title('Total Waiting Time: User vs Benchmark')
ax3.legend()
ax3.grid(axis='y', alpha=0.3)

# 4. Waiting Time Difference (%)
ax4 = plt.subplot(3, 2, 4)
colors = ['green' if x < 0 else 'red' for x in comparison['WT_Diff_Percent']]
ax4.bar(range(len(comparison)), comparison['WT_Diff_Percent'], color=colors, alpha=0.7)
ax4.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
ax4.set_xlabel('Dataset Index')
ax4.set_ylabel('Difference (%)')
ax4.set_title('Total Waiting Time Difference (User - Benchmark)\nGreen: User better | Red: Benchmark better')
ax4.grid(axis='y', alpha=0.3)

# 5. Performance by Dataset Size (CT)
ax5 = plt.subplot(3, 2, 5)
size_groups = comparison.groupby('Size')
sizes = sorted(comparison['Size'].unique())
ct_user_means = [size_groups.get_group(s)['CompletionTime_User'].mean() for s in sizes]
ct_bench_means = [size_groups.get_group(s)['CompletionTime_Benchmark'].mean() for s in sizes]
x_pos = np.arange(len(sizes))
ax5.bar(x_pos - width/2, ct_user_means, width, label='User', alpha=0.8, color='#2E86AB')
ax5.bar(x_pos + width/2, ct_bench_means, width, label='Benchmark', alpha=0.8, color='#A23B72')
ax5.set_xlabel('Number of Customers')
ax5.set_ylabel('Average Completion Time')
ax5.set_title('Average Completion Time by Dataset Size')
ax5.set_xticks(x_pos)
ax5.set_xticklabels(sizes)
ax5.legend()
ax5.grid(axis='y', alpha=0.3)

# 6. Performance by Dataset Size (WT)
ax6 = plt.subplot(3, 2, 6)
wt_user_means = [size_groups.get_group(s)['TotalWaitingTime_User'].mean() for s in sizes]
wt_bench_means = [size_groups.get_group(s)['TotalWaitingTime_Benchmark'].mean() for s in sizes]
ax6.bar(x_pos - width/2, wt_user_means, width, label='User', alpha=0.8, color='#2E86AB')
ax6.bar(x_pos + width/2, wt_bench_means, width, label='Benchmark', alpha=0.8, color='#A23B72')
ax6.set_xlabel('Number of Customers')
ax6.set_ylabel('Average Total Waiting Time')
ax6.set_title('Average Total Waiting Time by Dataset Size')
ax6.set_xticks(x_pos)
ax6.set_xticklabels(sizes)
ax6.legend()
ax6.grid(axis='y', alpha=0.3)

plt.tight_layout()
plt.savefig('comparison_charts.png', dpi=300, bbox_inches='tight')
print("Charts saved to: comparison_charts.png")

# Create scatter plot for multi-objective comparison
fig2, ax = plt.subplots(figsize=(12, 8))
ax.scatter(comparison['CompletionTime_User'], comparison['TotalWaitingTime_User'], 
           s=100, alpha=0.6, label='User', color='#2E86AB', marker='o')
ax.scatter(comparison['CompletionTime_Benchmark'], comparison['TotalWaitingTime_Benchmark'], 
           s=100, alpha=0.6, label='Benchmark', color='#A23B72', marker='s')

# Draw lines connecting user and benchmark solutions for same dataset
for idx, row in comparison.iterrows():
    ax.plot([row['CompletionTime_User'], row['CompletionTime_Benchmark']], 
            [row['TotalWaitingTime_User'], row['TotalWaitingTime_Benchmark']], 
            'k-', alpha=0.2, linewidth=0.5)

ax.set_xlabel('Completion Time')
ax.set_ylabel('Total Waiting Time')
ax.set_title('Multi-Objective Comparison: User vs Benchmark\n(Lines connect same dataset)')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('scatter_comparison.png', dpi=300, bbox_inches='tight')
print("Scatter plot saved to: scatter_comparison.png")

# Print top 10 best improvements and worst results
print("\n" + "="*80)
print("TOP 10 IMPROVEMENTS (User better than Benchmark)")
print("="*80)
best_improvements = comparison.nsmallest(10, 'CT_Diff_Percent')
print("\nBy Completion Time:")
for idx, row in best_improvements.iterrows():
    print(f"  {row['Dataset']:<12} | User: {row['CompletionTime_User']:8.2f} | Benchmark: {row['CompletionTime_Benchmark']:8.2f} | Diff: {row['CT_Diff_Percent']:6.2f}%")

print("\n" + "="*80)
print("TOP 10 AREAS FOR IMPROVEMENT (Benchmark better than User)")
print("="*80)
worst_results = comparison.nlargest(10, 'CT_Diff_Percent')
print("\nBy Completion Time:")
for idx, row in worst_results.iterrows():
    print(f"  {row['Dataset']:<12} | User: {row['CompletionTime_User']:8.2f} | Benchmark: {row['CompletionTime_Benchmark']:8.2f} | Diff: {row['CT_Diff_Percent']:+6.2f}%")

print("\n" + "="*80)
print("ANALYSIS COMPLETE!")
print("="*80)
print("\nGenerated files:")
print("  - comparison_detail.csv: Detailed comparison data")
print("  - comparison_charts.png: Main comparison charts (6 panels)")
print("  - scatter_comparison.png: Multi-objective scatter plot")
