import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

print("\n=== SO SÁNH ARCHIVE SIZE: 25% vs 50% EVALUATION ===\n")

# Read data
df_25 = pd.read_csv('results_60_instances.csv')
df_50 = pd.read_csv('result_50c_100c_doubled/results_summary.csv')

# Create comparison for 50C and 100C
sizes = [50, 100]
fig, axes = plt.subplots(1, 2, figsize=(14, 6))
fig.suptitle('Archive Size Improvement: 50C & 100C\n25% evals → 50% evals', 
             fontsize=16, fontweight='bold')

for idx, size in enumerate(sizes):
    ax = axes[idx]
    
    # Get data for this size
    data_25 = df_25[df_25['Size'] == size].sort_values('Instance')
    data_50 = df_50[df_50['Size'] == size].sort_values('Instance')
    
    instances = data_25['Instance'].values
    archive_25 = data_25['ArchiveSize'].values
    archive_50 = data_50['ArchiveSize'].values
    
    x = np.arange(len(instances))
    width = 0.35
    
    # Plot bars
    bars1 = ax.bar(x - width/2, archive_25, width, label='25% evals', 
                   color='lightcoral', alpha=0.8, edgecolor='black')
    bars2 = ax.bar(x + width/2, archive_50, width, label='50% evals', 
                   color='lightgreen', alpha=0.8, edgecolor='black')
    
    # Formatting
    ax.set_xlabel('Instance', fontsize=12, fontweight='bold')
    ax.set_ylabel('Archive Size', fontsize=12, fontweight='bold')
    ax.set_title(f'{size}C Instances', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([inst.split('.')[1] + '.' + inst.split('.')[2] 
                        for inst in instances], rotation=45, ha='right', fontsize=9)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add average line
    avg_25 = archive_25.mean()
    avg_50 = archive_50.mean()
    ax.axhline(y=avg_25, color='red', linestyle='--', linewidth=1.5, alpha=0.7,
               label=f'Avg 25%: {avg_25:.1f}')
    ax.axhline(y=avg_50, color='green', linestyle='--', linewidth=1.5, alpha=0.7,
               label=f'Avg 50%: {avg_50:.1f}')
    
    # Update legend to include averages
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels, fontsize=9, loc='upper left')
    
    # Print statistics
    print(f"\n{size}C STATISTICS:")
    print(f"  25% evals - Average: {avg_25:.1f}, Min: {archive_25.min()}, Max: {archive_25.max()}")
    print(f"  50% evals - Average: {avg_50:.1f}, Min: {archive_50.min()}, Max: {archive_50.max()}")
    improvement = ((avg_50 - avg_25) / avg_25 * 100)
    print(f"  Improvement: {improvement:+.1f}%")
    
    # Count improvements
    better = (archive_50 > archive_25).sum()
    worse = (archive_50 < archive_25).sum()
    same = (archive_50 == archive_25).sum()
    print(f"  Better: {better}/{len(instances)}, Worse: {worse}/{len(instances)}, Same: {same}/{len(instances)}")

plt.tight_layout()
plt.savefig('archive_size_comparison.png', dpi=300, bbox_inches='tight')
print(f"\n✅ Saved to: archive_size_comparison.png")

# Create summary comparison chart
fig, ax = plt.subplots(figsize=(10, 7))

sizes = [20, 50, 100, 200]
eval_types = ['25%', '50%', '50%', '25%']
colors = ['lightblue', 'lightgreen', 'lightgreen', 'lightblue']

# Get average archive sizes
avg_archives = []
for size in sizes:
    if size in [50, 100]:
        avg = df_50[df_50['Size'] == size]['ArchiveSize'].mean()
    else:
        avg = df_25[df_25['Size'] == size]['ArchiveSize'].mean()
    avg_archives.append(avg)

x = np.arange(len(sizes))
bars = ax.bar(x, avg_archives, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)

# Add eval type labels on bars
for i, (bar, eval_type) in enumerate(zip(bars, eval_types)):
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, height + max(avg_archives)*0.02,
            f'{height:.0f}\n({eval_type})',
            ha='center', va='bottom', fontsize=11, fontweight='bold')

ax.set_xlabel('Instance Size', fontsize=13, fontweight='bold')
ax.set_ylabel('Average Archive Size', fontsize=13, fontweight='bold')
ax.set_title('Average Archive Size by Instance Size\n(Hybrid Strategy: 20C/200C at 25%, 50C/100C at 50%)', 
             fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels([f'{s}C' for s in sizes], fontsize=12)
ax.grid(True, alpha=0.3, axis='y')

# Add legend for evaluation types
from matplotlib.patches import Patch
legend_elements = [Patch(facecolor='lightblue', edgecolor='black', label='25% evaluation'),
                   Patch(facecolor='lightgreen', edgecolor='black', label='50% evaluation')]
ax.legend(handles=legend_elements, fontsize=11, loc='upper left')

plt.tight_layout()
plt.savefig('archive_size_by_size.png', dpi=300, bbox_inches='tight')
print(f"✅ Saved to: archive_size_by_size.png")

# Overall summary
print("\n" + "="*60)
print("OVERALL SUMMARY")
print("="*60)
print(f"\n{'Size':<8} {'EvalType':<10} {'Instances':<10} {'Avg Archive':<15}")
print("-"*60)
for size in sizes:
    if size in [50, 100]:
        data = df_50[df_50['Size'] == size]
        eval_type = '50% evals'
    else:
        data = df_25[df_25['Size'] == size]
        eval_type = '25% evals'
    
    count = len(data)
    avg_archive = data['ArchiveSize'].mean()
    print(f"{size:3d}C     {eval_type:<10} {count:<10} {avg_archive:>10.1f}")

print("="*60)

plt.show()
