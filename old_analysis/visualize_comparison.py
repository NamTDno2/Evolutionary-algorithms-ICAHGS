import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read results
df = pd.read_csv('comparison_results.csv')

# Create figure with subplots
fig = plt.figure(figsize=(16, 10))

# 1. HV Comparison by Dataset Size
ax1 = plt.subplot(2, 3, 1)
sizes = [20, 50, 100, 200]
hv_by_size = [df[df['Dataset'] == s]['HV'].mean() for s in sizes]
colors = ['green' if h > 1 else 'red' for h in hv_by_size]
bars1 = ax1.bar(range(len(sizes)), hv_by_size, color=colors, alpha=0.7, edgecolor='black')
ax1.axhline(y=1.0, color='blue', linestyle='--', linewidth=2, label='Baseline')
ax1.set_xticks(range(len(sizes)))
ax1.set_xticklabels([f'{s}C' for s in sizes])
ax1.set_ylabel('HV Ratio')
ax1.set_title('Average HV Ratio by Dataset Size', fontweight='bold')
ax1.legend()
ax1.grid(axis='y', alpha=0.3)

# Add values on bars
for i, (bar, val) in enumerate(zip(bars1, hv_by_size)):
    ax1.text(bar.get_x() + bar.get_width()/2, val + 0.05, f'{val:.2f}', 
             ha='center', va='bottom', fontweight='bold')

# 2. Gap HV (%) Comparison
ax2 = plt.subplot(2, 3, 2)
gap_by_size = [df[df['Dataset'] == s]['GapHV(%)'].mean() for s in sizes]
colors2 = ['green' if g > 0 else 'red' for g in gap_by_size]
bars2 = ax2.bar(range(len(sizes)), gap_by_size, color=colors2, alpha=0.7, edgecolor='black')
ax2.axhline(y=0, color='blue', linestyle='--', linewidth=2)
ax2.set_xticks(range(len(sizes)))
ax2.set_xticklabels([f'{s}C' for s in sizes])
ax2.set_ylabel('Gap HV (%)')
ax2.set_title('Average Gap HV (%) by Dataset Size', fontweight='bold')
ax2.grid(axis='y', alpha=0.3)

for bar, val in zip(bars2, gap_by_size):
    ax2.text(bar.get_x() + bar.get_width()/2, val + (5 if val > 0 else -5), 
             f'{val:.1f}%', ha='center', va='bottom' if val > 0 else 'top', fontweight='bold')

# 3. Win Rate by Dataset Size
ax3 = plt.subplot(2, 3, 3)
win_rates = []
for s in sizes:
    size_df = df[df['Dataset'] == s]
    w = size_df['Wins'].sum()
    t = size_df['Ties'].sum()
    l = size_df['Loses'].sum()
    total = w + t + l
    win_rates.append((w/total*100) if total > 0 else 0)

bars3 = ax3.bar(range(len(sizes)), win_rates, color='steelblue', alpha=0.7, edgecolor='black')
ax3.set_xticks(range(len(sizes)))
ax3.set_xticklabels([f'{s}C' for s in sizes])
ax3.set_ylabel('Win Rate (%)')
ax3.set_title('Win Rate by Dataset Size', fontweight='bold')
ax3.set_ylim(0, 100)
ax3.grid(axis='y', alpha=0.3)

for bar, val in zip(bars3, win_rates):
    ax3.text(bar.get_x() + bar.get_width()/2, val + 2, f'{val:.1f}%', 
             ha='center', va='bottom', fontweight='bold')

# 4. HV by Grid Size (all datasets)
ax4 = plt.subplot(2, 3, 4)
grid_data = df.groupby('GridSize').agg({'HV': 'mean', 'Dataset': 'count'}).reset_index()
ax4.plot(grid_data['GridSize'], grid_data['HV'], marker='o', linewidth=2, markersize=8, color='darkgreen')
ax4.axhline(y=1.0, color='red', linestyle='--', linewidth=1, alpha=0.5)
ax4.set_xlabel('Grid Size')
ax4.set_ylabel('Average HV Ratio')
ax4.set_title('HV Ratio vs Grid Size (All Datasets)', fontweight='bold')
ax4.grid(True, alpha=0.3)

# 5. Domination Stacked Bar
ax5 = plt.subplot(2, 3, 5)
domination_data = []
for s in sizes:
    size_df = df[df['Dataset'] == s]
    w = size_df['Wins'].sum()
    t = size_df['Ties'].sum()
    l = size_df['Loses'].sum()
    total = w + t + l
    domination_data.append([w/total*100 if total>0 else 0, 
                            t/total*100 if total>0 else 0, 
                            l/total*100 if total>0 else 0])

domination_data = np.array(domination_data).T
x_pos = range(len(sizes))
ax5.bar(x_pos, domination_data[0], label='Wins', color='green', alpha=0.7, edgecolor='black')
ax5.bar(x_pos, domination_data[1], bottom=domination_data[0], label='Ties', 
        color='yellow', alpha=0.7, edgecolor='black')
ax5.bar(x_pos, domination_data[2], bottom=domination_data[0]+domination_data[1], 
        label='Loses', color='red', alpha=0.7, edgecolor='black')
ax5.set_xticks(x_pos)
ax5.set_xticklabels([f'{s}C' for s in sizes])
ax5.set_ylabel('Percentage (%)')
ax5.set_title('Domination Distribution by Dataset Size', fontweight='bold')
ax5.legend()
ax5.set_ylim(0, 100)

# 6. IGD Comparison
ax6 = plt.subplot(2, 3, 6)
igd_by_size = [df[df['Dataset'] == s]['IGD'].mean() for s in sizes]
bars6 = ax6.bar(range(len(sizes)), igd_by_size, color='coral', alpha=0.7, edgecolor='black')
ax6.set_xticks(range(len(sizes)))
ax6.set_xticklabels([f'{s}C' for s in sizes])
ax6.set_ylabel('Average IGD')
ax6.set_title('Average IGD by Dataset Size', fontweight='bold')
ax6.grid(axis='y', alpha=0.3)

for bar, val in zip(bars6, igd_by_size):
    ax6.text(bar.get_x() + bar.get_width()/2, val + 0.02, f'{val:.3f}', 
             ha='center', va='bottom', fontweight='bold')

plt.tight_layout()
plt.savefig('comparison_charts.png', dpi=300, bbox_inches='tight')
print("✓ Charts saved to: comparison_charts.png")
plt.show()
