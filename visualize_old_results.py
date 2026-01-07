import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read comparison results
df = pd.read_csv('old_results/comparison_results.csv')

# Create figure with 2 subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

# ===== LEFT: Domination Comparison =====
sizes = [20, 50, 100, 200]
wins = []
ties = []
loses = []

for size in sizes:
    size_df = df[df['Dataset'] == size]
    w = size_df['Wins'].sum()
    t = size_df['Ties'].sum()
    l = size_df['Loses'].sum()
    wins.append(w)
    ties.append(t)
    loses.append(l)

x = np.arange(len(sizes))
width = 0.6

p1 = ax1.bar(x, wins, width, label='Our > Benchmark', color='#2ecc71', edgecolor='black', linewidth=1.5)
p2 = ax1.bar(x, ties, width, bottom=wins, label='Equal', color='#f39c12', edgecolor='black', linewidth=1.5)
p3 = ax1.bar(x, loses, width, bottom=np.array(wins)+np.array(ties), 
             label='Benchmark > Our', color='#e74c3c', edgecolor='black', linewidth=1.5)

ax1.set_xlabel('Dataset Size', fontsize=14, fontweight='bold')
ax1.set_ylabel('Number of Solutions', fontsize=14, fontweight='bold')
ax1.set_title('Domination Comparison: ICAHGS vs Benchmark\n(Test on 4 Random Instances per Size)', 
              fontsize=15, fontweight='bold', pad=20)
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

# ===== RIGHT: HV Gap Comparison =====
hv_gaps = []
for size in sizes:
    size_df = df[df['Dataset'] == size]
    avg_gap = size_df['GapHV(%)'].mean()
    hv_gaps.append(avg_gap)

colors = ['#2ecc71' if g > 0 else '#e74c3c' for g in hv_gaps]
bars = ax2.bar(range(len(sizes)), hv_gaps, color=colors, edgecolor='black', linewidth=1.5, alpha=0.8, width=0.6)
ax2.axhline(y=0, color='blue', linestyle='--', linewidth=2, label='Baseline (0%)')
ax2.set_xticks(range(len(sizes)))
ax2.set_xticklabels([f'{s}C' for s in sizes], fontsize=12)
ax2.set_ylabel('Hypervolume Gap (%)', fontsize=14, fontweight='bold')
ax2.set_title('Average HV Gap by Dataset Size\n(Positive = Better than Benchmark)', 
              fontsize=15, fontweight='bold', pad=20)
ax2.legend(fontsize=11)
ax2.grid(axis='y', alpha=0.3, linestyle='--')

# Add values on bars
for i, (bar, val) in enumerate(zip(bars, hv_gaps)):
    y_pos = val + (3 if val > 0 else -5)
    va = 'bottom' if val > 0 else 'top'
    ax2.text(bar.get_x() + bar.get_width()/2, y_pos, f'{val:.1f}%', 
             ha='center', va=va, fontsize=11, fontweight='bold')

plt.tight_layout()
plt.savefig('old_results/comparison_visualization.png', dpi=300, bbox_inches='tight')
print("✅ Saved: old_results/comparison_visualization.png")

# ===== Generate HTML Table =====
html_content = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Hypervolume Comparison Table</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        h1 {
            color: #2c3e50;
            text-align: center;
            margin-bottom: 10px;
        }
        .subtitle {
            text-align: center;
            color: #7f8c8d;
            margin-bottom: 30px;
            font-size: 14px;
        }
        table {
            border-collapse: collapse;
            width: 100%;
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        th {
            background-color: #34495e;
            color: white;
            padding: 12px;
            text-align: center;
            font-weight: 600;
            border: 1px solid #2c3e50;
        }
        td {
            padding: 10px;
            text-align: center;
            border: 1px solid #ddd;
        }
        tr:nth-child(even) {
            background-color: #f9f9f9;
        }
        tr:hover {
            background-color: #ecf0f1;
        }
        .positive {
            color: #27ae60;
            font-weight: bold;
        }
        .negative {
            color: #e74c3c;
            font-weight: bold;
        }
        .neutral {
            color: #7f8c8d;
        }
        .dataset-header {
            background-color: #3498db !important;
            color: white;
            font-weight: bold;
            font-size: 16px;
        }
        .summary-row {
            background-color: #f39c12 !important;
            color: white;
            font-weight: bold;
        }
        .legend {
            max-width: 1200px;
            margin: 20px auto;
            padding: 15px;
            background-color: white;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-radius: 5px;
        }
        .legend h3 {
            margin-top: 0;
            color: #2c3e50;
        }
        .legend p {
            margin: 5px 0;
            color: #555;
        }
    </style>
</head>
<body>
    <h1>Hypervolume Comparison: ICAHGS vs Benchmark</h1>
    <div class="subtitle">Test Results on 4 Random Instances (20C, 50C, 100C, 200C)</div>
    
    <table>
        <thead>
            <tr>
                <th>Dataset</th>
                <th>Grid Size</th>
                <th>HV Ratio</th>
                <th>Gap HV (%)</th>
                <th>Domination<br/>(+/=/−)</th>
                <th>IGD</th>
                <th>Result</th>
            </tr>
        </thead>
        <tbody>
"""

current_size = None
size_data = {'20C': [], '50C': [], '100C': [], '200C': []}

for _, row in df.iterrows():
    size = f"{int(row['Dataset'])}C"
    grid = int(row['GridSize'])
    hv = row['HV']
    gap = row['GapHV(%)']
    dom = row['+/=/-']
    igd = row['IGD']
    
    # Determine result class
    if gap > 0:
        gap_class = 'positive'
        result = '✓ Better'
        result_class = 'positive'
    elif gap < -10:
        gap_class = 'negative'
        result = '✗ Worse'
        result_class = 'negative'
    else:
        gap_class = 'neutral'
        result = '≈ Similar'
        result_class = 'neutral'
    
    # Add dataset header if new size
    if current_size != size:
        if current_size is not None:
            html_content += f"""
            <tr class="dataset-header">
                <td colspan="7">{size} Summary</td>
            </tr>
"""
        current_size = size
    
    html_content += f"""
            <tr>
                <td><strong>{size}</strong></td>
                <td>{grid}</td>
                <td>{hv:.4f}</td>
                <td class="{gap_class}">{gap:+.2f}%</td>
                <td>{dom}</td>
                <td>{igd:.4f}</td>
                <td class="{result_class}">{result}</td>
            </tr>
"""
    
    size_data[size].append(gap)

# Add summary rows
html_content += """
            <tr class="summary-row">
                <td colspan="7">OVERALL SUMMARY</td>
            </tr>
"""

total_instances = 0
total_gap = 0
for size, gaps in size_data.items():
    if gaps:
        avg_gap = np.mean(gaps)
        total_instances += len(gaps)
        total_gap += sum(gaps)
        gap_class = 'positive' if avg_gap > 0 else 'negative'
        html_content += f"""
            <tr>
                <td><strong>{size}</strong></td>
                <td colspan="2">{len(gaps)} instances</td>
                <td class="{gap_class}"><strong>{avg_gap:+.2f}%</strong></td>
                <td colspan="3">Average HV Gap</td>
            </tr>
"""

overall_avg = total_gap / total_instances if total_instances > 0 else 0
overall_class = 'positive' if overall_avg > 0 else 'negative'
html_content += f"""
            <tr class="summary-row">
                <td><strong>OVERALL</strong></td>
                <td colspan="2">{total_instances} instances</td>
                <td class="{overall_class}"><strong>{overall_avg:+.2f}%</strong></td>
                <td colspan="3">Average Across All Sizes</td>
            </tr>
"""

html_content += """
        </tbody>
    </table>
    
    <div class="legend">
        <h3>Legend:</h3>
        <p><strong>HV Ratio:</strong> Hypervolume(Our) / Hypervolume(Benchmark). Values > 1.0 indicate better performance.</p>
        <p><strong>Gap HV (%):</strong> Percentage improvement over benchmark. Positive = better, negative = worse.</p>
        <p><strong>Domination (+/=/−):</strong> Number of solutions that dominate/equal/dominated by benchmark.</p>
        <p><strong>IGD:</strong> Inverted Generational Distance. Lower values indicate better convergence to benchmark front.</p>
    </div>
    
    <div class="subtitle" style="margin-top: 30px;">
        Generated on: """ + pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S') + """
    </div>
</body>
</html>
"""

with open('old_results/hv_comparison_table.html', 'w', encoding='utf-8') as f:
    f.write(html_content)

print("✅ Saved: old_results/hv_comparison_table.html")
print("\n=== SUMMARY ===")
print(f"Total instances tested: {total_instances}")
print(f"Overall HV Gap: {overall_avg:+.2f}%")
print(f"20C: {np.mean(size_data['20C']):+.2f}%")
print(f"50C: {np.mean(size_data['50C']):+.2f}%")
print(f"100C: {np.mean(size_data['100C']):+.2f}%")
print(f"200C: {np.mean(size_data['200C']):+.2f}%")

plt.show()
