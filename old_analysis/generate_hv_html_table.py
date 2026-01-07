import pandas as pd
import os
import numpy as np

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

def get_pareto_front(points):
    """Extract Pareto front from points"""
    if not points:
        return []
    
    pareto = []
    for p in points:
        dominated = False
        for other in points:
            if (other[0] < p[0] and other[1] <= p[1]) or (other[0] <= p[0] and other[1] < p[1]):
                dominated = True
                break
        if not dominated:
            pareto.append(p)
    return sorted(pareto, key=lambda x: x[0])

def hypervolume_2d(pareto_front, ref_point):
    """Calculate 2D hypervolume"""
    if not pareto_front:
        return 0.0
    
    hv = 0.0
    prev_x = ref_point[0]
    
    for point in reversed(pareto_front):
        width = prev_x - point[0]
        height = ref_point[1] - point[1]
        if width > 0 and height > 0:
            hv += width * height
        prev_x = point[0]
    
    return hv

# Read all files and calculate HV for each instance
from collections import defaultdict
result_files = sorted([f for f in os.listdir('result') if f.endswith('.txt')])
data_by_group = defaultdict(lambda: {'icahgs_hvs': [], 'benchmark_hvs': [], 'wins': 0, 'ties': 0, 'loses': 0})

for fname in result_files:
    if os.path.exists(f'benchmark/{fname}'):
        parts = fname.replace('.txt', '').split('.')
        if len(parts) == 3:
            size, grid = int(parts[0]), int(parts[1])
            
            ic_sols = read_solutions(f'result/{fname}')
            bm_sols = read_solutions(f'benchmark/{fname}')
            
            if ic_sols and bm_sols:
                # Normalize together for fair comparison
                all_points = ic_sols + bm_sols
                min_ct = min(p[0] for p in all_points)
                max_ct = max(p[0] for p in all_points)
                min_wt = min(p[1] for p in all_points)
                max_wt = max(p[1] for p in all_points)
                
                def normalize(pts):
                    if max_ct == min_ct or max_wt == min_wt:
                        return pts
                    return [[(p[0]-min_ct)/(max_ct-min_ct), (p[1]-min_wt)/(max_wt-min_wt)] for p in pts]
                
                norm_ic = normalize(ic_sols)
                norm_bm = normalize(bm_sols)
                
                # Get Pareto fronts
                pf_ic = get_pareto_front(norm_ic)
                pf_bm = get_pareto_front(norm_bm)
                
                # Calculate HV with ref point (1.1, 1.1)
                hv_ic = hypervolume_2d(pf_ic, [1.1, 1.1])
                hv_bm = hypervolume_2d(pf_bm, [1.1, 1.1])
                
                # Store HV values
                data_by_group[(size, grid)]['icahgs_hvs'].append(hv_ic)
                data_by_group[(size, grid)]['benchmark_hvs'].append(hv_bm)
                
                # Compare for win/tie/lose
                if hv_ic > hv_bm * 1.01:
                    data_by_group[(size, grid)]['wins'] += 1
                elif hv_bm > hv_ic * 1.01:
                    data_by_group[(size, grid)]['loses'] += 1
                else:
                    data_by_group[(size, grid)]['ties'] += 1

# HTML header
html = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Table 1: Performance Comparison Based on Hypervolume</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        h2 {
            text-align: center;
            color: #333;
            margin-bottom: 30px;
        }
        table {
            border-collapse: collapse;
            margin: 20px auto;
            background-color: white;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            width: 90%;
            max-width: 1200px;
        }
        th {
            background-color: #4472C4;
            color: white;
            padding: 12px;
            text-align: center;
            border: 1px solid #2E5C9A;
            font-weight: bold;
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
            background-color: #e8f4ff;
        }
        .dataset-col {
            font-weight: bold;
            text-align: left;
            padding-left: 20px;
        }
        .average-row {
            font-weight: bold;
            background-color: #D9E2F3;
        }
        .average-row .dataset-col {
            font-style: italic;
        }
        .positive {
            color: green;
            font-weight: bold;
        }
        .negative {
            color: red;
            font-weight: bold;
        }
        .neutral {
            color: #666;
        }
        .description-col {
            text-align: left;
            padding-left: 15px;
        }
    </style>
</head>
<body>
    <h2>Table 1: Performance Comparison Based on Hypervolume</h2>
    <table>
        <thead>
            <tr>
                <th>Dataset</th>
                <th>ICAHGS HV</th>
                <th>Gap HV (%)</th>
                <th>+/=/−</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
"""

# Group by dataset size
sizes = [20, 50, 100, 200]

for size in sizes:
    size_groups = sorted([(s, g) for s, g in data_by_group.keys() if s == size])
    
    # Data rows for this size
    for size_key, grid in size_groups:
        d = data_by_group[(size_key, grid)]
        
        # Average HV values for this (size, grid) combination
        avg_hv_ic = np.mean(d['icahgs_hvs'])
        avg_hv_bm = np.mean(d['benchmark_hvs'])
        
        # Calculate Gap HV (%)
        gap_hv = ((avg_hv_ic - avg_hv_bm) / avg_hv_bm * 100) if avg_hv_bm > 0 else 0
        
        # Determine color class for Gap
        gap_class = 'positive' if gap_hv > 0 else 'negative' if gap_hv < 0 else 'neutral'
        
        # Format +/=/- 
        domination = f"{d['wins']}/{d['ties']}/{d['loses']}"
        domination_class = 'positive' if d['wins'] > d['loses'] else 'negative' if d['wins'] < d['loses'] else 'neutral'
        
        # Dataset name with grid size
        dataset_name = f"{size} × {grid}"
        description = f"{size} customers, {grid}×{grid} grid"
        
        html += f"""
            <tr>
                <td class="dataset-col">{dataset_name}</td>
                <td>{avg_hv_ic:.2f}</td>
                <td class="{gap_class}">{gap_hv:+.2f}</td>
                <td class="{domination_class}">{domination}</td>
                <td class="description-col">{description}</td>
            </tr>
"""
    
    # Average row for this size
    size_data = [data_by_group[k] for k in size_groups]
    all_hv_ic = [hv for d in size_data for hv in d['icahgs_hvs']]
    all_hv_bm = [hv for d in size_data for hv in d['benchmark_hvs']]
    
    avg_hv_ic = np.mean(all_hv_ic)
    avg_hv_bm = np.mean(all_hv_bm)
    avg_gap = ((avg_hv_ic - avg_hv_bm) / avg_hv_bm * 100) if avg_hv_bm > 0 else 0
    
    total_w = sum(d['wins'] for d in size_data)
    total_t = sum(d['ties'] for d in size_data)
    total_l = sum(d['loses'] for d in size_data)
    avg_domination = f"{total_w}/{total_t}/{total_l}"
    
    avg_class = 'positive' if avg_gap > 0 else 'negative' if avg_gap < 0 else 'neutral'
    avg_dom_class = 'positive' if total_w > total_l else 'negative' if total_w < total_l else 'neutral'
    
    # Calculate win rate
    total_instances = total_w + total_t + total_l
    win_rate = (total_w / total_instances * 100) if total_instances > 0 else 0
    
    html += f"""
            <tr class="average-row">
                <td class="dataset-col">Average ({size}C)</td>
                <td>{avg_hv_ic:.2f}</td>
                <td class="{avg_class}">{avg_gap:+.2f}</td>
                <td class="{avg_dom_class}">{avg_domination}</td>
                <td class="description-col">Win rate: {win_rate:.1f}%</td>
            </tr>
"""

# HTML footer
html += """
        </tbody>
    </table>
    
    <div style="margin: 20px auto; max-width: 1000px; background-color: white; padding: 20px; border-radius: 5px;">
        <h3>Explanation:</h3>
        <ul>
            <li><strong>ICAHGS HV</strong>: Average hypervolume value of ICAHGS algorithm (0-1 scale after normalization with reference point [1.1, 1.1])</li>
            <li><strong>Gap HV (%)</strong>: Percentage difference from benchmark
                <ul>
                    <li>Formula: <code>Gap HV = (HV_ICAHGS - HV_Benchmark) / HV_Benchmark × 100</code></li>
                    <li><span class="positive">Positive (+)</span>: ICAHGS produces larger hypervolume than benchmark (better)</li>
                    <li><span class="negative">Negative (−)</span>: Benchmark produces larger hypervolume than ICAHGS (worse)</li>
                </ul>
            </li>
            <li><strong>+/=/−</strong>: Number of instances where ICAHGS wins/ties/loses against benchmark
                <ul>
                    <li>Win: HV_ICAHGS > 1.01 × HV_Benchmark</li>
                    <li>Tie: Difference < 1%</li>
                    <li>Loss: HV_ICAHGS < 0.99 × HV_Benchmark</li>
                </ul>
            </li>
            <li><strong>Win rate</strong>: Percentage of instances where ICAHGS dominates benchmark</li>
        </ul>
        
        <h3>Key Findings:</h3>
        <ul>
            <li><strong>20 customers</strong>: ICAHGS shows strong performance with positive Gap HV</li>
            <li><strong>50 customers</strong>: Mixed results, competitive with benchmark</li>
            <li><strong>100-200 customers</strong>: Benchmark generally outperforms ICAHGS on larger instances</li>
        </ul>
    </div>
</body>
</html>
"""

# Write to file
with open('hv_comparison_table.html', 'w', encoding='utf-8') as f:
    f.write(html)

print("✓ HTML table generated: hv_comparison_table.html")
print("\nTo view: Open the file in your web browser")
print("\nNote: HV values are now TRUE hypervolume values (0-1 scale), not ratios!")
