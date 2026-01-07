import os
import numpy as np
from collections import defaultdict
import pandas as pd

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
            i += 1
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
    """Extract Pareto front"""
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

def calculate_igd(obtained, reference):
    """Calculate IGD"""
    if not obtained or not reference:
        return float('inf')
    
    distances = []
    for ref_p in reference:
        min_dist = min(np.sqrt((ref_p[0]-obt_p[0])**2 + (ref_p[1]-obt_p[1])**2) 
                      for obt_p in obtained)
        distances.append(min_dist)
    
    return np.mean(distances)

# Read data
result_files = sorted([f for f in os.listdir('result') if f.endswith('.txt')])
data_by_group = defaultdict(lambda: {'icahgs': [], 'benchmark': []})

for fname in result_files:
    if os.path.exists(f'benchmark/{fname}'):
        parts = fname.replace('.txt', '').split('.')
        if len(parts) == 3:
            size, grid = int(parts[0]), int(parts[1])
            
            ic_sols = read_solutions(f'result/{fname}')
            bm_sols = read_solutions(f'benchmark/{fname}')
            
            if ic_sols and bm_sols:
                data_by_group[(size, grid)]['icahgs'].append(ic_sols)
                data_by_group[(size, grid)]['benchmark'].append(bm_sols)

# Calculate metrics
results = []

for (size, grid) in sorted(data_by_group.keys()):
    d = data_by_group[(size, grid)]
    
    hv_ratios = []
    igd_values = []
    wins, ties, loses = 0, 0, 0
    
    for ic, bm in zip(d['icahgs'], d['benchmark']):
        # Normalize
        all_pts = ic + bm
        min_ct = min(p[0] for p in all_pts)
        max_ct = max(p[0] for p in all_pts)
        min_wt = min(p[1] for p in all_pts)
        max_wt = max(p[1] for p in all_pts)
        
        def norm(pts):
            if max_ct == min_ct or max_wt == min_wt:
                return pts
            return [[(p[0]-min_ct)/(max_ct-min_ct), (p[1]-min_wt)/(max_wt-min_wt)] for p in pts]
        
        norm_ic = norm(ic)
        norm_bm = norm(bm)
        
        # Pareto fronts
        pf_ic = get_pareto_front(norm_ic)
        pf_bm = get_pareto_front(norm_bm)
        
        # HV
        hv_ic = hypervolume_2d(pf_ic, [1.1, 1.1])
        hv_bm = hypervolume_2d(pf_bm, [1.1, 1.1])
        
        if hv_bm > 0:
            hv_ratios.append(hv_ic / hv_bm)
        
        # IGD
        igd = calculate_igd(pf_ic, pf_bm)
        igd_values.append(igd)
        
        # Domination
        if hv_ic > hv_bm * 1.01:
            wins += 1
        elif hv_bm > hv_ic * 1.01:
            loses += 1
        else:
            ties += 1
    
    avg_hv = np.mean(hv_ratios) if hv_ratios else 1.0
    gap_hv = (avg_hv - 1.0) * 100
    avg_igd = np.mean(igd_values) if igd_values else 0
    
    results.append({
        'Dataset': size,
        'GridSize': grid,
        'HV': avg_hv,
        'GapHV(%)': gap_hv,
        '+/=/-': f"{wins}/{ties}/{loses}",
        'IGD': avg_igd,
        'Wins': wins,
        'Ties': ties,
        'Loses': loses
    })

# Create DataFrame
df = pd.DataFrame(results)

# Print main table (like the template)
print("\n" + "=" * 95)
print("TABLE 1: HYPERVOLUME COMPARISON")
print("=" * 95)
print(f"{'Dataset':<10} {'GridSize':<12} {'HV':<12} {'GapHV(%)':<15} {'+/=/-':<15}")
print("-" * 95)

for _, row in df.iterrows():
    print(f"{row['Dataset']:<10} {row['GridSize']:<12} {row['HV']:<12.2f} {row['GapHV(%)']:<15.2f} {row['+/=/-']:<15}")

print("-" * 95)

# Averages by size
for size in [20, 50, 100, 200]:
    size_df = df[df['Dataset'] == size]
    if not size_df.empty:
        avg_hv = size_df['HV'].mean()
        avg_gap = size_df['GapHV(%)'].mean()
        total_w = size_df['Wins'].sum()
        total_t = size_df['Ties'].sum()
        total_l = size_df['Loses'].sum()
        print(f"Average {size:<3}C {'':<8} {avg_hv:<12.2f} {avg_gap:<15.2f} {total_w}/{total_t}/{total_l}")

print("-" * 95)

# Overall summary
total_hv = df['HV'].mean()
total_gap = df['GapHV(%)'].mean()
total_w = df['Wins'].sum()
total_t = df['Ties'].sum()
total_l = df['Loses'].sum()

print(f"Summary      {'':<12} {total_hv:<12.2f} {total_gap:<15.2f} {total_w}/{total_t}/{total_l}")
print("=" * 95)

# IGD Table
print("\n" + "=" * 70)
print("TABLE 2: IGD (INVERTED GENERATIONAL DISTANCE)")
print("=" * 70)
print(f"{'Dataset':<15} {'GridSize':<15} {'Avg IGD':<15} {'Status':<15}")
print("-" * 70)

for _, row in df.iterrows():
    status = "Good" if row['IGD'] < 0.1 else "Moderate" if row['IGD'] < 0.3 else "Poor"
    print(f"{row['Dataset']:<15} {row['GridSize']:<15} {row['IGD']:<15.4f} {status:<15}")

print("=" * 70)

# Domination summary
print("\n" + "=" * 70)
print("TABLE 3: DOMINATION SUMMARY")
print("=" * 70)
print(f"{'Dataset':<15} {'Instances':<12} {'Wins':<10} {'Ties':<10} {'Loses':<10} {'Win Rate %':<12}")
print("-" * 70)

for size in [20, 50, 100, 200]:
    size_df = df[df['Dataset'] == size]
    if not size_df.empty:
        total = len(size_df)
        w = size_df['Wins'].sum()
        t = size_df['Ties'].sum()
        l = size_df['Loses'].sum()
        win_rate = (w / (w + t + l)) * 100 if (w + t + l) > 0 else 0
        print(f"{size}C{'':<12} {total:<12} {w:<10} {t:<10} {l:<10} {win_rate:<12.1f}")

print("-" * 70)
total_instances = len(df)
print(f"{'Total':<15} {total_instances:<12} {total_w:<10} {total_t:<10} {total_l:<10} "
      f"{(total_w/(total_w+total_t+total_l)*100):<12.1f}")
print("=" * 70)
print()

# Save to CSV
df.to_csv('comparison_results.csv', index=False)
print("✓ Results saved to: comparison_results.csv")
print()
