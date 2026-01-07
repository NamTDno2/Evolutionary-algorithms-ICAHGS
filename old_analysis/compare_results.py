import os
import numpy as np
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

def compare_instances(icahgs_list, benchmark_list):
    """Compare instances and return win/tie/lose"""
    wins, ties, loses = 0, 0, 0
    
    for ic, bm in zip(icahgs_list, benchmark_list):
        # Get combined min/max for normalization
        all_points = ic + bm
        min_ct = min(p[0] for p in all_points)
        max_ct = max(p[0] for p in all_points)
        min_wt = min(p[1] for p in all_points)
        max_wt = max(p[1] for p in all_points)
        
        # Normalize
        def normalize(pts):
            if max_ct == min_ct or max_wt == min_wt:
                return pts
            return [[(p[0]-min_ct)/(max_ct-min_ct), (p[1]-min_wt)/(max_wt-min_wt)] for p in pts]
        
        norm_ic = normalize(ic)
        norm_bm = normalize(bm)
        
        # Get Pareto fronts
        pf_ic = get_pareto_front(norm_ic)
        pf_bm = get_pareto_front(norm_bm)
        
        # Calculate HV with ref point (1.1, 1.1)
        hv_ic = hypervolume_2d(pf_ic, [1.1, 1.1])
        hv_bm = hypervolume_2d(pf_bm, [1.1, 1.1])
        
        # Compare
        if hv_ic > hv_bm * 1.01:
            wins += 1
        elif hv_bm > hv_ic * 1.01:
            loses += 1
        else:
            ties += 1
    
    return wins, ties, loses

# Read all files
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

# Print table
print("\n" + "=" * 90)
print("PERFORMANCE COMPARISON: ICAHGS vs BENCHMARK")
print("=" * 90)
print(f"{'Dataset':<12} {'GridSize':<12} {'HV Ratio':<12} {'GapHV (%)':<15} {'+/=/-'}")
print("-" * 90)

summary = defaultdict(lambda: {'ratios': [], 'gaps': [], 'w': 0, 't': 0, 'l': 0})

for (size, grid) in sorted(data_by_group.keys()):
    d = data_by_group[(size, grid)]
    
    # Calculate HV for each instance
    hv_ratios = []
    for ic, bm in zip(d['icahgs'], d['benchmark']):
        all_pts = ic + bm
        min_ct = min(p[0] for p in all_pts)
        max_ct = max(p[0] for p in all_pts)
        min_wt = min(p[1] for p in all_pts)
        max_wt = max(p[1] for p in all_pts)
        
        def norm(pts):
            if max_ct == min_ct or max_wt == min_wt:
                return pts
            return [[(p[0]-min_ct)/(max_ct-min_ct), (p[1]-min_wt)/(max_wt-min_wt)] for p in pts]
        
        pf_ic = get_pareto_front(norm(ic))
        pf_bm = get_pareto_front(norm(bm))
        
        hv_ic = hypervolume_2d(pf_ic, [1.1, 1.1])
        hv_bm = hypervolume_2d(pf_bm, [1.1, 1.1])
        
        if hv_bm > 0:
            hv_ratios.append(hv_ic / hv_bm)
    
    avg_ratio = np.mean(hv_ratios) if hv_ratios else 1.0
    gap = (avg_ratio - 1.0) * 100
    
    w, t, l = compare_instances(d['icahgs'], d['benchmark'])
    
    summary[size]['ratios'].append(avg_ratio)
    summary[size]['gaps'].append(gap)
    summary[size]['w'] += w
    summary[size]['t'] += t
    summary[size]['l'] += l
    
    print(f"{size:<12} {grid:<12} {avg_ratio:<12.2f} {gap:<15.2f} {w}/{t}/{l}")

print("-" * 90)

# Averages
for size in [20, 50, 100, 200]:
    if size in summary:
        s = summary[size]
        avg_r = np.mean(s['ratios'])
        avg_g = np.mean(s['gaps'])
        print(f"Average {size:<4} {'':<8} {avg_r:<12.2f} {avg_g:<15.2f} {s['w']}/{s['t']}/{s['l']}")

print("-" * 90)

# Overall
all_ratios = [r for s in summary.values() for r in s['ratios']]
all_gaps = [g for s in summary.values() for g in s['gaps']]
total_w = sum(s['w'] for s in summary.values())
total_t = sum(s['t'] for s in summary.values())
total_l = sum(s['l'] for s in summary.values())

print(f"Summary      {'':<8} {np.mean(all_ratios):<12.2f} {np.mean(all_gaps):<15.2f} {total_w}/{total_t}/{total_l}")
print("=" * 90)
print()
