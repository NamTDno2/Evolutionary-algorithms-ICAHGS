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

# Test with 20.20.1 instance
print("Testing 20.20 instances:")
print("=" * 80)

for i in range(1, 5):
    fname = f'20.20.{i}.txt'
    ic_sols = read_solutions(f'result/{fname}')
    bm_sols = read_solutions(f'benchmark/{fname}')
    
    if ic_sols and bm_sols:
        print(f"\n{fname}:")
        print(f"  ICAHGS solutions: {len(ic_sols)}")
        print(f"  Benchmark solutions: {len(bm_sols)}")
        
        # Normalize together
        all_points = ic_sols + bm_sols
        min_ct = min(p[0] for p in all_points)
        max_ct = max(p[0] for p in all_points)
        min_wt = min(p[1] for p in all_points)
        max_wt = max(p[1] for p in all_points)
        
        print(f"  CT range: [{min_ct:.2f}, {max_ct:.2f}]")
        print(f"  WT range: [{min_wt:.2f}, {max_wt:.2f}]")
        
        def normalize(pts):
            if max_ct == min_ct or max_wt == min_wt:
                return pts
            return [[(p[0]-min_ct)/(max_ct-min_ct), (p[1]-min_wt)/(max_wt-min_wt)] for p in pts]
        
        norm_ic = normalize(ic_sols)
        norm_bm = normalize(bm_sols)
        
        # Get Pareto fronts
        pf_ic = get_pareto_front(norm_ic)
        pf_bm = get_pareto_front(norm_bm)
        
        print(f"  ICAHGS Pareto front size: {len(pf_ic)}")
        print(f"  Benchmark Pareto front size: {len(pf_bm)}")
        
        # Calculate HV with ref point (1.1, 1.1)
        hv_ic = hypervolume_2d(pf_ic, [1.1, 1.1])
        hv_bm = hypervolume_2d(pf_bm, [1.1, 1.1])
        
        print(f"  HV ICAHGS: {hv_ic:.4f}")
        print(f"  HV Benchmark: {hv_bm:.4f}")
        print(f"  Gap: {((hv_ic - hv_bm) / hv_bm * 100):.2f}%")

# Calculate average for 20.20
hvs_ic = []
hvs_bm = []
for i in range(1, 5):
    fname = f'20.20.{i}.txt'
    ic_sols = read_solutions(f'result/{fname}')
    bm_sols = read_solutions(f'benchmark/{fname}')
    
    if ic_sols and bm_sols:
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
        
        pf_ic = get_pareto_front(norm_ic)
        pf_bm = get_pareto_front(norm_bm)
        
        hv_ic = hypervolume_2d(pf_ic, [1.1, 1.1])
        hv_bm = hypervolume_2d(pf_bm, [1.1, 1.1])
        
        hvs_ic.append(hv_ic)
        hvs_bm.append(hv_bm)

print("\n" + "=" * 80)
print("AVERAGE for 20 × 20:")
print(f"  Average HV ICAHGS: {np.mean(hvs_ic):.4f}")
print(f"  Average HV Benchmark: {np.mean(hvs_bm):.4f}")
print(f"  Average Gap: {((np.mean(hvs_ic) - np.mean(hvs_bm)) / np.mean(hvs_bm) * 100):.2f}%")
