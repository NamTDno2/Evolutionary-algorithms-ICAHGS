import os

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

def dominates(a, b):
    """Check if solution a dominates solution b"""
    return (a[0] <= b[0] and a[1] < b[1]) or (a[0] < b[0] and a[1] <= b[1])

def get_pareto_front(points):
    """Extract Pareto front"""
    if not points:
        return []
    
    pareto = []
    for p in points:
        dominated = False
        for other in points:
            if dominates(other, p):
                dominated = True
                break
        if not dominated:
            pareto.append(p)
    return sorted(pareto, key=lambda x: x[0])

# Check tie instances
tie_files = ['20.10.2.txt', '100.30.2.txt', '100.40.3.txt', '100.40.4.txt']

for fname in tie_files:
    print("\n" + "=" * 80)
    print(f"FILE: {fname}")
    print("=" * 80)
    
    ic_sols = read_solutions(f'result/{fname}')
    bm_sols = read_solutions(f'benchmark/{fname}')
    
    print(f"\nICACHGS: {len(ic_sols)} solutions")
    for i, sol in enumerate(ic_sols):
        print(f"  {i+1}. CT={sol[0]:.2f}, WT={sol[1]:.2f}")
    
    print(f"\nBENCHMARK: {len(bm_sols)} solutions")
    for i, sol in enumerate(bm_sols):
        print(f"  {i+1}. CT={sol[0]:.2f}, WT={sol[1]:.2f}")
    
    # Check domination
    print("\n--- DOMINATION ANALYSIS ---")
    
    # ICAHGS dominates Benchmark
    ic_dominates_bm = 0
    for bm in bm_sols:
        for ic in ic_sols:
            if dominates(ic, bm):
                ic_dominates_bm += 1
                break
    
    # Benchmark dominates ICAHGS
    bm_dominates_ic = 0
    for ic in ic_sols:
        for bm in bm_sols:
            if dominates(bm, ic):
                bm_dominates_ic += 1
                break
    
    print(f"ICAHGS dominates {ic_dominates_bm}/{len(bm_sols)} Benchmark solutions")
    print(f"Benchmark dominates {bm_dominates_ic}/{len(ic_sols)} ICAHGS solutions")
    print(f"C(ICAHGS, Benchmark) = {ic_dominates_bm/len(bm_sols):.4f}")
    print(f"C(Benchmark, ICAHGS) = {bm_dominates_ic/len(ic_sols):.4f}")
    
    # Get Pareto fronts
    pf_ic = get_pareto_front(ic_sols)
    pf_bm = get_pareto_front(bm_sols)
    
    print(f"\nPareto Front ICAHGS: {len(pf_ic)} points")
    print(f"Pareto Front Benchmark: {len(pf_bm)} points")
    
    # Check if they overlap
    print("\n--- OVERLAP ANALYSIS ---")
    for ic in pf_ic:
        for bm in pf_bm:
            if abs(ic[0] - bm[0]) < 0.01 and abs(ic[1] - bm[1]) < 0.01:
                print(f"  OVERLAP: IC {ic} ≈ BM {bm}")
    
    # Check relative positions
    if pf_ic and pf_bm:
        ic_min_ct = min(p[0] for p in pf_ic)
        ic_max_ct = max(p[0] for p in pf_ic)
        bm_min_ct = min(p[0] for p in pf_bm)
        bm_max_ct = max(p[0] for p in pf_bm)
        
        print(f"\nCT Range:")
        print(f"  ICAHGS: [{ic_min_ct:.2f}, {ic_max_ct:.2f}]")
        print(f"  Benchmark: [{bm_min_ct:.2f}, {bm_max_ct:.2f}]")
        
        if ic_max_ct < bm_min_ct or bm_max_ct < ic_min_ct:
            print("  → NO OVERLAP in CT dimension (completely separated)")
        else:
            print("  → OVERLAP in CT dimension")
