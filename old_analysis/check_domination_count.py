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

def dominates(a, b):
    """Check if solution a dominates solution b"""
    return (a[0] <= b[0] and a[1] < b[1]) or (a[0] < b[0] and a[1] <= b[1])

def coverage_metric(A, B):
    """Calculate C(A, B)"""
    if not B:
        return 0.0
    
    dominated_count = 0
    for b in B:
        for a in A:
            if dominates(a, b):
                dominated_count += 1
                break
    
    return dominated_count / len(B)

# Read all files
result_files = sorted([f for f in os.listdir('result') if f.endswith('.txt')])
data_by_size = defaultdict(lambda: {
    'icahgs_wins': 0,
    'benchmark_wins': 0,
    'ties': 0,
    'instances': []
})

total_instances = 0
for fname in result_files:
    if os.path.exists(f'benchmark/{fname}'):
        parts = fname.replace('.txt', '').split('.')
        if len(parts) == 3:
            size = int(parts[0])
            total_instances += 1
            
            ic_sols = read_solutions(f'result/{fname}')
            bm_sols = read_solutions(f'benchmark/{fname}')
            
            if ic_sols and bm_sols:
                c_ib = coverage_metric(ic_sols, bm_sols)
                c_bi = coverage_metric(bm_sols, ic_sols)
                
                result = "TIE"
                if c_ib > c_bi:
                    data_by_size[size]['icahgs_wins'] += 1
                    result = "ICAHGS"
                elif c_bi > c_ib:
                    data_by_size[size]['benchmark_wins'] += 1
                    result = "BENCHMARK"
                else:
                    data_by_size[size]['ties'] += 1
                
                data_by_size[size]['instances'].append({
                    'file': fname,
                    'c_ib': c_ib,
                    'c_bi': c_bi,
                    'result': result
                })

print("=" * 80)
print("DETAILED DOMINATION COUNT ANALYSIS")
print("=" * 80)

sizes = [20, 50, 100, 200]
for size in sizes:
    d = data_by_size[size]
    total = d['icahgs_wins'] + d['benchmark_wins'] + d['ties']
    
    print(f"\n{size}C: Total {total} instances")
    print(f"  ICAHGS wins: {d['icahgs_wins']}")
    print(f"  Benchmark wins: {d['benchmark_wins']}")
    print(f"  Ties: {d['ties']}")
    
    if d['ties'] > 0:
        print(f"\n  Tie instances:")
        for inst in d['instances']:
            if inst['result'] == 'TIE':
                print(f"    {inst['file']}: C(I,B)={inst['c_ib']:.4f}, C(B,I)={inst['c_bi']:.4f}")

print("\n" + "=" * 80)
print(f"TOTAL INSTANCES COUNTED: {total_instances}")
print(f"Expected: 60 (12+16+16+16)")
print("=" * 80)

# Count by size
counts_by_size = {}
for size in sizes:
    d = data_by_size[size]
    counts_by_size[size] = d['icahgs_wins'] + d['benchmark_wins'] + d['ties']

print(f"\nInstances by size:")
print(f"  20C: {counts_by_size[20]} (expected 12)")
print(f"  50C: {counts_by_size[50]} (expected 16)")
print(f"  100C: {counts_by_size[100]} (expected 16)")
print(f"  200C: {counts_by_size[200]} (expected 16)")
print(f"  Total: {sum(counts_by_size.values())}")
