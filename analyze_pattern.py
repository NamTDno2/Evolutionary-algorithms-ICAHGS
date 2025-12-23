#!/usr/bin/env python3
"""Analyze TRUE pattern of wins/losses"""

import pandas as pd

# Load data
results = pd.read_csv('results.csv')
benchmark = pd.read_csv('results_benchmark.csv')

# Get best CT for each dataset
results_best = results.groupby('Dataset')['CompletionTime'].min().reset_index()
benchmark_best = benchmark.groupby('Dataset')['CompletionTime'].min().reset_index()

# Merge
comparison = pd.merge(results_best, benchmark_best, on='Dataset', suffixes=('_ICAHGS', '_BENCH'))
comparison['WIN'] = comparison['CompletionTime_ICAHGS'] < comparison['CompletionTime_BENCH']

# Parse dataset name
comparison['Customers'] = comparison['Dataset'].str.split('.').str[0].astype(int)
comparison['Code'] = comparison['Dataset'].str.split('.').str[1].astype(int)
comparison['Instance'] = comparison['Dataset'].str.split('.').str[2].astype(int)

print("\n" + "="*80)
print("WIN/LOSE BY INSTANCE CODE (5/10/20/30/40)")
print("="*80)

for code in [5, 10, 20, 30, 40]:
    code_data = comparison[comparison['Code'] == code]
    if len(code_data) > 0:
        wins = code_data['WIN'].sum()
        total = len(code_data)
        win_rate = wins / total * 100
        color = '\033[92m' if win_rate >= 50 else '\033[91m'
        reset = '\033[0m'
        print(f"{color}Code {code:2d}: WIN {wins:2d}/{total:2d} ({win_rate:5.1f}%){reset}")

print("\n" + "="*80)
print("WIN/LOSE BY CUSTOMER COUNT (20/50/100/200)")
print("="*80)

for size in [20, 50, 100, 200]:
    size_data = comparison[comparison['Customers'] == size]
    wins = size_data['WIN'].sum()
    total = len(size_data)
    win_rate = wins / total * 100
    
    # Also show truck count
    from glob import glob
    sample_file = f'data/{size}.10.1.txt'
    try:
        with open(sample_file, 'r') as f:
            first_line = f.readline()
            trucks = int(first_line.split()[-1])
    except:
        trucks = '?'
    
    color = '\033[92m' if win_rate >= 50 else '\033[91m'
    reset = '\033[0m'
    print(f"{color}{size:3d} customers ({trucks} trucks): WIN {wins:2d}/{total:2d} ({win_rate:5.1f}%){reset}")

print("\n" + "="*80)
print("DETAILED BREAKDOWN: CODE × CUSTOMER SIZE")
print("="*80)

pivot = comparison.groupby(['Customers', 'Code'])['WIN'].agg(['sum', 'count'])
pivot['rate'] = (pivot['sum'] / pivot['count'] * 100).round(1)

print("\n       Code →")
print("Size ↓    5      10     20     30     40")
print("-" * 45)
for size in [20, 50, 100, 200]:
    row_str = f"{size:3d}   "
    for code in [5, 10, 20, 30, 40]:
        try:
            data = pivot.loc[(size, code)]
            wins = int(data['sum'])
            total = int(data['count'])
            rate = data['rate']
            row_str += f"{wins}/{total}({rate:4.0f}%) "
        except:
            row_str += "  N/A    "
    print(row_str)

print("\n" + "="*80)
