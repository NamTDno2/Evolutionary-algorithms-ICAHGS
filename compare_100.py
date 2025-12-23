import pandas as pd

# Load data
old = pd.read_csv('results_old.csv')
new = pd.read_csv('results.csv')
benchmark = pd.read_csv('results_benchmark.csv')

# Filter 100 customers only
old_100 = old[old['Dataset'].str.startswith('100.')]
new_100 = new[new['Dataset'].str.startswith('100.')]
bench_100 = benchmark[benchmark['Dataset'].str.startswith('100.')]

# Get best for each dataset
old_best = old_100.loc[old_100.groupby('Dataset')['CompletionTime'].idxmin()]
new_best = new_100.loc[new_100.groupby('Dataset')['CompletionTime'].idxmin()]
bench_best = bench_100.loc[bench_100.groupby('Dataset')['CompletionTime'].idxmin()]

# Merge
comparison = pd.merge(
    old_best[['Dataset', 'CompletionTime', 'TotalWaitingTime', 'ExecutionTime']],
    new_best[['Dataset', 'CompletionTime', 'TotalWaitingTime', 'ExecutionTime']],
    on='Dataset',
    suffixes=('_Old', '_New')
)

comparison = pd.merge(
    comparison,
    bench_best[['Dataset', 'CompletionTime', 'TotalWaitingTime']],
    on='Dataset'
)
comparison.rename(columns={'CompletionTime': 'CT_Bench', 'TotalWaitingTime': 'WT_Bench'}, inplace=True)

# Calculate improvements
comparison['CT_vs_Old'] = ((comparison['CompletionTime_New'] - comparison['CompletionTime_Old']) / comparison['CompletionTime_Old'] * 100).round(1)
comparison['WT_vs_Old'] = ((comparison['TotalWaitingTime_New'] - comparison['TotalWaitingTime_Old']) / comparison['TotalWaitingTime_Old'] * 100).round(1)
comparison['CT_vs_Bench'] = ((comparison['CompletionTime_New'] - comparison['CT_Bench']) / comparison['CT_Bench'] * 100).round(1)
comparison['WT_vs_Bench'] = ((comparison['TotalWaitingTime_New'] - comparison['WT_Bench']) / comparison['WT_Bench'] * 100).round(1)
comparison['Time_Speedup'] = (comparison['ExecutionTime_Old'] / comparison['ExecutionTime_New']).round(1)

print("="*100)
print("SO SÁNH KẾT QUẢ 100 CUSTOMERS: OLD vs NEW vs BENCHMARK")
print("="*100)
print(f"\n{'Dataset':<12} {'CT_Old':<10} {'CT_New':<10} {'CT_Bench':<10} {'vs_Old':<8} {'vs_Bench':<10} {'WT_vs_Old':<10} {'WT_vs_Bench':<12} {'Speedup':<8}")
print("-"*100)

for _, row in comparison.iterrows():
    ct_old_color = ""
    ct_bench_color = ""
    
    # Color coding
    vs_old_str = f"{row['CT_vs_Old']:+.1f}%"
    vs_bench_str = f"{row['CT_vs_Bench']:+.1f}%"
    wt_old_str = f"{row['WT_vs_Old']:+.1f}%"
    wt_bench_str = f"{row['WT_vs_Bench']:+.1f}%"
    
    print(f"{row['Dataset']:<12} {row['CompletionTime_Old']:<10.1f} {row['CompletionTime_New']:<10.1f} {row['CT_Bench']:<10.1f} {vs_old_str:<8} {vs_bench_str:<10} {wt_old_str:<10} {wt_bench_str:<12} {row['Time_Speedup']:<8}x")

print("-"*100)
print(f"\nTOTAL DATASETS: {len(comparison)}")
print(f"NEW better than OLD (CT): {(comparison['CT_vs_Old'] < 0).sum()}/{len(comparison)} datasets")
print(f"NEW better than Benchmark (CT): {(comparison['CT_vs_Bench'] < 0).sum()}/{len(comparison)} datasets")
print(f"NEW better than OLD (WT): {(comparison['WT_vs_Old'] < 0).sum()}/{len(comparison)} datasets")
print(f"NEW better than Benchmark (WT): {(comparison['WT_vs_Bench'] < 0).sum()}/{len(comparison)} datasets")
print(f"\nAverage CT change vs Old: {comparison['CT_vs_Old'].mean():.1f}%")
print(f"Average CT change vs Benchmark: {comparison['CT_vs_Bench'].mean():.1f}%")
print(f"Average WT change vs Old: {comparison['WT_vs_Old'].mean():.1f}%")
print(f"Average WT change vs Benchmark: {comparison['WT_vs_Bench'].mean():.1f}%")
print(f"Average speedup: {comparison['Time_Speedup'].mean():.1f}x (was {comparison['ExecutionTime_Old'].mean():.1f}s, now {comparison['ExecutionTime_New'].mean():.1f}s)")

# Show worst cases
print("\n" + "="*100)
print("TOP 5 WORST vs BENCHMARK (CT):")
print("="*100)
worst = comparison.nlargest(5, 'CT_vs_Bench')
for _, row in worst.iterrows():
    print(f"{row['Dataset']}: NEW={row['CompletionTime_New']:.1f} vs BENCH={row['CT_Bench']:.1f} ({row['CT_vs_Bench']:+.1f}%)")

print("\n" + "="*100)
print("TOP 5 BEST vs BENCHMARK (CT):")
print("="*100)
best = comparison.nsmallest(5, 'CT_vs_Bench')
for _, row in best.iterrows():
    print(f"{row['Dataset']}: NEW={row['CompletionTime_New']:.1f} vs BENCH={row['CT_Bench']:.1f} ({row['CT_vs_Bench']:+.1f}%)")
