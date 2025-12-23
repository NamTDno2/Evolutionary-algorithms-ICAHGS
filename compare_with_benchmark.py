#!/usr/bin/env python3
"""
Quick comparison script: ICAHGS results vs BENCHMARK
Generates summary statistics and visualizations
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    print("\n" + "="*80)
    print("COMPARISON: ICAHGS vs BENCHMARK")
    print("="*80 + "\n")
    
    # Load data
    print("Loading results...")
    new_results = pd.read_csv('results.csv')
    benchmark = pd.read_csv('results_benchmark.csv')
    
    # Get best solution for each dataset
    new_best = new_results.groupby('Dataset').apply(
        lambda x: x.loc[x['CompletionTime'].idxmin()]
    ).reset_index(drop=True)
    
    benchmark_best = benchmark.groupby('Dataset').apply(
        lambda x: x.loc[x['CompletionTime'].idxmin()]
    ).reset_index(drop=True)
    
    # Merge for comparison
    comparison = pd.merge(
        new_best[['Dataset', 'CompletionTime', 'TotalWaitingTime', 'ExecutionTime']],
        benchmark_best[['Dataset', 'CompletionTime', 'TotalWaitingTime']],
        on='Dataset',
        suffixes=('_ICAHGS', '_BENCH')
    )
    
    # Parse dataset components correctly
    comparison['Customers'] = comparison['Dataset'].str.split('.').str[0].astype(int)
    comparison['InstanceCode'] = comparison['Dataset'].str.split('.').str[1].astype(int)
    comparison['Instance'] = comparison['Dataset'].str.split('.').str[2].astype(int)
    
    # Get actual truck count from files
    def get_truck_count(dataset):
        try:
            with open(f'data/{dataset}.txt', 'r') as f:
                first_line = f.readline()
                return int(first_line.split()[-1])
        except:
            return 0
    
    comparison['ActualTrucks'] = comparison['Dataset'].apply(get_truck_count)
    
    # Calculate differences
    comparison['CT_Diff_%'] = ((comparison['CompletionTime_ICAHGS'] - comparison['CompletionTime_BENCH']) 
                                / comparison['CompletionTime_BENCH'] * 100)
    comparison['WT_Diff_%'] = ((comparison['TotalWaitingTime_ICAHGS'] - comparison['TotalWaitingTime_BENCH']) 
                                / comparison['TotalWaitingTime_BENCH'] * 100)
    
    # Win/Loss analysis
    ct_wins = (comparison['CT_Diff_%'] < 0).sum()
    ct_losses = (comparison['CT_Diff_%'] >= 0).sum()
    
    print(f"\n{'OVERALL SUMMARY':^80}")
    print("="*80)
    print(f"Total datasets: {len(comparison)}")
    print(f"\nCompletion Time (CT):")
    print(f"  ICAHGS better (lower CT): {ct_wins}/{len(comparison)} ({ct_wins/len(comparison)*100:.1f}%)")
    print(f"  BENCHMARK better: {ct_losses}/{len(comparison)} ({ct_losses/len(comparison)*100:.1f}%)")
    print(f"  Average difference: {comparison['CT_Diff_%'].mean():.2f}%")
    print(f"  Median difference: {comparison['CT_Diff_%'].median():.2f}%")
    
    # By dataset size with actual truck count
    print(f"\n{'BY DATASET SIZE (with actual truck counts)':^80}")
    print("="*80)
    
    for size in [20, 50, 100, 200]:
        size_data = comparison[comparison['Customers'] == size]
        if len(size_data) == 0:
            continue
        size_wins = (size_data['CT_Diff_%'] < 0).sum()
        size_total = len(size_data)
        avg_diff = size_data['CT_Diff_%'].mean()
        avg_time = size_data['ExecutionTime'].mean()
        actual_trucks = size_data['ActualTrucks'].iloc[0]
        
        print(f"\n{size} customers [{actual_trucks} trucks] ({size_total} datasets):")
        print(f"  WIN: {size_wins}/{size_total} ({size_wins/size_total*100:.1f}%)")
        print(f"  Average CT difference: {avg_diff:+.2f}%")
        print(f"  Average execution time: {avg_time:.1f}s")
    
    # By instance code (the REAL pattern!)
    print(f"\n{'BY INSTANCE CODE (5/10/20/30/40)':^80}")
    print("="*80)
    
    for code in [5, 10, 20, 30, 40]:
        code_data = comparison[comparison['InstanceCode'] == code]
        if len(code_data) == 0:
            continue
        code_wins = (code_data['CT_Diff_%'] < 0).sum()
        code_total = len(code_data)
        avg_diff = code_data['CT_Diff_%'].mean()
        
        print(f"\nCode {code} ({code_total} datasets):")
        print(f"  WIN: {code_wins}/{code_total} ({code_wins/code_total*100:.1f}%)")
        print(f"  Average CT difference: {avg_diff:+.2f}%")
    
    # Top improvements
    print(f"\n{'TOP 10 IMPROVEMENTS':^80}")
    print("="*80)
    top_improvements = comparison.nsmallest(10, 'CT_Diff_%')
    for idx, row in top_improvements.iterrows():
        print(f"  {row['Dataset']:12} | ICAHGS: {row['CompletionTime_ICAHGS']:8.2f} | "
              f"BENCH: {row['CompletionTime_BENCH']:8.2f} | Diff: {row['CT_Diff_%']:+.2f}%")
    
    # Areas for improvement
    print(f"\n{'TOP 10 AREAS FOR IMPROVEMENT':^80}")
    print("="*80)
    top_losses = comparison.nlargest(10, 'CT_Diff_%')
    for idx, row in top_losses.iterrows():
        print(f"  {row['Dataset']:12} | ICAHGS: {row['CompletionTime_ICAHGS']:8.2f} | "
              f"BENCH: {row['CompletionTime_BENCH']:8.2f} | Diff: {row['CT_Diff_%']:+.2f}%")
    
    # Create visualization
    print("\nGenerating charts...")
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('ICAHGS vs BENCHMARK Comparison', fontsize=16, fontweight='bold')
    
    # 1. Win/Loss bar chart
    ax = axes[0, 0]
    categories = ['WIN', 'LOSE']
    values = [ct_wins, ct_losses]
    colors = ['#2ecc71', '#e74c3c']
    ax.bar(categories, values, color=colors, edgecolor='black', linewidth=1.5)
    ax.set_ylabel('Number of Datasets')
    ax.set_title('Overall Win/Loss Count')
    ax.set_ylim(0, max(values) * 1.2)
    for i, v in enumerate(values):
        ax.text(i, v + 1, str(v), ha='center', va='bottom', fontweight='bold')
    
    # 2. Win rate by instance code (THE REAL PATTERN!)
    ax = axes[0, 1]
    instance_codes = [5, 10, 20, 30, 40]
    win_rates = []
    dataset_counts = []
    for code in instance_codes:
        code_data = comparison[comparison['InstanceCode'] == code]
        if len(code_data) > 0:
            win_rate = (code_data['CT_Diff_%'] < 0).sum() / len(code_data) * 100
            win_rates.append(win_rate)
            dataset_counts.append(len(code_data))
        else:
            win_rates.append(0)
            dataset_counts.append(0)
    
    colors_code = ['#2ecc71' if rate >= 50 else '#e74c3c' for rate in win_rates]
    bars = ax.bar([str(c) for c in instance_codes], win_rates, color=colors_code,
                   edgecolor='black', linewidth=1.5, alpha=0.8)
    ax.axhline(50, color='gray', linestyle='--', linewidth=1.5, label='50% threshold')
    ax.set_xlabel('Instance Code')
    ax.set_ylabel('Win Rate (%)')
    ax.set_title('Win Rate by Instance Code (Difficulty Level)')
    ax.set_ylim(0, 100)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for bar, rate, count in zip(bars, win_rates, dataset_counts):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 2,
                f'{rate:.1f}%\n(n={count})', ha='center', va='bottom', 
                fontsize=9, fontweight='bold')
    
    # 3. Win rate by size (with actual truck counts shown)
    ax = axes[1, 0]
    sizes = [20, 50, 100, 200]
    win_rates = []
    truck_labels = []
    for size in sizes:
        size_data = comparison[comparison['Customers'] == size]
        if len(size_data) > 0:
            win_rate = (size_data['CT_Diff_%'] < 0).sum() / len(size_data) * 100
            win_rates.append(win_rate)
            trucks = size_data['ActualTrucks'].iloc[0]
            truck_labels.append(f'{size}C\n({trucks}T)')
        else:
            win_rates.append(0)
            truck_labels.append(f'{size}C')
    
    bars = ax.bar(truck_labels, win_rates, color=['#3498db', '#9b59b6', '#e67e22', '#1abc9c'],
                   edgecolor='black', linewidth=1.5)
    ax.axhline(50, color='red', linestyle='--', linewidth=1, label='50% threshold')
    ax.set_xlabel('Customers (Trucks)')
    ax.set_ylabel('Win Rate (%)')
    ax.set_title('Win Rate by Dataset Size')
    ax.set_ylim(0, 100)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    for bar, rate in zip(bars, win_rates):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 2,
                f'{rate:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    # 4. Scatter plot
    ax = axes[1, 1]
    colors = ['green' if d < 0 else 'red' for d in comparison['CT_Diff_%']]
    ax.scatter(comparison['CompletionTime_BENCH'], comparison['CompletionTime_ICAHGS'], 
               c=colors, alpha=0.6, s=50, edgecolors='black', linewidth=0.5)
    max_val = max(comparison['CompletionTime_BENCH'].max(), comparison['CompletionTime_ICAHGS'].max())
    ax.plot([0, max_val], [0, max_val], 'k--', linewidth=1, label='Equal performance')
    ax.set_xlabel('Benchmark CT')
    ax.set_ylabel('ICAHGS CT')
    ax.set_title('CT Scatter Plot (Green=Better, Red=Worse)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('comparison_ICAHGS_vs_BENCHMARK.png', dpi=300, bbox_inches='tight')
    print("✓ Chart saved: comparison_ICAHGS_vs_BENCHMARK.png")
    
    # Save detailed comparison
    comparison_sorted = comparison.sort_values('CT_Diff_%')
    comparison_sorted.to_csv('comparison_ICAHGS_vs_BENCHMARK.csv', index=False)
    print("✓ Detailed data saved: comparison_ICAHGS_vs_BENCHMARK.csv")
    
    print("\n" + "="*80)
    print("ANALYSIS COMPLETE!")
    print("="*80 + "\n")

if __name__ == '__main__':
    main()
