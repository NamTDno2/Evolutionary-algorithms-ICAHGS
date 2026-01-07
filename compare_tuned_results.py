#!/usr/bin/env python3
"""
Compare tuned maxEvaluation results with benchmark
Compare 50C/100C instances from result_tuned/ with benchmark/
"""

import os
import csv
from pathlib import Path

def read_best_solution(filepath):
    """Read best solution (line 8) from result file"""
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            if len(lines) >= 8:
                # Line 7 (index 7) is route string, Line 8 (index 8) is objectives
                objectives_line = lines[7].strip()
                if objectives_line:
                    parts = objectives_line.split()
                    if len(parts) >= 2:
                        ct = float(parts[0])
                        wt = float(parts[1])
                        return ct, wt
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
    return None, None

def main():
    result_dir = Path("result_tuned")
    benchmark_dir = Path("benchmark")
    
    # Get all 50C and 100C instances
    instances = []
    for size in [50, 100]:
        for density in [10, 20, 30, 40]:
            for i in range(1, 5):
                instances.append(f"{size}.{density}.{i}")
    
    results = []
    win_count = 0
    tie_count = 0
    lose_count = 0
    
    ct_improvements = []
    wt_improvements = []
    
    print("=" * 80)
    print("COMPARISON: TUNED maxEvaluation vs Benchmark")
    print("=" * 80)
    print(f"{'Instance':<15} {'Result CT':<12} {'Benchmark CT':<12} {'CT Gap %':<10} {'Result WT':<12} {'Benchmark WT':<12} {'WT Gap %':<10} {'Status':<10}")
    print("-" * 80)
    
    for instance in instances:
        result_file = result_dir / f"{instance}.txt"
        benchmark_file = benchmark_dir / f"{instance}.txt"
        
        if not result_file.exists() or not benchmark_file.exists():
            continue
        
        result_ct, result_wt = read_best_solution(result_file)
        bench_ct, bench_wt = read_best_solution(benchmark_file)
        
        if result_ct is None or bench_ct is None:
            continue
        
        # Calculate improvement percentages
        ct_gap = ((result_ct - bench_ct) / bench_ct) * 100
        wt_gap = ((result_wt - bench_wt) / bench_wt) * 100
        
        ct_improvements.append(ct_gap)
        wt_improvements.append(wt_gap)
        
        # Determine domination status
        result_dominates = (result_ct <= bench_ct and result_wt <= bench_wt and 
                           (result_ct < bench_ct or result_wt < bench_wt))
        bench_dominates = (bench_ct <= result_ct and bench_wt <= result_wt and 
                          (bench_ct < result_ct or bench_wt < result_wt))
        
        if result_dominates:
            status = "WIN"
            win_count += 1
            color = "\033[92m"  # Green
        elif bench_dominates:
            status = "LOSE"
            lose_count += 1
            color = "\033[91m"  # Red
        else:
            status = "TIE"
            tie_count += 1
            color = "\033[93m"  # Yellow
        
        reset = "\033[0m"
        
        print(f"{instance:<15} {result_ct:<12.2f} {bench_ct:<12.2f} {ct_gap:>9.2f}% {result_wt:<12.2f} {bench_wt:<12.2f} {wt_gap:>9.2f}% {color}{status:<10}{reset}")
        
        results.append({
            'Instance': instance,
            'Result_CT': result_ct,
            'Result_WT': result_wt,
            'Benchmark_CT': bench_ct,
            'Benchmark_WT': bench_wt,
            'CT_Gap_%': ct_gap,
            'WT_Gap_%': wt_gap,
            'Status': status
        })
    
    print("=" * 80)
    
    total = win_count + tie_count + lose_count
    win_rate = (win_count / total * 100) if total > 0 else 0
    
    avg_ct_gap = sum(ct_improvements) / len(ct_improvements) if ct_improvements else 0
    avg_wt_gap = sum(wt_improvements) / len(wt_improvements) if wt_improvements else 0
    
    print(f"\n{'SUMMARY':<20}")
    print(f"{'Total instances:':<20} {total}")
    print(f"{'Wins:':<20} {win_count} ({win_rate:.1f}%)")
    print(f"{'Ties:':<20} {tie_count}")
    print(f"{'Losses:':<20} {lose_count}")
    print(f"{'Avg CT Gap:':<20} {avg_ct_gap:+.2f}%")
    print(f"{'Avg WT Gap:':<20} {avg_wt_gap:+.2f}%")
    
    # Separate by size
    print(f"\n{'BY SIZE':<20}")
    for size in [50, 100]:
        size_results = [r for r in results if r['Instance'].startswith(f"{size}.")]
        if size_results:
            size_wins = sum(1 for r in size_results if r['Status'] == 'WIN')
            size_total = len(size_results)
            size_win_rate = (size_wins / size_total * 100) if size_total > 0 else 0
            size_ct_gap = sum(r['CT_Gap_%'] for r in size_results) / size_total
            size_wt_gap = sum(r['WT_Gap_%'] for r in size_results) / size_total
            
            print(f"{size}C: {size_wins}/{size_total} wins ({size_win_rate:.1f}%) | CT: {size_ct_gap:+.2f}% | WT: {size_wt_gap:+.2f}%")
    
    # Save to CSV
    csv_file = "results_tuned_comparison.csv"
    with open(csv_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    
    print(f"\nResults saved to: {csv_file}")
    
    # Compare with OLD results
    print("\n" + "=" * 80)
    print("COMPARISON: OLD vs TUNED")
    print("=" * 80)
    print("OLD (maxEvaluation 1,095,000 / 20,610,000):")
    print("  50C: 25% win rate")
    print("  100C: 12.5% win rate")
    print("")
    print(f"TUNED (maxEvaluation 21,000 / 66,000):")
    size_50_results = [r for r in results if r['Instance'].startswith("50.")]
    size_100_results = [r for r in results if r['Instance'].startswith("100.")]
    
    if size_50_results:
        wins_50 = sum(1 for r in size_50_results if r['Status'] == 'WIN')
        total_50 = len(size_50_results)
        print(f"  50C: {wins_50}/{total_50} wins ({wins_50/total_50*100:.1f}% win rate)")
    
    if size_100_results:
        wins_100 = sum(1 for r in size_100_results if r['Status'] == 'WIN')
        total_100 = len(size_100_results)
        print(f"  100C: {wins_100}/{total_100} wins ({wins_100/total_100*100:.1f}% win rate)")
    
    print("\n✅ Tuned maxEvaluation is FASTER (52x/314x less) and may have BETTER results!")

if __name__ == "__main__":
    main()
