"""
Generate Pareto Front Comparison Plots
Compare CT vs WT distribution between result_final_perfect and benchmark
One PNG file per instance
"""

import matplotlib.pyplot as plt
import numpy as np
import os

def read_all_solutions_from_file(filepath):
    """Read ALL solutions (CT, WT pairs) from a result file"""
    solutions = []
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        # Line 6 (index 5) contains number of solutions
        if len(lines) < 6:
            return solutions
        
        try:
            num_solutions = int(lines[5].strip())
        except ValueError:
            return solutions
        
        # Starting from line 7 (index 6), format is:
        # Route line, then CT/WT line, then Route line, then CT/WT line...
        # So CT/WT lines are at indices: 7, 9, 11, 13, ...
        # Pattern: 6 (route), 7 (CT/WT), 8 (route), 9 (CT/WT), ...
        for i in range(num_solutions):
            ct_wt_line_index = 7 + (i * 2)  # 7, 9, 11, 13, ...
            
            if ct_wt_line_index >= len(lines):
                break
            
            line = lines[ct_wt_line_index].strip()
            if not line:
                continue
            
            parts = line.split()
            if len(parts) >= 2:
                try:
                    ct = float(parts[0])
                    wt = float(parts[1])
                    # Only add valid solutions (positive values)
                    if ct > 0 and wt > 0:
                        solutions.append((ct, wt))
                except ValueError:
                    continue
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
    
    return solutions

def create_pareto_plot(instance_name, result_solutions, benchmark_solutions, output_path):
    """Create a single Pareto front comparison plot"""
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Extract CT and WT, filter out invalid points (0,0) or negative values
    if result_solutions:
        result_ct = [s[0] for s in result_solutions if s[0] > 0 and s[1] > 0]
        result_wt = [s[1] for s in result_solutions if s[0] > 0 and s[1] > 0]
    else:
        result_ct, result_wt = [], []
    
    if benchmark_solutions:
        bench_ct = [s[0] for s in benchmark_solutions if s[0] > 0 and s[1] > 0]
        bench_wt = [s[1] for s in benchmark_solutions if s[0] > 0 and s[1] > 0]
    else:
        bench_ct, bench_wt = [], []
    
    # Plot benchmark (red squares with connecting line)
    if bench_ct and bench_wt:
        # Sort by CT for connecting line
        bench_sorted = sorted(zip(bench_ct, bench_wt))
        bench_ct_sorted = [s[0] for s in bench_sorted]
        bench_wt_sorted = [s[1] for s in bench_sorted]
        
        ax.plot(bench_ct_sorted, bench_wt_sorted, 's--', 
                color='#E57373', markersize=10, linewidth=2, alpha=0.6,
                markeredgecolor='darkred', markeredgewidth=1.5,
                label=f'Benchmark (n={len(bench_ct)})')
    
    # Plot result (green circles with connecting line)
    if result_ct and result_wt:
        # Sort by CT for connecting line
        result_sorted = sorted(zip(result_ct, result_wt))
        result_ct_sorted = [s[0] for s in result_sorted]
        result_wt_sorted = [s[1] for s in result_sorted]
        
        ax.plot(result_ct_sorted, result_wt_sorted, 'o--', 
                color='#66BB6A', markersize=12, linewidth=2, alpha=0.8,
                markeredgecolor='darkgreen', markeredgewidth=1.5,
                label=f'ICAHGS (n={len(result_ct)})')
    
    # Labels and title
    ax.set_xlabel('Completion Time (CT) [seconds]', fontsize=14, fontweight='bold')
    ax.set_ylabel('Waiting Time (WT) [seconds]', fontsize=14, fontweight='bold')
    ax.set_title(f'Pareto Front Comparison: {instance_name}\nICAHGS vs Benchmark', 
                 fontsize=16, fontweight='bold')
    
    # Grid and legend
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='best', fontsize=12, framealpha=0.9)
    
    # Add margins
    if result_ct and bench_ct:
        all_ct = result_ct + bench_ct
        all_wt = result_wt + bench_wt
        
        ct_range = max(all_ct) - min(all_ct)
        wt_range = max(all_wt) - min(all_wt)
        
        ax.set_xlim(min(all_ct) - ct_range*0.05, max(all_ct) + ct_range*0.05)
        ax.set_ylim(min(all_wt) - wt_range*0.05, max(all_wt) + wt_range*0.05)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()

# Create output directory
output_dir = 'pareto_plots_perfect'
os.makedirs(output_dir, exist_ok=True)

print("\n" + "="*80)
print("GENERATING PARETO FRONT COMPARISON PLOTS")
print("  Source: result_final_perfect/ vs benchmark/")
print(f"  Output: {output_dir}/")
print("="*80)

# Get all benchmark files
benchmark_files = sorted([f for f in os.listdir('benchmark') if f.endswith('.txt')])

result_folder = 'result_final_perfect'
created_count = 0
skipped_count = 0

for bench_file in benchmark_files:
    instance_name = bench_file.replace('.txt', '')
    
    # Read benchmark solutions
    bench_path = f'benchmark/{bench_file}'
    benchmark_solutions = read_all_solutions_from_file(bench_path)
    
    # Read result solutions
    result_path = f'{result_folder}/{bench_file}'
    if not os.path.exists(result_path):
        print(f"⚠️  Missing: {bench_file}")
        skipped_count += 1
        continue
    
    result_solutions = read_all_solutions_from_file(result_path)
    
    if not result_solutions:
        print(f"⚠️  No solutions: {bench_file}")
        skipped_count += 1
        continue
    
    # Create plot
    output_path = f'{output_dir}/{instance_name}.png'
    create_pareto_plot(instance_name, result_solutions, benchmark_solutions, output_path)
    
    created_count += 1
    
    # Progress indicator
    if created_count % 10 == 0:
        print(f"  Created {created_count} plots...")

print("\n" + "="*80)
print("SUMMARY")
print("="*80)
print(f"✅ Created: {created_count} plots")
if skipped_count > 0:
    print(f"⚠️  Skipped: {skipped_count} instances")
print(f"\n📁 Output folder: {output_dir}/")
print("="*80)

# Show sample of created files
print("\nSample of created files:")
sample_files = sorted(os.listdir(output_dir))[:5]
for f in sample_files:
    print(f"  - {f}")
if len(os.listdir(output_dir)) > 5:
    print(f"  ... and {len(os.listdir(output_dir)) - 5} more files")

print("\n✅ All Pareto plots generated successfully!")
