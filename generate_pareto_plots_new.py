import os
import matplotlib.pyplot as plt
import numpy as np

def read_solutions_from_file(filepath):
    """Read solutions (CT, WT) from result file"""
    solutions = []
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            
        if len(lines) < 6:
            return solutions
            
        num_solutions = int(lines[5].strip())
        
        # Read solutions (each solution: route line + CT WT line)
        i = 6
        while i < len(lines) and len(solutions) < num_solutions:
            # Skip route line
            i += 1
            if i < len(lines):
                # Parse CT WT line
                parts = lines[i].strip().split()
                if len(parts) >= 2:
                    ct = float(parts[0])
                    wt = float(parts[1])
                    solutions.append([ct, wt])
            i += 1
            
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        
    return solutions

# Create output directory
os.makedirs('pareto_plots_new', exist_ok=True)

# Get all instances
result_files = sorted([f for f in os.listdir('result') if f.endswith('.txt')])
benchmark_files = sorted([f for f in os.listdir('benchmark') if f.endswith('.txt')])

print(f"Found {len(result_files)} result files")
print(f"Found {len(benchmark_files)} benchmark files")
print("\nGenerating Pareto front plots...")

success_count = 0
failed_count = 0

for filename in result_files:
    if filename in benchmark_files:
        # Read solutions
        result_sols = read_solutions_from_file(f'result/{filename}')
        benchmark_sols = read_solutions_from_file(f'benchmark/{filename}')
        
        if not result_sols or not benchmark_sols:
            print(f"  ❌ {filename}: No solutions found")
            failed_count += 1
            continue
        
        # Convert to numpy arrays
        result_sols = np.array(result_sols)
        benchmark_sols = np.array(benchmark_sols)
        
        # Create plot
        fig, ax = plt.subplots(figsize=(10, 7))
        
        # Plot solutions
        ax.scatter(result_sols[:, 0], result_sols[:, 1], 
                  c='#2ecc71', marker='o', s=80, alpha=0.7, 
                  edgecolors='black', linewidth=1.5, 
                  label=f'ICAHGS (n={len(result_sols)})', zorder=3)
        
        ax.scatter(benchmark_sols[:, 0], benchmark_sols[:, 1], 
                  c='#e74c3c', marker='s', s=80, alpha=0.7, 
                  edgecolors='black', linewidth=1.5, 
                  label=f'Benchmark (n={len(benchmark_sols)})', zorder=2)
        
        # Connect points to show Pareto front
        result_sorted = result_sols[result_sols[:, 0].argsort()]
        benchmark_sorted = benchmark_sols[benchmark_sols[:, 0].argsort()]
        
        ax.plot(result_sorted[:, 0], result_sorted[:, 1], 
               'g--', alpha=0.5, linewidth=1.5, zorder=1)
        ax.plot(benchmark_sorted[:, 0], benchmark_sorted[:, 1], 
               'r--', alpha=0.5, linewidth=1.5, zorder=1)
        
        # Labels and title
        instance_name = filename.replace('.txt', '')
        ax.set_xlabel('Completion Time (CT) [seconds]', fontsize=12, fontweight='bold')
        ax.set_ylabel('Waiting Time (WT) [seconds]', fontsize=12, fontweight='bold')
        ax.set_title(f'Pareto Front Comparison: {instance_name}\nICAHGS vs Benchmark', 
                    fontsize=14, fontweight='bold', pad=15)
        
        ax.legend(loc='upper right', fontsize=11, framealpha=0.9)
        ax.grid(True, alpha=0.3, linestyle='--')
        
        # Save plot
        plt.tight_layout()
        plt.savefig(f'pareto_plots_new/{instance_name}.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        success_count += 1
        if success_count % 10 == 0:
            print(f"  Progress: {success_count}/{len(result_files)}")
    else:
        print(f"  ⚠️ {filename}: No matching benchmark file")
        failed_count += 1

print("\n" + "="*60)
print(f"✅ Successfully generated: {success_count} plots")
if failed_count > 0:
    print(f"❌ Failed: {failed_count} plots")
print(f"📁 Output folder: pareto_plots_new/")
print("="*60)
