import os
import matplotlib.pyplot as plt
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

# Create output folder
output_folder = 'pareto_plots'
if not os.path.exists(output_folder):
    os.makedirs(output_folder)
    print(f"Created folder: {output_folder}/")

# Get all result files
result_files = sorted([f for f in os.listdir('result') if f.endswith('.txt')])

print(f"\nGenerating Pareto front scatter plots for {len(result_files)} instances...")
print("=" * 80)

success_count = 0
for idx, fname in enumerate(result_files, 1):
    if os.path.exists(f'benchmark/{fname}'):
        # Read solutions
        ic_sols = read_solutions(f'result/{fname}')
        bm_sols = read_solutions(f'benchmark/{fname}')
        
        if ic_sols and bm_sols:
            # Extract data
            ic_ct = [s[0] for s in ic_sols]
            ic_wt = [s[1] for s in ic_sols]
            bm_ct = [s[0] for s in bm_sols]
            bm_wt = [s[1] for s in bm_sols]
            
            # Create figure
            plt.figure(figsize=(10, 8))
            
            # Plot Benchmark (orange squares)
            plt.scatter(bm_ct, bm_wt, color='orange', marker='s', s=100, 
                       alpha=0.7, edgecolors='black', linewidth=1, label='Benchmark')
            
            # Plot ICAHGS (blue circles)
            plt.scatter(ic_ct, ic_wt, color='blue', marker='o', s=120, 
                       alpha=0.8, edgecolors='black', linewidth=1.5, label='ICAHGS')
            
            # Labels and title
            instance_name = fname.replace('.txt', '')
            plt.xlabel('Completion Time (s)', fontsize=12, fontweight='bold')
            plt.ylabel('Total Waiting Time (s)', fontsize=12, fontweight='bold')
            plt.title(f'Instance {instance_name}', fontsize=14, fontweight='bold', pad=15)
            
            # Legend
            plt.legend(loc='best', fontsize=11, framealpha=0.9)
            
            # Grid
            plt.grid(True, alpha=0.3, linestyle='--')
            
            # Tight layout
            plt.tight_layout()
            
            # Save
            output_path = f'{output_folder}/{instance_name}.png'
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            plt.close()
            
            success_count += 1
            if idx % 10 == 0:
                print(f"  Progress: {idx}/{len(result_files)} plots generated...")

print("=" * 80)
print(f"✓ Successfully generated {success_count} plots")
print(f"✓ Saved to folder: {output_folder}/")
print(f"\nSample files:")
for fname in result_files[:5]:
    print(f"  - {output_folder}/{fname.replace('.txt', '.png')}")
print(f"  ... and {success_count - 5} more files")
