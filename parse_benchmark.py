# Python script to parse benchmark files and create results_benchmark.csv
# Format: Dataset,SolutionID,CompletionTime,TotalWaitingTime,ExecutionTime,ParetoSize,UniqueSolutions

import os
import re
from pathlib import Path

def parse_benchmark_file(filepath):
    """Parse a single benchmark file and extract all solutions."""
    dataset_name = Path(filepath).stem  # e.g., "20.5.1"
    solutions = []
    
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = [line.strip() for line in f.readlines()]
    
    i = 0
    run_count = 0
    
    while i < len(lines):
        # Look for "Time:" marker (start of a run)
        if lines[i].startswith("Time:"):
            run_count += 1
            i += 1
            
            # Skip metadata lines
            while i < len(lines) and not lines[i].isdigit():
                i += 1
            
            if i >= len(lines):
                break
            
            # Number of solutions in this run
            num_solutions = int(lines[i])
            i += 1
            
            # Parse each solution (2 lines: route + objectives)
            for sol_idx in range(num_solutions):
                if i >= len(lines):
                    break
                
                # Skip route line
                i += 1
                
                if i >= len(lines):
                    break
                
                # Parse objectives line
                obj_line = lines[i].strip()
                parts = obj_line.split()
                
                if len(parts) >= 2:
                    try:
                        completion_time = float(parts[0])
                        waiting_time = float(parts[1])
                        
                        solutions.append({
                            'completion_time': completion_time,
                            'waiting_time': waiting_time
                        })
                    except ValueError:
                        pass  # Skip invalid lines
                
                i += 1
        else:
            i += 1
    
    return dataset_name, solutions

def main():
    benchmark_dir = Path("benchmark")
    output_file = "results_benchmark.csv"
    
    if not benchmark_dir.exists():
        print(f"Error: {benchmark_dir} directory not found!")
        return
    
    # Get all .txt files
    benchmark_files = sorted(benchmark_dir.glob("*.txt"))
    
    if not benchmark_files:
        print(f"No benchmark files found in {benchmark_dir}")
        return
    
    print(f"Found {len(benchmark_files)} benchmark files")
    
    # Open output CSV
    with open(output_file, 'w', encoding='utf-8') as out:
        # Write header
        out.write("Dataset,SolutionID,CompletionTime,TotalWaitingTime,ExecutionTime,ParetoSize,UniqueSolutions\n")
        
        total_solutions = 0
        
        for filepath in benchmark_files:
            dataset_name, solutions = parse_benchmark_file(filepath)
            
            if not solutions:
                print(f"  {dataset_name}: No solutions found")
                continue
            
            # Remove duplicates based on objectives
            unique_solutions = []
            seen = set()
            
            for sol in solutions:
                key = (round(sol['completion_time'], 2), round(sol['waiting_time'], 2))
                if key not in seen:
                    seen.add(key)
                    unique_solutions.append(sol)
            
            pareto_size = len(solutions)
            unique_count = len(unique_solutions)
            
            # Write each unique solution
            for sol_id, sol in enumerate(unique_solutions):
                out.write(f"{dataset_name},{sol_id},{sol['completion_time']},{sol['waiting_time']},-1,{pareto_size},{unique_count}\n")
            
            total_solutions += unique_count
            print(f"  {dataset_name}: {unique_count} unique solutions (from {pareto_size} total)")
        
        print(f"\nTotal unique solutions across all datasets: {total_solutions}")
        print(f"Results written to: {output_file}")

if __name__ == "__main__":
    main()
