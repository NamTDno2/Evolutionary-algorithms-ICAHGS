import os
import numpy as np
import matplotlib.pyplot as plt

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

def analyze_distribution(solutions):
    """Analyze solution distribution patterns"""
    if len(solutions) < 2:
        return {}
    
    ct = np.array([s[0] for s in solutions])
    wt = np.array([s[1] for s in solutions])
    
    # Normalize to [0, 1] for analysis
    ct_norm = (ct - ct.min()) / (ct.max() - ct.min() + 1e-10)
    wt_norm = (wt - wt.min()) / (wt.max() - wt.min() + 1e-10)
    
    # Calculate metrics
    analysis = {
        'count': len(solutions),
        'ct_range': ct.max() - ct.min(),
        'wt_range': wt.max() - wt.min(),
        'ct_spread': np.std(ct_norm),
        'wt_spread': np.std(wt_norm),
        'correlation': np.corrcoef(ct_norm, wt_norm)[0, 1] if len(solutions) > 1 else 0,
    }
    
    # Calculate pairwise distances
    distances = []
    for i in range(len(solutions)):
        for j in range(i+1, len(solutions)):
            dist = np.sqrt((ct_norm[i] - ct_norm[j])**2 + (wt_norm[i] - wt_norm[j])**2)
            distances.append(dist)
    
    if distances:
        analysis['avg_distance'] = np.mean(distances)
        analysis['min_distance'] = np.min(distances)
        analysis['max_distance'] = np.max(distances)
    
    return analysis

# Analyze all instances
result_files = sorted([f for f in os.listdir('result') if f.endswith('.txt')])

print("=" * 100)
print("ANALYZING PARETO FRONT DISTRIBUTION PATTERNS")
print("=" * 100)

patterns = {
    'clustered': [],      # Nghiệm tập trung thành cụm
    'arc_shaped': [],     # Dải hình vòng cung (trade-off rõ ràng)
    'scattered': [],      # Phân tán rải rác
    'sparse': [],         # Ít nghiệm, xa nhau
    'dense': [],          # Nhiều nghiệm, gần nhau
    'outliers': []        # Có nghiệm lẻ
}

for fname in result_files:
    ic_sols = read_solutions(f'result/{fname}')
    bm_sols = read_solutions(f'benchmark/{fname}')
    
    if ic_sols and bm_sols:
        ic_analysis = analyze_distribution(ic_sols)
        bm_analysis = analyze_distribution(bm_sols)
        
        if not ic_analysis or not bm_analysis:
            continue
        
        instance = fname.replace('.txt', '')
        
        # Pattern classification
        data = {
            'instance': instance,
            'ic_count': ic_analysis.get('count', 0),
            'bm_count': bm_analysis.get('count', 0),
            'ic_corr': ic_analysis.get('correlation', 0),
            'bm_corr': bm_analysis.get('correlation', 0),
            'ic_avg_dist': ic_analysis.get('avg_distance', 0),
            'bm_avg_dist': bm_analysis.get('avg_distance', 0),
            'ic_min_dist': ic_analysis.get('min_distance', 0),
            'bm_min_dist': bm_analysis.get('min_distance', 0),
        }
        
        # Arc-shaped: strong negative correlation (trade-off curve)
        if ic_analysis.get('correlation', 0) < -0.7:
            patterns['arc_shaped'].append(data)
        
        # Clustered: small average distance
        elif ic_analysis.get('avg_distance', 1) < 0.2:
            patterns['clustered'].append(data)
        
        # Dense: many solutions close together
        elif ic_analysis['count'] > 8 and ic_analysis.get('min_distance', 1) < 0.05:
            patterns['dense'].append(data)
        
        # Sparse: few solutions far apart
        elif ic_analysis['count'] < 4 and ic_analysis.get('avg_distance', 0) > 0.4:
            patterns['sparse'].append(data)
        
        # Scattered: large spread
        elif ic_analysis['ct_spread'] > 0.3 and ic_analysis['wt_spread'] > 0.3:
            patterns['scattered'].append(data)

# Print representative examples
print("\n📊 PATTERN 1: ARC-SHAPED (Dải Vòng Cung - Trade-off Rõ Ràng)")
print("-" * 100)
print("Đặc điểm: Nghiệm phân bố thành đường cong, thể hiện trade-off rõ ràng giữa CT và WT")
print("Ý nghĩa: Khi giảm CT thì WT tăng và ngược lại - đây là Pareto front lý tưởng\n")
if patterns['arc_shaped']:
    for item in sorted(patterns['arc_shaped'], key=lambda x: abs(x['ic_corr']), reverse=True)[:5]:
        print(f"  {item['instance']}: {item['ic_count']} solutions, correlation={item['ic_corr']:.3f}")

print("\n📊 PATTERN 2: CLUSTERED (Chùm Tập Trung)")
print("-" * 100)
print("Đặc điểm: Nghiệm nằm gần nhau trong 1 vùng nhỏ")
print("Ý nghĩa: Algorithm hội tụ về 1 vùng cụ thể, có thể là local optimum hoặc vùng tốt nhất\n")
if patterns['clustered']:
    for item in sorted(patterns['clustered'], key=lambda x: x['ic_avg_dist'])[:5]:
        print(f"  {item['instance']}: {item['ic_count']} solutions, avg_dist={item['ic_avg_dist']:.3f}")

print("\n📊 PATTERN 3: DENSE (Đông Nghiệm Sát Nhau)")
print("-" * 100)
print("Đặc điểm: Nhiều nghiệm, khoảng cách giữa các nghiệm rất nhỏ")
print("Ý nghĩa: Diversity tốt nhưng có thể redundancy cao, nhiều nghiệm tương tự nhau\n")
if patterns['dense']:
    for item in sorted(patterns['dense'], key=lambda x: x['ic_min_dist'])[:5]:
        print(f"  {item['instance']}: {item['ic_count']} solutions, min_dist={item['ic_min_dist']:.4f}")

print("\n📊 PATTERN 4: SPARSE (Thưa Thớt - Nghiệm Lẻ)")
print("-" * 100)
print("Đặc điểm: Ít nghiệm, xa nhau")
print("Ý nghĩa: Coverage kém, algorithm chưa explore đủ hoặc bài toán khó\n")
if patterns['sparse']:
    for item in sorted(patterns['sparse'], key=lambda x: x['ic_count'])[:5]:
        print(f"  {item['instance']}: {item['ic_count']} solutions, avg_dist={item['ic_avg_dist']:.3f}")

print("\n📊 PATTERN 5: SCATTERED (Phân Tán Rải Rác)")
print("-" * 100)
print("Đặc điểm: Nghiệm rải đều trên không gian objective")
print("Ý nghĩa: Diversity tốt, explore nhiều vùng khác nhau\n")
if patterns['scattered']:
    for item in sorted(patterns['scattered'], key=lambda x: x['ic_avg_dist'], reverse=True)[:5]:
        print(f"  {item['instance']}: {item['ic_count']} solutions, avg_dist={item['ic_avg_dist']:.3f}")

# Find interesting cases for visualization
print("\n" + "=" * 100)
print("CASE STUDIES - TYPICAL EXAMPLES TO EXAMINE")
print("=" * 100)

interesting_cases = []

# Best arc-shaped
if patterns['arc_shaped']:
    best_arc = max(patterns['arc_shaped'], key=lambda x: abs(x['ic_corr']))
    interesting_cases.append(('Arc-Shaped (Perfect Trade-off)', best_arc['instance']))

# Most clustered
if patterns['clustered']:
    most_clustered = min(patterns['clustered'], key=lambda x: x['ic_avg_dist'])
    interesting_cases.append(('Clustered (Tập Trung)', most_clustered['instance']))

# Most sparse
if patterns['sparse']:
    most_sparse = min(patterns['sparse'], key=lambda x: x['ic_count'])
    interesting_cases.append(('Sparse (Thưa Nhất)', most_sparse['instance']))

# Most dense
if patterns['dense']:
    most_dense = max(patterns['dense'], key=lambda x: x['ic_count'])
    interesting_cases.append(('Dense (Đông Nhất)', most_dense['instance']))

print("\nRecommended instances to examine:")
for i, (pattern_type, instance) in enumerate(interesting_cases, 1):
    print(f"  {i}. {pattern_type}: pareto_plots/{instance}.png")

print("\n" + "=" * 100)
