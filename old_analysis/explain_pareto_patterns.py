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
            i += 1
            if i < len(lines):
                parts = lines[i].strip().split()
                if len(parts) >= 2:
                    solutions.append([float(parts[0]), float(parts[1])])
                    count += 1
            i += 1
    except:
        pass
    return solutions

# Select representative instances
examples = {
    '20.10.1': 'Arc-Shaped: Trade-off rõ ràng (2 nghiệm tạo đường cong)',
    '50.20.1': 'Dense với Benchmark: Benchmark có nhiều nghiệm gần nhau',
    '100.10.1': 'Mixed: ICAHGS ít nghiệm, Benchmark nhiều nghiệm',
    '200.10.1': 'Separated: 2 clusters ở 2 vùng khác nhau',
    '100.30.2': 'Completely Separated: Không overlap (C=0)',
}

print("=" * 100)
print("PHÂN TÍCH CHI TIẾT CÁC PATTERN PHÂN BỐ NGHIỆM PARETO FRONT")
print("=" * 100)

for idx, (instance, description) in enumerate(examples.items(), 1):
    fname = f'{instance}.txt'
    if not os.path.exists(f'result/{fname}'):
        continue
    
    ic_sols = read_solutions(f'result/{fname}')
    bm_sols = read_solutions(f'benchmark/{fname}')
    
    if not ic_sols or not bm_sols:
        continue
    
    print(f"\n{'='*100}")
    print(f"CASE {idx}: {instance} - {description}")
    print(f"{'='*100}")
    print(f"📁 File: pareto_plots/{instance}.png")
    
    # ICAHGS analysis
    ic_ct = np.array([s[0] for s in ic_sols])
    ic_wt = np.array([s[1] for s in ic_sols])
    
    print(f"\n🔵 ICAHGS: {len(ic_sols)} solutions")
    print(f"   CT range: [{ic_ct.min():.2f}, {ic_ct.max():.2f}] → spread = {ic_ct.max()-ic_ct.min():.2f}s")
    print(f"   WT range: [{ic_wt.min():.2f}, {ic_wt.max():.2f}] → spread = {ic_wt.max()-ic_wt.min():.2f}s")
    
    if len(ic_sols) > 1:
        ic_corr = np.corrcoef(ic_ct, ic_wt)[0, 1]
        print(f"   Correlation: {ic_corr:.3f}", end="")
        if ic_corr < -0.7:
            print(" → Strong trade-off (arc-shaped)")
        elif ic_corr > 0.7:
            print(" → Positive correlation (unusual)")
        else:
            print(" → Weak/no clear pattern")
    
    # Benchmark analysis
    bm_ct = np.array([s[0] for s in bm_sols])
    bm_wt = np.array([s[1] for s in bm_sols])
    
    print(f"\n🟠 BENCHMARK: {len(bm_sols)} solutions")
    print(f"   CT range: [{bm_ct.min():.2f}, {bm_ct.max():.2f}] → spread = {bm_ct.max()-bm_ct.min():.2f}s")
    print(f"   WT range: [{bm_wt.min():.2f}, {bm_wt.max():.2f}] → spread = {bm_wt.max()-bm_wt.min():.2f}s")
    
    if len(bm_sols) > 1:
        bm_corr = np.corrcoef(bm_ct, bm_wt)[0, 1]
        print(f"   Correlation: {bm_corr:.3f}", end="")
        if bm_corr < -0.7:
            print(" → Strong trade-off (arc-shaped)")
        elif bm_corr > 0.7:
            print(" → Positive correlation (unusual)")
        else:
            print(" → Weak/no clear pattern")
    
    # Overlap analysis
    print(f"\n📊 SPATIAL RELATIONSHIP:")
    
    # CT overlap
    ic_ct_range = (ic_ct.min(), ic_ct.max())
    bm_ct_range = (bm_ct.min(), bm_ct.max())
    
    if ic_ct_range[1] < bm_ct_range[0]:
        print(f"   ✓ ICAHGS CT < Benchmark CT (ICAHGS left side)")
        print(f"     → ICAHGS ưu tiên minimize CT")
    elif bm_ct_range[1] < ic_ct_range[0]:
        print(f"   ✓ Benchmark CT < ICAHGS CT (Benchmark left side)")
        print(f"     → Benchmark ưu tiên minimize CT")
    else:
        overlap_ct = min(ic_ct_range[1], bm_ct_range[1]) - max(ic_ct_range[0], bm_ct_range[0])
        total_ct = max(ic_ct_range[1], bm_ct_range[1]) - min(ic_ct_range[0], bm_ct_range[0])
        overlap_pct = (overlap_ct / total_ct * 100) if total_ct > 0 else 0
        print(f"   ≈ CT ranges overlap {overlap_pct:.1f}%")
        print(f"     → Cạnh tranh trong cùng vùng CT")
    
    # WT overlap
    ic_wt_range = (ic_wt.min(), ic_wt.max())
    bm_wt_range = (bm_wt.min(), bm_wt.max())
    
    if ic_wt_range[1] < bm_wt_range[0]:
        print(f"   ✓ ICAHGS WT < Benchmark WT (ICAHGS bottom)")
        print(f"     → ICAHGS ưu tiên minimize WT")
    elif bm_wt_range[1] < ic_wt_range[0]:
        print(f"   ✓ Benchmark WT < ICAHGS WT (Benchmark bottom)")
        print(f"     → Benchmark ưu tiên minimize WT")
    else:
        overlap_wt = min(ic_wt_range[1], bm_wt_range[1]) - max(ic_wt_range[0], bm_wt_range[0])
        total_wt = max(ic_wt_range[1], bm_wt_range[1]) - min(ic_wt_range[0], bm_wt_range[0])
        overlap_pct = (overlap_wt / total_wt * 100) if total_wt > 0 else 0
        print(f"   ≈ WT ranges overlap {overlap_pct:.1f}%")
        print(f"     → Cạnh tranh trong cùng vùng WT")
    
    # Density analysis for Benchmark
    if len(bm_sols) > 2:
        # Normalize
        bm_ct_norm = (bm_ct - bm_ct.min()) / (bm_ct.max() - bm_ct.min() + 1e-10)
        bm_wt_norm = (bm_wt - bm_wt.min()) / (bm_wt.max() - bm_wt.min() + 1e-10)
        
        # Calculate pairwise distances
        distances = []
        for i in range(len(bm_sols)):
            for j in range(i+1, len(bm_sols)):
                dist = np.sqrt((bm_ct_norm[i] - bm_ct_norm[j])**2 + (bm_wt_norm[i] - bm_wt_norm[j])**2)
                distances.append(dist)
        
        if distances:
            avg_dist = np.mean(distances)
            min_dist = np.min(distances)
            
            print(f"\n💡 DENSITY INSIGHT:")
            if avg_dist < 0.2:
                print(f"   → Benchmark nghiệm GẦN NHAU (avg_dist={avg_dist:.3f})")
                print(f"   → Có thể nhiều nghiệm tương tự (redundancy)")
            elif avg_dist > 0.5:
                print(f"   → Benchmark nghiệm XA NHAU (avg_dist={avg_dist:.3f})")
                print(f"   → Coverage tốt, explore nhiều vùng")
            
            if min_dist < 0.05:
                print(f"   → Có nghiệm RẤT SÁT NHAU (min_dist={min_dist:.4f})")
                print(f"   → Có thể là duplicate hoặc nearly-duplicate solutions")

print("\n" + "=" * 100)
print("SUMMARY - CÁC PATTERN ĐIỂN HÌNH")
print("=" * 100)

print("""
📌 PATTERN 1: ARC-SHAPED (Vòng Cung)
   - Ví dụ: 20.10.1, 20.5.1
   - Đặc điểm: Correlation âm mạnh (-0.9 đến -1.0)
   - Nghiệm tạo đường cong từ góc trên-trái sang góc dưới-phải
   - Ý nghĩa: Trade-off rõ ràng - giảm CT thì tăng WT và ngược lại
   - Đánh giá: ⭐⭐⭐⭐⭐ Pareto front lý tưởng!

📌 PATTERN 2: DENSE CLUSTER (Chùm Đông)
   - Ví dụ: 50.20.1, 100.20.1
   - Đặc điểm: Nhiều nghiệm (>10), khoảng cách trung bình nhỏ (<0.3)
   - Nghiệm tập trung trong 1 vùng hẹp
   - Ý nghĩa: Algorithm hội tụ tốt nhưng có thể redundancy
   - Đánh giá: ⭐⭐⭐ Tốt cho exploitation, kém cho exploration

📌 PATTERN 3: SEPARATED REGIONS (Tách Biệt)
   - Ví dụ: 100.30.2, 200.10.1
   - Đặc điểm: ICAHGS và Benchmark ở 2 vùng hoàn toàn khác nhau
   - Không có nghiệm nào dominate nhau (C=0)
   - Ý nghĩa: 2 thuật toán tối ưu các preference khác nhau
   - Đánh giá: ⭐⭐⭐ Không có winner rõ ràng, phụ thuộc mục tiêu

📌 PATTERN 4: SPARSE (Thưa Thớt)
   - Ví dụ: 50.10.2
   - Đặc điểm: Ít nghiệm (2-3), xa nhau
   - Coverage kém, nhiều vùng Pareto front chưa được explore
   - Ý nghĩa: Algorithm chưa converge đủ hoặc bài toán khó
   - Đánh giá: ⭐⭐ Cần tăng iterations hoặc population size

📌 PATTERN 5: OUTLIER (Nghiệm Lẻ)
   - Đặc điểm: Có 1 nghiệm cách xa các nghiệm khác
   - Thường là nghiệm extreme (rất tốt về 1 objective, kém về objective kia)
   - Ý nghĩa: Đại diện cho điểm đầu/cuối của Pareto front
   - Đánh giá: ⭐⭐⭐⭐ Tốt nếu cố ý, xấu nếu ngẫu nhiên
""")

print("=" * 100)
print("💡 RECOMMENDATION: Mở các file trong pareto_plots/ để quan sát trực quan!")
print("=" * 100)
