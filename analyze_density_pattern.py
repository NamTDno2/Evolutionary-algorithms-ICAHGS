import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read CSV
df = pd.read_csv('results_best_comparison.csv')

# Parse instance name to get density
df['Density'] = df['Instance'].apply(lambda x: int(x.split('.')[1]))

print("\n" + "="*80)
print("PHÂN TÍCH THEO DENSITY (Số nodes)")
print("="*80)
print("\nPattern: <Size>.<Density>.<Instance>")
print("Ví dụ: 50.20.1 = 50 customers, 20 nodes, instance 1")

# Group by Size and Density
grouped = df.groupby(['Size', 'Density'])

print("\n" + "-"*80)
print(f"{'Size-Density':<15} {'W/T/L':<10} {'WinRate':<10} {'CT Gap':<10} {'WT Gap':<10}")
print("-"*80)

for (size, density), group in sorted(grouped):
    wins = len(group[group['Status'] == 'win'])
    ties = len(group[group['Status'] == 'tie'])
    losses = len(group[group['Status'] == 'loss'])
    total = len(group)
    win_rate = (wins / total * 100) if total > 0 else 0
    
    ct_gap = group['CT_Gap_%'].mean()
    wt_gap = group['WT_Gap_%'].mean()
    
    status_str = f"{wins}/{ties}/{losses}"
    
    # Color coding
    if win_rate >= 50:
        color = '✅'
    elif win_rate >= 25:
        color = '⚠️'
    else:
        color = '❌'
    
    print(f"{color} {size:3d}C-{density:2d}nodes   {status_str:<10} {win_rate:6.1f}%   {ct_gap:+7.1f}%   {wt_gap:+7.1f}%")

# Analyze by density across all sizes
print("\n" + "="*80)
print("PATTERN THEO DENSITY")
print("="*80)

density_analysis = df.groupby('Density').agg({
    'Status': lambda x: (
        len(x[x == 'win']),
        len(x[x == 'tie']),
        len(x[x == 'loss'])
    ),
    'CT_Gap_%': 'mean',
    'WT_Gap_%': 'mean'
}).reset_index()

print(f"\n{'Density':<10} {'Instances':<12} {'W/T/L':<15} {'WinRate':<12} {'CT Gap':<12} {'WT Gap':<12}")
print("-"*80)

for _, row in density_analysis.iterrows():
    density = int(row['Density'])
    wins, ties, losses = row['Status']
    total = wins + ties + losses
    win_rate = (wins / total * 100) if total > 0 else 0
    ct_gap = row['CT_Gap_%']
    wt_gap = row['WT_Gap_%']
    
    # Count instances
    instances = len(df[df['Density'] == density])
    
    if win_rate >= 50:
        color = '✅'
    elif win_rate >= 25:
        color = '⚠️'
    else:
        color = '❌'
    
    print(f"{color} {density:2d} nodes   {instances:2d} instances  {wins:2d}/{ties:2d}/{losses:2d}        "
          f"{win_rate:6.1f}%      {ct_gap:+8.1f}%     {wt_gap:+8.1f}%")

# Key findings
print("\n" + "="*80)
print("KEY FINDINGS")
print("="*80)

print("\n1. PATTERN THEO DENSITY:")
for density in sorted(df['Density'].unique()):
    data = df[df['Density'] == density]
    wins = len(data[data['Status'] == 'win'])
    total = len(data)
    win_rate = (wins / total * 100) if total > 0 else 0
    wt_gap = data['WT_Gap_%'].mean()
    
    if density <= 10:
        print(f"   {density:2d} nodes: {win_rate:5.1f}% win rate, WT {wt_gap:+6.1f}% - ✅ TỐT (Low density)")
    elif density <= 20:
        print(f"   {density:2d} nodes: {win_rate:5.1f}% win rate, WT {wt_gap:+6.1f}% - ⚠️ TRUNG BÌNH")
    elif density <= 30:
        print(f"   {density:2d} nodes: {win_rate:5.1f}% win rate, WT {wt_gap:+6.1f}% - ❌ YẾU")
    else:
        print(f"   {density:2d} nodes: {win_rate:5.1f}% win rate, WT {wt_gap:+6.1f}% - ❌ RẤT YẾU (High density)")

print("\n2. TẠI SAO 200C TỐT HƠN 50C/100C?")
print(f"   200C: Có {len(df[df['Size']==200])} instances, phân bổ:")
for density in sorted(df[df['Size']==200]['Density'].unique()):
    data = df[(df['Size']==200) & (df['Density']==density)]
    wins = len(data[data['Status'] == 'win'])
    print(f"      {density:2d} nodes: {wins}/{len(data)} wins")

print(f"\n   → 200C có NHIỀU instances ở density THẤP (10-30 nodes)")
print(f"   → 200.10/20/30: WIN")
print(f"   → 200.40: LOSS (cùng pattern với 50C/100C)")

print(f"\n   50C/100C:")
for size in [50, 100]:
    print(f"      {size}C: {len(df[df['Size']==size])} instances")
    for density in sorted(df[df['Size']==size]['Density'].unique()):
        data = df[(df['Size']==size) & (df['Density']==density)]
        wins = len(data[data['Status'] == 'win'])
        print(f"         {density:2d} nodes: {wins}/{len(data)} wins")

print(f"\n   → 50C/100C có NHIỀU instances ở density CAO (20-40 nodes)")
print(f"   → Đặc biệt 30-40 nodes: 0-1 wins")

print("\n3. KẾT LUẬN:")
print("   ❌ KHÔNG PHẢI thuật toán tốt với bài toán lớn")
print("   ✅ Thuật toán TỐT với DENSITY THẤP (≤10-20 nodes)")
print("   ❌ Thuật toán YẾU với DENSITY CAO (≥30 nodes)")
print("   → Vấn đề: Không scale với SỐ NODES, không phải SIZE!")

print("\n4. TẠI SAO DENSITY CAO = KẾT QUẢ TỆ?")
print("   - Nhiều nodes → Search space phức tạp")
print("   - Truck phải dừng nhiều nơi → Drone routes phức tạp")
print("   - Synchronization constraints tăng")
print("   - Evaluation budget không đủ cho density cao")
print("="*80)
