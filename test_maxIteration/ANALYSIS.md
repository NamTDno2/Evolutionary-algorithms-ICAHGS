# Kết quả test maxIteration với 5 bộ 50C

## Thông tin từ terminal output:

| Instance | Iterations | Total Evaluations | Time (s) | numTrucks |
|----------|-----------|------------------|----------|-----------|
| 50.10.1  | 15        | 24,887           | 1.5      | 3 (10T)   |
| 50.20.1  | 15        | 22,253           | 1.3      | 3 (20T)   |
| 50.30.1  | 12        | 13,157           | 1.2      | 3 (30T)   |
| 50.40.1  | 12        | 15,066           | 1.1      | 3 (40T)   |
| 50.10.2  | 12        | 33,643           | 1.4      | 3 (10T)   |

## Phân tích:

### Số lượng evaluations theo cấu hình:

**Với 15 iterations (30-40T):**
- 50.10.1: 24,887 evals
- 50.20.1: 22,253 evals
- **Trung bình**: ~23,570 evaluations

**Với 12 iterations (10-20T):**
- 50.30.1: 13,157 evals
- 50.40.1: 15,066 evals
- 50.10.2: 33,643 evals
- **Trung bình**: ~20,622 evaluations

**Trung bình chung 5 bộ**: ~21,801 evaluations

### So sánh với cấu hình MỚI:

**Cấu hình CŨ (maxIteration):**
- 50C: 12-15 iterations
- Trung bình: ~21,801 evaluations
- Thời gian: ~1.3 giây/instance
- **Kết quả: 43.8% win rate** ✅

**Cấu hình MỚI (maxEvaluation):**
- 50C: 1,095,000 evaluations
- **Gấp 50x cấu hình cũ!**
- Thời gian: ~1.5 phút/instance (chậm hơn 69x)
- **Kết quả: 25% win rate** ❌

## Kết luận:

1. **12-15 iterations chỉ dùng ~22,000 evaluations**
2. **Cấu hình mới dùng 1,095,000 evals** (gấp 50x) nhưng kết quả **XẤU HƠN**
3. **Thời gian chạy**: 1.3s vs 90s (chậm hơn 69x)
4. **Win rate**: 43.8% vs 25% (giảm 18.8%)

### Nguyên nhân có thể:

- **Overfitting**: Quá nhiều evaluations khiến algorithm hội tụ vào local optima
- **Diversity loss**: Chạy quá lâu làm mất đa dạng quần thể
- **Early stopping tốt hơn**: Dừng sớm (12-15 iter) cho kết quả tốt hơn
- **Cân bằng exploration-exploitation**: Ít iterations giữ được exploration tốt hơn

## Khuyến nghị:

✅ **NÊN REVERT về maxIteration** với cấu hình cũ:
- 20C: 10 iterations
- 50C (30-40T): 15 iterations  
- 50C (10-20T): 12 iterations
- 100C: 20-25 iterations
- 200C: 18 iterations

Lý do: Nhanh hơn 69x, kết quả tốt hơn 18.8%!
