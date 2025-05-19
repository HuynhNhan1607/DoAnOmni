import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Đọc dữ liệu từ CSV
# df = pd.read_csv('logs/encoder_log_20250512_211356.csv') // FuzzyLog
df = pd.read_csv('logs/encoder_log_20250518_162937.csv')
# Lấy các cột RPM
rpm1 = df['RPM1'].values
rpm2 = df['RPM2'].values
rpm3 = df['RPM3'].values


# encoder_log_20250518_161650

# Tạo trục thời gian mới, mỗi dòng cách nhau 50ms
time_values = np.arange(len(rpm1)) * 0.05  # 50ms = 0.05s

# Vẽ biểu đồ
plt.figure(figsize=(12, 6))
plt.plot(time_values, rpm1, label='RPM1', linewidth=1.5)
plt.plot(time_values, rpm2, label='RPM2', linewidth=1.5)
plt.plot(time_values, rpm3, label='RPM3', linewidth=1.5)

plt.xlabel('Thời gian (s)')
plt.ylabel('RPM')
plt.title('Biểu đồ tốc độ động cơ (RPM) theo thời gian')
plt.grid(True, alpha=0.3)
plt.legend()

# Chỉ hiển thị phần có dữ liệu RPM khác 0 để thấy rõ hơn
non_zero_indices = np.where(np.abs(rpm1) + np.abs(rpm2) + np.abs(rpm3) > 0)[0]
if len(non_zero_indices) > 0:
    start_idx = max(0, non_zero_indices[0] - 10)
    # Đảm bảo end_idx không vượt quá giới hạn của mảng
    end_idx = min(len(time_values) - 1, non_zero_indices[-1] + 10)
    plt.xlim(time_values[start_idx], time_values[end_idx])

plt.tight_layout()
plt.show()