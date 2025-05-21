import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Đọc file CSV
df = pd.read_csv('logs/bno055_log_20250519_171518.csv')

# Tạo trục thời gian với mỗi điểm cách nhau 40ms
time_sec = np.arange(len(df)) * 0.04  # Thời gian tính bằng giây

# Vẽ đồ thị 3 trục gia tốc
plt.figure(figsize=(10, 6))
plt.plot(time_sec, df['AccelX'], label='AccelX', linewidth=1)
plt.plot(time_sec, df['AccelY'], label='AccelY', linewidth=1)
plt.plot(time_sec, df['AccelZ'], label='AccelZ', linewidth=1)

# Thêm nhãn và chú thích
plt.xlabel('Thời gian (s)')
plt.ylabel('Gia tốc (m/s²)')
plt.title('Biểu đồ 3 trục gia tốc theo thời gian')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()