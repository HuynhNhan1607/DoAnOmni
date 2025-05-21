import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

# Đọc dữ liệu từ file CSV
df = pd.read_csv('logs/bno055_log_20250519_163854.csv')

# Kiểm tra xem các cột gia tốc có tồn tại không
print("Các cột có trong dữ liệu:", df.columns.tolist())

# Trích xuất dữ liệu gia tốc 
accel_x = df['AccelX'].values
accel_y = df['AccelY'].values
accel_z = df['AccelZ'].values

# Thời gian lấy mẫu - giả định rằng tần số lấy mẫu là 100Hz
sampling_rate = 25  # Hz
N = len(accel_x)
T = 1.0 / sampling_rate

# Tính FFT cho các trục
fft_x = fft(accel_x)
fft_y = fft(accel_y)
fft_z = fft(accel_z)

# Tạo trục tần số
xf = fftfreq(N, T)[:N//2]

# Tính biên độ FFT (chỉ lấy nửa đầu của kết quả FFT vì dữ liệu thực)
magnitude_x = 2.0/N * np.abs(fft_x[:N//2])
magnitude_y = 2.0/N * np.abs(fft_y[:N//2])
magnitude_z = 2.0/N * np.abs(fft_z[:N//2])

# Vẽ tất cả trục trong cùng một biểu đồ
plt.figure(figsize=(12, 8))

# Giới hạn tần số hiển thị (0-25Hz)
freq_limit = 25  # Hz
indices = np.where(xf <= freq_limit)[0]

# Vẽ FFT cho cả 3 trục trên cùng một biểu đồ
plt.plot(xf[indices], magnitude_x[indices], 'r-', label='AccelX')
plt.plot(xf[indices], magnitude_y[indices], 'g-', label='AccelY')
plt.plot(xf[indices], magnitude_z[indices], 'b-', label='AccelZ')

plt.title('FFT của dữ liệu gia tốc (0-25Hz)')
plt.xlabel('Tần số (Hz)')
plt.ylabel('Biên độ')
plt.grid(True)
plt.legend()
plt.tight_layout()

# Thêm chú thích để hiển thị rõ giá trị tại các đỉnh
plt.show()