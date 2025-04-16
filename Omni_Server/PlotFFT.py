import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import fft, fftfreq
import pandas as pd

# BƯỚC 1: Đọc dữ liệu từ file .csv
file_path = "d:/Project/DoAn/Omni_Server/logs/encoder_log_20250406_183151.csv"
# Đọc dữ liệu sử dụng pandas
df = pd.read_csv(file_path, comment='/')  # Bỏ qua dòng comment bắt đầu bằng //

# BƯỚC 2: Tách dữ liệu động cơ
rpm_1 = df['RPM1'].values  # Động cơ 1
rpm_2 = df['RPM2'].values  # Động cơ 2
rpm_3 = df['RPM3'].values  # Động cơ 3

# Bỏ qua các giá trị 0 ban đầu (chỉ lấy dữ liệu từ khi có chuyển động)
non_zero_idx = np.where((rpm_1 != 0) | (rpm_2 != 0) | (rpm_3 != 0))[0]
if len(non_zero_idx) > 0:
    start_idx = np.min(non_zero_idx)
    rpm_1 = rpm_1[start_idx:]
    rpm_2 = rpm_2[start_idx:]
    rpm_3 = rpm_3[start_idx:]

# BƯỚC 3: Cài đặt thông số FFT
fs = 50  # Tần số lấy mẫu cố định (Hz) - tương ứng 20ms
N = len(rpm_1)  # Số mẫu dữ liệu
T = 1 / fs  # Khoảng thời gian giữa 2 mẫu (20ms)

# Tạo mảng thời gian mới
time = np.arange(0, N * T, T)

# Hàm tính FFT
def compute_fft(signal, N, fs):
    fft_values = fft(signal)  # Thực hiện FFT
    fft_magnitudes = np.abs(fft_values)[:N//2]  # Lấy biên độ nửa đầu phổ tần số
    frequencies = fftfreq(N, d=T)[:N//2]  # Lấy các tần số tương ứng
    return frequencies, fft_magnitudes

# Tính FFT cho từng động cơ
freqs_1, fft_rpm_1 = compute_fft(rpm_1, N, fs)
freqs_2, fft_rpm_2 = compute_fft(rpm_2, N, fs)
freqs_3, fft_rpm_3 = compute_fft(rpm_3, N, fs)

# BƯỚC 4: Vẽ biểu đồ dạng cột
plt.figure(figsize=(12, 5))
plt.bar(freqs_1, fft_rpm_1, width=0.05, alpha=0.6, label="FFT Động cơ 1", color='blue')
plt.bar(freqs_2, fft_rpm_2, width=0.05, alpha=0.6, label="FFT Động cơ 2", color='orange')
plt.bar(freqs_3, fft_rpm_3, width=0.05, alpha=0.6, label="FFT Động cơ 3", color='green')

plt.xlabel("Tần số (Hz)")
plt.ylabel("Biên độ")
plt.title("Phân tích phổ tần số của RPM động cơ (Dạng cột)")
plt.legend()
plt.grid()

# BƯỚC 5: Vẽ thêm biểu đồ giá trị RPM theo thời gian
plt.figure(figsize=(12, 5))
plt.plot(time, rpm_1, label="RPM Động cơ 1", color='blue')
plt.plot(time, rpm_2, label="RPM Động cơ 2", color='orange')
plt.plot(time, rpm_3, label="RPM Động cơ 3", color='green')
plt.xlabel("Thời gian (s)")
plt.ylabel("RPM")
plt.title("Vận tốc động cơ theo thời gian")
plt.legend()
plt.grid()

plt.show()