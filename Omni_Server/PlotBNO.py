import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Đường dẫn đến file log
file_path = r"D:\Project\DoAn\Omni_Server\logs\bno055_log_20250321_164237.txt"

# Đọc file log với các cột được phân tách bằng khoảng trắng
df = pd.read_csv(file_path, sep=" ", na_values="NA")

# Thay thế tên cột nếu cần
if 'W' in df.columns and 'X' in df.columns and 'Y' in df.columns and 'Z' in df.columns:
    df.rename(columns={'W': 'QuatW', 'X': 'QuatX', 'Y': 'QuatY', 'Z': 'QuatZ'}, inplace=True)

# Lọc các cột có giá trị (không phải tất cả các giá trị đều là NA)
valid_columns = [col for col in df.columns if not df[col].isna().all()]

if 'Time' in df.columns:
    time_diffs = df['Time'].diff().dropna()  # Tính khoảng thời gian giữa các gói tin
    avg_frequency = 1 / time_diffs.mean()  # Tần số trung bình (Hz)
    avg_frequency_text = f"Tần số gửi gói tin trung bình: {avg_frequency:.2f} Hz"
    print(avg_frequency_text)

# Tạo figure với kích thước lớn
plt.figure(figsize=(14, 10))

# Tạo subplots cho các loại dữ liệu khác nhau
num_plots = 2  # Euler và Quaternion
plt.subplot(num_plots, 1, 1)

# Vẽ đồ thị cho góc Euler
euler_cols = [col for col in valid_columns if col in ['Heading', 'Pitch', 'Roll']]
for col in euler_cols:
    plt.plot(df['Time'], df[col], label=col)
plt.title('Góc Euler theo thời gian')
plt.xlabel('Thời gian (giây)')
plt.ylabel('Độ (°)')
plt.grid(True)
plt.legend()

# Vẽ đồ thị cho quaternion
plt.subplot(num_plots, 1, 2)
quat_cols = [col for col in valid_columns if col in ['QuatW', 'QuatX', 'QuatY', 'QuatZ']]
for col in quat_cols:
    plt.plot(df['Time'], df[col], label=col)
plt.title('Quaternion theo thời gian')
plt.xlabel('Thời gian (giây)')
plt.ylabel('Giá trị quaternion')
plt.grid(True)
plt.legend()

# Điều chỉnh layout
plt.tight_layout()

# Hiển thị đồ thị
plt.show()