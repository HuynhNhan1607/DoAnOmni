import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Đường dẫn đến file log
file_path = r"D:\Project\DoAn\Omni_Server\logs\bno055_log_20250325_174625.txt"

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

# Hàm chuyển đổi từ quaternion sang Euler angles (degrees)
def quaternion_to_euler(w, x, y, z):
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    # Convert to degrees
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

# Tính góc Euler từ quaternion nếu có dữ liệu quaternion
has_quat_data = all(col in valid_columns for col in ['QuatW', 'QuatX', 'QuatY', 'QuatZ'])
if has_quat_data:
    df['Roll_from_Quat'], df['Pitch_from_Quat'], df['Yaw_from_Quat'] = quaternion_to_euler(
        df['QuatW'], df['QuatX'], df['QuatY'], df['QuatZ'])

# Tạo figure với kích thước lớn
plt.figure(figsize=(14, 15))  # Tăng chiều cao để chứa 3 plots


# Tạo subplots cho các loại dữ liệu khác nhau
num_plots = 3  # Euler, Quaternion, và Euler từ Quaternion
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

# Vẽ đồ thị cho Euler angles từ quaternion
if has_quat_data:
    plt.subplot(num_plots, 1, 3)
    plt.plot(df['Time'], df['Roll_from_Quat'], label='Roll (từ Quat)')
    plt.plot(df['Time'], df['Pitch_from_Quat'], label='Pitch (từ Quat)')
    plt.plot(df['Time'], df['Yaw_from_Quat'], label='Yaw (từ Quat)')
    plt.title('Góc Euler được tính từ Quaternion')
    plt.xlabel('Thời gian (giây)')
    plt.ylabel('Độ (°)')
    plt.grid(True)
    plt.legend()

# Điều chỉnh layout
plt.tight_layout()

# Hiển thị đồ thị
plt.show()