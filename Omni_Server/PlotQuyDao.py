import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import interp1d
import math

# Đọc dữ liệu từ file encoder
def read_encoder_data(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('Time') or line.startswith('//'):
                continue
            values = line.strip().split()
            if len(values) >= 4 and values[0] != "Time":
                try:
                    time = float(values[0])
                    rpm1 = float(values[1])
                    rpm2 = float(values[2])
                    rpm3 = float(values[3])
                    data.append([time, rpm1, rpm2, rpm3])
                except ValueError:
                    continue
    return np.array(data)

# Đọc dữ liệu từ file BNO055
def read_bno055_data(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('Time') or line.startswith('//'):
                continue
            values = line.strip().split()
            if len(values) >= 2 and values[0] != "Time":
                try:
                    time = float(values[0])
                    heading = float(values[1])
                    data.append([time, heading])
                except ValueError:
                    continue
    return np.array(data)

# Thông số robot
r = 0.03  # Bán kính bánh xe (m)
L = 0.153  # Khoảng cách từ tâm robot đến bánh xe (m)

# Đọc dữ liệu
encoder_data = read_encoder_data('logs/encoder_log_20250328_100755.txt')
bno055_data = read_bno055_data('logs/bno055_log_20250328_100755.txt')

# Lọc bỏ dữ liệu encoder không chuyển động (RPM = 0)
moving_indices = np.where(np.abs(encoder_data[:, 1]) > 1e-6)[0]
if len(moving_indices) > 0:
    start_idx = moving_indices[0]
    encoder_data = encoder_data[start_idx:]
    
    # Điều chỉnh thời gian về 0
    start_time = encoder_data[0, 0]
    encoder_data[:, 0] -= start_time
    
    # Điều chỉnh thời gian BNO055 cũng về 0
    bno055_data[:, 0] -= start_time

# Hàm chuyển đổi RPM sang vận tốc góc (rad/s)
def rpm_to_omega(rpm):
    return rpm * 2 * np.pi / 60

# Tìm gần nhất
def find_nearest_heading(time, bno055_data):
    nearest_index = -1  # Lưu chỉ số của giá trị phù hợp nhất
    for i in range(len(bno055_data)):
        if bno055_data[i, 0] <= time:
            nearest_index = i  
        else:
            break  # Khi timestamp lớn hơn `time`, dừng lại
    
    if nearest_index != -1:
        return np.radians(bno055_data[nearest_index, 1])  # Trả về giá trị tìm được
    return np.radians(bno055_data[0, 1]) if len(bno055_data) > 0 else 0  # Nếu không có giá trị phù hợp, trả về giá trị đầu tiên hoặc 0

# Tính quỹ đạo dựa vào tốc độ bánh xe và heading từ BNO055
def calculate_trajectory(encoder_data, bno055_data):
    n = len(encoder_data)
    
    # Khởi tạo vị trí
    x = np.zeros(n)
    y = np.zeros(n)
    theta = np.zeros(n)
    
    # Lấy góc heading ban đầu
    theta[0] = find_nearest_heading(encoder_data[0, 0], bno055_data)
    
    # Chuyển đổi RPM sang vận tốc góc (rad/s)
    omega1 = rpm_to_omega(encoder_data[:, 1])
    omega2 = rpm_to_omega(encoder_data[:, 2])
    omega3 = rpm_to_omega(encoder_data[:, 3])
    
    # Ma trận chuyển đổi từ vận tốc bánh xe sang vận tốc robot
    def compute_robot_velocity(omega_wheels, theta_current):
        # Hệ số chuyển đổi
        k1 = r 
        k2 = r 
        # Ma trận chuyển đổi (Jacobian thuận)
        M = np.array([
            [k1 * np.sin(theta_current), k1 * np.sin(theta_current - 2*np.pi/3), k1 * np.sin(theta_current + 2*np.pi/3)],
            [k1 * np.cos(theta_current), k1 * np.cos(theta_current - 2*np.pi/3), k1 * np.cos(theta_current + 2*np.pi/3)],
            [k2/L, k2/L, k2/L]
        ])
        
        # Tính vận tốc robot
        return M @ omega_wheels
    
    # Tính quỹ đạo
    for i in range(1, n):
        # Thời gian delta
        dt = encoder_data[i, 0] - encoder_data[i-1, 0]
        
        # Lấy góc heading từ BNO055
        theta[i] = find_nearest_heading(encoder_data[i, 0], bno055_data)
        
        # Vận tốc bánh xe hiện tại
        wheel_velocities = np.array([omega1[i], omega2[i], omega3[i]])
        
        # Tính vận tốc robot trong hệ tọa độ robot
        vx_robot, vy_robot, _ = compute_robot_velocity(wheel_velocities, theta[i])
        
        # Chuyển đổi sang hệ tọa độ toàn cục
        vx_global = vx_robot * np.cos(theta[i]) - vy_robot * np.sin(theta[i])
        vy_global = vx_robot * np.sin(theta[i]) + vy_robot * np.cos(theta[i])
        
        # Cập nhật vị trí
        x[i] = x[i-1] + vx_global * dt
        y[i] = y[i-1] + vy_global * dt
    
    return x, y, theta

# Tính quỹ đạo
x, y, theta = calculate_trajectory(encoder_data, bno055_data)

# Vẽ quỹ đạo
plt.figure(figsize=(12, 10))

# Quỹ đạo robot
plt.subplot(2, 1, 1)
plt.plot(x, y, 'b-', label='Quỹ đạo robot')
plt.scatter(x[0], y[0], color='green', s=100, marker='o', label='Điểm bắt đầu')
plt.scatter(x[-1], y[-1], color='red', s=100, marker='x', label='Điểm kết thúc')

# Vẽ mũi tên hướng theo mỗi 50 điểm dữ liệu
arrow_indices = np.arange(0, len(x), 50)
for i in arrow_indices:
    if i < len(x):
        plt.arrow(x[i], y[i], 0.1*np.cos(theta[i]), 0.1*np.sin(theta[i]), 
                 head_width=0.05, head_length=0.05, fc='red', ec='red')

plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Quỹ đạo di chuyển của robot (sử dụng heading từ BNO055)')
plt.grid(True)
plt.axis('equal')
plt.legend()

# Góc heading và tốc độ bánh xe theo thời gian
plt.subplot(2, 1, 2)
plt.plot(encoder_data[:, 0], np.degrees(theta), 'r-', label='Góc Heading BNO055 (độ)')
plt.plot(encoder_data[:, 0], encoder_data[:, 1], 'g-', label='RPM bánh 1')
plt.plot(encoder_data[:, 0], encoder_data[:, 2], 'b-', label='RPM bánh 2')
plt.plot(encoder_data[:, 0], encoder_data[:, 3], 'm-', label='RPM bánh 3')
plt.xlabel('Thời gian (s)')
plt.ylabel('Giá trị')
plt.title('Góc Heading và tốc độ bánh xe theo thời gian')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# Thêm thông tin về quỹ đạo
total_distance = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
print(f"Tổng quãng đường di chuyển: {total_distance:.3f} m")
print(f"Vị trí cuối cùng: X = {x[-1]:.3f} m, Y = {y[-1]:.3f} m")
print(f"Góc cuối cùng: {np.degrees(theta[-1]):.2f} độ")