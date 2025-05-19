import numpy as np
import matplotlib.pyplot as plt
import os
import csv

def forward_kinematics(omega1, omega2, omega3, wheel_radius, robot_radius, theta):
    """Tính toán động học thuận cho robot ba bánh đa hướng"""
    # Ma trận H
    # H_inv = np.array([
    #         [-np.sin(theta), np.cos(theta), robot_radius],
    #         [-np.sin(np.pi / 3 - theta), -np.cos(np.pi / 3 - theta), robot_radius],
    #         [np.sin(np.pi / 3 + theta), -np.cos(np.pi / 3 + theta), robot_radius]
    #     ])
    H = np.array([
                [-2/3 * np.sin(theta), -2/3 * np.cos(theta + np.pi/6), 2/3 * np.sin(theta + np.pi/3)],
                [2/3 * np.cos(theta), -2/3 * np.sin(theta + np.pi/6), -2/3 * np.cos(theta + np.pi/3)],
                [1/(3*robot_radius), 1/(3*robot_radius), 1/(3*robot_radius)]
                ])
    # Vector vận tốc góc của bánh xe
    omega = np.array([omega1, omega2, omega3]) * wheel_radius
    
    # Tính toán vận tốc của robot trong hệ global
    velocity = H.dot(omega)
    
    return velocity[0], velocity[1], velocity[2]  # Trả về dot_x, dot_y, dot_theta

def read_log_file(filename):
    """Đọc file log encoder và trích xuất timestamps và giá trị RPM"""
    timestamps = []
    rpm_values = []
    
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 4:  # Đảm bảo có đủ timestamp và 3 giá trị RPM
                try:
                    timestamp = float(parts[0])
                    rpm1 = float(parts[1])
                    rpm2 = float(parts[2])
                    rpm3 = float(parts[3])
                    
                    timestamps.append(timestamp)
                    rpm_values.append([rpm1, rpm2, rpm3])
                except ValueError:
                    # Bỏ qua các dòng không có giá trị số hợp lệ
                    continue
                    
    return timestamps, rpm_values

def calculate_trajectory(timestamps, rpm_values, wheel_radius, robot_radius):
    """Tính toán quỹ đạo robot từ giá trị RPM sử dụng động học thuận"""
    # Vị trí và hướng ban đầu
    x = 0.0
    y = 0.0
    theta = 0.0
    
    # Danh sách lưu các điểm quỹ đạo và vận tốc
    x_points = [x]
    y_points = [y]
    theta_points = [theta]
    vx_points = [0.0]
    vy_points = [0.0]
    omega_points = [0.0]
    dt_values = [0.0]
    
    # Chuyển đổi giá trị RPM thành rad/s và tính toán thay đổi vị trí
    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i-1]
        dt_values.append(dt)
        
        # Chuyển đổi RPM thành rad/s cho mỗi bánh xe
        omega_wheels = [(rpm * 2 * np.pi / 60) for rpm in rpm_values[i-1]]
        
        # Tính toán vận tốc sử dụng động học thuận
        vx, vy, omega = forward_kinematics(
            omega_wheels[0], omega_wheels[1], omega_wheels[2], 
            wheel_radius, robot_radius, theta
        )
        
        # Lưu giá trị vận tốc
        vx_points.append(vx)
        vy_points.append(vy)
        omega_points.append(omega)
        
        # Cập nhật vị trí và hướng
        x += vx * dt
        y += vy * dt
        theta += omega * dt
        
        # Lưu vị trí đã cập nhật
        x_points.append(x)
        y_points.append(y)
        theta_points.append(theta)
    
    return x_points, y_points, theta_points, vx_points, vy_points, omega_points, dt_values

def save_results_to_csv(filename, timestamps, rpm_values, x_points, y_points, theta_points, vx_points, vy_points, omega_points, dt_values):
    """Lưu kết quả tính toán vào file CSV"""
    with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        
        # Sử dụng tiêu đề tiếng Anh để tránh vấn đề mã hóa
        writer.writerow(['Time (s)', 'RPM1', 'RPM2', 'RPM3', 'dt (s)', 'Vx (m/s)', 'Vy (m/s)', 'Omega (rad/s)', 'X (m)', 'Y (m)', 'Theta (rad)'])
        
        # Ghi dữ liệu
        for i in range(len(timestamps)):
            if i < len(rpm_values):
                rpm_row = rpm_values[i]
            else:
                rpm_row = [0, 0, 0]
                
            writer.writerow([
                timestamps[i],
                rpm_row[0], rpm_row[1], rpm_row[2],
                dt_values[i],
                vx_points[i], vy_points[i], omega_points[i],
                x_points[i], y_points[i], theta_points[i]
            ])
def plot_trajectory(x_points, y_points, theta_points):
    """Vẽ quỹ đạo của robot"""
    plt.figure(figsize=(10, 8))
    
    # Vẽ đường quỹ đạo
    plt.plot(x_points, y_points, 'b-', linewidth=1.5, label='Quỹ đạo')
    
    # Vẽ điểm bắt đầu
    plt.plot(x_points[0], y_points[0], 'go', markersize=10, label='Điểm bắt đầu')
    
    # Vẽ điểm kết thúc
    plt.plot(x_points[-1], y_points[-1], 'ro', markersize=10, label='Điểm kết thúc')
    
    # Vẽ mũi tên hướng tại các khoảng đều nhau
    interval = max(1, len(x_points) // 30)  # Hiển thị khoảng 30 mũi tên hướng
    for i in range(0, len(x_points), interval):
        if i < len(theta_points):
            arrow_length = 0.05
            dx = arrow_length * np.cos(theta_points[i])
            dy = arrow_length * np.sin(theta_points[i])
            plt.arrow(x_points[i], y_points[i], dx, dy, 
                     head_width=0.02, head_length=0.02, fc='r', ec='r')
    
    # Đặt tỷ lệ trục bằng nhau để tránh biến dạng
    plt.axis('equal')
    plt.grid(True)
    plt.xlabel('Vị trí X (m)')
    plt.ylabel('Vị trí Y (m)')
    plt.title('Quỹ đạo Robot từ File Log')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

def main():
    # Thiết lập thông số
    log_file = "logs/encoder_log_20250331_184339.txt"
    wheel_radius = 0.03  # m
    robot_radius = 0.1543  # m
    results_file = "trajectory.csv"
    
    # Đọc file log
    timestamps, rpm_values = read_log_file(log_file)
    
    if not timestamps:
        print("Không tìm thấy dữ liệu hợp lệ trong file log.")
        return
    
    print(f"Đã đọc {len(timestamps)} mẫu dữ liệu từ file log.")
    
    # Tính toán quỹ đạo và vận tốc
    x_points, y_points, theta_points, vx_points, vy_points, omega_points, dt_values = calculate_trajectory(
        timestamps, rpm_values, wheel_radius, robot_radius
    )
    
    # Lưu kết quả vào file CSV
    save_results_to_csv(results_file, timestamps, rpm_values, x_points, y_points, 
                       theta_points, vx_points, vy_points, omega_points, dt_values)
    
    print(f"Đã lưu kết quả tính toán vào file {results_file}")
    
    # Vẽ quỹ đạo
    plot_trajectory(x_points, y_points, theta_points)

if __name__ == "__main__":
    main()