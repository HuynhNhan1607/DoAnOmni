import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def simpson_non_overlapping(velocity, dt=0.1):
    """
    Tích phân theo nhóm 3 điểm liên tiếp KHÔNG TRƯỢT với Simpson's Rule
    v0, v1, v2, sau đó v3, v4, v5, v.v...
    """
    position = 0.0
    positions = [0.0]
    
    # Cần ít nhất 3 điểm để áp dụng Simpson
    if len(velocity) < 3:
        return positions
    
    # Tích phân từng nhóm 3 điểm không trượt
    for i in range(0, len(velocity) - 2, 3):
        # Lấy 3 điểm liên tiếp
        v0 = velocity[i]
        v1 = velocity[i+1]
        v2 = velocity[i+2]
        
        # Simpson's Rule cho 3 điểm
        # Tổng khoảng thời gian là 2*dt
        h = dt  # khoảng thời gian giữa các điểm
        dx = (h/3.0) * (v0 + 4*v1 + v2)
        
        # Cập nhật vị trí
        position += dx
        
        # Thêm vị trí này cho cả 3 điểm trong nhóm
        positions.append(position)
        if i + 1 < len(velocity):
            positions.append(position)
        if i + 2 < len(velocity):
            positions.append(position)
    
    # Nếu số điểm không chia hết cho 3, xử lý các điểm còn lại
    remaining = len(velocity) % 3
    if remaining == 1:
        # Còn 1 điểm, dùng Euler
        position += velocity[-1] * dt
        positions.append(position)
    elif remaining == 2:
        # Còn 2 điểm, dùng hình thang
        position += (velocity[-2] + velocity[-1]) / 2.0 * dt
        positions.append(position)
        positions.append(position)
    
    # Cắt bớt nếu mảng kết quả quá dài
    if len(positions) > len(velocity):
        positions = positions[:len(velocity)]
    
    return positions

def main():
    # Đọc dữ liệu CSV
    file_path = 'd:/Project/DoAn/Omni_Server/logs/position_log_20250413_222648.csv'
    data = pd.read_csv(file_path)
    
    # Trích xuất các cột
    vel_x = data['Raw_VelX'].values
    vel_y = data['Raw_VelY'].values
    pos_x = data['Raw_X'].values
    pos_y = data['Raw_Y'].values
    time = data['Time'].values
    
    # Loại bỏ các giá trị NaN
    mask = ~np.isnan(vel_x) & ~np.isnan(vel_y)
    vel_x = vel_x[mask]
    vel_y = vel_y[mask]
    pos_x = pos_x[mask]
    pos_y = pos_y[mask]
    time = time[mask]
    
    # Tích phân bằng Simpson không trượt
    pos_x_simpson = simpson_non_overlapping(vel_x, dt=0.1)
    pos_y_simpson = simpson_non_overlapping(vel_y, dt=0.1)
    
    # Điều chỉnh giá trị ban đầu để khớp với điểm bắt đầu
    pos_x_simpson = np.array(pos_x_simpson) + pos_x[0]
    pos_y_simpson = np.array(pos_y_simpson) + pos_y[0]
    
    # Vẽ biểu đồ Simpson không trượt
    plt.figure(figsize=(12, 10))
    
    plt.subplot(2, 1, 1)
    plt.plot(time, pos_x, 'k-', label='Raw Position X', linewidth=2)
    plt.plot(time, pos_x_simpson, 'r--', label='Simpson Integration X', linewidth=1.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Position X (m)')
    plt.title('Non-overlapping Simpson Integration - X axis')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(time, pos_y, 'k-', label='Raw Position Y', linewidth=2)
    plt.plot(time, pos_y_simpson, 'r--', label='Simpson Integration Y', linewidth=1.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Position Y (m)')
    plt.title('Non-overlapping Simpson Integration - Y axis')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Tính sai số RMSE
    rmse_x = np.sqrt(np.mean((pos_x - pos_x_simpson)**2))
    rmse_y = np.sqrt(np.mean((pos_y - pos_y_simpson)**2))
    
    print("RMSE for Simpson's Rule (Non-overlapping):")
    print(f"RMSE for position X: {rmse_x:.6f} m")
    print(f"RMSE for position Y: {rmse_y:.6f} m")

if __name__ == "__main__":
    main()