import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Tham số của robot
r = 0.03  # Bán kính bánh xe (m)
L = 0.153   # Khoảng cách từ tâm robot đến bánh xe (m)
dt = 0.1  # Bước thời gian (s)
total_time = 10  # Tổng thời gian mô phỏng (s)
num_steps = int(total_time / dt)  # Số bước mô phỏng

# Vận tốc mong muốn trong global frame (m/s)
X_dot_c = 0  # Vận tốc theo hướng X (global frame)
Y_dot_c = 0.2199  # Vận tốc theo hướng Y (global frame)
theta_dot_c = 0.0  # Vận tốc góc (rad/s), ban đầu không quay

# Khởi tạo trạng thái ban đầu của robot
X_c = 0.0  # Vị trí X ban đầu (m)
Y_c = 0.0  # Vị trí Y ban đầu (m)
theta_c = 0.0  # Góc ban đầu (rad)

# Mảng lưu trữ quỹ đạo và các giá trị
trajectory = np.zeros((num_steps, 2))  # Lưu trữ quỹ đạo (X_c, Y_c)
thetas = np.zeros(num_steps)  # Lưu trữ góc theta_c
omegas = np.zeros((num_steps, 3))  # Lưu trữ omega_1, omega_2, omega_3

# Hàm tính ma trận H' (Jacobian)
def compute_H_prime(theta_c, L):
    return np.array([
        [-np.sin(theta_c), np.cos(theta_c), L],
        [-np.sin(np.pi/3 - theta_c), -np.cos(np.pi/3 - theta_c), L],
        [np.sin(np.pi/3 + theta_c), -np.cos(np.pi/3 + theta_c), L]
    ])

# Hàm chuyển đổi vận tốc từ global frame sang body frame - ĐÃ SỬA
def global_to_body(X_dot_c, Y_dot_c, theta_c):
    # Ma trận quay đúng từ global sang body
    R = np.array([
        [np.cos(theta_c), np.sin(theta_c)],
        [-np.sin(theta_c), np.cos(theta_c)]
    ])
    vel_global = np.array([X_dot_c, Y_dot_c])
    vel_body = R @ vel_global
    return vel_body[0], vel_body[1]

# Hàm chuyển đổi vận tốc từ body frame sang global frame - THÊM MỚI
def body_to_global(X_dot_b, Y_dot_b, theta_c):
    # Ma trận quay đúng từ body sang global
    R_inv = np.array([
        [np.cos(theta_c), -np.sin(theta_c)],
        [np.sin(theta_c), np.cos(theta_c)]
    ])
    vel_body = np.array([X_dot_b, Y_dot_b])
    vel_global = R_inv @ vel_body
    return vel_global[0], vel_global[1]

# Hàm tính vận tốc góc của bánh xe
def compute_wheel_speeds(X_dot_b, Y_dot_b, theta_dot_c, H_prime, r):
    vel_body = np.array([X_dot_b, Y_dot_b, theta_dot_c])
    omegas = (1/r) * H_prime @ vel_body
    return omegas

# Mô phỏng chuyển động
for t in range(num_steps):
    # Lưu trữ vị trí và góc hiện tại
    trajectory[t, 0] = X_c
    trajectory[t, 1] = Y_c
    thetas[t] = theta_c

    # Thêm nhiễu ngẫu nhiên để mô phỏng drift (góc lệch theta_c)
    drift = np.random.normal(0, 1)  # Nhiễu ngẫu nhiên (rad/s)
    theta_c += drift * dt
    
    # Không áp dụng điều khiển PID cho góc theta vì đây là robot omni-directional
    # theta_dot_c giữ nguyên là 0 hoặc giá trị mong muốn
    
    # Chuyển đổi vận tốc từ global frame sang body frame
    X_dot_b, Y_dot_b = global_to_body(X_dot_c, Y_dot_c, theta_c)

    # Tính ma trận H' tại thời điểm hiện tại
    H_prime = compute_H_prime(theta_c, L)

    # Tính vận tốc góc của bánh xe
    omegas[t, :] = compute_wheel_speeds(X_dot_b, Y_dot_b, theta_dot_c, H_prime, r)

    # Chuyển vận tốc từ body frame về global frame để cập nhật vị trí
    X_dot_global, Y_dot_global = body_to_global(X_dot_b, Y_dot_b, theta_c)

    # Cập nhật vị trí robot
    X_c += X_dot_global * dt
    Y_c += Y_dot_global * dt
    
    # Cập nhật góc theo nhiễu và vận tốc góc
    theta_c += theta_dot_c * dt  # Nếu theta_dot_c = 0, thì chỉ có nhiễu ảnh hưởng
    print(theta_c)
# Vẽ quỹ đạo và các thông số
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))

# Quỹ đạo của robot
ax1.plot(trajectory[:, 0], trajectory[:, 1], label='Quỹ đạo robot')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_title('Quỹ đạo chuyển động của robot')
ax1.grid(True)
ax1.legend()
ax1.axis('equal')

# Góc theta_c theo thời gian
time = np.arange(num_steps) * dt
ax2.plot(time, thetas, label='Góc theta_c (rad)')
ax2.set_xlabel('Thời gian (s)')
ax2.set_ylabel('Góc theta_c (rad)')
ax2.set_title('Góc theta_c thay đổi theo thời gian do nhiễu')
ax2.grid(True)
ax2.legend()

# Vận tốc góc của bánh xe
ax3.plot(time, omegas[:, 0], label='omega_1')
ax3.plot(time, omegas[:, 1], label='omega_2')
ax3.plot(time, omegas[:, 2], label='omega_3')
ax3.set_xlabel('Thời gian (s)')
ax3.set_ylabel('Vận tốc góc (rad/s)')
ax3.set_title('Vận tốc góc của các bánh xe')
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.show()

# Animation
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-1, 11)
ax.set_ylim(-5, 5)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Mô phỏng chuyển động của robot')
ax.grid(True)
ax.axis('equal')

line, = ax.plot([], [], 'b-', label='Quỹ đạo')
robot, = ax.plot([], [], 'ro', label='Robot')
ax.legend()

def init():
    line.set_data([], [])
    robot.set_data([], [])
    return line, robot

def animate(i):
    # Ensure i is within bounds
    if i >= len(trajectory):
        i = len(trajectory) - 1
        
    # Update trajectory line
    line.set_data(trajectory[:i, 0], trajectory[:i, 1])
    
    # Update robot position
    robot.set_data([trajectory[i, 0]], [trajectory[i, 1]])
    
    return line, robot

ani = animation.FuncAnimation(fig, animate, init_func=init, frames=num_steps, 
                             interval=dt*1000, blit=True)
plt.show()