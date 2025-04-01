import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
import tkinter as tk
from tkinter import ttk
import math
import queue

class TrajectoryVisualizer:
    def __init__(self):
        self.trajectory_points = []
        self.trajectory_window = None
        self.canvas = None
        self.fig = None
        self.ax = None
        self.is_active = False
        self.lock = threading.Lock()
        self.update_queue = queue.Queue()
        
        # Robot parameters
        self.wheel_radius = 0.03  # m
        self.robot_radius = 0.1543  # m
        self.wheel_angles = [0, 2*np.pi/3, 4*np.pi/3]
        
        # Starting position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update_time = None
        
        # Cài đặt thời gian cập nhật đồ thị (miliseconds)
        self.update_interval = 500
        self.last_plot_update = 0
        
    def initialize_plot(self):
        """Initialize the trajectory visualization window"""
        if self.trajectory_window is not None:
            return
        
        self.trajectory_window = tk.Toplevel()
        self.trajectory_window.title("Robot Trajectory")
        self.trajectory_window.geometry("800x600")
        self.trajectory_window.protocol("WM_DELETE_WINDOW", self.close)
        
        # Create matplotlib figure and canvas
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot Trajectory')
        
        # Set initial plot limits
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.trajectory_window)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # Add control buttons
        button_frame = ttk.Frame(self.trajectory_window)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        ttk.Button(button_frame, text="Reset", command=self.reset_trajectory).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(button_frame, text="Close", command=self.close).pack(side=tk.RIGHT, padx=5, pady=5)
        
        # Thêm slider điều chỉnh khoảng thời gian cập nhật
        update_frame = ttk.Frame(button_frame)
        update_frame.pack(side=tk.LEFT, padx=20, fill=tk.X, expand=True)
        
        ttk.Label(update_frame, text="Update Interval:").pack(side=tk.LEFT, padx=5)
        
        self.interval_var = tk.IntVar(value=self.update_interval)
        interval_scale = ttk.Scale(update_frame, from_=100, to=2000, 
                                  orient=tk.HORIZONTAL, 
                                  variable=self.interval_var,
                                  command=lambda v: self.interval_var.set(int(float(v))))
        interval_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        ttk.Label(update_frame, textvariable=self.interval_var).pack(side=tk.LEFT, padx=5)
        ttk.Label(update_frame, text="ms").pack(side=tk.LEFT)
        
        self.is_active = True
        self.last_update_time = time.time()
        self.last_plot_update = time.time() * 1000  # ms
        
        # Start the update checker
        self.check_update_queue()
        
    def check_update_queue(self):
        """Check for updates in the queue and apply them"""
        try:
            # Xử lý các cập nhật vị trí
            positions_updated = False
            while not self.update_queue.empty():
                rpm_values = self.update_queue.get_nowait()
                # Cập nhật vị trí mà không cập nhật đồ thị
                self._update_position(rpm_values)
                positions_updated = True
            
            # Chỉ cập nhật đồ thị sau khoảng thời gian đã đặt
            current_time = time.time() * 1000  # ms
            if positions_updated and (current_time - self.last_plot_update > self.interval_var.get()):
                self._update_plot()
                self.last_plot_update = current_time
                
        except Exception as e:
            print(f"Error in update queue processing: {e}")
        
        # Schedule next check
        if self.trajectory_window:
            self.trajectory_window.after(50, self.check_update_queue)
    
    def _update_position(self, rpm_values):
        """Chỉ cập nhật vị trí mà không vẽ đồ thị"""
        if not self.is_active:
            return
            
        current_time = time.time()
        
        # Skip if this is the first update
        if self.last_update_time is None:
            self.last_update_time = current_time
            return
            
        dt = current_time - self.last_update_time
        
        # Convert RPM to rad/s for each wheel
        omega_wheels = [(rpm * 2 * np.pi / 60) for rpm in rpm_values]
        
        # Sử dụng động học thuận để tính toán vận tốc robot
        vx_global, vy_global, omega = self.forward_kinematics(
            omega_wheels[0], omega_wheels[1], omega_wheels[2], 
            self.wheel_radius, self.robot_radius, self.theta
        )
        
        # Integrate velocity to get position
        self.x += vx_global * dt
        self.y += vy_global * dt
        self.theta += omega * dt
        
        # Thêm điểm vào quỹ đạo
        with self.lock:
            self.trajectory_points.append((self.x, self.y))
        
        self.last_update_time = current_time
    
    def _update_plot(self):
        """Chỉ cập nhật đồ thị với dữ liệu hiện tại"""
        if not self.is_active or not self.ax:
            return
        
        with self.lock:
            # Get trajectory points
            x_points = [p[0] for p in self.trajectory_points]
            y_points = [p[1] for p in self.trajectory_points]
            
            # Clear and redraw
            self.ax.clear()
            self.ax.plot(x_points, y_points, 'b-')
            
            # Draw robot position and orientation
            robot_size = 0.1
            self.ax.plot(self.x, self.y, 'ro')
            
            cos_theta = np.cos(self.theta)
            sin_theta = np.sin(self.theta)
            
            self.ax.plot([self.x, self.x + robot_size * cos_theta], 
                         [self.y, self.y + robot_size * sin_theta], 'r-')
            
            # Format plot
            self.ax.grid(True)
            self.ax.set_aspect('equal')
            self.ax.set_xlabel('X Position (m)')
            self.ax.set_ylabel('Y Position (m)')
            self.ax.set_title('Robot Trajectory')
            
            # Adjust plot limits if needed
            x_min, x_max = self.ax.get_xlim()
            y_min, y_max = self.ax.get_ylim()
            
            if self.x < x_min or self.x > x_max or self.y < y_min or self.y > y_max:
                self.ax.set_xlim(min(x_min, self.x - 1), max(x_max, self.x + 1))
                self.ax.set_ylim(min(y_min, self.y - 1), max(y_max, self.y + 1))
            
            self.canvas.draw()
    
    def show(self):
        """Show the trajectory window"""
        if not self.is_active:
            self.initialize_plot()
        else:
            self.trajectory_window.deiconify()
    
    def close(self):
        """Close the trajectory window"""
        if self.trajectory_window:
            self.trajectory_window.withdraw()
            self.is_active = False
    
    def reset_trajectory(self):
        """Reset the trajectory plot"""
        with self.lock:
            self.trajectory_points = []
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.last_update_time = time.time()
            
            # Clear the plot
            if self.ax:
                self.ax.clear()
                self.ax.grid(True)
                self.ax.set_aspect('equal')
                self.ax.set_xlabel('X Position (m)')
                self.ax.set_ylabel('Y Position (m)')
                self.ax.set_title('Robot Trajectory')
                self.ax.set_xlim(-2, 2)
                self.ax.set_ylim(-2, 2)
                self.canvas.draw()
    
    def update_trajectory(self, rpm_values):
        """Update trajectory based on encoder readings"""
        self.update_queue.put(rpm_values)

    def forward_kinematics(self, omega1, omega2, omega3, wheel_radius, robot_radius, theta):
        """Tính toán động học thuận cho robot ba bánh đa hướng"""
        # Ma trận H
        H_inv = np.array([
                [-np.sin(theta), np.cos(theta), robot_radius],
                [-np.sin(np.pi / 3 - theta), -np.cos(np.pi / 3 - theta), robot_radius],
                [np.sin(np.pi / 3 + theta), -np.cos(np.pi / 3 + theta), robot_radius]
            ])
            
        # Vector vận tốc góc của bánh xe
        omega = np.array([omega1, omega2, omega3]) * wheel_radius
        
        # Tính toán vận tốc của robot trong hệ global
        velocity = np.linalg.inv(H_inv).dot(omega)
        
        return velocity[0], velocity[1], velocity[2]  # Trả về dot_x, dot_y, dot_theta

# Create a singleton instance
trajectory_visualizer = TrajectoryVisualizer()