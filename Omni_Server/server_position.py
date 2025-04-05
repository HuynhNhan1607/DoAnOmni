import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import threading
import time
import tkinter as tk
from tkinter import ttk
import math
import queue

class RobotPositionVisualizer:
    def __init__(self):
        self.position_window = None
        self.canvas = None
        self.fig = None
        self.ax = None
        self.is_active = False
        self.lock = threading.Lock()
        self.update_queue = queue.Queue()
        
        # Plot limits and grid size
        self.PLOT_X_MIN = -1.2
        self.PLOT_X_MAX = 5.4
        self.PLOT_Y_MIN = -1.2
        self.PLOT_Y_MAX = 5.4
        self.GRID_SIZE = 0.6  # m
        
        # Robot parameters
        self.robot_radius = 0.1543  # m
        
        # Position data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.trajectory_points = []
        
        # Update interval
        self.update_interval = 50  # ms - Giảm xuống để cập nhật nhanh hơn
        self.last_plot_update = 0
        
    def initialize_plot(self):
        """Initialize robot position visualization window"""
        if self.position_window is not None:
            return
        
        self.position_window = tk.Toplevel()
        self.position_window.title("Robot Position (from Robot)")
        self.position_window.geometry("800x600")
        self.position_window.protocol("WM_DELETE_WINDOW", self.close)
        
        # Create matplotlib figure and canvas
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        # Setup grid
        grid_size = self.GRID_SIZE
        x_ticks = np.arange(self.PLOT_X_MIN, self.PLOT_X_MAX + grid_size/2, grid_size)
        y_ticks = np.arange(self.PLOT_Y_MIN, self.PLOT_Y_MAX + grid_size/2, grid_size)
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)
        self.ax.grid(True, linewidth=0.8)
        self.ax.set_facecolor('#f8f8f8')
        
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot Position (Direct from Robot)')
        
        # Set plot limits
        self.ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
        self.ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.position_window)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # Thanh công cụ Matplotlib
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.position_window)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # Add control buttons
        button_frame = ttk.Frame(self.position_window)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        ttk.Button(button_frame, text="Close", command=self.close).pack(side=tk.RIGHT, padx=5, pady=5)
        
        # Initial robot drawing
        self._draw_robot(0, 0, 0)
        
        self.is_active = True
        self.last_plot_update = time.time() * 1000  # ms
        
        # Start the update checker
        self.check_update_queue()
    
    def check_update_queue(self):
        """Check for updates in the queue and apply them"""
        try:
            # Process position updates
            update_needed = False
            while not self.update_queue.empty():
                x, y, theta = self.update_queue.get_nowait()
                self.x = x
                self.y = y
                self.theta = theta
                self.trajectory_points.append((x, y))
                update_needed = True
            
            # Update plot if needed and enough time has passed
            current_time = time.time() * 1000  # ms
            if update_needed and (current_time - self.last_plot_update > self.update_interval):
                self._update_plot()
                self.last_plot_update = current_time
                
        except Exception as e:
            print(f"Error in position update queue: {e}")
        
        # Schedule next check
        if self.position_window:
            self.position_window.after(50, self.check_update_queue)
    
    def update_robot_position(self, x, y, theta):
        """Update robot position with data from robot"""
        if not self.is_active:
            return
        self.update_queue.put((x, y, theta))
    
    def _update_plot(self):
        """Update the plot with current position data"""
        if not self.is_active or not self.ax:
            return
        
        with self.lock:
            # Clear and redraw
            self.ax.clear()
            
            # Draw trajectory
            x_points = [p[0] for p in self.trajectory_points]
            y_points = [p[1] for p in self.trajectory_points]
            self.ax.plot(x_points, y_points, 'b-')
            
            # Setup grid
            grid_size = self.GRID_SIZE
            self.ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
            self.ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
            x_ticks = np.arange(self.PLOT_X_MIN, self.PLOT_X_MAX + grid_size/2, grid_size)
            y_ticks = np.arange(self.PLOT_Y_MIN, self.PLOT_Y_MAX + grid_size/2, grid_size)
            self.ax.set_xticks(x_ticks)
            self.ax.set_yticks(y_ticks)
            self.ax.grid(True, linewidth=0.8)
            self.ax.set_facecolor('#f8f8f8')
            
            # Sử dụng theta ở radian cho việc vẽ (numpy sử dụng radian)
            theta_rad = math.radians(self.theta)
            
            # Draw robot at current position using theta in radians
            self._draw_robot(self.x, self.y, theta_rad)
            
            # Format plot
            self.ax.set_aspect('equal')
            self.ax.set_xlabel('X Position (m)')
            self.ax.set_ylabel('Y Position (m)')
            self.ax.set_title('Robot Position (Data from Robot)')
            
            # Hiển thị vị trí và góc gốc (đã là degree)
            self.ax.text(self.PLOT_X_MAX - 1.0, self.PLOT_Y_MAX - 0.5,
                        f'Vị trí: ({self.x:.2f}m, {self.y:.2f}m)\n'
                        f'Góc: {self.theta:.1f}°', 
                        fontsize=10, fontweight='bold',
                        bbox=dict(facecolor='yellow', alpha=0.8, boxstyle='round,pad=0.5'))
            
            self.canvas.draw()
    
    def _draw_robot(self, x, y, theta):
        """Draw robot at specified position and orientation
        theta is in radians
        """
        # Draw robot body
        robot_body = plt.Circle((x, y), self.robot_radius, fill=False, color='r', linewidth=1.5)
        self.ax.add_patch(robot_body)
        
        # Draw direction arrows
        arrow_length = self.robot_radius * 1.2
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # X direction (heading) - red
        self.ax.arrow(x, y, arrow_length * cos_theta, arrow_length * sin_theta,
                    head_width=0.05, head_length=0.07, fc='r', ec='r', linewidth=1.5)
        # Y direction - green
        self.ax.arrow(x, y, -arrow_length * sin_theta, arrow_length * cos_theta,
                    head_width=0.05, head_length=0.07, fc='g', ec='g', linewidth=1.5)
        
        # Chỉ hiển thị label cho bánh xe
        wheel_angles = [0, 2*np.pi/3, 4*np.pi/3]  # 0°, 120°, 240°
        
        for i, angle in enumerate(wheel_angles):
            # Calculate global wheel angle
            global_angle = theta + angle
            
            # Calculate wheel position
            wheel_x = x + self.robot_radius * np.cos(global_angle)
            wheel_y = y + self.robot_radius * np.sin(global_angle)
            
            # Add wheel number
            label_offset = 0.03
            label_x = wheel_x + label_offset * np.cos(global_angle)
            label_y = wheel_y + label_offset * np.sin(global_angle)
            self.ax.text(label_x, label_y, str(i+1), fontsize=8, 
                        ha='center', va='center', color='black', 
                        bbox=dict(facecolor='white', alpha=0.7, boxstyle='circle'))
    
    def normalize_angle(self, angle):
        """Normalize angle to range -pi to pi"""
        return ((angle + np.pi) % (2 * np.pi)) - np.pi
    
    def show(self):
        """Show the position window"""
        if not self.is_active or not self.position_window:
            self.initialize_plot()
        else:
            self.position_window.deiconify()  # Hiện lại cửa sổ đã ẩn
            self.is_active = True
    
    def close(self):
        """Close the position window"""
        if self.position_window:
            self.position_window.withdraw()
            #self.is_active = False

# Create singleton instance
robot_position_visualizer = RobotPositionVisualizer()