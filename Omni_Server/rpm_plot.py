import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import numpy as np

TIME_INTERVAL = 50

class RPMPlotter:
    def __init__(self):
        self.rpm_data = [[] for _ in range(3)]  # Data storage for plotting RPM
        self.time_data = []
        self.start_time = time.time()
        
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], label=f"Motor {i+1}")[0] for i in range(3)]
        self.ax.set_xlim(0, 30)
        self.ax.set_ylim(-200, 200)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("RPM")
        self.ax.legend()
        self.ani = None  # Chỉ tạo animation khi cần

    def show_plot(self):
        if self.ani is None:
            self.ani = FuncAnimation(self.fig, self.update_plot, interval=TIME_INTERVAL, cache_frame_data=False)
        plt.show()

    def update_plot(self, frame):
        if len(self.time_data) == 0:
            return
        
        min_length = min(len(self.time_data), min(len(self.rpm_data[i]) for i in range(3)))
        self.time_data = self.time_data[-min_length:]
        
        for i in range(3):
            self.rpm_data[i] = self.rpm_data[i][-min_length:]
            self.lines[i].set_data(self.time_data, self.rpm_data[i])
        
        self.ax.set_xlim(max(0, self.time_data[-1] - 30), self.time_data[-1] + 1)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()

    def add_rpm_data(self, encoders):
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        
        for i in range(3):
            if len(self.rpm_data[i]) == 0:
                self.rpm_data[i].append(0)
            try:
                self.rpm_data[i].append(int(float(encoders[i])))  # Convert to float first, then int
            except ValueError:
                self.rpm_data[i].append(0)  # Default to 0 if conversion fails
        
        min_length = min(len(self.time_data), min(len(self.rpm_data[i]) for i in range(3)))
        self.time_data = self.time_data[-min_length:]
        for i in range(3):
            self.rpm_data[i] = self.rpm_data[i][-min_length:]

rpm_plotter = RPMPlotter()

def update_rpm_plot(encoders):
    rpm_plotter.add_rpm_data(encoders)
