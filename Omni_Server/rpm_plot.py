import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import numpy as np

TIME_INTERVAL = 100

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
        plt.ion()  # Bật chế độ interactive để không bị treo
        if self.ani is None:
            self.ani = FuncAnimation(self.fig, self.update_plot, interval=TIME_INTERVAL, cache_frame_data=False)
        self.fig.canvas.draw()  # Đảm bảo vẽ xong trước khi hiển thị
        plt.show(block=False)


    def update_plot(self, frame):
        """Cập nhật dữ liệu trên biểu đồ."""
        if not self.time_data or any(len(self.rpm_data[i]) != len(self.time_data) for i in range(3)):
            return  # Không cập nhật nếu số lượng phần tử không khớp

        for i in range(3):
            self.lines[i].set_data(self.time_data, self.rpm_data[i])

        if len(self.time_data) > 1:
            self.ax.set_xlim(max(0, self.time_data[-1] - 30), self.time_data[-1] + 1)

        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()


    def add_rpm_data(self, encoders):
        """Nhận dữ liệu và cập nhật danh sách."""
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)

        for i in range(3):
            try:
                value = float(encoders[i]) if encoders[i] not in [None, ""] else 0
                self.rpm_data[i].append(value)
            except ValueError:
                self.rpm_data[i].append(0)

        # Đồng bộ hóa dữ liệu, tránh lỗi shape mismatch
        min_length = min(len(self.time_data), *(len(self.rpm_data[i]) for i in range(3)))

        self.time_data = self.time_data[-min_length:]
        for i in range(3):
            self.rpm_data[i] = self.rpm_data[i][-min_length:]


rpm_plotter = RPMPlotter()

def update_rpm_plot(encoders):
    rpm_plotter.add_rpm_data(encoders)