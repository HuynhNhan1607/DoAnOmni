import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
import io

def plot_fuzzy_log_data(csv_content):
    # Đọc dữ liệu CSV, bỏ qua dòng comment
    lines = csv_content.split('\n')
    data_lines = [line for line in lines if not line.startswith('//') and line.strip()]
    csv_data = '\n'.join(data_lines)
    df = pd.read_csv(io.StringIO(csv_data))
    
    # Tạo trục thời gian (mỗi dòng cách nhau 50ms)
    time_axis = np.arange(len(df)) * 0.05  # 50ms = 0.05s
    
    # Định nghĩa các nhóm thông số
    groups = {
        'PID Motor 1': ['M1_Kp', 'M1_Ki', 'M1_Kd'],
        'PID Motor 2': ['M2_Kp', 'M2_Ki', 'M2_Kd'],
        'PID Motor 3': ['M3_Kp', 'M3_Ki', 'M3_Kd'],
        'Errors': ['Error1', 'Error2', 'Error3'],
        'Deltas': ['Delta1', 'Delta2', 'Delta3'],
        'Execution': ['Exec_Time']
    }
    
    # Tạo figure và axes
    fig, ax = plt.subplots(figsize=(12, 8))
    plt.subplots_adjust(left=0.3)  # Để không gian cho các checkbox
    
    # Vẽ tất cả đường (ban đầu ẩn)
    lines = {}
    colors = {
        'M1_Kp': 'red', 'M1_Ki': 'green', 'M1_Kd': 'blue',
        'M2_Kp': 'red', 'M2_Ki': 'green', 'M2_Kd': 'blue',
        'M3_Kp': 'red', 'M3_Ki': 'green', 'M3_Kd': 'blue',
        'Error1': 'red', 'Error2': 'green', 'Error3': 'blue',
        'Delta1': 'red', 'Delta2': 'green', 'Delta3': 'blue',
        'Exec_Time': 'black'
    }
    
    line_styles = {
        'M1_Kp': '-', 'M1_Ki': '--', 'M1_Kd': '-.',
        'M2_Kp': '-', 'M2_Ki': '--', 'M2_Kd': '-.',
        'M3_Kp': '-', 'M3_Ki': '--', 'M3_Kd': '-.',
        'Error1': '-', 'Error2': '--', 'Error3': '-.',
        'Delta1': '-', 'Delta2': '--', 'Delta3': '-.',
        'Exec_Time': '-'
    }
    
    for group_columns in groups.values():
        for column in group_columns:
            line, = ax.plot(time_axis, df[column], 
                           label=column, 
                           color=colors[column],
                           linestyle=line_styles[column],
                           visible=False)
            lines[column] = line
    
    # Tạo các checkbox theo nhóm
    num_groups = len(groups)
    height_per_group = 0.8 / num_groups
    
    # Hàm xử lý khi checkbox được click
    def toggle_visibility(label):
        # Bật/tắt đường tương ứng
        lines[label].set_visible(not lines[label].get_visible())
        
        # Cập nhật legend
        visible_lines = [line for line in lines.values() if line.get_visible()]
        if visible_lines:
            ax.legend(visible_lines, [line.get_label() for line in visible_lines], 
                     loc='upper right')
        else:
            ax.legend_ = None
        
        # Tự động điều chỉnh trục y khi các đường thay đổi
        if visible_lines:
            ymin = min([min(line.get_ydata()) for line in visible_lines])
            ymax = max([max(line.get_ydata()) for line in visible_lines])
            margin = (ymax - ymin) * 0.1
            ax.set_ylim(ymin - margin, ymax + margin)
        
        plt.draw()
    
    checkboxes = []
    for i, (group_name, columns) in enumerate(groups.items()):
        # Vị trí cho nhóm checkbox
        checkbox_ax = plt.axes([0.01, 0.1 + i * height_per_group, 0.2, height_per_group - 0.02])
        checkbox_ax.set_title(group_name)
        
        # Tạo checkbox với trạng thái ban đầu là ẩn
        checkbox = CheckButtons(checkbox_ax, columns, [False] * len(columns))
        checkbox.on_clicked(toggle_visibility)
        checkboxes.append(checkbox)
    
    # Thiết lập tiêu đề và nhãn
    ax.set_title('Fuzzy Logic Controller Data')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Value')
    ax.grid(True)
    
    plt.show()

# Đọc nội dung từ file CSV và vẽ biểu đồ
with open("logs/fuzzy_log_20250512_205907.csv", "r") as f:
    csv_content = f.read()
    plot_fuzzy_log_data(csv_content)