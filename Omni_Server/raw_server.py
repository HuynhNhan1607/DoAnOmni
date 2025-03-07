import socket
import threading
import re
import keyboard  # Requires the 'keyboard' library. Install it via `pip install keyboard`
import time
import tkinter as tk
from tkinter import filedialog
from raw_control import ControlGUI

from rpm_plot import update_rpm_plot
from rpm_plot import rpm_plotter


class Server:
    def __init__(self, gui):
        self.gui = gui
        
        self.control_active = False
        self.firmware_active = False
        self.file_path = None
        self.speed = [0, 0, 0]  # Speed for three motors
        self.encoders = [0, 0, 0]  # Encoder values for three motors
        self.pid_values = [[0, 0, 0] for _ in range(3)]  # PID values for three motors
        self.server_socket = None
        self.client_socket = None
        self.sending_firmware = False

    def start_firmware_server(self):
        self.firmware_active = True
        threading.Thread(target=self.firmware_server_thread).start()

    def firmware_server_thread(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(("0.0.0.0", 12345))
        self.server_socket.listen(1)
        self.gui.update_monitor("Firmware server started, waiting for client...")
        self.client_socket, addr = self.server_socket.accept()
        self.gui.update_monitor(f"Client connected for firmware: {addr}")
        threading.Thread(target=self.receive_upgrade, args=(self.client_socket,), daemon=True).start()
        self.gui.enable_file_selection()

    def send_firmware(self):
        if self.file_path and self.client_socket:
            try:
                self.sending_firmware = True
                with open(self.file_path, "rb") as f:
                    while True:
                        chunk = f.read(1024)
                        if not chunk:
                            break
                        self.client_socket.sendall(chunk)  # Use sendall to ensure complete sending
                self.gui.update_monitor("Firmware sent successfully.")
            except Exception as e:
                self.gui.update_monitor(f"Error sending firmware: {e}")
            finally: 
                self.sending_firmware = False
                self.client_socket.shutdown(socket.SHUT_WR)  # Ensure the socket is closed
        else:
            self.gui.update_monitor("No file selected or client not connected.")
    
    def receive_upgrade(self, sock):
        try:
            while True:
                if self.sending_firmware :
                    time.sleep(0.1)
                    print("Waiting for firmware to be sent...")
                    continue

                data = sock.recv(1024).decode()
                if not data:
                    break
                print(data)
        except Exception as e:
            self.gui.update_monitor(f"Error receiving upgrade status: {e}")

    def send_upgrade_command(self):
        if self.client_socket:
            try:
                self.client_socket.sendall(b"Upgrade")
                self.gui.update_monitor("Sent 'Upgrade' command to the client.")
            except Exception as e:
                self.gui.update_monitor(f"Failed to send 'Upgrade' command: {e}")
        else:
            self.gui.update_monitor("No client connected to send 'Upgrade' command.")

    def start_control_server(self):
        self.control_active = True
        threading.Thread(target=self.control_server_thread).start()

    def control_server_thread(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(("0.0.0.0", 12346))
        self.server_socket.listen(1)
        self.gui.update_monitor("Control server started, waiting for client...")
        self.client_socket, addr = self.server_socket.accept()
        self.gui.update_monitor(f"Client connected for control: {addr}")

        threading.Thread(target=self.receive_encoder_data, args=(self.client_socket,), daemon=True).start()

    def receive_encoder_data(self, sock):
        buffer = ""  # Lưu dữ liệu bị phân mảnh
        
        try:
            while True:
                data = sock.recv(1024).decode()
                if not data:
                    break

                buffer += data  # Thêm dữ liệu mới vào buffer
                
                while "\n" in buffer:  # Kiểm tra nếu có dòng kết thúc
                    line, buffer = buffer.split("\n", 1)  # Lấy một dòng hoàn chỉnh
                    line = line.strip()  # Loại bỏ ký tự trắng thừa

                    if line.startswith("1:"):
                        pattern = r"(\d):(-?\d+(?:\.\d+)?)"
                        matches = re.findall(pattern, line)

                        for motor_id, value in matches:
                            motor_index = int(motor_id) - 1
                            self.encoders[motor_index] = float(value)  # Ép kiểu float

                        self.gui.update_encoders(self.encoders)
                        update_rpm_plot(self.encoders)  # Cập nhật biểu đồ

                        with open("encoder_data.txt", "a") as file:
                            file.write(" ".join([str(e) for e in self.encoders]) + "\n")

                    elif line.startswith("LOG:"):
                        log_message = line[4:]  
                        print(log_message)

                    else:
                        print(f"Unknown data format: {line}")
                        
        except Exception as e:
            self.gui.update_monitor(f"Encoder reception error: {e}")
            print(buffer)

    def send_command(self, dot_x, dot_y, dot_theta):
        # Gửi lệnh điều khiển đến client
        command = f"dot_x:{dot_x:.4f} dot_y:{dot_y:.4f} dot_theta:{dot_theta:.4f}"
        print(command)
        if self.client_socket:
            try:
                self.client_socket.sendall(command.encode())
                # Log lệnh đã gửi
            except Exception as e:
                print(f"Send command reception error: {e}")
        

    def set_speed(self, motor_index, speed):
        self.speed[motor_index] = speed
        if self.client_socket:
            self.client_socket.sendall(f"MOTOR_{motor_index + 1}_SPEED:{speed}".encode())
        self.gui.update_monitor(f"Motor {motor_index + 1} speed updated to {speed}")

    def send_set_pid(self):
        rpm_plotter.show_plot()
        if self.client_socket:
            try:
                self.client_socket.sendall(b"Set PID")
                self.gui.update_monitor("Sent 'Set PID' command to the client.")
            except Exception as e:
                self.gui.update_monitor(f"Failed to send 'Set PID' command: {e}")
        else:
            self.gui.update_monitor("No client connected to send 'Set PID' command.")

    def set_pid_values(self, motor_index, p, i, d):
        self.pid_values[motor_index] = [p, i, d]
        if self.client_socket:
            pid_command = f"MOTOR:{motor_index + 1} Kp:{p} Ki:{i} Kd:{d}"
            self.client_socket.sendall(pid_command.encode())
        self.gui.update_monitor(f"MOTOR:{motor_index + 1} Kp:{p} Ki:{i} Kd:{d}")

class ServerGUI:
    def __init__(self, root):
        self.root = root
        self.server = Server(self)
        self.encoder_labels = []
        self.pid_entries = []
        self.control_gui = ControlGUI(root)
        self.control_gui.set_server(self.server)
        self.setup_gui()

    def setup_gui(self):
        self.root.title("Server Control")
        self.root.geometry("800x600")

        self.canvas = tk.Canvas(self.root, bg="white", width=800, height=400)
        self.canvas.pack(fill="both", expand=True)

        # Firmware Upgrade
        self.canvas.create_text(100, 30, text="Firmware Upgrade", font=("Arial", 12, "bold"))
        self.start_firmware_button = tk.Button(self.root, text="Start Firmware Server", command=self.server.start_firmware_server)
        self.canvas.create_window(100, 60, window=self.start_firmware_button)

        self.choose_file_button = tk.Button(self.root, text="Choose Firmware", command=self.choose_file, state="disabled")
        self.canvas.create_window(250, 60, window=self.choose_file_button)

        self.send_firmware_button = tk.Button(self.root, text="Send Firmware", command=self.server.send_firmware, state="disabled")
        self.canvas.create_window(400, 60, window=self.send_firmware_button)
        
        # Switch Upgrade
        self.canvas.create_text(600, 30, text="Switch Upgrade", font=("Arial", 12, "bold"))

        self.switch_upgrade_button = tk.Button(self.root, text="Switch Upgrade", command=self.server.send_upgrade_command)
        self.canvas.create_window(600, 60, window=self.switch_upgrade_button)

        #Manual Control
        self.manual_control_button = tk.Button(self.root, text="Manual Control", command=self.manual_control)
        self.canvas.create_window(600, 120, window=self.manual_control_button)

        #Set PID
        self.set_pid_button = tk.Button(self.root, text="Set PID", command=self.server.send_set_pid)
        self.canvas.create_window(600, 180, window=self.set_pid_button)

        # Control Section
        self.canvas.create_text(100, 100, text="Remote Control", font=("Arial", 12, "bold"))
        self.start_control_button = tk.Button(self.root, text="Start Control Server", command=self.server.start_control_server)
        self.canvas.create_window(100, 130, window=self.start_control_button)

        # PID Control
        self.canvas.create_text(350, 100, text="PID Control", font=("Arial", 12, "bold"))
        for i in range(3):
            self.canvas.create_text(260 + i * 100, 150, text=f"PID {i + 1}")
            p_entry = tk.Entry(self.root, width=5)
            i_entry = tk.Entry(self.root, width=5)
            d_entry = tk.Entry(self.root, width=5)
            p_entry.insert(0, "0")
            i_entry.insert(0, "0")
            d_entry.insert(0, "0")
            self.canvas.create_window(260 + i * 100, 180, window=p_entry)
            self.canvas.create_window(260 + i * 100, 210, window=i_entry)
            self.canvas.create_window(260 + i * 100, 240, window=d_entry)
            self.pid_entries.append((p_entry, i_entry, d_entry))
            set_button = tk.Button(self.root, text="Set", command=lambda idx=i: self.set_pid(idx))
            self.canvas.create_window(260 + i * 100, 270, window=set_button)

        for i in range(3):
            text_x = 27   
            entry_x = 90
            button_x = 125
            encoder_x = 160
            Y_Display = 170

            self.canvas.create_text(text_x, Y_Display + i * 40, text=f"Motor {i + 1}:", anchor="w")
            
            speed_entry = tk.Entry(self.root, width=7)
            speed_entry.insert(0, "0")
            self.canvas.create_window(entry_x, Y_Display + i * 40, window=speed_entry, anchor="w")

            set_button = tk.Button(self.root, text="Set", command=lambda idx=i, e=speed_entry: self.set_motor_speed(idx, e))
            self.canvas.create_window(button_x, Y_Display + i * 40, window=set_button, anchor="w")

            encoder_label = tk.Label(self.root, text=f"RPM: 0")
            self.encoder_labels.append(encoder_label)
            self.canvas.create_window(encoder_x, Y_Display + i * 40, window=encoder_label, anchor="w")


        # Monitor Section
        self.canvas.create_text(100, 300, text="Monitor", font=("Arial", 12, "bold"))
        self.monitor_text = tk.Text(self.root, height=8, width=70, state="disabled")
        self.canvas.create_window(400, 400, window=self.monitor_text)

    def set_pid(self, motor_index):
        try:
            p = float(self.pid_entries[motor_index][0].get())
            i = float(self.pid_entries[motor_index][1].get())
            d = float(self.pid_entries[motor_index][2].get())
            self.server.set_pid_values(motor_index, p, i, d)
        except ValueError:
            self.update_monitor(f"Invalid PID values for Motor {motor_index + 1}.")

    def update(self):
        self.control_gui.update()
        if not self.control_gui.handle_events():
            self.root.quit()

    def manual_control(self):
        # self.control_gui.show_control_gui()
        # self.control_gui.run()
        manual_thread = threading.Thread(target=self.control_gui.run, daemon=True)
        manual_thread.start()



    def enable_file_selection(self):
        self.choose_file_button.config(state="normal")

    def choose_file(self):
        self.server.file_path = filedialog.askopenfilename(filetypes=[("Binary files", "*.bin")])
        if self.server.file_path:
            self.send_firmware_button.config(state="normal")

    def set_motor_speed(self, motor_index, entry):
        try:
            speed = int(entry.get())
            if -200 <= speed <= 200:
                #speed = int(speed * 1020 / 200)
                self.server.set_speed(motor_index, speed)
            else:
                self.update_monitor(f"Motor {motor_index + 1} speed must be between -200 and 200.")
        except ValueError:
            self.update_monitor(f"Invalid speed value for Motor {motor_index + 1}.")

    def update_encoders(self, encoders):
        for i, value in enumerate(encoders):
            self.encoder_labels[i].config(text=f"RPM: {value}")

    def update_monitor(self, message):
        self.monitor_text.config(state="normal")
        self.monitor_text.insert("end", message + "\n")
        self.monitor_text.config(state="disabled")
        self.monitor_text.see("end")

if __name__ == "__main__":
    root = tk.Tk()
    app = ServerGUI(root)
    root.mainloop()
