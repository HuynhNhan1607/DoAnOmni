import pygame
import math
import time

WIDTH, HEIGHT = 350, 250

ANGLE_ROTATION =  2 * math.pi / 3 #(Rad/s)
TIME_ROTATION = 2 # 2s
RPM = 70
M_PER_ROUND = 0.06 * math.pi

class ControlGUI:
    def __init__(self, root):
        self.root = root
        self.server = None
        self.setup_gui()

    def setup_gui(self):
        # Màu sắc
        self.WHITE = (255, 255, 255)
        self.BLUE = (0, 0, 255)
        self.RED = (255, 0, 0)

        # Thiết lập robot
        self.radius = 15
        self.x, self.y = WIDTH // 2, HEIGHT // 2  # Vị trí robot ban đầu
        self.max_speed = RPM * M_PER_ROUND / 60  

    def show_control_gui(self):
        pygame.init()
        self.window = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Omnidirectional Robot Control 360°")

    def update(self):
        # Vẽ màn hình
        self.window.fill(self.WHITE)  # Làm mới màn hình
        pygame.draw.circle(self.window, self.BLUE, (int(self.x), int(self.y)), self.radius)  # Robot
        pygame.display.update()  # Cập nhật màn hình

    def handle_events(self):
        pygame.time.delay(100)  # Delay để giới hạn tốc độ vòng lặp

        # Kiểm tra sự kiện
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

        # Lấy trạng thái các phím
        keys = pygame.key.get_pressed()

        # Tính toán vận tốc tuyến tính
        dot_x, dot_y, dot_theta = 0, 0, 0

        if keys[pygame.K_w]:  # Lên -> Tăng Y (Y hướng lên là dương)
            dot_y += self.max_speed  
        if keys[pygame.K_s]:  # Xuống -> Giảm Y
            dot_y -= self.max_speed  
        if keys[pygame.K_a]:  # Trái -> Giảm X
            dot_x -= self.max_speed
        if keys[pygame.K_d]:  # Phải -> Tăng X
            dot_x += self.max_speed

        # Tính toán vận tốc góc (quay)
        if keys[pygame.K_q]:  # Quay trái
            dot_theta = - (ANGLE_ROTATION / TIME_ROTATION)  # 5.1 = L/r 
            self.send_command(dot_x, dot_y, dot_theta)  # Gửi lệnh quay
            time.sleep(TIME_ROTATION)  # Chờ 1 giây
            self.send_command(dot_x, dot_y, 0)  # Gửi lệnh dừng quay

        if keys[pygame.K_e]:  # Quay phải
            dot_theta =  ANGLE_ROTATION  / TIME_ROTATION
            self.send_command(dot_x, dot_y, dot_theta)  # Gửi lệnh quay
            time.sleep(TIME_ROTATION)  # Chờ 1 giây
            self.send_command(dot_x, dot_y, 0)  # Gửi lệnh dừng quay

        if keys[pygame.K_c]:  # Dừng
            self.send_command(dot_x, dot_y, dot_theta)
        # Gửi lệnh nếu có di chuyển
        if ( dot_x != 0 or dot_y != 0 ) and dot_theta == 0:
            self.send_command(dot_x, dot_y, dot_theta)

        # Cập nhật vị trí mô phỏng
        scale = 15
        self.x += dot_x * scale  # X giữ nguyên
        self.y -= dot_y * scale  # Y giữ nguyên vì Y hướng lên là dương

        # Giới hạn robot không ra khỏi màn hình
        self.x = max(self.radius, min(WIDTH - self.radius, self.x))
        self.y = max(self.radius, min(HEIGHT - self.radius, self.y))

        # Vẽ màn hình
        self.window.fill(self.WHITE)  # Làm mới màn hình
        pygame.draw.circle(self.window, self.BLUE, (int(self.x), int(self.y)), self.radius)
        pygame.display.update()  # Cập nhật màn hình

        return True


    def send_command(self, dot_x, dot_y, dot_theta):
        # Gửi lệnh điều khiển đến server
        if self.server:
            self.server.send_command(dot_x, dot_y, dot_theta)
        else:
            print("Server not found!")

    def set_server(self, server):
        self.server = server

    def run(self):
        self.show_control_gui()
        clock = pygame.time.Clock()
        while True:
            self.update()
            if not self.handle_events():
                break
            clock.tick(20)
        pygame.quit()