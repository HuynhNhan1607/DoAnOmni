import pygame
import numpy as np

# Khởi tạo Pygame
pygame.init()

# Cấu hình lưới (10 cm x 10 cm)
GRID_WIDTH = 30   # Số ô theo chiều rộng (tương ứng 3m)
GRID_HEIGHT = 54  # Số ô theo chiều dài (tương ứng 5.4m)
CELL_SIZE = 15    # Kích thước mỗi ô trên màn hình (pixel)
WINDOW_WIDTH = GRID_WIDTH * CELL_SIZE
WINDOW_HEIGHT = GRID_HEIGHT * CELL_SIZE
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT + 50))  # Thêm không gian cho text
pygame.display.set_caption("Vẽ bản đồ lưới cho ngôi nhà")

# Kích thước thực tế của khu vực (mét)
REAL_WIDTH = 3.0   # Chiều rộng thực tế: 3 mét
REAL_HEIGHT = 5.4  # Chiều dài thực tế: 5.4 mét

# Tính kích thước thực tế của 1 ô
CELL_REAL_WIDTH = REAL_WIDTH / GRID_WIDTH   # mét
CELL_REAL_HEIGHT = REAL_HEIGHT / GRID_HEIGHT  # mét
print(f"Mỗi ô nhỏ: {CELL_REAL_WIDTH*100:.1f} cm x {CELL_REAL_HEIGHT*100:.1f} cm")

# Tạo mảng lưới (0: trống, 1: vật cản)
grid = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=int)

# Màu sắc
WHITE = (255, 255, 255)  # Trống
BLACK = (0, 0, 0)        # Vật cản
GREEN = (0, 255, 0)      # Điểm Start
GRAY = (200, 200, 200)   # Viền ô nhỏ
DARK_GRAY = (50, 50, 50) # Viền ô lớn

# Font để hiển thị text
font = pygame.font.Font(None, 36)

# Biến để theo dõi chế độ vẽ
drawing_obstacle = False
erasing_obstacle = False  # Chế độ xóa
setting_start = False
start_pos = None

# Vòng lặp chính
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_s:  # Nhấn 'S' để lưu bản đồ
                # Tạo dictionary chứa dữ liệu cần lưu
                map_data = {
                    "grid": grid,
                    "start_pos": start_pos
                }
                np.save("house_map.npy", map_data)
                print("Đã lưu bản đồ (10cm x 10cm) vào house_map.npy")
                print("Kích thước lưới:", grid.shape)
                if start_pos:
                    print(f"Điểm Start: {start_pos}")
            elif event.key == pygame.K_o:  # Nhấn 'O' để vẽ vật cản
                drawing_obstacle = not drawing_obstacle
                erasing_obstacle = False
                setting_start = False
                print("Chế độ vẽ vật cản:", drawing_obstacle)
            elif event.key == pygame.K_e:  # Nhấn 'E' để xóa vật cản
                erasing_obstacle = not erasing_obstacle
                drawing_obstacle = False
                setting_start = False
                print("Chế độ xóa vật cản:", erasing_obstacle)
            elif event.key == pygame.K_p:  # Nhấn 'P' để đặt điểm Start
                setting_start = not setting_start
                drawing_obstacle = False
                erasing_obstacle = False
                print("Chế độ đặt điểm Start:", setting_start)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            x, y = event.pos
            if y < WINDOW_HEIGHT:  # Chỉ xử lý click trong khu vực lưới
                grid_x, grid_y = x // CELL_SIZE, y // CELL_SIZE
                if drawing_obstacle:
                    grid[grid_y, grid_x] = 1  # Đặt vật cản
                elif erasing_obstacle:
                    grid[grid_y, grid_x] = 0  # Xóa vật cản
                elif setting_start:
                    start_pos = (grid_x, grid_y)
                    print(f"Điểm Start: {start_pos}")

    # Vẽ lưới
    screen.fill(WHITE)
    for i in range(GRID_HEIGHT):
        for j in range(GRID_WIDTH):
            rect = pygame.Rect(j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            if grid[i, j] == 1:
                pygame.draw.rect(screen, BLACK, rect)  # Vật cản
            # Vẽ viền ô nhỏ
            pygame.draw.rect(screen, GRAY, rect, 1)
            # Vẽ viền đậm cho ô lớn (60 cm x 60 cm)
            if j % 6 == 0:  # Viền dọc
                pygame.draw.line(screen, DARK_GRAY, (j * CELL_SIZE, i * CELL_SIZE), (j * CELL_SIZE, (i + 1) * CELL_SIZE), 3)
            if i % 6 == 0:  # Viền ngang
                pygame.draw.line(screen, DARK_GRAY, (j * CELL_SIZE, i * CELL_SIZE), ((j + 1) * CELL_SIZE, i * CELL_SIZE), 3)

    # Vẽ viền đậm cho các cạnh ngoài cùng của ô lớn
    for i in range(0, GRID_HEIGHT + 1, 6):
        pygame.draw.line(screen, DARK_GRAY, (0, i * CELL_SIZE), (WINDOW_WIDTH, i * CELL_SIZE), 3)
    for j in range(0, GRID_WIDTH + 1, 6):
        pygame.draw.line(screen, DARK_GRAY, (j * CELL_SIZE, 0), (j * CELL_SIZE, WINDOW_HEIGHT), 3)

    # Vẽ điểm Start
    if start_pos:
        start_x, start_y = start_pos
        pygame.draw.circle(screen, GREEN, (start_x * CELL_SIZE + CELL_SIZE // 2, start_y * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 3)

    # Hiển thị thông tin kích thước ô
    text = font.render(f"Ô nhỏ: 1 ô = {CELL_REAL_WIDTH*100:.1f}cm x {CELL_REAL_HEIGHT*100:.1f}cm | Ô lớn: 60cm x 60cm", True, BLACK)
    screen.blit(text, (10, WINDOW_HEIGHT + 10))

    pygame.display.flip()

pygame.quit()