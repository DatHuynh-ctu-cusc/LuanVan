# lidar_map_drawer.py
import math
from PIL import Image, ImageTk

def draw_lidar_on_canvas(canvas, data):
    if not canvas:
        return

    width = canvas.winfo_width()
    height = canvas.winfo_height()
    map_size = 6.0  # mét
    resolution = 0.1  # m/cell
    cells = int(map_size / resolution)

    # === Tạo ảnh bản đồ trắng ===
    img = Image.new("RGB", (cells, cells), "white")
    pixels = img.load()

    robot_x = cells // 2
    robot_y = cells // 2

    angle = data.get("angle_min", 0)
    angle_inc = data.get("angle_increment", 0.01)

    for r in data.get("ranges", []):
        if 0.05 < r < 6.0:
            x_m = r * math.cos(angle)
            y_m = r * math.sin(angle)
            gx = int((x_m + map_size / 2) / resolution)
            gy = int((y_m + map_size / 2) / resolution)

            if 0 <= gx < cells and 0 <= gy < cells:
                # === Vẽ ray từ tâm robot đến điểm vật cản ===
                x0, y0 = robot_x, robot_y
                x1, y1 = gx, gy
                steps = max(abs(x1 - x0), abs(y1 - y0))
                for i in range(steps):
                    x = int(x0 + (x1 - x0) * i / steps)
                    y = int(y0 + (y1 - y0) * i / steps)
                    if 0 <= x < cells and 0 <= y < cells and pixels[x, y] == (255, 255, 255):
                        pixels[x, y] = (200, 200, 200)  # xám: vùng tự do

                # === Vẽ điểm va chạm (đen) ===
                pixels[gx, gy] = (0, 0, 0)

        angle += angle_inc

    # === Vẽ thân robot (màu đỏ) ở giữa bản đồ ===
    robot_width = int(0.2 / resolution)
    robot_height = int(0.3 / resolution)
    for dx in range(-robot_width // 2, robot_width // 2 + 1):
        for dy in range(-robot_height // 2, robot_height // 2 + 1):
            px = robot_x + dx
            py = robot_y + dy
            if 0 <= px < cells and 0 <= py < cells:
                pixels[px, py] = (255, 0, 0)

    # === Vẽ hướng phía trước nằm trong thân robot (màu xanh) ===
    try:
        angle_forward = math.pi  # 180 độ = hướng trước
        dx = int(round(math.cos(angle_forward)))   # dx = -1
        dy = int(round(math.sin(angle_forward)))   # dy =  0

        gx = robot_x + dx
        gy = robot_y + dy

        if 0 <= gx < cells and 0 <= gy < cells:
            pixels[gx, gy] = (0, 255, 0)  # màu xanh: hướng trước 1 pixel trong thân robot
    except:
        pass


    # === Resize ảnh và hiển thị lên canvas ===
    img = img.resize((width, height), resample=Image.NEAREST)
    tk_img = ImageTk.PhotoImage(img)

    # === Tối ưu cập nhật ảnh không bị giật ===
    if hasattr(canvas, 'map_image'):
        canvas.itemconfig(canvas.map_image, image=tk_img)
    else:
        canvas.map_image = canvas.create_image(0, 0, anchor="nw", image=tk_img)

    canvas.image = tk_img  # giữ tham chiếu ảnh
