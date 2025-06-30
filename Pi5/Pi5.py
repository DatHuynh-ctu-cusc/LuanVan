import os
import tkinter as tk
from tkinter import filedialog
import socket
import threading
import json
import time
import math
import numpy as np
from collections import deque
from gpiozero import DigitalInputDevice, Button

# === ENCODER SETUP ===
ENCODERS = {
    'E1': {'A': 21, 'B': 20},
    'E2': {'A': 5,  'B': 6},
    'E3': {'A': 12, 'B': 18},
    'E4': {'A': 23, 'B': 24}
}

LIMIT_SWITCHES = {
    'L1': Button(4),
    'L2': Button(17),
    'L3': Button(22),
    'L4': Button(13),
}

positions = {key: 0 for key in ENCODERS}
encoders = {}
for key, pins in ENCODERS.items():
    encoders[key] = {
        'A': DigitalInputDevice(pins['A']),
        'B': DigitalInputDevice(pins['B'])
    }
lock = threading.Lock()

def make_callback(key):
    def callback():
        with lock:
            if encoders[key]['B'].value == 0:
                positions[key] += 1
            else:
                positions[key] -= 1
    return callback

for key in ENCODERS:
    encoders[key]['A'].when_activated = make_callback(key)
    encoders[key]['A'].when_deactivated = make_callback(key)


class SimpleApp:
    def __init__(self, root):
        self.root = root
        self.root.title("App Don Gian Co Sidebar")
        self.root.geometry("1000x600")

        self.running = threading.Event()
        self.running.set()

        # Sidebar
        self.sidebar = tk.Frame(root, width=250, bg="#2c3e50")
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)

        tk.Label(self.sidebar, text="\U0001F4CB MENU", bg="#2c3e50", fg="white", font=("Arial", 14, "bold")).pack(pady=20)
        self.add_sidebar_button("\U0001F3E1 Trang chu", self.show_home)
        self.add_sidebar_button("\U0001F6F9 Ban do", self.show_map)
        self.add_sidebar_button("\U0001F4BE Du lieu", self.show_data)
        self.add_sidebar_button("\U0001F4C2 Thu muc", self.show_folder)
        self.add_sidebar_button("\U0001F916 Robot", self.show_robot)
        self.add_sidebar_button("\U0001F6E0 Cai dat", self.show_settings)

        self.content = tk.Frame(root, bg="white")
        self.content.pack(side="right", expand=True, fill="both")

        self.encoder_labels = {}
        self.lidar_canvas = None
        self.ray_history = deque(maxlen=3000)
        self.grid_size = 0.1  # m / cell
        self.map_width = 120  # 12m
        self.map_height = 120
        self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.uint8)

        self.drawing_mode = False
        self.path_points = []

        self.show_home()
        threading.Thread(target=self.send_to_pi4, daemon=True).start()
        threading.Thread(target=self.receive_lidar_from_pi4, daemon=True).start()

    def add_sidebar_button(self, label, command):
        btn = tk.Button(self.sidebar, text=label, bg="#34495e", fg="white", font=("Arial", 12), height=2, anchor="w", padx=20, relief="flat")
        btn.pack(fill="x", pady=2)
        btn.config(command=lambda b=btn, c=command: self.on_sidebar_click(b, c))

    def on_sidebar_click(self, button, command):
        for child in self.sidebar.winfo_children():
            if isinstance(child, tk.Button):
                child.config(bg="#34495e")
        button.config(bg="#1abc9c")
        command()

    def show_home(self):
        self.update_content("\U0001F3E1 Day la Trang chu")

    def show_map(self):
        for widget in self.content.winfo_children():
            widget.destroy()

        top_frame = tk.Frame(self.content)
        top_frame.pack(side="top", fill="x")

        draw_btn = tk.Button(top_frame, text="🖊️ Vẽ đường đi", command=self.toggle_drawing_mode)
        draw_btn.pack(side="left", padx=5, pady=5)

        clear_path_btn = tk.Button(top_frame, text="🗑️ Xóa đường đi", command=self.clear_path)
        clear_path_btn.pack(side="left", padx=5, pady=5)

        save_path_btn = tk.Button(top_frame, text="💾 Lưu đường đi", command=self.save_path)
        save_path_btn.pack(side="left", padx=5, pady=5)

        self.lidar_canvas = tk.Canvas(self.content, bg="white")
        self.lidar_canvas.pack(fill="both", expand=True)
        self.lidar_canvas.bind("<Configure>", lambda e: self.redraw_lidar_map())
        self.lidar_canvas.bind("<Button-1>", self.on_canvas_click)

    def show_data(self):
        for widget in self.content.winfo_children():
            widget.destroy()
        tk.Label(self.content, text="\U0001F4BE Du lieu robot", font=("Arial", 20), bg="white").pack(pady=20)

    def show_folder(self):
        self.update_content("\U0001F4C2 Thu muc luu file")

    def show_robot(self):
        for widget in self.content.winfo_children():
            widget.destroy()
        self.canvas = tk.Canvas(self.content, bg="white")
        self.canvas.pack(expand=True, fill="both")
        self.encoder_labels = {}
        self.canvas.bind("<Configure>", self.draw_robot_centered)

    def show_settings(self):
        self.update_content("\U0001F6E0 Cai dat he thong")

    def update_content(self, text):
        for widget in self.content.winfo_children():
            widget.destroy()
        tk.Label(self.content, text=text, font=("Arial", 20), bg="white").pack(expand=True)

    def draw_robot_centered(self, event):
        self.canvas.delete("all")
        for lbl in self.encoder_labels.values():
            lbl.destroy()
        self.encoder_labels.clear()
        width, height = event.width, event.height
        car_w, car_h = 200, 300
        x0 = (width - car_w) // 2
        y0 = (height - car_h) // 2
        x1 = x0 + car_w
        y1 = y0 + car_h
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="#bdc3c7", outline="black", width=2)
        wheel_pos = {
            "E1": (x0 - 45, y0 + 10),
            "E2": (x0 - 45, y1 - 40),
            "E3": (x1 + 5, y0 + 10),
            "E4": (x1 + 5, y1 - 40),
        }
        for name, (x, y) in wheel_pos.items():
            self.canvas.create_rectangle(x, y, x + 40, y + 30, fill="#2c3e50")
            self.canvas.create_text(x + 20, y - 10, text=name, font=("Arial", 9))
            lbl = tk.Label(self.canvas, text=f"Encoder: {positions[name]}", bg="white", font=("Arial", 9))
            lbl.place(x=x - 5, y=y + 35)
            self.encoder_labels[name] = lbl

    def send_to_pi4(self):
        HOST_PI4 = '192.168.100.1'
        PORT = 9999
        while self.running.is_set():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((HOST_PI4, PORT))
                    print("[App] Ket noi encoder/limit toi Pi4")
                    while self.running.is_set():
                        with lock:
                            pos_str = ';'.join([f"{k}:{v}" for k, v in positions.items()])
                        limit_state = ';'.join([f"{k}:{'1' if btn.is_pressed else '0'}" for k, btn in LIMIT_SWITCHES.items()])
                        msg = f"ENC{{{pos_str}}};LIMITS{{{limit_state}}}\n"
                        s.sendall(msg.encode())
                        time.sleep(0.1)
            except:
                time.sleep(2)

    def receive_lidar_from_pi4(self):
        PORT = 8899
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', PORT))
        server.listen(1)
        print("[App] Dang cho Lidar tu Pi4...")
        while self.running.is_set():
            conn, addr = server.accept()
            print(f"[App] Da ket noi Lidar: {addr}")
            with conn:
                conn.settimeout(5)
                buffer = ""
                while self.running.is_set():
                    try:
                        chunk = conn.recv(4096).decode()
                        if not chunk:
                            break
                        buffer += chunk
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line == "PING":
                                conn.sendall(b"PONG\n")
                            else:
                                try:
                                    parsed = json.loads(line)
                                    self.update_lidar_map(parsed)
                                except:
                                    print("[App] Loi parse JSON")
                    except socket.timeout:
                        print("[App] Lidar socket timeout")
                        break
                    except:
                        break

    def update_lidar_map(self, data):
        if not self.lidar_canvas:
            return

        angle = data.get("angle_min", 0)
        angle_inc = data.get("angle_increment", 0.01)
        ranges = data.get("ranges", [])

        width = self.lidar_canvas.winfo_width()
        height = self.lidar_canvas.winfo_height()
        cx, cy = width // 2, height // 2
        scale = min(width, height) / 6.0

        new_rays = []
        for r in ranges:
            if 0.05 < r < 6.0:
                x_m = r * math.cos(angle)
                y_m = r * math.sin(angle)
                px = cx + x_m * scale
                py = cy - y_m * scale
                new_rays.append(((cx, cy), (px, py)))
                self.mark_occupancy_map(x_m, y_m)
            angle += angle_inc

        self.ray_history.extend(new_rays)
        self.redraw_lidar_map()

    def mark_occupancy_map(self, x_m, y_m):
        gx = int(self.map_width // 2 + x_m / self.grid_size)
        gy = int(self.map_height // 2 - y_m / self.grid_size)
        if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
            self.occupancy_map[gy, gx] = 255

    def redraw_lidar_map(self):
        if not self.lidar_canvas:
            return
        self.lidar_canvas.delete("all")
        w, h = self.lidar_canvas.winfo_width(), self.lidar_canvas.winfo_height()
        cx, cy = w // 2, h // 2
        scale = min(w, h) / 6.0

        # draw occupancy grid
        cell_w = w / self.map_width
        cell_h = h / self.map_height
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.occupancy_map[y, x] > 0:
                    px = x * cell_w
                    py = y * cell_h
                    self.lidar_canvas.create_rectangle(px, py, px + cell_w, py + cell_h, fill="black", outline="")

        # draw lidar rays
        for ray in self.ray_history:
            x0, y0 = ray[0]
            x1, y1 = ray[1]
            self.lidar_canvas.create_line(x0, y0, x1, y1, fill="gray", width=1)
            self.lidar_canvas.create_oval(x1 - 1, y1 - 1, x1 + 1, y1 + 1, fill="red", outline="")

        # draw manual path
        if self.path_points:
            coords = []
            for x_m, y_m in self.path_points:
                px = cx + x_m * scale
                py = cy - y_m * scale
                coords.append((px, py))
            for i in range(1, len(coords)):
                self.lidar_canvas.create_line(coords[i-1][0], coords[i-1][1],
                                              coords[i][0], coords[i][1],
                                              fill="green", width=2)

    def toggle_drawing_mode(self):
        self.drawing_mode = not self.drawing_mode
        print(f"[App] 🖊️ Chế độ vẽ đường: {'BẬT' if self.drawing_mode else 'TẮT'}")

    def on_canvas_click(self, event):
        if not self.drawing_mode:
            return
        w, h = self.lidar_canvas.winfo_width(), self.lidar_canvas.winfo_height()
        cell_w = w / self.map_width
        cell_h = h / self.map_height
        gx = int(event.x / cell_w)
        gy = int(event.y / cell_h)
        x_m = (gx - self.map_width // 2) * self.grid_size
        y_m = (self.map_height // 2 - gy) * self.grid_size
        self.path_points.append((x_m, y_m))
        print(f"[App] 🖊️ Thêm điểm: ({x_m:.2f}, {y_m:.2f})")
        self.redraw_lidar_map()

    def clear_path(self):
        self.path_points.clear()
        print("[App] 🗑️ Đã xóa đường đi")
        self.redraw_lidar_map()

    def save_path(self):
        if not self.path_points:
            print("[App] ⚠️ Không có điểm đường để lưu")
            return
        path_file = filedialog.asksaveasfilename(defaultextension=".json",
                                                  filetypes=[("JSON files", "*.json")],
                                                  title="Lưu đường đi")
        if not path_file:
            return
        with open(path_file, "w") as f:
            json.dump([{"x": x, "y": y} for x, y in self.path_points], f, indent=2)
        print(f"[App] ✅ Đã lưu đường đi: {path_file}")


# === MAIN ===
if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = SimpleApp(root)
        root.mainloop()
    except KeyboardInterrupt:
        print("[App] ⛔ Đóng ứng dụng.")
    finally:
        print("[App] 🧹 Cleanup GPIO...")
        try:
            for btn in LIMIT_SWITCHES.values():
                btn.close()
            for enc in encoders.values():
                enc['A'].close()
                enc['B'].close()
        except Exception as e:
            print(f"[App] ⚠️ Lỗi khi dọn GPIO: {e}")
        os._exit(0)
