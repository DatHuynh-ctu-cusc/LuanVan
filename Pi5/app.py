import tkinter as tk
import threading
from encoder_handler import positions
from lidar_map_drawer import draw_lidar_on_canvas

class SimpleApp:
    def __init__(self, root):
        self.root = root
        self.root.title("App don gian co sidebar")
        self.root.geometry("1000x600")

        self.running = threading.Event()
        self.running.set()

        self.sidebar = tk.Frame(root, width=250, bg="#2c3e50")
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)

        self.buttons = []
        self.active_button = None

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

        self.show_home()

    def add_sidebar_button(self, label, command):
        btn = tk.Button(self.sidebar, text=label, bg="#34495e", fg="white", font=("Arial", 12), height=2, anchor="w", padx=20, relief="flat")
        btn.pack(fill="x", pady=2)
        btn.config(command=lambda b=btn, c=command: self.on_sidebar_click(b, c))
        self.buttons.append(btn)

    def on_sidebar_click(self, button, command):
        if self.active_button:
            self.active_button.config(bg="#34495e")
        button.config(bg="#1abc9c")
        self.active_button = button
        command()

    def show_home(self):
        self.update_content("\U0001F3E1 Day la Trang chu")

    def show_map(self):
        for widget in self.content.winfo_children():
            widget.destroy()
        self.lidar_canvas = tk.Canvas(self.content, bg="white")
        self.lidar_canvas.pack(fill="both", expand=True)

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

    def update_lidar_map(self, data):
        draw_lidar_on_canvas(self.lidar_canvas, data)
