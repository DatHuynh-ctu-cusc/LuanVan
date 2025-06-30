
# main.py
import subprocess
import threading
import rclpy
from autonomous_node import LiDARNode
from receiver import start_receiver
from motor_control import stop_all


def start_lidar_ros():
    return subprocess.Popen(['ros2', 'launch', 'sllidar_ros2', 'sllidar_c1_launch.py'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    counts = {"E1": 0, "E2": 0, "E3": 0, "E4": 0}
    start_receiver(counts)
    lidar_proc = start_lidar_ros()
    rclpy.init()
    node = LiDARNode(counts)
    # Khởi động nhận bản đồ mini từ Pi5


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[Pi4] ⛔ Dừng hệ thống")
    finally:
        rclpy.shutdown()
        stop_all()
        lidar_proc.terminate()

if __name__ == '__main__':
    main()
