import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LiDARDirectionTester(Node):
    def __init__(self):
        super().__init__('lidar_direction_test')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # In giá trị tại các góc: 0°, 90°, 180°, 270°
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        def get_distance_at_deg(deg):
            # Chuyển độ sang radian, tính index
            rad = math.radians(deg)
            index = int((rad - angle_min) / angle_increment)
            if 0 <= index < len(ranges):
                return ranges[index]
            return None

        d_front = get_distance_at_deg(0)      # phía trước
        d_left = get_distance_at_deg(90)      # bên trái
        d_back = get_distance_at_deg(180)     # phía sau
        d_right = get_distance_at_deg(270)    # bên phải

        print(f"⬆️ Trước (0°):  {d_front:.2f} m")
        print(f"⬅️ Trái (90°):   {d_left:.2f} m")
        print(f"⬇️ Sau (180°):   {d_back:.2f} m")
        print(f"➡️ Phải (270°):  {d_right:.2f} m")
        print("-" * 40)

def main(args=None):
    rclpy.init(args=args)
    node = LiDARDirectionTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
