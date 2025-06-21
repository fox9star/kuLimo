# sudo apt install ros-humble-tf-transformations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy import time
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from tf2_ros import TransformException


class FrameListener(Node):
    def __init__(self):
        super().__init__("tf2_listener")  # node name
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform("joint", "world", time.Time())
        except Exception:
            self.get_logger().info("lookup 실패!!")
            return
        self.get_logger().info(f"{t.transform.translation.x}")
        self.get_logger().info(f"{t.transform.translation.y}")
        self.get_logger().info(f"{t.transform.translation.z}")


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()