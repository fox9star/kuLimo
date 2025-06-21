# /turtle1/cmd_vel [geometry_msgs/msg/Twist]
import rclpy
from rclpy.node import Node
from user_interface.msg import MyTopic


class Simple_pub(Node):
    def __init__(self):
        super().__init__("myTopicPub")  # node name
        self.create_timer(0.1, self.pub_turtle)
        self.pub = self.create_publisher(MyTopic, "/message", 10)
        self.count = 0

    def pub_turtle(self):
        msg = MyTopic()
        msg.a = 10
        msg.b = 20
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"내가 만든 토픽 입니다.{self.count}"
        self.pub.publish(msg)
        self.count += 1


def main():
    rclpy.init()
    node = Simple_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()