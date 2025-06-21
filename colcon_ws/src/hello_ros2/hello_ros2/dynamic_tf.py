# sudo apt install ros-humble-tf-transformations

import rclpy
import rclpy.logging
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class DynamicFramePublisher(Node):
    def __init__(self):
        super().__init__("dynamic_tf2_broadcaster")  # node name
        self.create_timer(1 / 30, self.pub_cb)
        self.tf_br = TransformBroadcaster(self)
        self.t = 0.0

    def pub_cb(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "map"
        t.transform.translation.x = 1.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, self.t)  # r y p
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = "map"
        t2.child_frame_id = "joint"
        t2.transform.translation.x = 3.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        self.t += 1 / 60
        print(self.t)

        self.tf_br.sendTransform(t)
        self.tf_br.sendTransform(t2)


def main():
    rclpy.init()
    node = DynamicFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()