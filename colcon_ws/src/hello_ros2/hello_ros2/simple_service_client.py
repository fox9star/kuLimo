# ros2 run hello_ros2 simple_service_client
# ros2 run hello_ros2 simple_service_server2
# ros2 run hello_ros2 simple_service_server
# 화면의 결과를 비교해 보세요.

from asyncio import Future

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class Service_client(Node):
    def __init__(self):
        super().__init__("service_client")  # 노드 이름
        self.client = self.create_client(SetBool, "setBool")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")
        self.create_timer(1, self.update)
        self.create_timer(2, self.send_request)
        self.bool = False
        self.cnt = 0
        self.request = SetBool.Request()
        self.future = Future()

    def update(self):
        self.get_logger().info("main Thread is running!!")

    def send_request(self):
        self.get_logger().info(f"{self.cnt} 번째 요청")
        self.request.data = not self.request.data
        self.future = self.client.call_async(self.request)  # ros1 대기
        self.future.add_done_callback(self.done_callback)
        self.cnt += 1

    def done_callback(self, future):
        response: SetBool.Response = future.result()
        self.get_logger().info(f"처리 상태: {response.success}")
        self.get_logger().info(f"서버에서 온 메세지: {response.message}")


def main():
    rclpy.init()
    node = Service_client()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()

    