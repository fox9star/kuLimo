import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class Service_server(Node):
    def __init__(self):
        super().__init__("service_server")  # 노드 이름
        self.create_service(SetBool, "setBool", self.setBool_callback)
        self.bool = False
        self.cnt = 0

    def setBool_callback(self, request: SetBool.Request, response: SetBool.Response):
        # 서버의 작동 알고리즘...
        self.get_logger().info(f"{self.cnt}번째 요청 처리")
        self.get_logger().info(f"현재 bool 정보 {self.bool}")
        self.get_logger().info(f"변경 요청 값 {request.data}")
        if request.data != self.bool:
            self.bool = not self.bool  # 실제 데이터 변경
            response.success = True
            response.message = f"{self.cnt}번째 요청 {self.bool} 변경 성공"
        else:
            response.success = False
            response.message = f"{self.cnt}번째 요청 {self.bool} 변경 실패"
        time.sleep(5)
        self.cnt += 1
        return response


def main():
    rclpy.init()
    node = Service_server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()