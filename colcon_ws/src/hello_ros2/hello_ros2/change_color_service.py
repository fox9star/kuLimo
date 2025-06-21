import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty  # 기본 Empty 서비스 사용

class ColorService(Node):
    def __init__(self):
        super().__init__('color_service')
        self.service = self.create_service(Empty, 'change_color', self.change_color_callback)
        self.get_logger().info('Color Service Ready')

    def change_color_callback(self, request, response):
        # 색상 변경 로직 (예제에서는 단순 로그 출력)
        self.get_logger().info('Changing color...')
        return response

def main():
    rclpy.init()
    node = ColorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()