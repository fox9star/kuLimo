# service client 를 만드세요.
# service 이름 타입(turtlesim_node color 변경) 실시간으로 다양한 색상을 표현한다.

# move_turtle.py -> 파라미터를 설정 각속도 움직임을 외부에서 변화 시킨다.
# 위 파라미터도 yaml 에 넣어서 작동 시킨다.


import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ColorClient(Node):
    def __init__(self):
        super().__init__('color_client')
        self.client = self.create_client(Empty, 'change_color')

    def send_request(self):
        req = Empty.Request()
        self.client.call_async(req)
        self.get_logger().info('Requested color change')

def main():
    rclpy.init()
    node = ColorClient()
    node.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()