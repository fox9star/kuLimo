#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# 이동 목표 위치에 따라 속도 설정
def handle_command(msg):
    cmd = msg.data.lower()
    twist = Twist()

    if cmd == "table1":
        rospy.loginfo("➡️ 1번 테이블로 이동 명령 수신")
        twist.linear.x = 0.3  # 앞으로 이동
        twist.angular.z = 0.0
    elif cmd == "table2":
        rospy.loginfo("⬅️ 2번 테이블로 이동 명령 수신")
        twist.linear.x = -0.3  # 뒤로 이동
        twist.angular.z = 0.0
    elif cmd == "stop":
        rospy.loginfo("⛔ 정지 명령 수신")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    else:
        rospy.logwarn(f"알 수 없는 명령어 수신: {cmd}")
        return
t 엔드포인트
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            pub.publish(data)  # ROS 토픽으로 메시지 발행
    except Exception as e:
    cmd_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('limo_command_executor')
    rospy.Subscriber('/limo/move_command', String, handle_command)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("📡 Limo 제어 노드 시작됨 (명령 수신 대기 중...)")
    rospy.spin()