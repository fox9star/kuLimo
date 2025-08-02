import asyncio
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import uvicorn
import threading

app = FastAPI()
rospy.init_node('web_interface_node', anonymous=True)
pub = rospy.Publisher('message', String, queue_size=10)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
clients = set()
loop = None  # FastAPI 이벤트 루프 저장 변수

html = """
<!DOCTYPE html>
<html>
<head>
    <title>ROS Web Interface</title>
</head>
<body>
    <h1>ROS Message Interface</h1>
    <div id="messages"></div>
    <input type="text" id="inputMessage" placeholder="Enter message">
    <button onclick="sendMessage()">Send</button>
    <button onclick="sendGoCommand()">Go</button>
    <button onclick="sendForwardCommand()">Forward</button>
    <script>
        const ws = new WebSocket("ws://192.168.2.24:8000/ws");
        ws.onmessage = function(event) {
            const messages = document.getElementById("messages");
            const message = document.createElement("p");
            message.textContent = event.data;
            messages.appendChild(message);
        };
        function sendMessage() {
            const input = document.getElementById("inputMessage");
            ws.send(input.value);
            input.value = "";
        }
        function sendGoCommand() {
            ws.send("go");
        }
        function sendForwardCommand() {
            ws.send("forward");
        }
    </script>
</body>
</html>
"""

@app.get("/")
async def get():
    return HTMLResponse(html)

def move_limo():
    twist = Twist()
    twist.linear.x = 0.2
    twist.angular.z = 1.5
    rate = rospy.Rate(10)
    count = 0

    while count < 100 and not rospy.is_shutdown():  # 약 10초간 회전
        cmd_vel_pub.publish(twist)
        rate.sleep()
        count += 1

    cmd_vel_pub.publish(Twist())  # 정지

def move_forward():
    twist = Twist()
    twist.linear.x = 0.2
    twist.angular.z = 0.0
    rate = rospy.Rate(10)
    count = 0

    while count < 70 and not rospy.is_shutdown():  # 약 7초간 전진
        cmd_vel_pub.publish(twist)
        rate.sleep()
        count += 1

    cmd_vel_pub.publish(Twist())  # 정지

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            message = data.strip().lower()

            if message == "go":
                await websocket.send_text("LIMO is moving in a circle for ~10 seconds 🌀")
                threading.Thread(target=move_limo, daemon=True).start()
            elif message == "forward":
                await websocket.send_text("LIMO is moving straight for 7 seconds ➡️")
                threading.Thread(target=move_forward, daemon=True).start()
            else:
                pub.publish(message)
                await websocket.send_text(f'Message "{message}" published to ROS ✅')
    except Exception:
        clients.remove(websocket)

def message_callback(data):
    message = data.data
    if loop is not None:
        asyncio.run_coroutine_threadsafe(broadcast(message), loop)

async def broadcast(message):
    for client in clients.copy():
        try:
            await client.send_text(message)
        except:
            clients.remove(client)

def start_ros_subscriber():
    rospy.Subscriber("message", String, message_callback)
    rospy.spin()

@app.on_event("startup")
async def startup_event():
    global loop
    loop = asyncio.get_running_loop()

if __name__ == "__main__":
    threading.Thread(target=start_ros_subscriber, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8000)
