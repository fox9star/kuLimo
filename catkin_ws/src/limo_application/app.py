# Backend: main.py

import asyncio
import rospy
import math
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import uvicorn

app = FastAPI()
rospy.init_node('web_interface_node', anonymous=True)

# Publishers
pub = rospy.Publisher('message', String, queue_size=10)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# State for sensors
total = 0.0
current_distance = float('inf')  # in meters
current_yaw = 0.0                 # in radians

# Control
stop_event = threading.Event()

# WebSocket clients
clients = set()
loop = None

# Movement logic
STOP_DISTANCE = 0.20  # 20 cm
RATE_HZ = 10
YAW_TOL = 0.1        # radians tolerance for alignment
ROT_SPEED = 0.5      # angular speed for rotation (rad/s)

# Helper to normalize angle to [-pi, pi]
def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# Move: rotate to heading then go straight until obstacle or emergency
def move_with_heading(target_yaw):
    stop_event.clear()
    rate = rospy.Rate(RATE_HZ)
    # Rotation phase
    while not rospy.is_shutdown() and not stop_event.is_set():
        err = normalize_angle(target_yaw - current_yaw)
        if abs(err) < YAW_TOL:
            break
        twist = Twist()
        twist.angular.z = ROT_SPEED * (1 if err > 0 else -1)
        cmd_vel_pub.publish(twist)
        rate.sleep()
    # Stop rotation
    cmd_vel_pub.publish(Twist())
    rospy.sleep(0.1)
    # Forward phase
    twist = Twist(linear=Twist().linear, angular=Twist().angular)
    twist.linear.x = 0.2
    twist.angular.z = 0.0
    while not rospy.is_shutdown() and current_distance > STOP_DISTANCE and not stop_event.is_set():
        cmd_vel_pub.publish(twist)
        rate.sleep()
    # Stop movement
    cmd_vel_pub.publish(Twist())

# Button handlers
def handle_table1():
    move_with_heading(0.0)  # 0° front

def handle_table2():
    move_with_heading(math.pi)  # 180° back

def handle_kitchen():
    move_with_heading(math.pi/2*3)  # 90° right

def handle_emergency():
    # Signal all movements to stop
    stop_event.set()
    cmd_vel_pub.publish(Twist())

# ROS Callbacks
def scan_callback(msg: LaserScan):
    global current_distance
    total_dist = 0.0
    count = 0
    min_angle = 0.0
    max_angle = math.radians(10)
    for i, r in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        if min_angle <= angle <= max_angle:
            if not math.isinf(r) and not math.isnan(r):
                total_dist += r
                count += 1
    current_distance = total_dist / count if count > 0 else float('inf')

def imu_callback(msg: Imu):
    global current_yaw
    import tf.transformations as tft
    quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    _, _, yaw = tft.euler_from_quaternion(quat)
    current_yaw = yaw

# Message broadcast to WebSocket
def message_callback(data):
    message = data.data
    if loop:
        asyncio.run_coroutine_threadsafe(broadcast_message(message), loop)

# Async broadcast helpers
async def broadcast_message(message: str):
    for ws in set(clients):
        try:
            await ws.send_text(message)
        except:
            clients.remove(ws)

async def broadcast_status():
    # Periodically send sensor status
    while True:
        await asyncio.sleep(0.1)
        status = f"{current_yaw:.2f},{current_distance:.2f}"
        for ws in set(clients):
            try:
                await ws.send_text('status:' + status)
            except:
                clients.remove(ws)

# Single-file HTML with log cap and status display
html = '''<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <title>ROS Control Panel</title>
  <style>
    body { text-align: center; font-family: sans-serif; }
    h1 { margin-top: 20px; }
    .btn { padding: 20px; font-size: 1.5rem; margin: 10px; width: 200px; border-radius: 10px; }
    .stop { background-color: #f55; }
    #messages { margin-top: 20px; max-height: 200px; overflow-y: auto; border: 1px solid #ccc; padding: 10px; width: 50%; margin: 0 auto; }
    #messages p { margin: 5px 0; }
    #status { margin-top: 20px; font-size: 1.2rem; }
  </style>
</head>
<body>
  <h1>ROS Control Panel</h1>
  <div>
    <button class="btn" onclick="send('table1')">테이블1</button>
    <button class="btn" onclick="send('table2')">테이블2</button>
    <button class="btn" onclick="send('kitchen')">주방</button>
    <button class="btn stop" onclick="send('긴급정지')">긴급정지</button>
  </div>
  <div id="messages"></div>
  <div id="status">Yaw: -- , Distance: --</div>
  <script>
    const ws = new WebSocket(`ws://${location.host}/ws`);
    const msgDiv = document.getElementById('messages');
    const statusDiv = document.getElementById('status');
    ws.onmessage = event => {
      const data = event.data;
      if (data.startsWith('status:')) {
        const parts = data.split(':')[1].split(',');
        const yaw = parts[0];
        const dist = parts[1];
        statusDiv.textContent = `Yaw: ${yaw} rad, Distance: ${dist} m`;
      } else {
        const p = document.createElement('p');
        p.textContent = data;
        msgDiv.appendChild(p);
        while (msgDiv.children.length > 5) {
          msgDiv.removeChild(msgDiv.firstChild);
        }
      }
    };
    function send(cmd) {
      ws.send(cmd);
    }
  </script>
</body>
</html>'''

@app.get('/')
async def index():
    return HTMLResponse(html)

@app.websocket('/ws')
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            cmd = (await ws.receive_text()).strip().lower()
            if cmd == 'table1':
                threading.Thread(target=handle_table1, daemon=True).start()
                await ws.send_text('Table 1: moving at 0° until obstacle')
            elif cmd == 'table2':
                threading.Thread(target=handle_table2, daemon=True).start()
                await ws.send_text('Table 2: moving at 180° until obstacle')
            elif cmd == 'kitchen':
                threading.Thread(target=handle_kitchen, daemon=True).start()
                await ws.send_text('Kitchen: moving at 90° until obstacle')
            elif cmd == '긴급정지':
                handle_emergency()
                await ws.send_text('Emergency stop!')
            else:
                pub.publish(cmd)
                await ws.send_text(f'Published: {cmd}')
    except:
        clients.remove(ws)

@app.on_event('startup')
async def startup_event():
    global loop
    loop = asyncio.get_running_loop()
    # start periodic status updates
    asyncio.create_task(broadcast_status())

if __name__ == '__main__':
    threading.Thread(target=lambda: (
        rospy.Subscriber('scan', LaserScan, scan_callback),
        rospy.Subscriber('imu', Imu, imu_callback),
        rospy.Subscriber('message', String, message_callback),
        rospy.spin()
    ), daemon=True).start()
    uvicorn.run(app, host='0.0.0.0', port=8000)