#!/usr/bin/env python3
"""Webots hexapod controller using WebSocket to communicate with ROS2"""

from controller import Robot
import json
import time
import math

try:
    import websocket
except ImportError:
    print("Installing websocket-client...")
    import subprocess
    subprocess.check_call(['pip3', 'install', 'websocket-client'])
    import websocket

TIME_STEP = 8

robot = Robot()

# Get all motors
hip_motors = []
knee_motors = []
for i in range(6):
    side = 'l' if i < 3 else 'r'
    idx = i if i < 3 else i - 3
    hip_motors.append(robot.getDevice(f'hip_motor_{side}{idx}'))
    knee_motors.append(robot.getDevice(f'knee_motor_{side}{idx}'))

target_linear = 0.0
target_angular = 0.0
t = 0

def on_message(ws, message):
    global target_linear, target_angular
    try:
        data = json.loads(message)
        if data.get('topic') == '/cmd_vel':
            msg = data.get('msg', {})
            target_linear = msg.get('linear', {}).get('x', 0.0)
            target_angular = msg.get('angular', {}).get('z', 0.0)
    except:
        pass

def on_error(ws, error):
    print(f"WebSocket error: {error}")

def on_close(ws, close_status_code, close_msg):
    print("WebSocket closed")

def on_open(ws):
    print("WebSocket connected to ROS2 bridge")
    subscribe_msg = json.dumps({
        "op": "subscribe",
        "topic": "/cmd_vel",
        "type": "geometry_msgs/Twist"
    })
    ws.send(subscribe_msg)

print("Connecting to ROS2 bridge at ws://localhost:9090...")
ws = websocket.WebSocketApp("ws://localhost:9090",
                            on_open=on_open,
                            on_message=on_message,
                            on_error=on_error,
                            on_close=on_close)

import threading
ws_thread = threading.Thread(target=ws.run_forever)
ws_thread.daemon = True
ws_thread.start()

time.sleep(1)
print("Hexapod controller started with WebSocket")

while robot.step(TIME_STEP) != -1:
    t += TIME_STEP / 1000.0
    
    # Simple tripod gait
    phase = math.sin(t * target_linear * 2)
    
    for i in range(6):
        # Alternate tripod pattern: 0,2,4 vs 1,3,5
        offset = 0 if i % 2 == 0 else math.pi
        
        # Hip swing for forward movement
        hip_angle = target_angular * 0.3 + target_linear * 0.3 * math.sin(t * 3 + offset)
        hip_motors[i].setPosition(hip_angle)
        
        # Knee lift for walking
        knee_pos = -0.01 * abs(target_linear) * (1 + math.sin(t * 3 + offset))
        knee_motors[i].setPosition(knee_pos)
