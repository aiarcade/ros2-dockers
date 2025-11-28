#!/usr/bin/env python3
"""Webots humanoid controller using WebSocket to communicate with ROS2"""

from controller import Robot
import json
import time

try:
    import websocket
except ImportError:
    print("Installing websocket-client...")
    import subprocess
    subprocess.check_call(['pip3', 'install', 'websocket-client'])
    import websocket

TIME_STEP = 8

robot = Robot()

# List all available devices first
num_devices = robot.getNumberOfDevices()
print(f"Total devices: {num_devices}")
print("Available devices:")
for i in range(num_devices):
    device = robot.getDeviceByIndex(i)
    if device:
        print(f"  Device #{i}: {device.getName()}")

# Get motors using the correct Atlas motor naming (PascalCase)
motors = {}
atlas_motor_names = [
    'LArmShz', 'LArmShx', 'LArmEly', 'LArmElx', 'LArmWry', 'LArmWrx', 'LArmWry2',
    'RArmShz', 'RArmShx', 'RArmEly', 'RArmElx', 'RArmWry', 'RArmWrx', 'RArmWry2',
    'LLegHpz', 'LLegHpx', 'LLegHpy', 'LLegKny', 'LLegAky', 'LLegAkx',
    'RLegHpz', 'RLegHpx', 'RLegHpy', 'RLegKny', 'RLegAky', 'RLegAkx',
    'BackBkz', 'BackBky', 'BackBkx',
    'NeckRy'
]

for name in atlas_motor_names:
    motor = robot.getDevice(name)
    if motor:
        motors[name] = motor
        print(f"Found motor: {name}")

print(f"Total motors found: {len(motors)}")

target_arms_up = 0.0

def on_message(ws, message):
    global target_arms_up
    try:
        data = json.loads(message)
        if data.get('topic') == '/cmd_vel':
            msg = data.get('msg', {})
            # Use linear.x to control arms up position (1.0 = arms up, 0.0 = arms down)
            target_arms_up = msg.get('linear', {}).get('x', 0.0)
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
print("Humanoid controller started with WebSocket")

while robot.step(TIME_STEP) != -1:
    # Move arms up when target_arms_up > 0.5
    if target_arms_up > 0.5:
        # Raise arms - shoulder and elbow motors
        if 'LArmShx' in motors:
            motors['LArmShx'].setPosition(-1.396)  # Left arm out
        if 'RArmShx' in motors:
            motors['RArmShx'].setPosition(-0.77)  # Right arm out
        if 'LArmEly' in motors:
            motors['LArmEly'].setPosition(-1.5)  # Left elbow up
        if 'RArmEly' in motors:
            motors['RArmEly'].setPosition(-1.5)  # Right elbow up
    else:
        # Arms down - neutral position
        if 'LArmShx' in motors:
            motors['LArmShx'].setPosition(0.0)
        if 'RArmShx' in motors:
            motors['RArmShx'].setPosition(0.0)
        if 'LArmEly' in motors:
            motors['LArmEly'].setPosition(0.0)
        if 'RArmEly' in motors:
            motors['RArmEly'].setPosition(0.0)
