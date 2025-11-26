#!/usr/bin/env python3
"""Webots controller using WebSocket to communicate with ROS2"""

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

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025
TIME_STEP = 64

robot = Robot()

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0)

ds0 = robot.getDevice('ds0')
ds1 = robot.getDevice('ds1')
ds0.enable(TIME_STEP)
ds1.enable(TIME_STEP)

target_linear = 0.0
target_angular = 0.0

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
    
    advertise_left = json.dumps({
        "op": "advertise",
        "topic": "/left_sensor",
        "type": "sensor_msgs/Range"
    })
    ws.send(advertise_left)
    
    advertise_right = json.dumps({
        "op": "advertise",
        "topic": "/right_sensor",
        "type": "sensor_msgs/Range"
    })
    ws.send(advertise_right)

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
print("Robot controller started with WebSocket")

while robot.step(TIME_STEP) != -1:
    left_vel = (target_linear - target_angular * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
    right_vel = (target_linear + target_angular * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
    
    left_motor.setVelocity(left_vel)
    right_motor.setVelocity(right_vel)
    
    # Convert sensor values: lookupTable gives 1020 at close range, 0 at far
    # Invert this: high value = close obstacle, low value = far/no obstacle
    left_sensor_value = ds0.getValue()
    right_sensor_value = ds1.getValue()
    
    # Convert to distance: 1020 -> 0.05m (close), 0 -> 0.15m (far/clear)
    # Formula: distance = 0.15 - (sensor_value / 1020.0) * 0.10
    if left_sensor_value > 0:
        left_distance = max(0.0, 0.15 - (left_sensor_value / 1020.0) * 0.10)
    else:
        left_distance = 0.15  # No obstacle detected
    
    if right_sensor_value > 0:
        right_distance = max(0.0, 0.15 - (right_sensor_value / 1020.0) * 0.10)
    else:
        right_distance = 0.15  # No obstacle detected
    
    left_sensor_msg = json.dumps({
        "op": "publish",
        "topic": "/left_sensor",
        "msg": {
            "range": left_distance,
            "min_range": 0.0,
            "max_range": 0.15
        }
    })
    
    right_sensor_msg = json.dumps({
        "op": "publish",
        "topic": "/right_sensor",
        "msg": {
            "range": right_distance,
            "min_range": 0.0,
            "max_range": 0.15
        }
    })
    
    try:
        ws.send(left_sensor_msg)
        ws.send(right_sensor_msg)
    except:
        pass
