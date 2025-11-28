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

# Get distance sensors
sonar_front = robot.getDevice('sonar_front')
sonar_left = robot.getDevice('sonar_left')
sonar_right = robot.getDevice('sonar_right')

# Enable sensors
sonar_front.enable(TIME_STEP)
sonar_left.enable(TIME_STEP)
sonar_right.enable(TIME_STEP)

print("Distance sensors enabled")

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

# Publish sensor data
def publish_sensor_data():
    front_val = sonar_front.getValue()
    left_val = sonar_left.getValue()
    right_val = sonar_right.getValue()
    
    sensor_msg = json.dumps({
        "op": "publish",
        "topic": "/hexapod/sensors",
        "msg": {
            "data": f"Front:{front_val:.2f} Left:{left_val:.2f} Right:{right_val:.2f}"
        }
    })
    try:
        ws.send(sensor_msg)
    except:
        pass

sensor_counter = 0

while robot.step(TIME_STEP) != -1:
    t += TIME_STEP / 1000.0
    
    # Publish sensor data every 50 steps (approx 0.4 seconds)
    sensor_counter += 1
    if sensor_counter >= 50:
        publish_sensor_data()
        sensor_counter = 0
    
    if target_linear > 0:
        # Tripod gait: legs 0,2,4 move together, legs 1,3,5 move together
        gait_frequency = 1.5  # Hz
        phase = (t * gait_frequency) % 1.0
        
        for i in range(6):
            # Group A (0,2,4) and Group B (1,3,5)
            leg_phase = phase if i % 2 == 0 else (phase + 0.5) % 1.0
            
            # Determine left or right side (affects direction)
            is_left = i < 3
            direction = 1 if is_left else -1
            
            # Hip: rotate to push robot forward
            # Left legs push backward (positive angle), right legs push backward (negative angle)
            if leg_phase < 0.5:
                # Swing phase (leg lifted and moving forward)
                hip_angle = direction * (-0.4 + (leg_phase * 2) * 0.8)  # -0.4 to +0.4
            else:
                # Stance phase (leg on ground pushing backward)
                hip_angle = direction * (0.4 - ((leg_phase - 0.5) * 2) * 0.8)  # +0.4 to -0.4
            
            hip_motors[i].setPosition(hip_angle)
            
            # Knee: lift during swing phase, lower during stance phase
            if leg_phase < 0.3:
                # Lift leg at start of swing
                knee_angle = -0.015
            elif leg_phase < 0.5:
                # Lower leg at end of swing
                knee_angle = -0.008
            else:
                # Leg on ground during stance
                knee_angle = 0.0
            
            knee_motors[i].setPosition(knee_angle)
    else:
        # Stop - all legs at neutral position
        for i in range(6):
            hip_motors[i].setPosition(0.0)
            knee_motors[i].setPosition(0.0)
