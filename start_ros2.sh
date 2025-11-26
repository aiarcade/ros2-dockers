#!/bin/bash

echo "=== Starting ROS2 with Rosbridge ==="

docker compose exec dev bash -c "
cd /root && \
source /opt/ros/humble/setup.bash && \
source install/local_setup.bash && \
ros2 launch bot simulate.launch.py
"
