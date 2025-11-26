#!/bin/bash

# Get the primary container IP dynamically
CONTAINER_IP=$(hostname -I | awk '{print $1}')

echo "Container IP detected: $CONTAINER_IP"

# Export for ROS 2 / DDS auto networking
export ROS_IP=$CONTAINER_IP
export ROS_HOSTNAME=$CONTAINER_IP

# Allow other commands (bash, ros2, etc.)
exec "$@"
