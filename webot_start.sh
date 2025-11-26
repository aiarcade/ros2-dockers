#!/bin/bash

echo "=== Starting Webots ==="
echo "Webots will connect to ROS2 via WebSocket (ws://localhost:9090)"
echo ""

if [[ "$(uname -s)" == "Darwin" ]]; then
    open -a Webots
else
    webots &
fi

echo "Webots started. Open world: ws/src/bot/worlds/my_world.wbt"

