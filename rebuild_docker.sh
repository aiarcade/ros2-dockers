#!/bin/bash

echo "=== Rebuilding Docker with Rosbridge ==="
docker compose build dev
docker compose up -d dev
echo "=== Building bot package ==="
docker compose exec dev bash -c "cd /root && colcon build --packages-select bot"
echo "=== Done ==="
echo "Now run: ./start_ros2.sh"
