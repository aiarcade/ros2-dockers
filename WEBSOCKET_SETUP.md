# WebSocket Setup for Webots-ROS2 Communication

## Architecture
- **Webots (macOS)**: Runs controller with WebSocket client
- **Docker**: Runs rosbridge_server on port 9090
- **ROS2 nodes (Docker)**: obstacle_avoider subscribes/publishes via ROS2

## Setup Steps

### 1. Rebuild Docker Image
```bash
docker compose build dev
docker compose up -d dev
```

### 2. Build ROS2 Package
```bash
docker compose exec dev bash -c "cd /root && colcon build --packages-select bot"
```

### 3. Start ROS2 with Rosbridge
```bash
docker compose exec dev bash -c "source /opt/ros/humble/setup.bash && source /root/install/local_setup.bash && ros2 launch bot simulate.launch.py"
```

This starts:
- rosbridge_server on ws://localhost:9090
- obstacle_avoider node

### 4. Start Webots
```bash
./webot_start.sh
```
Then open `ws/src/bot/worlds/my_world.wbt` and click Play ▶️

## How It Works

1. **Webots controller** connects to ws://localhost:9090
2. **Subscribes** to `/cmd_vel` topic
3. **Publishes** `/left_sensor` and `/right_sensor` topics
4. **obstacle_avoider** in Docker reads sensors and publishes cmd_vel
5. **Webots** receives cmd_vel and moves robot motors

## Testing

**Check rosbridge is running:**
```bash
docker compose exec dev bash -c "ros2 node list | grep rosbridge"
```

**Monitor topics:**
```bash
docker compose exec dev bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
```

**Echo cmd_vel:**
```bash
docker compose exec dev bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /cmd_vel"
```

## Ports
- 9090: WebSocket (rosbridge)
- 7400-7700: DDS/ROS2 discovery
