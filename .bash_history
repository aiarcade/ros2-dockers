cd ws0/src/
ls
source /opt/ros/humble/setup.bash 
source  install/setup.bash 
ros2 run hello_world listener
exit
ls
cd ws0
ls
source  /opt/ros/humble/setup.bash 
ls
cd src/
ros2 pkg create hello_world    --build-type ament_python   --dependencies rclpy std_msgs geometry_msgs
colcon build --symlink-install
ros2 run hello_world minimal_publisher
ros2 run hello_world talker
colcon build --symlink-install
ros2 run hello_world talker
ls
source  install/setup.bash 
ros2 run hello_world talker
ros2 run hello_world talker
colcon build --symlink-install
source  install/setup.bash 
ros2 run hello_world talker
colcon build --symlink-install
ros2 launch 
ros2 launch  hello_world simulate.launch.py
colcon build --symlink-install
ros2 launch  hello_world simulate.launch.py
colcon build --symlink-install
ros2 launch  hello_world simulate.launch.py
colcon build --symlink-install
ros2 launch  hello_world simulate.launch.py
rm -rf build/ install/
ros2 launch  hello_world simulate.launch.py
colcon build --symlink-install
source  install/setup.bash 
colcon build --symlink-install
source  install/setup.bash 
ros2 launch  hello_world simulate.launch.py
colcon build --symlink-install
ros2 launch  hello_world simulate.launch.py
ros2 pkg create --build-type ament_python py_pubsub
exit
