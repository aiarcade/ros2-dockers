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
cd ws/src
colcon build --symlink-install
rm -rf build/ install/
colcon build --symlink-install
source  /opt/ros/rolling/setup.bash 
source  install/local_setup.
source  install/local_setup.bash 
ros2 launch  simplebot simulate.launch.py
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
source  install/local_setup.bash 
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
source  install/local_setup.bash 
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
colcon build --symlink-install
source  install/local_setup.bash 
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
source  install/local_setup.bash 
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
source  install/local_setup.bash 
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
source  install/local_setup.bash 
ros2 launch  simplebot simulate.launch.py
colcon build --symlink-install
source  install/local_setup.bash 
colcon build --symlink-install
source  install/local_setup.bash 
ros2 launch  simplebot simulate.launch.py
exit
exit
