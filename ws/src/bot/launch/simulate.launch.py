from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen'
    )
    
    obstacle_avoider = Node(
        package='bot',
        executable='obstacle_avoider',
    )

    return LaunchDescription([rosbridge, obstacle_avoider])