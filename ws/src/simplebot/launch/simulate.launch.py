from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen'
    )
    
    simple_drive = Node(
        package='simplebot',
        executable='drive',
    )

    return LaunchDescription([rosbridge, simple_drive])