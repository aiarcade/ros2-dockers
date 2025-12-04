import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod',
            executable='rollover_drive',
            output='screen'
        ),
    ])
