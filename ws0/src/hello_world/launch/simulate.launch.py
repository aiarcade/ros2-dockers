import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    package_dir = get_package_share_directory('hello_world')


    talker_node = Node(
        package='hello_world',
        executable='talker',
    )

    return LaunchDescription([
        talker_node ,
        
    ])