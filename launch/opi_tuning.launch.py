import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blimp_vision_tuning',
            executable='blimp_vision_tuning_node',
            name='blimp_vision_tuning_node',
            namespace='/',
            output='screen'
        )
    ])
