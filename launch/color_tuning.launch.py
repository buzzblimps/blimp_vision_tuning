import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace='MaryO',
            parameters=[os.path.join(get_package_share_directory('blimp_vision'), 'param', 'elp_config.yaml')],
            output='screen'
        ),
        Node(
            package='blimp_vision_tuning',
            executable='blimp_vision_tuning_node',
            name='blimp_vision_tuning_node',
            namespace='MaryO',
            output='screen'
        )
    ])
