import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sdk_command'),
        'config',
        'param.yaml'
    )

    return LaunchDescription([
        Node(
            package='sdk_command',
            executable='sdk_command_node',
            name='sdk_command_node',
            output='screen',
            parameters=[config]
        )
    ])
