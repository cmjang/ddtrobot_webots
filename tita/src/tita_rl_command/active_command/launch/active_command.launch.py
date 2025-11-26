import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import sys
sys.path.insert(0, os.path.join(get_package_share_directory('tita_bringup'), 'launch'))
from launch_utils import tita_namespace

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('active_command'),
        'config',
        'param.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='active_command',
            executable='active_command_node',
            name='active_command_node',
            namespace=tita_namespace,
            output='screen',
            parameters=[config]
        )
    ])
