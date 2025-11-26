# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription, LaunchContext
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

import platform
import sys
sys.path.insert(0, os.path.join(get_package_share_directory('tita_bringup'), 'launch'))
from launch_utils import tita_namespace

def generate_launch_description():
    # Declare arguments
    system_arch = platform.machine()
    print(system_arch)
    joy_controller_node = Node(
        package="joy_controller",
        executable="joy_controller_node",
        namespace=tita_namespace,
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        namespace="",
        condition=IfCondition(PythonExpression(["'", system_arch, "' == 'x86_64' "]))
    )
    joy_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('teleop_command'),
            'launch',
            'teleop_command.launch.py'
        )),
        condition=IfCondition(
            PythonExpression(["'", system_arch, "' == 'aarch64' "]))
    )    

    nodes = [
        joy_controller_node,
        joy_node,
        joy_launch,
    ]

    return LaunchDescription(nodes)
