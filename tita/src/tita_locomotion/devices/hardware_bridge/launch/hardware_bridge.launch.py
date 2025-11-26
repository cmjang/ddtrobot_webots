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
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


import sys
sys.path.insert(0, os.path.join(get_package_share_directory('tita_bringup'), 'launch'))
from launch_utils import tita_namespace

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tower",
            default_value="false",
            description="Decide use tita tower or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="mcu",
            choices=["wbc", "sdk", "mcu"],
            description="Enable sdk of joint effort input",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf",
            default_value="robot.xacro",
            description="Select urdf file in locomotion_bringup/description/urdf",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yaml",
            default_value="tita_controllers_ros2_control.yaml",
            description="Select yaml file in locomotion_bringup/config",
        )
    )

    use_tower = LaunchConfiguration("use_tower")
    prefix=tita_namespace

    # Initialize Arguments
    # save urdf
    robot_description_content_dir = PathJoinSubstitution(
        [FindPackageShare("tita_description"), "tita" , "xacro",  LaunchConfiguration('urdf')]
    )
    xacro_executable = FindExecutable(name="xacro")

    robot_description_content = Command(
        [
            PathJoinSubstitution([xacro_executable]),
            " ",
            robot_description_content_dir,
            " ",
            "use_tower:=",
            use_tower,
            " ",
            "ctrl_mode:=",
            LaunchConfiguration("ctrl_mode"),
            " ",
            "sim_env:=",
            "none",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    # robot_controllers = os.path.join(
    #     get_package_share_directory("locomotion_bringup"),
    #     "config",
    #     "tita_controllers_ros2_control.yaml",
    # )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("locomotion_bringup"), "config", LaunchConfiguration('yaml')]
    )
    # TODO: if launch in here
    command_node = Node(
        package="commands_can",
        executable="commands_can_node",
        namespace=prefix,
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ctrl_mode'), "' == 'mcu'"]))
    )

    motors_status_node = Node(
        package="motors_can",
        executable="motors_can_node",
        namespace=prefix,
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            robot_controllers,
        ],
        namespace=prefix, 
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        # output="both",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"frame_prefix": prefix+"/"},
        ],
        namespace=prefix, 
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )
    
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )

    nodes = [
        command_node,
        # motors_status_node,
        control_node,
        robot_state_pub_node,
        # joint_state_broadcaster_spawner,
        # imu_sensor_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
