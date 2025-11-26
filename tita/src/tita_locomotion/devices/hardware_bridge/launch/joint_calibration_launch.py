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

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess


def exec_cmd(command):
    ret = subprocess.run(command,
                         shell=True,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE,
                         encoding='utf-8',
                         timeout=1)
    if ret.returncode == 0:
        print('success:', ret)
    else:
        print('error:', ret)
    return ret.returncode, ret.stdout

def GetNamespace():
    ret, ns = exec_cmd('cat /sys/firmware/devicetree/base/serial-number')
    ns = ns.replace('\x00', '')
    if ret == 0:
        ns = ns.strip()
    else:
        ns = ''
    ns = ns[0:-1]
    n = len(ns)
    if n > 0:
        n = min(n, 7)
        ns = ns[-n:]
    print('serial number:%s' % (ns))
    ns_ = 'tita' + ns
    return ns_

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Initialize Arguments

    # Get URDF via xacro
    control_dir = get_package_share_directory('joint_calibration')
    description_dir = get_package_share_directory('tita_description')
    tita_description_path = os.path.join(description_dir, "urdf", "tita_hw_ros2control.urdf")
    with open(tita_description_path) as f:
        robot_description = f.read()

    robot_controllers = os.path.join(control_dir, "config", "joint_calibration_config.yaml")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{
            'robot_description_path': tita_description_path,
            },
            robot_controllers
        ],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        # namespace=GetNamespace()  
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description}],
        # namespace=GetNamespace(),
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    joint_calibration_spawner = Node(
            package='controller_manager',
            executable='spawner',
            # output='screen',
            arguments=["joint_calibration", "--controller-manager", "/controller_manager"], #TODO: change: cheater_tita_control
        )   
    
    start_joint_calibration = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_calibration_spawner],
        )
    )
    
    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        start_joint_calibration
    ]

    return LaunchDescription(declared_arguments + nodes)
