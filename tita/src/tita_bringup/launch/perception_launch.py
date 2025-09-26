#!/usr/bin/python3
#
# Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

from launch_ros.actions import Node
import os
import sys

from ament_index_python.packages import get_package_share_directory
sys.path.insert(0, os.path.join(get_package_share_directory('tita_bringup'), 'launch'))
from launch_utils import *



def generate_launch_description():
    # Define the nodes to be launched
    dual_tof_device_node = return_launch_file('dual_tof_device','launch','dual_tof_device.launch.py')

    obstacle_detector_node = return_launch_file('obstacle_detector','launch','obstacle_detector.launch.py')

    stereo_camera_node = return_launch_file('argus_camera_device','launch','stereo_cam.launch.py')
    
    # angle_detector_node = Node(
    #         package='angle_detector',
    #         executable='angle_detector_node',
    #         name='angle_detector_node',
    #         namespace=tita_namespace,
    #         output='screen',
    #         parameters=[get_config_param("angle_detector","param.yaml")]
    #     )
    
    # obstacle_detector_node = Node(
    #         package='obstacle_detector',
    #         executable='obstacle_detector_node',
    #         name='obstacle_detector_node',
    #         namespace=tita_namespace,
    #         output='screen',
    #         parameters=[get_config_param("obstacle_detector","param.yaml")]
    #     )
    
    # point_generator_node = Node(
    #         package='point_generator',
    #         executable='point_generator_node',
    #         name='point_generator_node',
    #         namespace=tita_namespace,
    #         output='screen',
    #         parameters=[get_config_param("point_generator","param.yaml")]
    #     )
    

    ld = LaunchDescription()
    ld.add_action(dual_tof_device_node)
    ld.add_action(obstacle_detector_node)
    ld.add_action(stereo_camera_node)
    # ld.add_action(ultrawave_device_node)
    # # Add dependent nodes using wait_and_launch
    # ld.add_action(RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=dual_tof_device_node,
    #         on_start=[angle_detector_node]
    #     )
    # ))

    # ld.add_action(RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=dual_tof_device_node,
    #         on_start=[obstacle_detector_node]
    #     )
    # ))

    # ld.add_action(RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=stereo_camera_device_node,
    #         on_start=[point_generator_node]
    #     )
    # ))

    return ld
