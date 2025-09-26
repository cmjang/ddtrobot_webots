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
    battery_device_node = return_launch_file('battery_device','launch','battery_device_node.launch.py')
    power_controller_node = return_launch_file('power_controller','launch','power_controller_node.launch.py')
    temperature_controller_node = return_launch_file('temperature_controller','launch','temperature_controller_node.launch.py')
    rgb_light_controller_node = return_launch_file('rgb_light_controller','launch','rgb_light_controller_node.launch.py')
    audio_controller_node = return_launch_file('audio_controller','launch','audio_controller_node.launch.py')
    system_status_node = return_launch_file('system_status','launch','system_status.launch.py')

    ld = LaunchDescription()
    ld.add_action(battery_device_node)
    ld.add_action(power_controller_node)
    ld.add_action(audio_controller_node)
    ld.add_action(temperature_controller_node)
    ld.add_action(rgb_light_controller_node)
    ld.add_action(system_status_node)

    return ld
