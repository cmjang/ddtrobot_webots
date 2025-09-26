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

from launch.actions import IncludeLaunchDescription , RegisterEventHandler,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart

sys.path.insert(0, os.path.join(get_package_share_directory('tita_bringup'), 'launch'))
from launch_utils import *

def generate_launch_description():

    # hc_command_node = return_launch_file('hc_command','launch','hc_command.launch.py')

    # path_follower_node = return_launch_file('path_follower','launch','path_follower.launch.py')

    # path_controller_node = return_launch_file('path_controller','launch','path_controller.launch.py')

    # passive_cmd_node = TimerAction(
    #     period=1.0,
    #     actions=[return_launch_file('passive_command', 'launch', 'passive_command.launch.py')]
    # )
    # mqtt_bridge_node = return_launch_file('mqtt_ros_bridge','launch','mqtt_bridge.launch.py')
    
    # scale_bridge_node = return_launch_file('scale_msgs_bridge','launch','scale_bridge.launch.py')

    audio_interaction_node = return_launch_file('audio_interaction','launch','audio_interaction.launch.py')
    
    ld = LaunchDescription()
    # # Add independent nodes to LaunchDescription
    # ld.add_action(hc_command_node)
    # ld.add_action(mqtt_bridge_node)
    # ld.add_action(path_controller_node)
    # ld.add_action(scale_bridge_node)
    ld.add_action(audio_interaction_node)
    
    return ld