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

import os
import sys
launch_dir = os.path.dirname(__file__)
sys.path.append(launch_dir)

from launch import LaunchDescription
from launch_utils import *

def generate_launch_description():

    hardware_bridge_launch = return_launch_file('hardware_bridge','launch','hardware_bridge.launch.py')
    
    command_launch = return_launch_file('tita_bringup','launch','command_launch.py')

    perception_launch = return_launch_file('tita_bringup','launch','perception_launch.py')

    system_launch = return_launch_file('tita_bringup','launch','system_launch.py')

    interaction_launch = return_launch_file('tita_bringup','launch','interaction_launch.py')

    location_launch = return_launch_file('tita_bringup','launch','location_launch.py')

    return LaunchDescription([
        hardware_bridge_launch,
        system_launch,
        interaction_launch,
        perception_launch,
        location_launch,
        command_launch
    ])
