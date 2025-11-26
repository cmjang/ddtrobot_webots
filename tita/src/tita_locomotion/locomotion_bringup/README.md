<p align="center"><strong>locomotion_bringup</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

用于拉起locomotion模块，主要通过`tita_controllers.launch.py`来拉启所需功能, 可以输入的参数如下：
```bash
robot@tita:~/lkx/tita_ros2$ ros2 launch locomotion_bringup tita_controllers.launch.py --show-args
Arguments (pass arguments as '<name>:=<value>'):

    'sim_env':
        Select simulation environment. Valid choices are: ['none', 'webots', 'gazebo']
        (default: 'none')

    'ctrl_mode':
        Select wheel-legged robot control methods, mcu means on mcu-board control. Valid choices are: ['wbc', 'sdk', 'mcu']
        (default: 'mcu')

    'urdf':
        Select urdf file in locomotion_bringup/description/urdf
        (default: 'robot.xacro')

    'yaml':
        Select yaml file in locomotion_bringup/config
        (default: 'tita_controllers_ros2_control.yaml')

    'use_tower':
        Decide use tita tower or not
        (default: 'false')

```


## Basic Information

| Installation method | Supported platform[s]    |
| ------------------- | ------------------------ |
| Source              | Jetpack 6.0 , ros-humble |

------

## Subscribed

|          ROS Topic          |           Interface           | Frame ID |    Description     |
| :-------------------------: | :---------------------------: | :------: | :----------------: |
|  `command/manager/cmd_key`  |      std_msgs/msg/String      |    \\    | 指令机器人切换状态 |
| `command/manager/cmd_pose`  | geometry_msgs/msg/PoseStamped |    \\    | 指令机器人头部位姿 |
| `command/manager/cmd_twist` |    geometry_msgs/msg/Twist    |    \\    |   指令机器人速度   |

## Published

|          ROS Topic           |             Interface              | Frame ID |        Description         |
| :--------------------------: | :--------------------------------: | :------: | :------------------------: |
|    `robot_description  `     |        std_msgs/msg/String         |    \\    |        urdf描述信息        |
|    `   joint_states     `    |     sensor_msgs/msg/JointState     |    \\    |       机器人关节信息       |
| `imu_sensor_broadcaster/imu` |        sensor_msgs/msg/Imu         |    \\    |          imu信息           |
|   `dynamic_joint_states  `   | control_msgs/msg/DynamicJointState |    \\    | ros2_control发出的关节信息 |



## Build Package

```bash
# if have extra dependencies
# apt install ros-humble-pluginlib ros-humble-ros2-control ros-humble-ros2-controllers
colcon build --packages-up-to locomotion_bringup
source install/setup.bash
# add environment variable to use sim in pc environment, cause in board don't have siumulation installed
export SIM_COMPILE=true

ros2 launch locomotion_bringup tita_controllers.launch.py [sim_env:=gazebo, webots, none] [extra_inertia:=false]

```

