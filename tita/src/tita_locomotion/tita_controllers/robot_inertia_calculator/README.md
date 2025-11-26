<p align="center"><strong>robot_inertia_calculator</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

计算额外增加机械臂的质量参数以及整体质心的功能包

## Basic Information

| Installation method | Supported platform[s]    |
| ------------------- | ------------------------ |
| Source              | Jetpack 6.0 , ros-humble |

------

## Subscribed

|          ROS Topic           |          Interface           | Frame ID |      Description       |
| :--------------------------: | :--------------------------: | :------: | :--------------------: |
| `imu_sensor_broadcaster/imu` |    sensor_msgs::msg::Imu     |          |     tita的imu信息      |
|        `joint_states`        | sensor_msgs::msg::JointState |          | tita的joint_states信息 |

## Published

|                 ROS Topic                 |            Interface            | Frame ID |                           Description                            |
| :---------------------------------------: | :-----------------------------: | :------: | :--------------------------------------------------------------: |
|    `robot_inertia_calculator/inertia`     | locomotion_msgs::msg::RigidBody |          |                 额外增加机械臂计算的惯量参数信息                 |
| `robot_inertia_calculator/whole_body_com` |   geometry_msgs::msg::Vector3   |          | 整体机器人的质心位置，在控制坐标系下（没有yaw旋转以及xyz的平移） |

## Build Package

```bash
# if have extra dependencies
# apt install ros-$ROS_DISTRO-pinocchio
colcon build --packages-up-to robot_inertia_calculator
source install/setup.bash
ros2 launch robot_inertia_calculator robot_inertia_calculator.launch.py

ros2 service call /${tita_ns}/robot_inertia_calculator/enable_calculate std_srvs/srv/SetBool "{data: true}" # true or false, true mean enable caculator of extra tilt

```
