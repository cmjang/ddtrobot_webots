<p align="center"><strong>motor_device</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

wbc控制轮式运动的controller interface，



## Basic Information

| Installation method | Supported platform[s]    |
| ------------------- | ------------------------ |
| Source              | Jetpack 6.0 , ros-humble |

------

## Subscribed

| ROS Topic |       Interface        | Frame ID | Description |
| :-------: | :--------------------: | :------: | :---------: |
| `topic1`  | std_msgs::msg::Float64 |  frameA  |  接收数据A  |
| `topic2`  | std_msgs::msg::Float64 |  frameB  |  接收数据B  |

## Published

| ROS Topic |       Interface        | Frame ID |    Description    |
| :-------: | :--------------------: | :------: | :---------------: |
|    sum    | std_msgs::msg::Float64 |  frameC  | 求和结果 1 Hz发布 |

## Service

| Service Topic |     Call Interface     |    Return Interface    |       Description        |
| :-----------: | :--------------------: | :--------------------: | :----------------------: |
| service name  | std_msgs::msg::Float64 | std_msgs::msg::Float64 |    一个service的案例     |
|               | std_msgs::msg::Float64 |                        | Service 的第二个输入描述 |



## Build Package

```bash
# if have extra dependencies
# apt install ros-humble-pluginlib ros-humble-ros2-control ros-humble-ros2-controllers ros-$ROS_DISTRO-pinocchio
colcon build --packages-up-to tita_controller
# if in debug 
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --parallel-workers=10
# 如果使用webots仿真
apt-get install ros-humble-webots-ros2
## 验证webots_ros2安装ok
ros2 launch webots_ros2_universal_robot multirobot_launch.py
## clone仓库
cd src/tita_locomotion
git clone git@git.ddt.dev:rbt/alg/tita_webots_ros2.git
colcon build --packages-up-to webots_bridge
source install/setup.bash
ros2 launch webots_bridge webot_bridge.launch.py
```

## debug
apt install ros-humble-rqt-robot-steering
apt install ros-humble-rqt-controller-manager

配置vscode中 `launch.json`，attach程序选择

```json
{
  "version": "0.2.0",
  "configurations": [
      {
          "name": "Attach to tita_controller (Local)",
          "type": "cppdbg",
          "request": "attach",
          "program": "/opt/ros/humble/lib/controller_manager/ros2_control_node",  // controller_manager的可执行文件路径
          "processId": "${command:pickProcess}",
          "MIMode": "gdb",
          "miDebuggerPath": "/usr/bin/gdb",
          "setupCommands": [
              {
                  "description": "Enable pretty-printing for gdb",
                  "text": "-enable-pretty-printing",
                  "ignoreFailures": true
              }
          ],
      },
      {
        "name": "Attach to tita_controller (Remote)",
        "type": "cppdbg",
        "request": "attach",
        "program": "/opt/ros/humble/lib/controller_manager/ros2_control_node",  // controller_manager的可执行文件路径
        "processId": "${command:pickRemoteProcess}",
        "MIMode": "gdb",
        "miDebuggerPath": "/usr/bin/gdb",
        "useExtendedRemote": true,
        "setupCommands": [
            {
                "description": "Enable pretty-printing for gdb",
                "text": "-enable-pretty-printing",
                "ignoreFailures": true
            }
        ],
        "logging": { "engineLogging": true },
          "targetArchitecture": "ARM",
          "pipeTransport": {
              "pipeCwd": "",
              "pipeProgram": "ssh",
              "pipeArgs": ["lkxbot"],
              "debuggerPath": "/usr/bin/gdb",
              "pipeEnv": {}
          },
          "sourceFileMap": {
              "/usr": "${workspaceFolder}/../../install"
          }
        
    }
  ]
}

```

# test
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON