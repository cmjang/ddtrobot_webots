<p align="center"><strong>tita_command</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

​	机器人控制指令管理器。其中分为用户输入指令和传感器被动控制指令。用户输入指令包含了遥控器指令和用户的 `SDK` 输入指令。被动指令是结合了机器人的各种传感器信息，用于限制当前用户指令的控制状态，作为指令的限制器而存在。 `command manager` 节点将主动指令与被动指令整合发送给机器人作为控制机器人的唯一指令。

## Basic Information

| Installation method | Supported platform[s]      |
| ------------------- | -------------------------- |
| Source              | Jetpack 6.0 , ros-humble |

------

## Build Package

```bash
# if have extra dependencies
apt install ros-humble-joy
colcon build --packages-select teleop_command passive_command comman_manager active_command
```
