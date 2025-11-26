<p align="center"><strong>devices</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>
​
主要为与硬件层通信相关package, 包括ros2_control的hardware_interface, 以及其他硬件接口。

- **hardware_bridge**: tita使用的ros2_control的hardware_interface，can通信．
- **airbot_hardware_bridge**: airbot使用的ros2_control的hardware_interface．
- **command_can**：用于ros2的遥控指令通过can发送到运控板（暂），
- **motors_can**：用来读取下层can的电机以及imu信息发送到上层，
- **protocol**：cansocket，
- **udp**：udp通信，给airbot使用(暂)．

## Basic Information

| Installation method | Supported platform[s]    |
| ------------------- | ------------------------ |
| Source              | Jetpack 6.0 , ros-humble |

------
