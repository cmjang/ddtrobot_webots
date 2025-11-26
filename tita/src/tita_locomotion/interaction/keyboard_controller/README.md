<p align="center"><strong>keyboard_controller</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>
    
    keyboard_controller 是一个键盘控制节点，用于接收键盘输入并发布相应的指令。

## Build Package

```bash
colcon build --packages-up-to keyboard_controller
ros2 run oros_keyboard_controller keyboard_controller_node --ros-args -r __ns:=/tita # according to your tita namespace
```
