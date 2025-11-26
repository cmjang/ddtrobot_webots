<p align="center"><strong>joy_controller</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

    暂时用于wbc调试的joy启动

## Build Package

```bash
colcon build --packages-up-to joy_controller
source install/setup.bash
ros2 launch joy_controller joy_controller.launch.py 
```
