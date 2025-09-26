# DDTRobot Webots Simulation

[![Linux platform](https://img.shields.io/badge/platform-Linux-green.svg)](#)[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg)](https://opensource.org/license/apache-2-0)


## Overview

This project is based on the open-source **Webots** simulation environment developed by **[Direct Drive Technology Robotics](https://github.com/DDTRobot)** , and it supports one-click deployment of robot training weight files from **[DDTRobotLab](https://github.com/cmjang/ddtrobot_lab)**.For more information about **tita** **sim2sim**, and **sim2real**, please refer to [tita_rl_sim2sim2real](https://github.com/DDTRobot/tita_rl_sim2sim2real) 

<div align="center">

| <div align="center"> Isaac Lab </div> | <div align="center">  Webots </div> |
|--- | --- |
| [<img src="./img/isaaclab.gif" width="240px">](isaaclab.gif) | [<img src="./img/webots.gif" width="240px">](webots.gif) |
</div>

## Installation

### Dowload the Docker

```bash
docker pull registry.cn-guangzhou.aliyuncs.com/ddt_robot/ubuntu:webot2023b-v1
```

### **Clone the repository**

```bash
git clone https://github.com/cmjang/ddtrobot_webots.git
```

### Install library

1. **start the docker**

   ```bash
   sudo docker run -v path/above/your/project:/mnt/dev -w /mnt/dev --rm  --gpus all --net=host --privileged -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1  -e CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda -it registry.cn-guangzhou.aliyuncs.com/ddt_robot/ubuntu:webot2023b-v1
   
   # For example: Dowloads is the directory above the project ddtrobot_webots
   sudo docker run -v ~/Downloads:/mnt/dev -w /mnt/dev --rm  --gpus all --net=host --privileged -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1  -e CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda -it registry.cn-guangzhou.aliyuncs.com/ddt_robot/ubuntu:webot2023b-v1
   ```

2. **Try starting the webots，and shall see the webots GUI:**

   ```bash
   webots
   ```

3. **Check ros2** 

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

4. **Install webots_ros2**

   ```bash
   cd webots_ros2
   colcon build
   source install/setup.bash
   ```

5. **Install pinocchio** 

   ```bash
   apt update
   apt install ros-humble-pinocchio
   #Then delete the build file and build again
   ```

6. **Enable xhost**

   Add `xhost +` to your `~/.bashrc` (for your **local machine, not inside Docker**):

   ```bash
   nano ~/.bashrc
   ```

   Add `xhost +` at the end of the file.

   Reload your bash configuration：

   ```bash
   source ~/.bashrc
   ```

### Change the path of policy

By default, if your Docker is loading the project path, then you don’t need to change it.Please adjust it according to your actual setup. The file is located at:
 `tita/src/tita_locomotion/tita_controllers/tita_controller/src/fsm/FSMState_RL.cpp`, line 17.

```c++
policy_stand=std::make_shared<CudaTest>("/mnt/dev/stand.engine");
//change the "/mnt/dev/stand.engine" to your own path
```

### Run sim

```bash
cd tita
colcon build
```

1. **Copy robot mesh**

   ```bash
   sudo mkdir -p /usr/share/robot_description
   sudo cp -r src/tita_locomotion/tita_description/tita /usr/share/robot_description/
   ```

2. **Run webots simulatior**

   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch locomotion_bringup sim_bringup.launch.py
   ```

3. **Run command terminal**

   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run keyboard_controller keyboard_controller_node --ros-args -r __ns:=/tita
   ```

## Add your policy

The ONNX weight file obtained from  **[DDTRobotLab](https://github.com/cmjang/ddtrobot_lab)** is converted using **TensorRT**.

```bash
/usr/src/tensorrt/bin/trtexec --onnx=policy.onnx --saveEngine=stand.engine
```

### Acknowlegments

This project is built with reference to related open-source projects. Special thanks to:

- [webots](https://github.com/cyberbotics/webots) :open-source robot simulator
- [tita_rl_sim2sim2real](https://github.com/DDTRobot/tita_rl_sim2sim2real) :a framework for simulation verification and physical deployment 

