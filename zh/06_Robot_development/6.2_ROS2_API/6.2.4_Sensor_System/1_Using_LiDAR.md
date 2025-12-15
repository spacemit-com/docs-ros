sidebar_position: 1

# 激光雷达使用

## 简介

本章节介绍在 ROS 2 Humble 下集成主流 2D 激光雷达（Lidar）设备的使用方法，包括 YDLidar 与 RPLidar 两类雷达的 SDK 编译、驱动功能包构建及后续使用说明。

## YDLidar 驱动配置

### 编译 YDLidar SDK

YDLidar ROS 2 驱动依赖其官方提供的 **YDLidar-SDK**，需先手动编译安装：

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
cmake --build . -- -j8
sudo cmake --install .
```

### 构建 ROS2 功能包

```bash
mkdir -p ~/demo_ws/src && cd ~/demo_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git --branch=humble --depth 1
cd ~/demo_ws
source /opt/ros/humble/setup.bash
colcon build
```

## RPLidar 驱动配置（rdlidar）

### 编译 ROS2 功能包

```bash
mkdir -p ~/demo_ws/src && cd ~/demo_ws/src
git clone https://github.com/allenh1/rplidar_ros.git --branch=ros2 --depth 1
cd ~/demo_ws
source /opt/ros/humble/setup.bash
colcon build
```