sidebar_position: 1

# Lidar Usage

## Introduction

This chapter describes how to integrate and use mainstream 2D ​​Lidar devices within ROS 2 Humble. It covers the compilation of SDKs for two specific types of Lidars—YDLidar and RPLidar—as well as the construction of their driver packages and subsequent usage instructions.

## YDLidar Driver Configuration

### Compiling the YDLidar SDK

The YDLidar ROS 2 driver relies on the official **YDLidar-SDK** provided by the manufacturer; therefore, you must first manually compile and install it:

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
cmake --build . -- -j8
sudo cmake --install .
```

### Building the ROS 2 Package

```bash
mkdir -p ~/demo_ws/src && cd ~/demo_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git --branch=humble --depth 1
cd ~/demo_ws
source /opt/ros/humble/setup.bash
colcon build
```

## RPLidar Driver Configuration (rdlidar)

### Building the ROS 2 Package

```bash
mkdir -p ~/demo_ws/src && cd ~/demo_ws/src
git clone https://github.com/allenh1/rplidar_ros.git --branch=ros2 --depth 1
cd ~/demo_ws
source /opt/ros/humble/setup.bash
colcon build
```