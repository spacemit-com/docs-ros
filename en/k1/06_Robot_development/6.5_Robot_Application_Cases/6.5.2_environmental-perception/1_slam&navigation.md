sidebar_position: 1

# AGV Mapping and Navigation

Watch [SpacemiT Official Video](https://m.bilibili.com/video/BV15M6UYPEPe?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&p=1&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=C1945F6E-C110-40A1-AFD3-0E442722A9AD&share_source=WEIXIN&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1735914458&unique_k=Wg9Qmqd&up_id=3537125114906665)

## Overview

This demo showcases how the SpacemiT K1 chip drives a small car to perform **mapping and navigation**, covering core capabilities such as **SLAM (Simultaneous Localization and Mapping), path planning, and obstacle avoidance**.

## Supported Platforms

- **Hardware**: Any development board based on the SpacemiT K1 chip (e.g., Muse Book, Muse Pi, Muse Pi Pro, BPi, etc.).
- **Operating System**: Recommended version is Bianbu OS 2.0 or higher.
- **ROS System**: ROS 2 Humble.

## System Modules

The system relies on the following modules working together to achieve mapping and navigation:

- **LiDAR Module**。
  - Scans the surroundings and provides real-time laser data for the mapping and navigation modules, enabling accurate localization and map construction.

- **Chassis Module (Wheeltec ROS Education Car)**
  - Publishes real-time status updates of the robot’s chassis for use by the mapping and navigation modules.
  - Receives movement commands from the control module and converts them into motor control signals to follow the planned path.

- **Mapping Module**
  - Combines LiDAR data and chassis information to build and update the robot’s environment map in real time.
  - Continuously improves the map for a more accurate environmental model.

- **Navigation Module**
  - Uses the built map, along with LiDAR and chassis data, to calculate the optimal path from the current location to the target.
  - Ensures the robot navigates smoothly to the desired destination.

- **Obstacle Detection Module**
  - Uses LiDAR to monitor the surroundings and detect potential obstacles in real time.
  - Provides feedback to enable obstacle avoidance, ensuring safe operation in complex environments.

- **Control Module（Muse Pi Development Board）**
  - Receives path information from the navigation module and controls the robot’s precise movements.
  - Converts high-level commands into control signals to drive the chassis motors.

## Environment Setup

Refer to **[Section 6.1.2](../../6.1_OS_Preparation/6.1.2_ROS2_Installation.md)** to set up the ROS 2 environment on the Muse Pi development board.

## Compiling Drivers and ROS Packages

### Step 1: Configure the CH343 (CP9102) Driver

Sensors such as LiDAR can be connected to the **Muse Pi** board via a USB-to-serial adapter, enabling data exchange between the sensor and the board.。

- **Download the Source Code**:

  ```Shell
  git clone https://github.com/WCHSoftGroup/ch343ser_linux.git
  ```

- **Compile and Install**:

  ```Shell
  cd ch343ser_linux/driver
  sudo make install
  ```

### Step 2: Compile the LiDAR Driver

- **Download the Driver Source**:
  [YDLidar-SDK.tar.gz](https://archive.spacemit.com/ros2/code/YDLidar-SDK.tar.gz)

- **Extract the Files**:

  ```bash
  mkdir -p ~/ros2_humble_space
  tar -zxvf YDLidar-SDK.tar.gz -C ~/ros2_humble_space
  ```

- **Compile and Install**:

  ```
  cd ~/ros2_humble_space/YDLidar-SDK
  mkdir build && cd  build
  cmake ..
  cmake --build .
  cmake --install .
  ```

### Step 3: Compile Mapping and Navigation Packages

- **Download ROS Package Source**:

  [k1_origin_src.tar.gz](https://archive.spacemit.com/ros2/code/k1_origin_src.tar.gz)。

- **Extract the Source Code into workspace**:

  ```bash
  mkdir -p ~/ros2_humble_space/k1_origin_ws
  tar -zxvf k1_origin_src.tar.gz -C ~/ros2_humble_space/k1_origin_ws
  ```

#### Compile the `serial_ros2` Package

`serial_ros2` is a ROS 2 driver package for serial communication, enabling data exchange with external hardware connected via UART (sensors, actuators, control boards, etc.).

**Compile and Install**:

```bash
cd ~/ros2_humble_space/k1_origin_ws/src/originbot_driver/serial_ros2
make && sudo make install
make clean
```

#### Compile the `qpOASES` Package

The `qpOASES` package is a library for solving quadratic programming (QP) problems.

**Compile and Install**:

```bash
cd ~/ros2_humble_space/k1_origin_ws/src/originbot_driver/qpOASES
mkdir build && cd build
cmake ..
make && sudo make install
cd .. && rm -r build/
```

### Step 4: Compile All Packages

- Activate the ROS Environment:

  ```bash
  source ~/ros2_humble_space/ros2_humble/setup.zsh
  source ~/ros2_humble_space/ros2_humble_extra/local_setup.zsh
  ```

- Execute the `colcon` command to compile all packages in the workspace:

  ```bash
  cd ~/ros2_humble_space/k1_origin_ws/
  colcon build
  ```

## Compiling Source Code

### On K1 Board

- **Install ROS Dependencies**:

  ```bash
  sudo apt install -y ros-dev-tools ros-humble-ros-base \
  python3-numpy 'ros-humble-cartographer*' 'ros-humble-nav*' libpcap0.8-dev libuvc-dev \
  ros-humble-filters ros-humble-turtlesim ros-humble-camera-info-manager ros-humble-pcl-ros \
  ros-humble-image-common ros-humble-image-geometry ros-humble-robot-localization \
  ros-humble-joint-state-publisher
  ```

- **Download Source Code Package**:

  [src_k1.tar.gz](https://archive.spacemit.com/ros2/code/src_k1.tar.gz)

- **Extract and Compile**:

  ```bash
  mkdir -p ~/wheeltec_ws
  tar -zxvf src_k1.tar.gz -C ~/wheeltec_ws
  cd ~/wheeltec_ws/src
  colcon build
  ```

- **Configure serial port rules**
   Update udev rules according to your device IDs:

  ```bash
  udevadm info --query=all --name=/dev/ttyCH343USB1
  echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0002", MODE:="0777", SYMLINK+="wheeltec_controller"' > /etc/udev/rules.d/wheeltec_controller2.rules

  # CH9102 Serial-Port Setup (ensure the CH9102 driver is installed), device index `0001` is given the alias wheeltec_lidar
  echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0001", MODE:="0777", SYMLINK+="wheeltec_lidar"' > /etc/udev/rules.d/wheeltec_lidar2.rules

  sudo udevadm control --reload-rules
  sudo udevadm trigger
  ```

### On PC

- **Install ROS dependencies**

  ```bash
  sudo apt install -y ros-dev-tools ros-humble-ros-base \
  python3-numpy 'ros-humble-cartographer*' 'ros-humble-nav*' libpcap0.8-dev libuvc-dev \
  ros-humble-filters ros-humble-turtlesim ros-humble-camera-info-manager ros-humble-pcl-ros \
  ros-humble-image-common ros-humble-image-geometry ros-humble-robot-localization \
  ros-humble-joint-state-publisher
  ```

- **Download Source Code Package**:

  [src_pc.tar.gz](https://archive.spacemit.com/ros2/code/src_pc.tar.gz)

- **Extract and Compile**:

  ```bash
  mkdir -p ~/wheeltec_ws
  tar -zxvf src_k1.tar.gz -C ~/wheeltec_ws
  cd ~/wheeltec_ws/src
  colcon build
  ```

## Launch Commands

### Simultaneous Mapping & Navigation

- **PC Side: Launch Visualization**:

  ```bash
  ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
  ```

- **K1 Side: Start Mapping and Navigation**:

  ```bash
  ros2 launch wheeltec_nav2 wheeltec_nav2_for_slam.launch.py
  ```

### Run Mapping and Navigation Separately

- **PC Side: Launch Visualization**:

  ```bash
  ros2 launch originbot_viz display_slam.launch.py
  ```

#### Mapping

- On the Muse Pi board, run one of the following commands:
  - **Using slam_gmapping**:

    ```bash
    ros2 launch slam_gmapping slam_gmapping.launch.py
    ```

  - **Using cartographer**:

    ```bash
    ros2 launch wheeltec_cartographer cartographer.launch.py
    ```

- **Save the Map**:

  ```bash
  ros2 run nav2_map_server map_saver_cli -f spacemit1
  ```

#### Navigation

- First, disable the mapping function.

- **PC Side: Launch Visualization**:

  ```bash
  ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
  ```

- **Muse Pi**: After replacing the map, rebuild with `colcon build`, then start navigation:

  ```bash
  ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
  ```
