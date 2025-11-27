# 小车建图导航

点击观看 [进迭时空官方视频](https://m.bilibili.com/video/BV15M6UYPEPe?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&p=1&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=C1945F6E-C110-40A1-AFD3-0E442722A9AD&share_source=WEIXIN&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1735914458&unique_k=Wg9Qmqd&up_id=3537125114906665)

## 案例简介

该 Demo 展示了进迭时空 K1 芯片驱动小车进行建图和导航的应用案例，涵盖了建图、导航和避障等核心功能。

## 支持平台

- 平台：基于进迭时空 K1 芯片的开发板均可（Muse Book、Muse Pi、Muse Pi Pro、BPi等）。
- 操作系统：推荐使用 Bianbu OS 2.0 以上版本。
- ROS 系统：ROS2 Humble。

## 模块清单

为了实现上述功能，系统需要以下各模块的协同工作：

- **激光雷达模块**
  该模块负责扫描并获取周围环境的激光数据，为建图模块和导航模块提供实时的环境信息，帮助系统进行准确的定位和地图构建。

- **底盘模块（轮趣科技 ROS 教育小车）**
  底盘模块发布机器人底盘的实时状态信息，供建图和导航模块使用。同时，接收来自控制模块的运动指令，将其转化为各电机的控制命令，确保机器人按预定轨迹移动。

- **建图模块**
  建图模块结合激光雷达数据和底盘信息，实时构建并更新机器人所处环境的地图。通过持续积累和优化数据，帮助系统获得更加精确的环境模型。

- **导航模块**
  导航模块基于已构建的地图以及激光雷达和底盘数据，计算出从当前位置到目标位置的最优路径。它负责路径规划，确保机器人能够顺利导航到指定位置。

- **障碍检测模块**
  该模块利用激光雷达实时监测周围环境，检测并识别潜在的障碍物。通过障碍物信息的反馈，帮助系统实现避障功能，保证机器人在复杂环境中的安全运行。

- **控制模块（Muse Pi 开发板）**
  控制模块接收导航模块规划的路径信息，并根据这些指令控制机器人进行精准的运动。它负责将高层指令转化为实际的控制信号，调度底盘模块驱动各个电机实现机器人的运动。

## 环境搭建

参考 [6.1.2节](../../6.1_OS_Preparation/6.1.2_ROS2_Installation.md) 在 Muse Pi 开发板上搭建 ROS2 环境。

## 编译驱动和 ROS 包

### 配置 CH343(CP9102) 驱动

雷达等传感器可通过 USB 转串口设备连接到 Muse Pi 开发板，实现传感器与开发板之间的数据收发。

- 下载源码

```Shell
git clone https://github.com/WCHSoftGroup/ch343ser_linux.git
```

- 编译并安装

```Shell
cd ch343ser_linux/driver
sudo make install
```

### 编译雷达驱动

- 下载驱动源码：[YDLidar-SDK.tar.gz](https://archive.spacemit.com/ros2/code/YDLidar-SDK.tar.gz)

- 解压

```
mkdir -p ~/ros2_humble_space
tar -zxvf YDLidar-SDK.tar.gz -C ~/ros2_humble_space
```

- 编译

```
cd ~/ros2_humble_space/YDLidar-SDK
mkdir build && cd  build
cmake ..
cmake --build .
cmake --install .
```

### 编译建图导航相关包

- **下载 ROS 包源码**

首先，下载相关的源码包 [k1_origin_src.tar.gz](https://archive.spacemit.com/ros2/code/k1_origin_src.tar.gz)。

- **解压源码**

解压下载的源码包，并将其存放在指定的工作空间中：

```bash
mkdir -p ~/ros2_humble_space/k1_origin_ws
tar -zxvf k1_origin_src.tar.gz -C ~/ros2_humble_space/k1_origin_ws
```

#### 编译 `serial_ros2` 包

`serial_ros2` 包是一个用于串口通信的 ROS2 驱动包，主要用于与通过串口连接的外部硬件设备（如传感器、执行器、控制板等）进行数据交换。

进入 `serial_ros2` 目录，执行编译命令：

```bash
cd ~/ros2_humble_space/k1_origin_ws/src/originbot_driver/serial_ros2
make && sudo make install
make clean
```

#### 编译 `qpOASES` 包

`qpOASES` 包是一个用于求解二次规划问题（Quadratic Programming, QP）的库。

首先创建 `build` 目录，然后执行编译：

```bash
cd ~/ros2_humble_space/k1_origin_ws/src/originbot_driver/qpOASES
mkdir build && cd build
cmake ..
make && sudo make install
cd .. && rm -r build/
```

### 编译所有包

最后，编译整个工作空间的所有包。首先激活 ROS 环境：

```bash
source ~/ros2_humble_space/ros2_humble/setup.zsh
source ~/ros2_humble_space/ros2_humble_extra/local_setup.zsh
```

然后执行 `colcon` 命令，编译工作空间中的所有包：

```bash
cd ~/ros2_humble_space/k1_origin_ws/
colcon build
```

## 编译源码

### K1端

- **安装 ROS 依赖包**

```bash
sudo apt install -y ros-dev-tools ros-humble-ros-base \
python3-numpy 'ros-humble-cartographer*' 'ros-humble-nav*' libpcap0.8-dev libuvc-dev \
ros-humble-filters ros-humble-turtlesim ros-humble-camera-info-manager ros-humble-pcl-ros \
ros-humble-image-common ros-humble-image-geometry ros-humble-robot-localization \
ros-humble-joint-state-publisher
```

- **下载源码压缩包**

下载源码包：[src_k1.tar.gz](https://archive.spacemit.com/ros2/code/src_k1.tar.gz)。

- **解压并编译源码**

```bash
mkdir -p ~/wheeltec_ws
tar -zxvf src_k1.tar.gz -C ~/wheeltec_ws
cd ~/wheeltec_ws/src
colcon build
```

- **设置串口（注意设备号）**

根据实际设备号设置串口规则。

```bash
udevadm info --query=all --name=/dev/ttyCH343USB1
echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0002", MODE:="0777", SYMLINK+="wheeltec_controller"' > /etc/udev/rules.d/wheeltec_controller2.rules

# CH9102 串口设置（确保系统安装了对应驱动），串口号为 0001，设置别名为 wheeltec_lidar
echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0001", MODE:="0777", SYMLINK+="wheeltec_lidar"' > /etc/udev/rules.d/wheeltec_lidar2.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### PC端

- **安装 ROS 依赖包**

```bash
sudo apt install -y ros-dev-tools ros-humble-ros-base \
python3-numpy 'ros-humble-cartographer*' 'ros-humble-nav*' libpcap0.8-dev libuvc-dev \
ros-humble-filters ros-humble-turtlesim ros-humble-camera-info-manager ros-humble-pcl-ros \
ros-humble-image-common ros-humble-image-geometry ros-humble-robot-localization \
ros-humble-joint-state-publisher
```

- **下载源码压缩包**

下载源码包：[src_pc.tar.gz](https://archive.spacemit.com/ros2/code/src_pc.tar.gz)

- **解压并编译源码**

```bash
mkdir -p ~/wheeltec_ws
tar -zxvf src_k1.tar.gz -C ~/wheeltec_ws
cd ~/wheeltec_ws/src
colcon build
```

## 启动命令

### 同时建图导航

- PC 端开启可视化

```bash
ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
```

- K1 端启动建图与导航

```bash
ros2 launch wheeltec_nav2 wheeltec_nav2_for_slam.launch.py
```

### 分别建图导航

- PC 端开启可视化

```bash
ros2 launch originbot_viz display_slam.launch.py
```

#### 建图

在 Muse Pi 开发板上执行以下命令：

- 使用 slam_gmapping

```bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

- 使用 cartographer

```bash
ros2 launch wheeltec_cartographer cartographer.launch.py
```

- 保存地图

```bash
ros2 run nav2_map_server map_saver_cli -f spacemit1
```

#### 导航

- 首先关闭建图功能

- 在 PC 端开启可视化

```bash
ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
```

- 在 Muse Pi 开发板上，替换地图后执行 `colcon build` 一次，再启动导航

```bash
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
```