sidebar_position: 2

# CMP10A IMU usage

## Hardware Connection

Hardware Documentation：(https://item.jd.com/10052180610725.html)、（https://www.yahboom.com/study/IMU）

**Hardware Connection Diagram：**

![](./images/imu1.png)

**View Device Nodes：**

```
bianbu@bianbu:~$ ls /dev/ttyUSB*
/dev/ttyUSB0
```

**Grant permissions to the device node.**

```
sudo chmod 666 /dev/ttyUSB0
```



## Environment Setup

### Install Dependencies

Ensure the development environment dependencies are installed.：(https://bianbu.spacemit.com/brdk/system_configuration/2.3_System_Dependency_Installation)

```bash
sudo apt install python3-serial ros-humble-rviz2 ros-humble-rviz-imu-plugin
```



### Source the ROS2 environment.

The following steps assume that the environment has already been sourced.

```bash
source /opt/bros/humble/setup.bash
```



## Start the IMU Node

### Publish TF 

```
ros2 launch rdk_sensors wit_imu_rviz.launch.py port:=/dev/ttyUSB0
```
Publishes the IMU data and simultaneously broadcasts a TF transform from `base_link` to `imu_link`, facilitating visualization using RViz 2.

### Do Not Publish TF

```
ros2 launch rdk_sensors wit_imu.launch.py port:=/dev/ttyUSB0
```

Publishes only the IMU data, making it suitable for use when integrating with other ROS 2 packages.



## Launch Visualization

```
export QT_QPA_PLATFORM=xcb# Using the Humble Version
ros2 launch rdk_visualization display_imu.launch.py
```

![](./images/imu2.png)

Shake the IMU, and the pose of the small red cube will change accordingly.
