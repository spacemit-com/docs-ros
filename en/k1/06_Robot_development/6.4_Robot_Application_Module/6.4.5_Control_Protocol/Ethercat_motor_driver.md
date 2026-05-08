sidebar_position: 1

# Control Motor Based on EtherCAT

This document explains how to control an integrated servo motor based on the EtherCAT protocol, using the JMC IHSS42-24-05-EC as an example.

## Hardware Connection

![Hardware connection](images/ethercat_motor.jpg)
As shown in the figure, connect the motor power supply. Connect one end of an Ethernet cable to the motor IN port and the other to the development board Ethernet port. A continuously illuminated power indicator confirms normal motor operation.

## Environment Description

The ROS 2 firmware already integrates the EtherCAT environment. Before startup, perform the following steps to verify that the service is enabled.

**After the wiring is completed, check whether the device node appears.**

```bash
ls /dev/EtherCAT0
```

If the node appears, proceed to [Download Software Package](#download-software-package) for the next step.

If the node does not appear, go to [Enable EtherCAT](#enable-ethercat) for configuration.

### Enable EtherCAT

#### 1. Open the Device Tree File Folder

```bash
cd /boot/spacemit/6.6.63/
```

#### 2. Obtain the Development Board DTS File

```bash
sudo dtc -I dtb -O dts -o k1-x_MUSE-Pi-Pro.dts k1-x_MUSE-Pi-Pro.dtb


# Run the following command if the dtc tool is not installed.
sudo apt install device-tree-compiler
```

**Note:** This example uses the MUSE-Pi-Pro development board. If a different development board is used, replace the device tree filename in the command with the actual board name.

#### 3. Modify DTS

```bash
vi k1-x_MUSE-Pi-Pro.dts
```

* Locate the **&eth0** node (same as `ethernet@cac80000`).

  Modify the `compatible` property to `spacemit,k1x-ec-emac`.

```bash
 ethernet@cac80000 {
        compatible = "spacemit,k1x-ec-emac";
        reg = <0x00 0xcac80000 0x00 0x420>;
        k1x,apmu-base-reg = <0xd4282800>;
        ctrl-reg = <0x3e4>;
        dline-reg = <0x3e8>;
        clocks = <0x03 0xa5 0x03 0xa6>;
        clock-names = "emac-clk\0ptp-clk";
        resets = <0x1d 0x5d>;
        reset-names = "emac-reset";
        interrupts-extended = <0x1e 0x83>;
        mac-address = [00 00 00 00 00 00];
        ptp-support;
        ptp-clk-rate = <0x989680>;
        power-domains = <0x20 0x00>;
        clk,pm-runtime,no-sleep;
        cpuidle,pm-runtime,sleep;
        interconnects = <0x42>;
        interconnect-names = "dma-mem";
        status = "okay";
        pinctrl-names = "default";
        pinctrl-0 = <0x43>;
        emac,reset-gpio = <0x32 0x6e 0x00>;
        emac,reset-active-low;
        emac,reset-delays-us = <0x00 0x2710 0x186a0>;
        tx-threshold = <0x5ee>;
        rx-threshold = <0x0c>;

```

* Locate **&ec_master** (same as `ethercat_master`).

  Modify `compatible = "igh,k1x-ec-master";`

  Modify `status = "okay";`

```bash
ethercat_master {
        compatible = "igh,k1x-ec-master";
        run-on-cpu = <0x01>;
        debug-level = <0x00>;
        master-count = <0x01>;
        ec-devices = <0x40 0x41>;
        master-indexes = <0x00 0x00>;
        modes = "ec_main\0ec_backup";
        status = "okay";
        };

```

#### 4. Compile New DTB

```bash
sudo dtc -I dts -O dtb -o k1-x_MUSE-Pi-Pro.dtb k1-x_MUSE-Pi-Pro.dts
```

#### 5. Reboot the Development Board

```bash
sudo reboot
```

The changes will take effect after a system reboot.

#### 6. Modify Device Node Permissions

```bash
sudo chown bianbu:bianbu /dev/EtherCAT0
```
The default owner of the device node is `root`. To allow access for a regular user, you must manually modify the permissions after each reboot.

Then, check the `EtherCAT0` device again.

```bash
bianbu@bianbu:~$ ls -l /dev/EtherCAT0
crw------- 1 bianbu bianbu 240, 0 Sep 29 20:32 /dev/EtherCAT0
```

### Download Software Package

**Note:** If you switch to the root user from this step onward, no permission-related errors will occur when launching the node, and no additional file modifications are required.

#### 1. Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create a ROS 2 functional package
ros2 pkg create motor_driver --build-type ament_cmake --dependencies ethercat_driver_ros2 rclcpp std_msg
```

#### 2. Download the Package

```bash
cd ~

# Clone the entire repository
git clone https://gitlab.dc.com:8443/bianbu/bianbu-robot/brdk.git

# Copy the resource to the workspace
cp -r brdk/core_pkgs/jobot_bringup_pkgs/jobot_protocol_bringup ~/ros_ws/src/
```

#### 3. Install the Build Environment

```bash
cd ~/ros2_ws

# Install the build environment
apt install -y colcon python3-rosdep gcc g++

# Install dependencies
apt install -y ros-humble-control-msgs ros-humble-hardware-interface

apt install -y ros-humble-hardware-interface \
ros-humble-xacro \
ros-humble-imu-sensor-broadcaster \
ros-humble-diff-drive-controller  \
ros-humble-position-controllers  \
ros-humble-gripper-controllers  \
ros-humble-joint-state-broadcaster  \
ros-humble-joint-trajectory-controller  \
ros-humble-controller-manager \
ros-humble-velocity-controllers \
ros-humble-ros2-control
```

#### 4. Compile ROS 2 EtherCAT Packages

```bash

cd ~/ros2_ws/

sudo rosdep init    # Non-root users need to add sudo
rosdep update
rosdep install --ignore-src --from-paths . -y -r

colcon build

```

## Launch and Control

### 1. Launch the EtherCAT Master Service

```bash
sudo igh_driver          # ROS 2 is pre-integrated and can be run directly.
```

If the `igh_driver` is not integrated in your flashed firmware version:

```bash
cd brdk/core_pkgs/jobot_bringup_pkgs/jobot_protocol_bringup/resource
```

The executable file is available in this path.

Execute:

```bash
sudo chmod +x igh_driver

./igh_driver
```

You can check the status of the master and slave devices using the following commands:

```bash
ethercat master
#Output
Master0
  Phase: Idle
  Active: no
  Slaves: 1
  Ethernet devices:
    Main: fe:fe:fe:ab:82:c0 (attached)
      Link: UP
      Tx frames:   10316433
      Tx bytes:    643986548
      Rx frames:   10316432
      Rx bytes:    643986488
      Tx errors:   0
      Tx frame rate [1/s]:    116    119    119
      Tx rate [KByte/s]:      6.8    7.0    7.0
      Rx frame rate [1/s]:    116    119    119
      Rx rate [KByte/s]:      6.8    7.0    7.0
    Common:
      Tx frames:   10316433
      Tx bytes:    643986548
      Rx frames:   10316432
      Rx bytes:    643986488
      Lost frames: 0
      Tx frame rate [1/s]:    116    119    119
      Tx rate [KByte/s]:      6.8    7.0    7.0
      Rx frame rate [1/s]:    116    119    119
      Rx rate [KByte/s]:      6.8    7.0    7.0
      Loss rate [1/s]:          0      0      0
      Frame loss [%]:         0.0    0.0    0.0
  Distributed clocks:
    Reference clock:   Slave 0
    DC reference time: 0
    Application time:  0
                       2000-01-01 00:00:00.000000000


ethercat slaves
#output
0  0:0  PREOP  +  IHSS42-EC
```

### 2. Launch the Node

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jobot_protocol_bringup motor_drive.launch.py
```

**Note:** Running as the root user avoids the following errors and warnings.

* If the launch fails, check the device node permissions.

```bash
ls -l /dev/EtherCAT0

If the device is owned by `root`, run the following command to make it available to regular users.

sudo chown bianbu:bianbu /dev/EtherCAT0
```

* If the launch fails because of a real-time scheduling warning, modify the file and reboot the development board.

```bash
sudo vi /etc/security/limits.conf


Add the following lines below the line `#@student        -       maxlogins       4`.

bianbu - rtprio 99
bianbu - memlock unlimited
bianbu - nice -20

# End of file
```

### 3. Control the Motor

Open a new terminal.

```bash
# Continuously send position
cd ~/ros2_ws

source install/setup.bash
# Execute the source command every time you open a new terminal to set the ROS environment variables.

ros2 topic pub -r 0.2 /trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, joint_names: ["joint_1"], points: [{positions: [100.0], velocities: [0.0], accelerations: [0.0], time_from_start: {sec: 1, nanosec: 0}},{positions: [10000.0], velocities: [0.0], accelerations: [0.0],time_from_start: {sec: 5, nanosec: 0}}]}'
```

The motor will rotate at a fixed frequency.

**Parameter Description**

```bash
1.
- ros2 topic pub

  ---  Publish a topic

2.
- -r 0.2

  ---  Specify the publish rate at 0.2 Hz (i.e., once every 5 seconds)

3.
- /trajectory_controller/joint_trajectory

--- Topic name
  Composed of the controller name `trajectory_controller` and the function `joint_trajectory`.
  Must match the configuration in the controller configuration file (`controllers.yaml`).

4.
- trajectory_msgs/msg/JointTrajectory

--- Standard message type in ROS2 for describing joint trajectories.

5.
## Message content, format defined in ROS.
- {header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}

--- Message header with metadata.

Parameters:
 - stamp: {sec: 0, nanosec: 0} 
 
 --- Message timestamp, (0,0) indicates using the current time when the controller receives messages as the trajectory start time. 

- frame_id: ""   

 --- Reference coordinate frame name. An empty string indicates no coordinate frame is specified.


6.
- joint_names: ["joint_1"]

--- List of joint names to be controlled. It must be completely consistent with the joint names defined in the configuration files. 


7.
- points: [
{positions: [100.0], velocities: [0.0], accelerations: [0.0], time_from_start: {sec: 1, nanosec: 0}},

{positions: [10000.0], velocities: [0.0], accelerations: [0.0],time_from_start: {sec: 5, nanosec: 0}}
]

--- List of trajectory points. Each point defines joint's state at a specific time. 

Parameters:
   - positions: [100.0]      --- Target position of the joint
   - velocities: [0.0]       --- Target velocity of the joint when reaching this point.
   - accelerations: [0.0]    --- Target acceleration of the joint when reaching this point. 
   - time_from_start: {sec: 1, nanosec: 0}   --- Time elapsed from the start of the trajectory to reaching this point. 
```

### 4. Read Position Information

Open a new terminal:

```bash
cd ~/ros_ws
source install/setup.bash
ros2 topic echo /joint_states
```

**Parameter Description**

```bash
ros3 topic echo --- Print real-time topic content

/joint_states   --- Topic Name
```
