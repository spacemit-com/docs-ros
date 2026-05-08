sidebar_position: 2

# Navigation2

## Overview

Navigation2 is the ROS 2–based navigation framework that safely drives a mobile robot from point A to point B.

It integrates multiple modules, including **pose estimation, behavior planning, path planning, and obstacle avoidance**.

GitHub: [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)

This example demonstrates Navigation2 in two scenarios:

- **simulation** (Gazebo + rviz on the PC)
- **real-vehicle** (physical robot with SpaceMiT board)

## Simulation Navigation

In this section, a **simulated robot-car model** is used to demonstrate Navigation2 navigation. The simulation lets you observe **the robot moving in Gazebo** and **visualizing navigation in rviz**.

- **Navigation2 algorithms** run on the SpacemiT RISC-V board.
- The **simulated robot model, Gazebo environment, and rviz visualization** run on a PC connected to the same network.

### Preparation

1. Flash the **SpacemiT board** with the **Bianbu ROS system image**.
2. Install **ROS 2 Humble** and **Bianbu Robot SDK** on the PC.

### Usage Guide

#### Launch the Simulation Environment

On the PC, open a terminal and install the robot-car model and Gazebo environment:

```shell
sudo apt install ros-humble-gazebo*
sudo apt install ros-humble-turtlebot3
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-bringup
sudo apt install ros-humble-turtlebot3-simulations
```

After installation is complete, run the following command to load the robot model and start the Gazebo simulation environment:

```shell
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Once launched successfully, the simulation environment will appear as shown below:

![](images/sim_gazebo.jpg)

#### Start Navigation2

Next, start Navigation2 on the **SpacemiT board** to enable autonomous robot navigation.

Open a terminal on on the **SpacemiT board**, then run the following command to install Navigation2:

```shell
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

After installation is complete, start Navigation2 by running:

```shell
source /opt/bros/humble/setup.bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
```

#### Visualization on PC

On the PC, open a new terminal and start rviz visualization with:

```shell
source /opt/ros/humble/setup.bash
source ~/ros2_demo_ws/install/setup.bash
ros2 launch br_visualization display_navigation.launch.py
rviz2
```

![](images/sim_nav2_rviz.jpg)

Initially, you will only see an empty environment map, because the robot’s initial pose has not been set.

In rviz2, click **2D Pose Estimate** to set the robot’s starting position and orientation:

![](images/sim_nav2_set_pose1.jpg)

Once set, rviz will load the robot’s coordinate frames and costmap information:

![](images/sim_nav2_set_pose2.jpg)

To Click **2D Nav Goal** in rviz to set the navigation target:

![](images/sim_nav2_set_goal1.jpg)

You can then observe the robot navigating to the target:

![](images/sim_nav2_set_goal2.jpg)

## Real-Vehicle Navigation

This section assumes you have already built and saved an environment map using [SLAM](1_SLAM_Mapping.md).

We will now start Navigation2 on an actual robot equipped with a SpacemiT RISC-V board and visualize the process on the PC.

### Preparation

1. Flash the SpacemiT board with **Bianbu Desktop 24.04** and install the **Bianbu Robot SDK**.
2. Install **ros-humble** and the **Bianbu Robot SDK** on the PC.
3. Build and save the environment map as described in [SLAM](1_SLAM_Mapping.md).

### Usage Guide

#### Install Navigation2

Run the following commands on the **SpacemiT board terminal** to install Navigation2:

```shell
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

#### Start Navigation2 Navigation

Launch the real-robot configuration and the Navigation2 stack in one step by running:

```shell
source /opt/bros/humble/setup.bash
ros2 launch br_navigation nav2.launch.py
```

#### Visualization on PC

Open a new terminal **on the PC** and start RViz for visualization:

```shell
ros2 launch br_visualization display_navigation.launch.py
```

![](images/nav2_rviz.jpg)

The launch file is preconfigured to set the robot’s initial pose at the SLAM map origin.
If needed, you can adjust the robot’s position and orientation in RViz by clicking **2D Pose Estimate**:

![](images/nav2_set_pose.jpg)

Click **2D Nav Goal** to specify a navigation target and monitor the progress in rviz2 on the PC.

![](images/nav2_set_goal.jpg)

## Run SLAM and Navigation2 Together

If you don’t already have a prebuilt map, you can run **SLAM and Navigation2 simultaneously** to perform navigation while the map is being generated and updated in real time.

Run below command on the SpacemiT board:

```shell
ros2 launch br_navigation nav2_for_slam.launch.py
```

![](images/slam_with_nav2.jpg)

On the PC, use RViz to set a navigation target by clicking **2D Nav Goal**.
The robot will navigate through the unknown environment while SLAM continuously updates the map.

![](images/slam_with_nav2_set_goal.jpg)
