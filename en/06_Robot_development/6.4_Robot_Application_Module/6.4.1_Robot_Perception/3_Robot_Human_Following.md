# Mobile Robot Human Following

```
Last Version: 12/09/2025
```

## Overview

This section demonstrates human-following functionality for a mobile robot, which includes USB-camera image acquisition, human detection, and velocity decision-making.

For convenience, we observe the robot’s motion in the Gazebo simulator; you can also deploy it on a real robot that supports ROS 2 chassis velocity control.

- The human-following algorithm runs on a **SpacemiT RISC-V board**.
- The Gazebo simulation runs on a **PC** connected to the same network.

## Environment Setup

### On SpacemiT RISC-V Board

1. Flash the **Bianbu ROS system image** to the board.
2. Install required dependencies:

   ```bash
   sudo apt install python3-opencv ros-humble-cv-bridge ros-humble-camera-info-manager \
   ros-humble-image-transport python3-spacemit-ort python3-yaml libyaml-dev python3-numpy
   ```

### On x86 PC

1. Install **Ubuntu 22.04**.
2. Set up **ROS 2 Humble**.
3. Install the mobile robot model and Gazebo simulation packages:

```
sudo apt install ros-humble-gazebo*
sudo apt install ros-humble-turtlebot3
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-bringup
sudo apt install ros-humble-turtlebot3-simulations
```

## Usage Guide

### Launch the Simulation Environment

Run the following command to load the robot model and start the Gazebo simulation environment:

```shell
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

After successful launch, the simulation environment will look like this:

![](images/follow_person_sim.jpg)

### Launch the Human-Following Algorithm

On the **SpacemiT RISC-V board**, launch the human-following algorithm; make sure a USB camera is plugged in beforehand.

The robot’s policy is to follow the target closest to the image center until it reaches a short-range threshold, then stop.
Default linear speed is **0.1 m/s** and angular speed is **0.37 rad/s**; both can be tuned via the launch-file parameters.

The robot's following strategy is to select the target closest to the center of its view and then follow it until it reaches a relatively close distance, at which point it stops.
- **Default motion parameters:**
  - Linear speed: **0.1 m/s**
  - Angular speed: **0.37 rad/s**
- These can be adjusted in the launch file parameters.

**Hardware Connection**

![](images/follow_hardware_usb.jpg)

Check the camera device ID:

```bash
➜  ~ ls /dev/video*
/dev/video0  /dev/video10  /dev/video12  /dev/video14  /dev/video16  /dev/video18  /dev/video2   /dev/video21  /dev/video4  /dev/video50  /dev/video6  /dev/video8  /dev/video-dec0
/dev/video1  /dev/video11  /dev/video13  /dev/video15  /dev/video17  /dev/video19  /dev/video20  /dev/video3   /dev/video5  /dev/video51  /dev/video7  /dev/video9
```

Unplug and replug the camera to confirm the device index.
The default device used in the program is `/dev/video20`.

**Start the Following Node**

Open a terminal and run:

```bash
# Source ROS 2 environment
source /opt/bros/humble/setup.bash
```

```bash
ros2 launch br_application agv_person_follow.launch.py
```

You should see logs similar to:

```
[INFO] [launch]: All log files can be found below /home/zq-pi/.ros/log/2025-06-12-16-07-34-737169-spacemit-k1-x-MUSE-Pi-board-89537
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [agv_follow_node-1]: process started with pid [89544]
[agv_follow_node-1] [INFO] [1749715663.222242636] [agv_follow_node]: AI tracking-control service started, current state: paused
```

**To accommodate multitasking scenarios, the following node launches a service that can control whether following is enabled or not.**

You can quickly enable this service from a terminal:

```bash
source /opt/bros/humble/setup.bash
ros2 service call /toggle_follow std_srvs/srv/SetBool "{data: true}"
```

Expected output:

```
[agv_follow_node-1] [INFO] [...] [agv_follow_node]: Received request: data=True → AI tracking module ON
```

To disable the service, use:

```bash
ros2 service call /toggle_follow std_srvs/srv/SetBool "{data: false}"
```

To use this service within your code:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class FollowClient(Node):
    def __init__(self):
        super().__init__('follow_control_client')
        self.cli = self.create_client(SetBool, 'toggle_follow')

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for toggle_follow service...')

    def send_request(self, enable: bool):
        req = SetBool.Request()
        req.data = enable
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Response: success={future.result().success}, message="{future.result().message}"')
        else:
            self.get_logger().error('Service call failed')

def main():
    rclpy.init()
    client = FollowClient()
    client.send_request(True)  # True = start tracking
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Follow-node terminal output:

```
[agv_follow_node-1] cmd_vel -- linear_x:0.0, angular_z:0.0
[agv_follow_node-1] cmd_vel -- linear_x:0.0, angular_z:0.0
[agv_follow_node-1] cmd_vel -- linear_x:0.4, angular_z:0.0
[agv_follow_node-1] cmd_vel -- linear_x:0.4, angular_z:0.0
[agv_follow_node-1] cmd_vel -- linear_x:0.4, angular_z:0.0
[agv_follow_node-1] cmd_vel -- linear_x:0.4, angular_z:0.0
[agv_follow_node-1] cmd_vel -- linear_x:0.4, angular_z:0.0
```

Move away from the camera or move side to side. You should observe the robot moving accordingly in Gazebo.

### View Detection Results

To view the human-target detection results, relaunch with the following parameter:

```bash
ros2 launch br_application agv_person_follow.launch.py publish_result_img:=true
```

The topic is published to `/detection_image`. To view it on PC, run

```bash
rqt_image-view rqt_image-view
```

### Configurable Parameters

**Parameter reference for `agv_person_follow.launch.py`**

| Parameter          | Purpose                        | Default            |
|--------------------|--------------------------------|--------------------|
| `video_device`     | Camera device index            | `/dev/video20`     |
| `publish_result_img` | Whether to publish the detection image    | `False`            |
| `linear_x`         | Following linear speed         | `0.4 m/s`          |
| `angular_z`        | Following angular speed        | `0.37 rad/s`       |

## Human-Following on a Real Robot

![](images/follow_object.jpg)

On a real robot, tilt the camera upward slightly for better recognition performance.
