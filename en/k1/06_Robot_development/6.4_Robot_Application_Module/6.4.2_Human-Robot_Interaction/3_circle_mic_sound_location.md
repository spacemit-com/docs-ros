sidebar_position: 3

# Sound Localization

This guide demonstrates how to implement sound source localization using the iFlytek M260C 6-Microphone Array Board. The hardware detects the direction of sound sources and outputs an angle between 0° and 360°.

## Hardware Specifications

**Product:** iFlytek Co-branded Far-field Microphone Array (6-Mic)
**Model:** M260C Voice Interaction Module
**Official Link:** [Product Page](https://item.m.jd.com/product/10054882134702.html?gx=RnAomTM2b2fan85Hp41wX4inMaW5_TE&gxd=RnAoy2BbaWDZyZwcrIImVA6-xKLxjrc&ad_od=share&utm_source=androidapp&utm_medium=appshare&utm_campaign=t_335139774&utm_term=CopyURL)

![](./images/mic_hard.jpg)

It outputs an angle **between 0° and 360°**.

## Hardware Connection

Connect the microphone array to your system as shown below:
![](./images/mic_connect.jpg)

## Device Configuration

To allow your system to recognize the microphone array, you need to create a device rule

```bash
sudo su

echo  'KERNEL=="ttyACM*", ATTRS{serial}=="0004", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777",SYMLINK+="wheeltec_mic"' >/etc/udev/rules.d/wheeltec_mic.rules

udevadm control --reload-rules
udevadm trigger
```

Verify the configuration:

```bash
ls /dev/wheeltec_mic -lh
```

Expected successful output:

![](./images/ls_res1.png)

## Launching Sound Localization

Start the microphone node:

```bash
ros2 launch br_sensors ring_mic.launch.py
```

Testing the Functionality
1. Speak the default wake-up phrase "小微小微" / "Xiao Wei Xiao Wei" (in any direction).
2. The system detects the wake-up phrase and outputs the angle of the sound source (0° to 360°)

Example terminal output:

![](./images/mic_print.png)

**Note:**
- The default wake-up phrase is "Xiao Wei Xiao Wei". You can change it by referring to the official hardware documentation.
- A trigger flag is set to `1` when wake word is detected, then resets to `0`
- Use this flag to determine wake phrase detection in your subscribers

### Topic Subscription

Topic name: `angle_topic`

Below is a simple Python script to subscribe to the `angle_topic` and display the angle and wake-up flag:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class AngleSubscriber(Node):
    def __init__(self):
        super().__init__('angle_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'angle_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if msg.data and len(msg.data) >= 2:
            angle = msg.data[0]
            trigger_flag = msg.data[1]
            self.get_logger().info(f"Received angle: {angle}, trigger: {trigger_flag}")

def main(args=None):
    rclpy.init(args=args)
    node = AngleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save the script as `angle_sub.py`

Source your ROS 2 environment and run the script:

```bash
source /opt/bros/humble/setup.bash
python3 angle_sub.py
```

The subscriber will continuously print received angle and trigger data whenever the wake-up phrase is detected.
