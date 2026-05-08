sidebar_position: 3

# VAD Node

## Function Description

The VAD (Voice Activity Detection) node is used to detect segments of human speech within a continuous audio stream, outputting both the probability of speech presence and a binary determination of whether speech is present.
This node serves as an auxiliary module for Wake Word and ASR front-ends, reducing invalid audio data processing and improving system efficiency.

Core functions:

* Subscribe to audio data（`jobot_interfaces/msg/AudioFrame`）
* Perform real-time inference using the**Silero VAD model (ONNX)**
* Publish Voice Activity Detection results（`jobot_interfaces/msg/VADResult`）

## Software and Hardware Environment

* **Operating System**：ROS2_LXQT 
* **Recommended Hardware**：SpacemiT MUSE Pi Pro（with built-in NPU acceleration, suitable for edge inference scenarios）
* **Dependency**：
  * ROS 2 Humble or higher
  * SpacemiT ONNX Runtime（CPU/NPU inference）
  * PortAudio / ALSA（for audio input）

### Common Testing Tools (Development/Debugging Phase)

* `rqt_graph`：Visualizes ROS node and topic connections
* `ros2 topic echo`：Views message content
* `ros2 topic hz`：Checks topic publishing frequency

## Dependency Installation Check

```
sudo apt install -y ros-humble-ros-base python3-pyaudio python3-spacemit-ort libfftw3-dev
```

## Subscribed Topics

* **Audio Input**：
  * Default subscribed topic name：`/audio/raw`
  * Message type：`jobot_interfaces/msg/AudioFrame`
  * Content: Includes the sample rate, number of channels, sample format（pcm/float32）and audio data array

## Launch Command

Use `ros2 launch` to start the VAD node:

```bash
ros2 launch rdk_hri vad.launch.py
```

## Published Topics

* Topic name：`/audio/vad_out`
* Message type：`jobot_interfaces/msg/VADResult`
* Field Descriptions：

  * `std_msgs/Header header`：Timestamp and frame\_id
  * `float32 prob`：The probability that the current frame belongs to speech
  * `bool is_speech`：Whether speech is detected

## Parameter Details

| Parameter Name      | Type   | Default Value         | Description                       |
| ----------- | ------ | -------------- | -------------------------- |
| `trig_on`   | float  | 0.23           | Probability threshold for detecting the presence of speech |
| `sub_topic` | string | /audio/raw     | Topic for subscribing to the audio stream           |
| `pub_topic` | string | /audio/vad_out | Topic for publishing VAD results      |

## C++ Subscription Example

```cpp
#include "rclcpp/rclcpp.hpp" 
#include "jobot_interfaces/msg/vad_result.hpp"

class VadResultListener : public rclcpp::Node
{
public:
    VadResultListener() : Node("vad_result_listener")
    {
        sub_ = this->create_subscription<jobot_interfaces::msg::VADResult>(
            "/audio/vad_out", 10,
            [this](const jobot_interfaces::msg::VADResult::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(),
                            "Speech prob: %.2f, is_speech: %s",
                            msg->prob, msg->is_speech ? "true" : "false");
            });
    }

private:
    rclcpp::Subscription<jobot_interfaces::msg::VADResult>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VadResultListener>());
    rclcpp::shutdown();
    return 0;
}
```

## Python Subscription Example

```python
import rclpy
from rclpy.node import Node
from jobot_interfaces.msg import VADResult

class VadResultListener(Node):
    def __init__(self):
        super().__init__('vad_result_listener')
        self.subscription = self.create_subscription(
            VADResult,
            '/audio/vad_out',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: VADResult):
        self.get_logger().info(
            f"Speech prob: {msg.prob:.2f}, is_speech: {msg.is_speech}")

def main(args=None):
    rclpy.init(args=args)
    node = VadResultListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
