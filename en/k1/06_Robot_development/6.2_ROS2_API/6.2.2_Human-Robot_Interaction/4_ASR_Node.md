sidebar_position: 4

# ASR Node

## Function Description

ASR（Automatic Speech Recognition）node is used to convert audio streams into text.

This node supports online speech recognition and can be used in conjunction with a VAD node to initiate recognition only when a speech segment is detected, thereby reducing unnecessary computation.

## Hardware and Software Environment

* **Recommended Hardware**：

  * SpacemiT MUSE Pi Pro or any other RISC-V64 platform that supports ROS2_LXQT
  * USB microphone (or other audio capture device)
* **Software Environment**：

  * ROS2_LXQT system

## Dependency Installation Check

```
sudo apt update
sudo apt install -y libopenblas-dev \
 portaudio19-dev \
 python3-dev \
 ffmpeg \
 python3-spacemit-ort \
 libcjson-dev \
 libasound2-dev \
 python3-pip \
 python3-venv
```

Configuring the Virtual Environment

```
# Set up mirror sources
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple

# Create the virtual environment
python3 -m venv ~/asr_env
source ~/asr_env/bin/activate

# Install dependencies
pip install -r /opt/bros/humble/share/jobot_voice/requirements.txt
```

## Required Subscribed Topics

* `/audio/raw` (`jobot_interfaces/msg/AudioFrame`)
* Input audio frame data, sourced from the audio capture node.

* `/audio/vad_out` (`jobot_interfaces/msg/VADResult`)
  * Optional VAD subscription.

## Launch Command

Launch using `ros2 launch` ：

```
source /opt/bros/humble/setup.bash
source ~/asr_env/bin/activate
export PYTHONPATH=~/asr_env/lib/python3.12/site-packages/:$PYTHONPATH
```

```bash
ros2 launch rdk_hri asr.launch.py
```

## Published Topics

* `/voice_text` (`std_msgs/msg/String`)
* Output the recognized text result.

## Parameter List

|     Parameter Name     |  Type  |      Default Value      |         Candidate Value         |               Description                         |
| :------------: | :----: | :--------------: | :--------------------: | :----------------------------------------------------: |
| `result_topic` | string |   `voice_text`   |         Customizable       |                  ASR recognition result output topic                  |
|  `sub_topic`   | string |   `/audio/raw`   | Must match audio capture node |          Input audio stream topic (provided by audio capture node)          |
|   `language`   | string |       `zh`       |         zh、en         |           Recognition language (`zh`: Chinese / `en`: English)            |
|     `sld`      | double |      `1.0`       |          > 0           | The maximum duration of mute after recording starts (seconds), if exceeded, the recording ends and text conversion begins |
|    `min_db`    |  int   |      `2000`      |      Recommended > 2000       |                 Energy threshold for detecting sound                 |
|   `max_time`   | double |      `10.0`      |          > 0           |                 Maximum duration (seconds) for a single recording                 |
|   `use_vad`    |  bool  |     `false`      |      false、true       |                 Whether to enable external VAD results                  |
|  `vad_topic`   | string | `/audio/vad_out` |   Consistent with VAD node   |                   Subscribe to VAD topic name                     |

## C++ Subscription Example

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ASRClient : public rclcpp::Node {
public:
    ASRClient() : Node("asr_client") {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/voice_text", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Recognition result: %s", msg->data.c_str());
            });
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ASRClient>());
    rclcpp::shutdown();
    return 0;
}
```

## Python Subscription Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ASRClient(Node):
    def __init__(self):
        super().__init__('asr_client')
        self.subscription = self.create_subscription(
            String,
            '/voice_text',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Recognition result: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ASRClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
