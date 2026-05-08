sidebar_position: 1

# Message Interface Specifications

## Audio Messages

Message file location：`jobot_interfaces/msg/AudioFrame.msg`

Used for transmitting audio data streams; contains timestamps, sampling information, and raw PCM data.

### Message Definition

```text
# Header; contains timestamp and frame_id
std_msgs/Header header

# Sample rate
int32 sample_rate

# Number of channels
int32 channels

# Format（0=PCM16, 1=float32）
int32 sample_format

# Audio data for a single frame (int16 PCM)
int16[] data
```

### Field Descriptions

| Field Name          | Type              | Description                                    |
| --------------- | ----------------- | --------------------------------------- |
| `header`        | `std_msgs/Header` | ROS standard message header; contains timestamp and `frame_id` |
| `sample_rate`   | `int32`           | Sample rate (Hz), e.g., `16000`, `48000`     |
| `channels`      | `int32`           | Number of channels (1=mono, 2=stereo, etc.)          |
| `sample_format` | `int32`           | Data format: `0=PCM16`, `1=float32`        |
| `data`          | `int16[]`         | Audio frame data (PCM 16-bit, little-endian)          |

### Usage Instructions

When subscribing to this message, the timestamp can be retrieved via `header.stamp`, which is used for synchronization with other sensors;
`sample_rate` and `channels` are used for decoding the audio data;
`data` is an array of raw audio samples, which can be written directly to a WAV file or fed into an ASR model.

### Python Subscription Example (rclpy)

```python
import rclpy
from rclpy.node import Node
from jobot_interfaces.msg import AudioFrame

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            AudioFrame,
            'audio_frames',   # Topic name
            self.callback,
            10
        )

    def callback(self, msg: AudioFrame):
        self.get_logger().info(
            f"Received audio: rate={msg.sample_rate}, "
            f"channels={msg.channels}, "
            f"format={msg.sample_format}, "
            f"len(data)={len(msg.data)}"
        )
        # Example: Retrieve the first 10 samples
        print("Samples:", msg.data[:10])

def main(args=None):
    rclpy.init(args=args)
    node = AudioSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Subscription Example (rclcpp)

```cpp
#include "rclcpp/rclcpp.hpp"
#include "jobot_interfaces/msg/audio_frame.hpp"

using std::placeholders::_1;

class AudioSubscriber : public rclcpp::Node
{
public:
    AudioSubscriber() : Node("audio_subscriber")
    {
        subscription_ = this->create_subscription<jobot_interfaces::msg::AudioFrame>(
            "audio_frames", 10, std::bind(&AudioSubscriber::callback, this, _1));
    }

private:
    void callback(const jobot_interfaces::msg::AudioFrame::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received audio: rate=%d, channels=%d, format=%d, len(data)=%zu",
            msg->sample_rate, msg->channels, msg->sample_format, msg->data.size());

        // Example: Output the first 10 sampling points
        for (size_t i = 0; i < std::min((size_t)10, msg->data.size()); i++)
        {
            std::cout << msg->data[i] << " ";
        }
        std::cout << std::endl;
    }

    rclcpp::Subscription<jobot_interfaces::msg::AudioFrame>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

## VAD

Message File Location: `jobot_interfaces/msg/VADResult.msg`

This message represents the result of Voice Activity Detection (VAD), including the probability of speech occurrence and whether speech was detected.

### Message Definition

```text
std_msgs/Header header
float32 prob        # Speech probability
bool is_speech      # Whether speech was detected
```

### Field Descriptions

| Field Name      | Type              | Description                                                         |
| ----------- | ----------------- | ------------------------------------------------------------ |
| `header`    | `std_msgs/Header` | ROS standard message header, containing a timestamp and `frame_id`, used for time synchronization with audio frames. |
| `prob`      | `float32`         | Probability of speech activity; range [0.0, 1.0], indicating the confidence that the current frame contains speech.    |
| `is_speech` | `bool`            | Whether speech was detected: `true` = speech present, `false` = no speech present.                |

### Use Cases

- **Voice Triggering**： Can be used for voice segment detection before wake-up word detection, reducing false triggering.
- **ASR Preprocessing**： Filter out silent frames before passing them to speech recognition (ASR) to save computational resources.
- **Robot Dialogue Systems**：In multimodal interaction scenarios, determines whether the user is currently speaking.

### Python Subscription Example

```python
import rclpy
from rclpy.node import Node
from jobot_interfaces.msg import VADResult

class VADSubscriber(Node):
    def __init__(self):
        super().__init__('vad_subscriber')
        self.subscription = self.create_subscription(
            VADResult,
            'vad_result',
            self.callback,
            10
        )

    def callback(self, msg: VADResult):
        self.get_logger().info(
            f"[VAD] prob={msg.prob:.2f}, is_speech={msg.is_speech}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = VADSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Subscription Example

```cpp
#include "rclcpp/rclcpp.hpp"
#include "jobot_interfaces/msg/vad_result.hpp"

using std::placeholders::_1;

class VADSubscriber : public rclcpp::Node
{
public:
    VADSubscriber() : Node("vad_subscriber")
    {
        subscription_ = this->create_subscription<jobot_interfaces::msg::VADResult>(
            "vad_result", 10, std::bind(&VADSubscriber::callback, this, _1));
    }

private:
    void callback(const jobot_interfaces::msg::VADResult::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "[VAD] prob=%.2f, is_speech=%s",
            msg->prob, msg->is_speech ? "true" : "false");
    }

    rclcpp::Subscription<jobot_interfaces::msg::VADResult>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VADSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```
