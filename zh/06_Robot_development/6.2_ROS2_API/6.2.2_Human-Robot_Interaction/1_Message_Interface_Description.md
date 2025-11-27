
# 消息接口说明



## 音频消息

消息文件位置：`jobot_interfaces/msg/AudioFrame.msg`

用于传输音频数据流，包含时间戳、采样信息和原始 PCM 数据。



### 消息定义

```text
# 头部，包含时间戳和frame_id
std_msgs/Header header

# 采样率
int32 sample_rate

# 声道数
int32 channels

# 格式（0=PCM16, 1=float32）
int32 sample_format

# 每一帧音频数据（int16 PCM）
int16[] data
```



### 字段说明

| 字段名          | 类型              | 说明                                    |
| --------------- | ----------------- | --------------------------------------- |
| `header`        | `std_msgs/Header` | ROS 标准消息头，包含时间戳和 `frame_id` |
| `sample_rate`   | `int32`           | 采样率（Hz），如 `16000`、`48000`       |
| `channels`      | `int32`           | 声道数（1=单声道，2=立体声等）          |
| `sample_format` | `int32`           | 数据格式：`0=PCM16`，`1=float32`        |
| `data`          | `int16[]`         | 音频帧数据（PCM 16bit 小端序）          |



### 使用说明

订阅该消息时，可以通过 `header.stamp` 获取时间戳，用于和其他传感器同步；
 `sample_rate` 和 `channels` 用于解码音频数据；
 `data` 则是原始音频样本数组，可以直接写入 WAV 文件或输入到 ASR 模型。



### Python 订阅示例（rclpy）

```python
import rclpy
from rclpy.node import Node
from jobot_interfaces.msg import AudioFrame

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            AudioFrame,
            'audio_frames',   # 话题名
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
        # 例如：取前 10 个采样点
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



### C++ 订阅示例（rclcpp）

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

        // 示例：输出前 10 个采样点
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

消息文件位置：`jobot_interfaces/msg/VADResult.msg`

该消息用于表示语音活动检测（Voice Activity Detection, VAD）的结果，包含语音出现的概率和是否检测到语音。

### 消息定义

```text
std_msgs/Header header
float32 prob        # 语音概率
bool is_speech      # 是否检测到语音
```



### 字段说明

| 字段名      | 类型              | 说明                                                         |
| ----------- | ----------------- | ------------------------------------------------------------ |
| `header`    | `std_msgs/Header` | ROS 标准消息头，包含时间戳和 `frame_id`，用于与音频帧进行时间同步 |
| `prob`      | `float32`         | 语音活动概率，范围 [0.0, 1.0]，表示当前帧包含语音的置信度    |
| `is_speech` | `bool`            | 是否检测到语音，`true`=有人声，`false`=无人声                |



### 使用场景

- **语音触发**：可用于唤醒词检测前的语音段检测，减少误触发。
- **ASR 前处理**：在传给语音识别（ASR）前过滤掉静音帧，节省计算资源。
- **机器人对话系统**：在多模态交互中，判断用户是否正在说话。



### Python 订阅示例

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



### C++ 订阅示例

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



