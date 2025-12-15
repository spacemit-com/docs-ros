sidebar_position: 3

# VAD 节点

## 功能说明

VAD（Voice Activity Detection，语音活动检测）节点用于从连续的音频流中检测人声语音片段，输出人声概率以及是否存在人声的判定结果。
该节点可作为语音唤醒（Wake Word）、ASR 前端的辅助模块，用于减少无效音频数据处理，提高系统效率。

核心功能：

* 订阅音频数据（`jobot_interfaces/msg/AudioFrame`）
* 利用 **Silero VAD 模型（ONNX）** 进行实时推理
* 发布语音活动检测结果（`jobot_interfaces/msg/VADResult`）

## 软硬件环境

* **操作系统**：ROS2_LXQT
* **硬件推荐**：SpacemiT MUSE Pi Pro（内置 NPU 加速，适合边缘推理场景）
* **依赖库**：
  * ROS 2 Humble 或更高版本
  * SpacemiT ONNX Runtime（CPU/NPU 推理）
  * PortAudio / ALSA（用于音频输入）

### 常用测试工具（开发/调试阶段）

* `rqt_graph`：可视化 ROS 节点和话题连接关系
* `ros2 topic echo`：查看消息内容
* `ros2 topic hz`：检查话题发布频率

## 依赖安装检查

```
sudo apt install -y ros-humble-ros-base python3-pyaudio python3-spacemit-ort libfftw3-dev
```

## 需要订阅的话题

* **音频输入**：
  * 默认订阅话题名称：`/audio/raw`
  * 消息类型：`jobot_interfaces/msg/AudioFrame`
  * 内容：包含采样率、声道数、采样格式（pcm/float32）和音频数据数组

## 启动命令

使用 `ros2 launch` 启动 VAD 节点：

```bash
ros2 launch rdk_hri vad.launch.py
```

## 发布的话题

* 话题名称：`/audio/vad_out`
* 消息类型：`jobot_interfaces/msg/VADResult`
* 字段说明：

  * `std_msgs/Header header`：时间戳和 frame\_id
  * `float32 prob`：当前帧属于语音的概率
  * `bool is_speech`：是否检测到语音

## 参数详解

| 参数名      | 类型   | 默认值         | 说明                       |
| ----------- | ------ | -------------- | -------------------------- |
| `trig_on`   | float  | 0.23           | 判定是否存在人声的概率阈值 |
| `sub_topic` | string | /audio/raw     | 订阅的音频流话题           |
| `pub_topic` | string | /audio/vad_out | 发布的 VAD 结果话题名      |

## C++ 订阅示例

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

## Python 订阅示例

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
