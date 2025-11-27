# ASR 节点

## 功能说明

ASR（Automatic Speech Recognition，自动语音识别）节点用于将语音流转换为文本。
该节点支持在线语音识别，可结合 VAD 节点使用，在检测到语音段落时启动识别，减少无效计算。
典型应用场景包括语音对话、人机交互和语音控制等。

---

## 软硬件环境

* **硬件推荐**：

  * SpacemiT MUSE Pi Pro 或其他支持 ROS2_LXQT 的 RISC-V64 平台
  * USB 麦克风（或其他音频采集设备）
* **软件环境**：

  * ROS2_LXQT 系统

## 依赖安装检查

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

配置虚拟环境

```
# 设置镜像源
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple

# 创建虚拟环境
python3 -m venv ~/asr_env
source ~/asr_env/bin/activate

# 安装依赖
pip install -r /opt/bros/humble/share/jobot_voice/requirements.txt
```



## 需要订阅的话题

* `/audio/raw` (`jobot_interfaces/msg/AudioFrame`)
* 输入音频帧数据，来自音频采集节点。

- `/audio/vad_out` (`jobot_interfaces/msg/VADResult`)
  * 可选的 VAD 订阅。

## 启动命令

使用 `ros2 launch` 启动：

```
source /opt/bros/humble/setup.bash
source ~/asr_env/bin/activate
export PYTHONPATH=~/asr_env/lib/python3.12/site-packages/:$PYTHONPATH
```

```bash
ros2 launch rdk_hri asr.launch.py
```



## 发布的话题

* `/voice_text` (`std_msgs/msg/String`)
* 输出识别后的文本结果。



## 参数列表

|     参数名     |  类型  |      默认值      |         候选值         |                          说明                          |
| :------------: | :----: | :--------------: | :--------------------: | :----------------------------------------------------: |
| `result_topic` | string |   `voice_text`   |         自定义         |                  ASR 识别结果输出话题                  |
|  `sub_topic`   | string |   `/audio/raw`   | 与音频采集节点发布一致 |          输入音频流话题（由音频采集节点提供）          |
|   `language`   | string |       `zh`       |         zh、en         |           识别语言（`zh` 中文 / `en` 英文）            |
|     `sld`      | double |      `1.0`       |          > 0           | 录音启动后静音最大时长（秒），超过则结束录音开始转文字 |
|    `min_db`    |  int   |      `2000`      |      建议 > 2000       |                 判定为有声音的能量阈值                 |
|   `max_time`   | double |      `10.0`      |          > 0           |                 单段录音最长时长（秒）                 |
|   `use_vad`    |  bool  |     `false`      |      false、true       |                 是否启用外部 VAD 结果                  |
|  `vad_topic`   | string | `/audio/vad_out` |  与 VAD 节点保持一致   |                    订阅 VAD 话题名                     |



## C++ 订阅示例

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ASRClient : public rclcpp::Node {
public:
    ASRClient() : Node("asr_client") {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/voice_text", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "识别结果: %s", msg->data.c_str());
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



## Python 订阅示例

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
        self.get_logger().info(f"识别结果: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ASRClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

