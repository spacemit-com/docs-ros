---
sidebar_position: 1
---

# 语音转文字（ASR）

## 功能简介

本模块基于 SenseVoice ONNX 模型 实现语音转文字（Automatic Speech Recognition, ASR）功能。系统通过麦克风获取音频输入，结合 SpacemiT 智算核进行模型推理，并将识别结果通过 ROS2 消息发布。

默认支持 唤醒词检测 + 语音识别 联动流程，实现语音控制的人机交互能力。

本模块的应用场景可覆盖：

- 智能家居语音控制
- 服务机器人语音指令交互
- 工业语音辅助系统
- 无接触式语音控制设备

## 环境准备

### 安装系统依赖项

```bash
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

### 配置 Python 虚拟环境并安装依赖

```bash
# 设置镜像源（建议使用清华镜像）
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple

# 创建虚拟环境
python3 -m venv ~/asr_env
source ~/asr_env/bin/activate

# 安装依赖
pip install -r /opt/bros/humble/share/jobot_voice/requirements.txt
```

`jobot_voice/` 目录已预装到系统，直接使用即可。

### 配置用户音频权限

```bash
sudo usermod -aG audio $USER
```

### 激活运行环境

```bash
# 激活 ROS 2 与虚拟环境
source /opt/bros/humble/setup.bash
source ~/asr_env/bin/activate
export PYTHONPATH=~/asr_env/lib/python3.12/site-packages/:$PYTHONPATH
```

### 查看录音设备信息

```bash
arecord -l
```

输出示例：

```
**** List of CAPTURE Hardware Devices ****
card 0: sndes8326 [snd-es8326], device 0: i2s-dai0-ES8326 HiFi ES8326 HiFi-0 []
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: XFMDPV0018 [XFM-DP-V0.0.18], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

示例中 `card 1` 为 USB 麦克风设备，后续启动时需将 `device_index` 参数设为 `1`。

## 启动语音识别

运行以下命令启动语音转文字功能：

```bash
ros2 launch rdk_perception asr_wakeword.launch.py \
  rate:=16000 \
  device_index:=1 \
  min_db:=4000
```

注意事项：

- 启动后监听唤醒词（默认为：**你好**）
- 检测到唤醒词后自动开始录音并识别语音内容
- 识别结果通过 ROS2 消息发布
- 部分 USB 麦克风仅支持 `48000 Hz`，请根据实际设备适配 `rate` 参数。

终端输出示例：

```
[wakeword_asr_node-1] jack server is not running or cannot be started
[wakeword_asr_node-1] JackShmReadWritePtr::~JackShmReadWritePtr - Init not done for -1, skipping unlock
[wakeword_asr_node-1] JackShmReadWritePtr::~JackShmReadWritePtr - Init not done for -1, skipping unlock
[wakeword_asr_node-1] compiler_depend.ts(51) LOG(INFO) precompiled_charsmap is empty. use identity normalization.
[wakeword_asr_node-1] ** 已检测到优化后模型，直接加载: /home/zq-pi/.cache/sensevoice/model_quant_optimized.onnx
[wakeword_asr_node-1] Listening for wake word...
[wakeword_asr_node-1] 检测到语音，开始录制...
[wakeword_asr_node-1] 静音超过 1.0 秒，停止录制。
[wakeword_asr_node-1] 关闭音频流
[wakeword_asr_node-1] wake check: 你好
[wakeword_asr_node-1] 唤醒词已检测到！
[wakeword_asr_node-1] 开始监听用户语音...
[wakeword_asr_node-1] 检测到语音，开始录制...
[wakeword_asr_node-1] 静音超过 1.0 秒，停止录制。
[wakeword_asr_node-1] 关闭音频流
[wakeword_asr_node-1] 用户说: 今天天气怎么样
```

## 订阅识别结果

识别结果将发布至 ROS 2 话题 `/voice_text`，其消息类型为标准 `std_msgs/msg/String`：

```bash
ros2 topic echo /voice_text
```

输出示例：

```
data: 今天天气怎么样
---
```

## 参数说明

| 参数名称       | 说明                                               | 默认值       |
| -------------- | -------------------------------------------------- | ------------ |
| `sld`          | 检测到语音后静音持续时间（秒）达到该值时结束录音   | `1.0`        |
| `min_db`       | 开始录音的音量阈值，数值越大，触发录音所需音量越大 | `2000`       |
| `max_time`     | 单次最长录音时长（秒），超时后自动返回唤醒模式     | `30`         |
| `channels`     | 音频声道数                                         | `1`          |
| `rate`         | 音频采样率（如 `16000`、`48000`）                  | `48000`      |
| `device_index` | 音频采集设备编号，通过 `arecord -l` 查询           | `1`          |
| `result_topic` | 发布识别结果的 ROS 话题名称                        | `voice_text` |
