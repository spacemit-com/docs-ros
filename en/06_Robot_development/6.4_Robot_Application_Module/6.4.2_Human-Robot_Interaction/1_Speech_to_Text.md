

# Automatic Speech Recognition (ASR)

```
Last Version: 12/09/2025
```

## Overview

This module converts spoken audio into text using the SenseVoice ONNX model — a process known as **Automatic Speech Recognition (ASR)**

It captures audio from a microphone, performs inference on the SpacemiT computing cores, and publishes recognition results as ROS 2 messages.

The system supports a **wake word + speech recognition pipeline** out of the box, enabling voice-controlled interaction.

**Typical applications:**

- Smart home voice control
- Service robot voice command interaction
- Industrial voice assistance systems
- Touchless voice-controlled devices

## Environment Setup

### Install System Dependencies

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

### Configure Python Virtual Environment & Install Dependencies

```bash
# Set mirror source (recommend Tsinghua mirror)
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple

# Create and activate a virtual environment
python3 -m venv ~/asr_env
source ~/asr_env/bin/activate

# Install dependencies
pip install -r /opt/bros/humble/share/jobot_voice/requirements.txt
```

The `jobot_voice/` directory is pre-installed in the system and ready for use.

### Configure User Audio Permissions

```bash
sudo usermod -aG audio $USER
```

### Activate Runtime Environment

```bash
# Activate ROS 2 and Python virtual environment
source /opt/bros/humble/setup.bash
source ~/asr_env/bin/activate
export PYTHONPATH=~/asr_env/lib/python3.12/site-packages/:$PYTHONPATH
```

### Check Available Recording Devices

```bash
arecord -l
```

**Example output:**

```
**** List of CAPTURE Hardware Devices ****
card 0: sndes8326 [snd-es8326], device 0: i2s-dai0-ES8326 HiFi ES8326 HiFi-0 []
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: XFMDPV0018 [XFM-DP-V0.0.18], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

Here, `card 1` is the USB microphone device.
Assign `1` for the `device_index` in later launch commands.

## Start Speech Recognition

Launch the ASR node with:

```bash
ros2 launch br_perception asr_wakeword.launch.py \
  rate:=16000 \
  device_index:=1 \
  min_db:=4000
```

Important Notes:

- After launch, the system listens for wake words (default: **"你好/Nihao"**)
- Automatically records and recognizes speech upon wake word detection
- Recognition results are published via ROS2 messages
- Some USB microphones only support `48000` Hz - adjust the `rate` parameter accordingly

**Example terminal output:**

```
[wakeword_asr_node-1] jack server is not running or cannot be started
[wakeword_asr_node-1] JackShmReadWritePtr::~JackShmReadWritePtr - Init not done for -1, skipping unlock
[wakeword_asr_node-1] JackShmReadWritePtr::~JackShmReadWritePtr - Init not done for -1, skipping unlock
[wakeword_asr_node-1] compiler_depend.ts(51) LOG(INFO) precompiled_charsmap is empty. use identity normalization.
[wakeword_asr_node-1] Detected optimized model, loading directly: /home/zq-pi/.cache/sensevoice/model_quant_optimized.onnx
[wakeword_asr_node-1] Listening for wake word...
[wakeword_asr_node-1] Voice activity detected, starting recording...
[wakeword_asr_node-1] Silence detected for 1.0s, stopping recording.
[wakeword_asr_node-1] Closing audio stream
[wakeword_asr_node-1] wake check: Hello
[wakeword_asr_node-1] Wake word detected!
[wakeword_asr_node-1] Listening for user speech...
[wakeword_asr_node-1] Voice activity detected, starting recording...
[wakeword_asr_node-1] Silence detected for 1.0s, stopping recording.
[wakeword_asr_node-1] Closing audio stream
[wakeword_asr_node-1] User said: How's the weather today
```

## Subscribe to Recognition Results

The recognition results are published to the ROS 2 topic `/voice_text`, using the standard `std_msgs/msg/String` message type:

```bash
ros2 topic echo /voice_text
```

**Example:**

```
data: What's the weather like today?
---
```

## Parameter Description

| Parameter Name | Description | Default Value |
|---------------|------------|--------------|
| `sld` | SSilence duration (seconds) to stop recording after speech ends | `1.0` |
| `min_db` | Minimum volume threshold to start recording (higher = louder trigger)  | `2000` |
| `max_time` | Max duration (seconds) for a single recording before returning to wake mode | `30` |
| `channels` | Number of audio channels | `1` |
| `rate` | Audio sampling rate (e.g. `16000`, `48000`) | `48000` |
| `device_index` | Audio capture device ID (query via `arecord -l`) | `1` |
| `result_topic` | ROS 2 topic for recognition results | `voice_text` |
