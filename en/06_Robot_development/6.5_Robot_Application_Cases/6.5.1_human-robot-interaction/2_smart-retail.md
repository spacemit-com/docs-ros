# Smart Retail with Robotic Arm

```
Last Version: 19/09/2025
```

Watch the [Demo Video](https://archive.spacemit.com/ros2/Video_examples/smart-retail.mp4)

## Overview

This demo showcases the **K1 platform** controls an **Elephant robotic arm** for a **smart retail scenario**.
It integrates:

- **Function Call LLM**
- **YOLOv8** for product detection
- **OCR** for barcode and payment recognition

The robotic arm accurately identifies products through object detection and uses OCR for barcode scanning and checkout, providing an efficient and intelligent shopping experience.

The system supports **voice interaction**, significantly enhancing automation and convenience in shopping, and demonstrates the future intelligent retail.

## Hardware

- 1 × Development board with **K1 SoC** (with power supply)
- 1 × **Elephant Robotics myCobot 280 robotic arm** (with flange camera and vacuum pump)
- 1 × **Elephant Aikits AI kit**
- 1 × **USB microphone**
- 1 × **USB sound card or speaker**

## Software Modules

This demo consists of the following core modules:

- **Smart Product Recognition Module**
  - Combines **voice interaction** and **computer vision**.
  - Uses the **Function Call large language model** to interprets voice commands like "Pick up an orange".
  - Uses **YOLOv8** to detect the target product in the camera feed.
  - Supports detection of multiple product categories and 2D image coordinates.

- **Robotic Arm Motion Control Module**
  - Controls the robotic arm’s joints, angles, and suction pump.
  - Converts YOLOv8’s 2D image coordinates into 3D joint angles for precise movement.

- **Product Scanning Module**
  - Uses the flange camera to scan product barcodes and retrieve pricing information.

- **Payment Processing Module**
  - Uses the Paddle OCR engine to recognize payment vouchers.

## Environment Setup

Follow these steps to set up the environment for the smart retail system.

### Step 1: Download the Project Code

Clone the project repository:

```bash
git clone https://github.com/elephantrobotics/jobot-ai-elephant.git ~/
```

### Step 2: Install System Dependencies

Update the system and install required libraries:

```bash
sudo apt update
sudo apt install -y \
    spacemit-ollama-toolkit \
    portaudio19-dev \
    python3-dev \
    libopenblas-dev \
    ffmpeg \
    python3-venv \
    python3-spacemit-ort \
    libceres-dev \
    libopencv-dev
```

### Step 3: Install Python Dependencies

1. Create a Python virtual environment:

   ```bash
   virtualenv ~/asr-env
   ```

2. Configure pip to use the Spacemit mirror source:

   ```bash
   pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple
   ```

3. Install project dependencies:

   ```bash
   cd ~/jobot-ai-elephant
   source ~/asr-env/bin/activate
   pip install -r requirements.txt
   ```

### Step 4: Download and Deploy LLMs to Ollama

1. Create a directory for model storage:

   ```bash
   mkdir -p ~/models && cd ~/models
   ```

2. Download the model and configuration files

   ```bash
   # Download the Qwen2.5 0.5B model
   wget -c https://archive.spacemit.com/spacemit-ai/gguf/Qwen2.5-0.5B-Instruct-Q4_0.gguf --no-check-certificate
   wget -c https://archive.spacemit.com/spacemit-ai/modelfile/qwen2.5:0.5b.modelfile --no-check-certificate

   # Download the Qwen2.5 0.5B-FC model
   wget -c https://archive.spacemit.com/spacemit-ai/gguf/qwen2.5-0.5b-f16-elephant-fc-Q4_0.gguf --no-check-certificate
   wget -c https://archive.spacemit.com/spacemit-ai/modelfile/qwen2.5-0.5b-elephant-fc.modelfile --no-check-certificate
   ```

3. Deploy the models to Ollama:

   ```bash
   ollama create qwen2.5:0.5b -f qwen2.5:0.5b.modelfile
   ollama create qwen2.5-0.5b-elephant-fc -f qwen2.5-0.5b-elephant-fc.modelfile
   ```

4. Verify model deployment:

   ```bash
   ollama list
   ```

   Expected Output with both models listed, as shown below

   ![image-20250428153224566](../resources/smart-retail-ollama-status.png)

5. Remove the model storage directory to save space:

   ```bash
   rm -rf ~/models
   ```

### Step 5: Set Audio Permissions

Run the following command to add the current user to the `audio` group, granting access to and management of audio devices:

```bash
sudo usermod -aG audio $USER
```

## Audio Module Configuration Guide

### Step 1: Detect and Configure Microphone

List available microphone devices:

```bash
arecord -l
```

**Sample Output**:

```
**** List of CAPTURE Hardware Devices ****
card 1: Camera [USB Camera], device 0: USB Audio [USB Audio]
    Subdevices: 1/1
    Subdevice #0: subdevice #0
card 2: Camera_1 [USB 2.0 Camera], device 0: USB Audio [USB Audio]
    Subdevices: 1/1
    Subdevice #0: subdevice #0
card 3: Device [USB PnP Sound Device], device 0: USB Audio [USB Audio]
    Subdevices: 1/1
    Subdevice #0: subdevice #0
```

**Note**: Avoid selecting devices labeled "Camera" as they are typically camera microphones.

In this example, choose `card 3` as the main microphone.

Update the microphone configuration in `~/jobot-ai-elephant/smart_main_asr.py`:

```python
record_device = 3  # Recording device index, should be modified according to the actual detection results
rec_audio = RecAudioThreadPipeLine(
    vad_mode=1,
    sld=2,
    max_time=2,
    channels=1,
    rate=48000,
    device_index=record_device
)
```

### Step 2: Detect and Configure the Speaker

List available speaker devices:

```bash
aplay -l
```

Example output:

```
card 0: sndes8326 [snd-es8326], device 0: i2s-dai0-ES8326 HiFi ES8326 HiFi-0 []
    Subdevices: 1/1
    Subdevice #0: subdevice #0
card 2: Device [USB Audio Device], device 0: USB Audio [USB Audio]
    Subdevices: 1/1
    Subdevice #0: subdevice #0
```

Update the speaker configuration in the following files:

- **File 1**: `~/jobot-ai-elephant/smart_main_asr.py`

   ```python
   play_device='plughw:2,0'  # Speaker sittings
   ```

- **File 2**: `~/jobot-ai-elephant/spacemit_audio/play.py`

   ```python
   play_device='plughw:2,0'  # Speaker sittings
   ```

### Step 3: Configure Recording Parameters

Set the maximum recording duration in `~/jobot-ai-elephant/smart_main_asr.py`:

```python
rec_audio.max_time_record = 3  # Maximum recording time in seconds
```

Understand recording modes:

- By default, **non-blocking mode** is used for recording
- To wait for recording to complete, use the `join()` method

```python
# Start recording
rec_audio.max_time_record = 3
rec_audio.frame_is_append = True
rec_audio.start_recording()
rec_audio.thread.join()  # Wait for recording to complete
```

## Run the Program

```bash
cd ~/jobot-ai-elephant
source ~/asr_env/bin/activate
python smart_main_asr.py
```

Usage Instructions:

- Press **Enter** directly to enter recording mode
- Default recording duration is **3 seconds**
- Supports fuzzy command matching (e.g., "Grab an orange", "Grab an apple", "Checkout", etc.)
- Natural language commands supported by the large language model, such as:
  - "Give me an apple", "One more orange", etc.
  - Can recognize various object names
