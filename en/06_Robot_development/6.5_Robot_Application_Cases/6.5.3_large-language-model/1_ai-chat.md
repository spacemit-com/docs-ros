# AI Chatbot

```
Last Version: 19/09/2025
```

Watch [SpacemiT Official Video](https://archive.spacemit.com/ros2/Video_examples/ai-chat.mp4)

## Overview

This demo showcases a fully local intelligent voice assistant system.
It integrates **speech recognition**, **natural language processing (NLP)**, and **speech synthesis** into one workflow.

All AI models are converted to the **ONNX format** and optimized through quantization. With the **K1** chip’s hardware acceleration, the system ensures fast response while maintaining high-quality interactions.

## Hardware

### Supported Development Boards

- Muse Book
- Muse Pi
- Muse Pi Pro
- Banana Pi（K1 version）

### System Requirements

- **Operating System:** Bianbu OS 2.0 or higher
- **Storage:** At least 8GB available
- **Memory:** 4GB or more recommended

## Core Modules

This project includes the following core modules:

- **Audio Capture Module**
   Captures audio input via an external USB microphone.

- **ASR Module (Automatic Speech Recognition)**
   Locally deploys ASR models to convert speech to text for subsequent processing.

- **LLM Module (Large Language Model)**
   Deploys the Deepseek R1 model locally to process and understand text input, then generate responses.

- **TTS Module (Text-to-Speech)**
   Runs a local speech synthesis model to convert text responses from the LLM into spoken output.

- **Audio Playback Module**
   Outputs audio through an external speaker to complete the interaction loop.

## Environment Setup

### Step 1: Download the Code

[asr-llm-tts.zip](https://archive.spacemit.com/ros2/code/asr-llm-tts.zip)

Extract the files:

```bash
unzip asr-llm-tts.zip -d ~/
```

### Step 2: Install System Dependencies

Update the system and install required packages:

```bash
sudo apt update
sudo apt install -y python3-spacemit-ort spacemit-ollama-toolkit virtualenv wget
```

### Step 3: Install Python Dependencies

1. Create a Python Virtual Environment:

   ```bash
   virtualenv ~/demo-env
   ```

2. Configure pip to Use Spacemit Mirror Source:

   ```bash
   pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple
   ```

3. Install project dependencies

   ```bash
   cd ~/asr-llm-tts
   source ~/demo-env/bin/activate
   pip install -r requirements.txt
   ```

### Step 4: Deploy the LLM Model to Ollama

1. **Ensure Ollama Service is Running:**
   - Verify the service status

     ```bash
     systemctl status ollama
     ```

   - Expected output:
  ![image-20250422140656034](../resources/ai-chat-ollama-status.png)

   - If the status is `inactive`, start the service:

     ```bash
     systemctl start ollama
     ```

2. **Download the Deepseek model and configuration files**

   ```bash
   mkdir -p ~/my-ollama
   cd ~/my-ollama
   wget https://archive.spacemit.com/spacemit-ai/openwebui/deepseek-r1-1.5b.modelfile
   wget https://www.modelscope.cn/models/ggml-org/DeepSeek-R1-Distill-Qwen-1.5B-Q4_0-GGUF/resolve/master/deepseek-r1-distill-qwen-1.5b-q4_0.gguf
   ```

3. **Load the model into Ollama**

   ```bash
   ollama create deepseek-r1-1.5b -f deepseek-r1-1.5b.modelfile
   ```

4. **Copy NLTK data**

   ```bash
   cp -r ~/asr-llm-tts/nltk_data ~/
   ```

## Launch Commands

- Navigate to the project directory and activate the virtual environment

   ```bash
   cd ~/asr-llm-tts/src
   source ~/demo-env/bin/activate
   ```

- Launch the voice output version

   ```python
   python main_tts.py
   ```

- Launch the text output version

   ```
   python main.py
   ```
