

# LLM Chat Module

```
Last Version: 12/09/2025
```

## Overview

This module enables **natural language conversation** using a locally deployed **Large Language Model (LLM)**.

It supports two modes of interaction:

- **Non-streaming service**
- **Streaming Action**

Typical application scenarios include:

- Human-machine natural language interaction
- Intelligent Q&A and knowledge query
- Multi-turn dialogue state management
- Command intent understanding and generation

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

### Runtime Environment Configuration & Dependency Installation

```bash
# Install LLM model runtime environment (includes model download and configuration)
bash /opt/bros/humble/share/br_chat/llm_setup.sh

# Create and activate Python virtual environment
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple

python3 -m venv ~/ai_env
source ~/ai_env/bin/activate

# Install Python dependencies
pip install -r /opt/bros/humble/share/br_chat/requirements.txt
```

## Non-Streaming Chat Service

### Import Environment

```bash
source /opt/bros/humble/setup.bash
source ~/ai_env/bin/activate
export PYTHONPATH=~/ai_env/lib/python3.12/site-packages/:$PYTHONPATH
```

### Start the Server

```bash
ros2 launch br_chat chat_service.launch.py
```

- **Service name:** `chat_service`
- **Service type:** `jobot_ai_msgs/srv/LLMChat`

### Client Call Example

Create a new file named `llm_client.py`:

```python
import rclpy
from rclpy.node import Node
from jobot_ai_msgs.srv import LLMChat

class LLMClientNode(Node):
    def __init__(self):
        super().__init__('llm_client')
        self.cli = self.create_client(LLMChat, 'chat_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for LLM service...')
        self.req = LLMChat.Request()

    def send_request(self, prompt_text):
        self.req.prompt = prompt_text
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = LLMClientNode()
    future = node.send_request("Hello, tell me what ROS2 is?")
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print("Model response:", future.result().response)
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run:

```bash
python3 llm_client.py
```

**Example output:**

```
Model response: Hello! ROS (Robot Operating System) is a set of development tools and runtime environment developed by the ROS Core Team. It provides complete system for building real-time autonomous robots.

It consists of three main components: **Core**, **Dynamic** and **Toolbox**.
- The core includes fundamental systems like sensor abstraction, command line interface, communication protocol handling, etc.
- Dynamic component is responsible to control robot execution based on user input
- Toolbox provides a set of tools for building custom modules or applications with the help of ROS.

With ROS2 you can create robots that have self-awareness and autonomous decision-making capabilities. It also has libraries like **Bullet**, which are used by ROS to simulate real-time systems, etc.
Is there anything else I should know about it?
```

## Streaming Chat Action

Supports real-time, streaming-response LLM chat actions, ideal for voice or multi-turn interactive scenarios that require incremental output.

### Import Environment

```
source /opt/bros/humble/setup.bash
source ~/ai_env/bin/activate
export PYTHONPATH=~/ai_env/lib/python3.12/site-packages/:$PYTHONPATH
```

### Start the Action Server

```bash
ros2 launch br_chat chat_action.launch.py
```

By default
- **Action name:** `chat_action`
- **Action type:** `jobot_ai_msgs/action/LLMChatAction`

### Client Call Example

Create a new `llm_client_action.py` file:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from jobot_ai_msgs.action import LLMChatAction

class LLMChatClient(Node):
    def __init__(self):
        super().__init__('llm_chat_client')
        self._action_client = ActionClient(self, LLMChatAction, 'chat_action')

    def send_prompt(self, prompt):
        goal_msg = LLMChatAction.Goal()
        goal_msg.prompt = prompt

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f'{feedback.partial}', end='', flush=True)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        print(f'\n\n[Result] {result.response}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = LLMChatClient()

    import sys
    if len(sys.argv) > 1:
        prompt = ' '.join(sys.argv[1:])
    else:
        prompt = "Hello, please introduce yourself"

    client.send_prompt(prompt)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

Run:

```bash
python3 llm_client_action.py
```

**Example terminal output:**

```
[INFO] [1748593600.839893117] [llm_chat_client]: Goal accepted
I am a large-scale multilingual model developed by Alibaba Cloud. I can provide various types of information and answer your questions—such as news, financial updates, tech knowledge, and more. If you have any specific questions or need help with anything, feel free to let me know!

[Result] I am a large-scale multilingual model developed by Alibaba Cloud. I can provide various types of information and answer your questions—such as news, financial updates, tech knowledge, and more. If you have any specific
```

## Summary

| Mode | Interface Type | Advantages | Use Cases |
|------|---------------|------------|-----------|
| Non-streaming | Service |  Easy to implement, complete response | Q&A applications, task calls |
| Streaming Output | Action | Real-time feedback, suitable for voice interaction and long text generation | Multi-turn dialogues, voice broadcasting and other streaming interactions |
