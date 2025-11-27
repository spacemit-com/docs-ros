---
sidebar_position: 2
---

# LLM 模型聊天

```
最新版本：2025/09/12
```

## 功能简介

本模块基于本地部署的大语言模型（LLM, Large Language Model），实现自然语言处理对话能力，支持非流式服务与流式 Action 两种交互模式。

本模块可广泛应用于以下场景：

- 人机自然语言交互
- 智能问答与知识查询
- 多轮对话状态管理
- 指令意图理解与生成

## 环境准备

### 安装系统依赖

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

### 运行环境配置与依赖安装

```bash
# 安装 LLM 模型运行环境（含模型下载与配置）
bash /opt/bros/humble/share/rdk_chat/llm_setup.sh

# 创建并激活 Python 虚拟环境
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple

python3 -m venv ~/ai_env
source ~/ai_env/bin/activate

# 安装 Python 依赖
pip install -r /opt/bros/humble/share/rdk_chat/requirements.txt
```

## 非流式聊天服务接口

### 导入环境

```
source /opt/bros/humble/setup.bash
source ~/ai_env/bin/activate
export PYTHONPATH=~/ai_env/lib/python3.12/site-packages/:$PYTHONPATH
```

### 启动服务端

```bash
ros2 launch rdk_chat chat_service.launch.py
```

默认服务名为：`chat_service`，服务类型为 `jobot_interfaces/srv/LLMChat`

### 客户端调用示例

新建 `llm_client.py` 文件：

```python
import rclpy
from rclpy.node import Node
from jobot_interfaces.srv import LLMChat

class LLMClientNode(Node):
    def __init__(self):
        super().__init__('llm_client')
        self.cli = self.create_client(LLMChat, 'chat_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 LLM 服务中...')
        self.req = LLMChat.Request()

    def send_request(self, prompt_text):
        self.req.prompt = prompt_text
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = LLMClientNode()
    future = node.send_request("你好，告诉我什么是 ROS2？")
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print("模型回复:", future.result().response)
    else:
        node.get_logger().error('调用服务失败')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

执行：

```bash
source /opt/bros/humble/setup.bash
python3 llm_client.py
```

输出示例：

```
模型回复: Hello! ROS (Robot Operating System) is a set of development tools and runtime environment developed by the ROS Core Team. It provides complete system for building real-time autonomous robots.

It consists of three main components: **Core**, **Dynamic** and **Toolbox**.
- The core includes fundamental systems like sensor abstraction, command line interface, communication protocol handling, etc.
- Dynamic component is responsible to control robot execution based on user input
- Toolbox provides a set of tools for building custom modules or applications with the help of ROS.

With ROS2 you can create robots that have self-awareness and autonomous decision-making capabilities. It also has libraries like **Bullet**, which are used by ROS to simulate real-time systems, etc.
Is there anything else I should know about it?
```

## 流式聊天 Action 接口

支持实时响应输出的 LLM 聊天动作，适用于语音或多轮交互响应流式推送场景。

### 导入环境

```
source /opt/bros/humble/setup.bash
source ~/ai_env/bin/activate
export PYTHONPATH=~/ai_env/lib/python3.12/site-packages/:$PYTHONPATH
```

### 启动 Action 服务端

```bash
ros2 launch rdk_chat chat_action.launch.py
```

默认 Action 名称为 `chat_action`，类型为 `jobot_interfaces/action/LLMChatAction`。

### 客户端调用示例

新建 `llm_client_action.py` 文件：

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from jobot_interfaces.action import LLMChatAction

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
            self.get_logger().info('目标被拒绝')
            return

        self.get_logger().info('目标已被接受')
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
        prompt = "你好，请介绍一下你自己"

    client.send_prompt(prompt)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

执行：

```bash
source /opt/bros/humble/setup.bash
python3 llm_client_action.py
```

终端输出示例：

```
[INFO] [1748593600.839893117] [llm_chat_client]: 目标已被接受
我是由阿里云开发的大规模多语言模型，可以提供各种类型的信息和解答您的问题，如新闻、财经资讯、科技知识等。如果您有任何具体的问题或需要帮助的事项，请随时告诉我！

[Result] 我是由阿里云开发的大规模多语言模型，可以提供各种类型的信息和解答您的问题，如新闻、财经资讯、科技知识等。如果您有任何具体的问题或需要帮助的事项，请随时告诉我！
```

## 总结对比

| 模式       | 接口类型 | 优点                               | 场景                         |
| ---------- | -------- | ---------------------------------- | ---------------------------- |
| 非流式服务 | Service  | 简单易用，响应完整                 | 问答类应用、任务调用         |
| 流式输出   | Action   | 实时反馈，适配语音交互与长文本生成 | 多轮对话、语音播报等流式交互 |
