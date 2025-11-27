# ROS2 MIPI Camera 节点 API

---

## 1. 概览

- **节点组件**：`camera_node`（采集）、`venc_node`（硬编）、`vdec_node`（硬解）、`vo_node`（显示）、`infer_node`（推理）  
- **核心消息**：`jdk_interfaces/msg/JdkFrameMsg` 在各节点之间传递帧（含 DMA/FD 方式）

### 1.1 节点与话题总览

| 节点         | 订阅话题         | 发布话题          | 说明 |
|--------------|------------------|-------------------|------|
| `camera_node`| —                | `/camera_frames`  | 采集 NV12 帧并发布，支持 FD/DMA 共享；可提供 FD 套接字服务。 |
| `venc_node`  | `/camera_frames` | `/encoded_frames` | 将原始帧硬件编码为 H.264/H.265。 |
| `vdec_node`  | `/encoded_frames`| `/decoded_frames` | 将编码帧解码为 NV12。 |
| `vo_node`    | `/camera_frames` | —                 | 硬件直出显示画面（JdkVo）。 |
| `infer_node` | `/camera_frames` | —（打印/自处理）  | 对相机帧进行 YOLOv8 目标检测（示例模型）。 |

> 注：所有消息字段（如 `width/height/stride/pix_format/is_dma/dma_fd/data` 等）随 `JdkFrameMsg` 传递。

---

## 2. `camera_node`（相机采集）

### 2.1 功能说明
打开 V4L2 设备采集 NV12 帧，发布到 `/camera_frames`；同时启动 UNIX Socket（`socket_path`）用于 FD 传输（零拷贝）。

### 2.2 订阅的话题
- 无

### 2.3 发布的话题
- `/camera_frames` (`jdk_interfaces/msg/JdkFrameMsg`)：NV12 或 FD 共享帧。

### 2.4 参数列表

| 参数名         | 类型   | 默认值                   | 说明 |
|----------------|--------|--------------------------|------|
| `device`       | string | `"/dev/video50"`         | V4L2 设备路径（如 `/dev/video0`）。 |
| `width`        | int    | `1280`                   | 帧宽。 |
| `height`       | int    | `720`                    | 帧高。 |
| `socket_path`  | string | `"/tmp/jdk_fd_socket"`   | FD 共享套接字路径。 |

### 2.5 启动示例
```bash
ros2 run jdk_camera_node camera_node --ros-args \
  -p device:=/dev/video50 -p width:=1280 -p height:=720
```

### 2.6 订阅示例
订阅`/camera_frames`，详细见 §6 通用订阅示例。

---

## 3. `venc_node`（硬件编码）

### 3.1 功能说明
订阅 `/camera_frames`，将原始帧硬件编码为 H.264/H.265，发布到 `/encoded_frames`；支持 DMA/FD。

### 3.2 订阅的话题
- `/camera_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 3.3 发布的话题
- `/encoded_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 3.4 参数列表

| 参数名            | 类型   | 默认值                        | 说明 |
|-------------------|--------|-------------------------------|------|
| `width`           | int    | `1920`                        | 输入帧宽。 |
| `height`          | int    | `1080`                        | 输入帧高。 |
| `payload`         | int    | `CODING_H264`                 | 编码格式（H.264/H.265）。 |
| `format`          | int    | `PIXEL_FORMAT_NV12`           | 输入像素格式。 |
| `use_dma`         | bool   | `true`                        | 是否使用 DMA。 |
| `venc_fd_socket`  | string | `"/tmp/jdk_fd_enc2out.sock"`  | 编码输出 FD 套接字。 |
| （内部订阅套接字）| —      | `"/tmp/jdk_fd_socket"`        | 通常不需修改。 |

### 3.5 启动示例
```bash
ros2 run jdk_venc_node venc_node
```

---

## 4. `vdec_node`（硬件解码）

### 4.1 功能说明
订阅 `/encoded_frames`，解码至 NV12 并发布 `/decoded_frames`；可与编码节点通过 FD 套接字衔接。

### 4.2 订阅的话题
- `/encoded_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 4.3 发布的话题
- `/decoded_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 4.4 参数列表

| 参数名           | 类型   | 默认值                       | 说明 |
|------------------|--------|------------------------------|------|
| `width`          | int    | `1920`                       | 解码后帧宽。 |
| `height`         | int    | `1080`                       | 解码后帧高。 |
| `payload`        | int    | `CODING_H264`                | 输入码流类型。 |
| `format`         | int    | `PIXEL_FORMAT_NV12`          | 输出像素格式。 |
| `use_dma`        | bool   | `true`                       | 是否使用 DMA。 |
| `dec_fd_socket`  | string | `"/tmp/jdk_fd_dec2out.sock"` | 解码输出 FD 套接字。 |
| `venc_fd_socket` | string | `"/tmp/jdk_fd_enc2out.sock"` | 对接编码节点输出。 |

### 4.5 启动示例
```bash
ros2 run jdk_vdec_node vdec_node
```

---

## 5. `vo_node`（硬件显示）

### 5.1 功能说明
订阅 `/camera_frames`，使用 `JdkVo` 进行硬件直出显示。

### 5.2 订阅的话题
- `/camera_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 5.3 发布的话题
- 无

### 5.4 参数列表

| 参数名          | 类型   | 默认值                  | 说明 |
|-----------------|--------|-------------------------|------|
| `width`         | int    | `1920`                  | 输入帧宽。 |
| `height`        | int    | `1080`                  | 输入帧高。 |
| `format`        | int    | `PIXEL_FORMAT_NV12`     | 输入像素格式。 |
| `use_dma`       | bool   | `true`                  | 是否使用 DMA。 |
| `cam_fd_socket` | string | `"/tmp/jdk_fd_socket"`  | 摄像头 FD 套接字路径。 |

### 5.5 启动示例
```bash
ros2 run jdk_vo_node vo_node
```

---

## 6. `infer_node`（目标检测推理）

### 6.1 功能说明
订阅 `/camera_frames`，示例使用 YOLOv8（`yolov8n.q.onnx`）进行目标检测，并在终端打印结果。

### 6.2 订阅的话题
- `/camera_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 6.3 发布的话题
- 无（示例为日志/自处理）。

### 6.4 参数列表

| 参数名        | 类型   | 默认值                 | 说明 |
|---------------|--------|------------------------|------|
| `socket_path` | string | `"/tmp/jdk_fd_socket"` | 获取帧的 FD 套接字路径。 |

### 6.5 启动示例
```bash
ros2 run jdk_infer_node infer_node
```

---

## 7. 统一订阅示例

### 7.1 Python 订阅 NV12 并显示（非 DMA）

```python
import rclpy
from rclpy.node import Node
from jdk_interfaces.msg import JdkFrameMsg
import numpy as np
import cv2

class CameraListener(Node):
    def __init__(self):
        super().__init__('cam_listener')
        self.sub = self.create_subscription(
            JdkFrameMsg, 'camera_frames', self.callback, 10)

    def callback(self, msg):
        if not msg.is_dma:
            w, h = msg.width, msg.height
            nv12 = np.frombuffer(msg.data, dtype=np.uint8)
            yuv  = nv12.reshape((h * 3 // 2, w))
            bgr  = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            cv2.imshow('Camera', bgr)
            cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 8. 常见问题（FAQ，精要）

- **如何切换 H.265？** 将 `venc_node.payload` 设置为 H.265 对应编码类型。  
- **如何启用零拷贝？** 各节点 `use_dma=true` 且通过 FD 套接字互通（`socket_path` / `*_fd_socket`）。  
- **显示节点不出图？** 确认 `vo_node` 订阅的是 `/camera_frames`，并与 `camera_node` 的分辨率/格式匹配。
