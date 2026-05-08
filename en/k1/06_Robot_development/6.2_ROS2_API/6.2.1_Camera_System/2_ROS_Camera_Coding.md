sidebar_position: 2

# ROS2 MIPI Camera Node API

## 1. Overview

- **Node components**：`camera_node`（acquisition）、`venc_node`（hard-coded）、`vdec_node`（hard-decoded）、`vo_node`（display）、`infer_node`（inference）  
- **Core message**：`jdk_interfaces/msg/JdkFrameMsg` transfers frames between nodes (including DMA/FD mode)

### 1.1 Overview of nodes and topics

| Node         | Subscribe to topic         | Publish topic          | Description |
|--------------|------------------|-------------------|------|
| `camera_node`| —                | `/camera_frames`  | Collect NV12 frames and publish them, support FD/DMA sharing; can provide FD socket service. |
| `venc_node`  | `/camera_frames` | `/encoded_frames` | Hardware encode raw frames to H.264/H.265. |
| `vdec_node`  | `/encoded_frames`| `/decoded_frames` | Decode encoded frames to NV12. |
| `vo_node`    | `/camera_frames` | —                 | Hardware direct display screen (JdkVo). |
| `infer_node` | `/camera_frames` | —(print/self-processing)  | YOLOv8 object detection on camera frames (example model). |

> Note: All message fields (such as `width/height/stride/pix_format/is_dma/dma_fd/data`, etc.) are passed with `JdkFrameMsg`.

## 2. `camera_node`(Camera Capture)

### 2.1 Function Description
Opens a V4L2 device to capture NV12 frames and publishes them to `/camera_frames`; simultaneously launches a UNIX Socket (`socket_path`) for File Descriptor (FD) transfer (zero-copy).

### 2.2 Subscribed Topics
- None

### 2.3 Published Topics
- `/camera_frames` (`jdk_interfaces/msg/JdkFrameMsg`): NV12 or FD-shared frames.

### 2.4 Parameter List

| Parameter Name         | Type   | Default Value                  | Description |
|----------------|--------|--------------------------|------|
| `device`       | string | `"/dev/video50"`         |V4L2 device path (e.g., `/dev/video0`). |
| `width`        | int    | `1280`                   | Frame width. |
| `height`       | int    | `720`                    | Frame height. |
| `socket_path`  | string | `"/tmp/jdk_fd_socket"`   | FD sharing socket path. |

### 2.5 Launch Example
```bash
ros2 run jdk_camera_node camera_node --ros-args \
  -p device:=/dev/video50 -p width:=1280 -p height:=720
```

### 2.6 Subscription Example
Subscribe to `/camera_frames`; for details, see §6 General Subscription Examples.

## 3. `venc_node`（Hardware Encoding）

### 3.1 Function Description
Subscribes to `/camera_frames`, hardware-encodes the raw frames into H.264/H.265 format, and publishes them to `/encoded_frames`; supports DMA/FD.

### 3.2 Subscribed Topics
- `/camera_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 3.3 Published Topics
- `/encoded_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 3.4 Parameter List

| Parameter Name            | Type   | Default Value                      | Description |
|-------------------|--------|-------------------------------|------|
| `width`           | int    | `1920`                        | Input frame width. |
| `height`          | int    | `1080`                        | Input frame height. |
| `payload`         | int    | `CODING_H264`                 | Encoding format (H.264/H.265). |
| `format`          | int    | `PIXEL_FORMAT_NV12`           | Input pixel format. |
| `use_dma`         | bool   | `true`                        | Whether to use DMA. |
| `venc_fd_socket`  | string | `"/tmp/jdk_fd_enc2out.sock"`  | Encoding output FD socket. |
| (Internal Subscription Socket)| —      | `"/tmp/jdk_fd_socket"`        | Typically does not require modification. |

### 3.5 Launch Example
```bash
ros2 run jdk_venc_node venc_node
```

## 4. `vdec_node`（Hardware Decoding）

### 4.1 Function Description
Subscribes to `/encoded_frames`, decodes the data into NV12 format, and publishes it to `/decoded_frames`; can be interfaced with the encoding node via an FD socket.

### 4.2 Subscribed Topics
- `/encoded_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 4.3 Published Topics
- `/decoded_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 4.4 Parameter List

| Parameter Name           | Type   | Default Value                      | Description |
|------------------|--------|------------------------------|------|
| `width`          | int    | `1920`                       | Width of the decoded frame. |
| `height`         | int    | `1080`                       | Height of the decoded frame. |
| `payload`        | int    | `CODING_H264`                | Input bitstream type. |
| `format`         | int    | `PIXEL_FORMAT_NV12`          | Output pixel format. |
| `use_dma`        | bool   | `true`                       | Whether to use DMA. |
| `dec_fd_socket`  | string | `"/tmp/jdk_fd_dec2out.sock"` | FD socket for decoder output. |
| `venc_fd_socket` | string | `"/tmp/jdk_fd_enc2out.sock"` | Interface for the encoding node's output. |

### 4.5 Launch Example
```bash
ros2 run jdk_vdec_node vdec_node
```

## 5. `vo_node`（Hardware Display）

### 5.1 Function Description
Subscribes to `/camera_frames` and utilizes `JdkVo` for direct hardware display output.

### 5.2 Subscribed Topics
- `/camera_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 5.3 Published Topics
- None

### 5.4 Parameter List

| Parameter Name          | Type   | Default Value                  | Description |
|-----------------|--------|-------------------------|------|
| `width`         | int    | `1920`                  | Input frame width. |
| `height`        | int    | `1080`                  | Input frame height. |
| `format`        | int    | `PIXEL_FORMAT_NV12`     | Input pixel format. |
| `use_dma`       | bool   | `true`                  | Whether to use DMA. |
| `cam_fd_socket` | string | `"/tmp/jdk_fd_socket"`  | Camera FD socket path. |

### 5.5 Launch Example
```bash
ros2 run jdk_vo_node vo_node
```

## 6. `infer_node`（Object Detection Inference）

### 6.1 Function Description
Subscribes to `/camera_frames`. This example uses YOLOv8 (`yolov8n.q.onnx`) to perform object detection and prints the results to the terminal.

### 6.2 Subscribed Topics
- `/camera_frames` (`jdk_interfaces/msg/JdkFrameMsg`)。

### 6.3 Published Topics
- None (This example focuses on logging/internal processing).

### 6.4 Parameter List

| Parameter Name        | Type   | Default Value                 | Description |
|---------------|--------|------------------------|------|
| `socket_path` | string | `"/tmp/jdk_fd_socket"` | The path to the FD socket used for retrieving frames. |

### 6.5 Launch Example
```bash
ros2 run jdk_infer_node infer_node
```


## 7. Unified Subscription Example

### 7.1 Python Subscription to NV12 and Display (Non-DMA)

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

## 8. Frequently Asked Questions (FAQ - Highlights)

- **How to switch to H.265?** Set `venc_node.payload` to the encoding type corresponding to H.265.  
- **How to enable zero copy?** Ensure that `use_dma=true` for each node and that they communicate via FD sockets (`socket_path` / `*_fd_socket`).  
- **The display node isn't showing any images?** Verify that `vo_node` is subscribed to `/camera_frames` and that its resolution/format matches that of `camera_node`.
