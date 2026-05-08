sidebar_position: 4

# OCR (Optical Character Recognition)

## PaddleOCR

PaddleOCR (**PPOCR**) is an **end-to-end optical character recognition (OCR) system** open-sourced by PaddlePaddle. It provides a complete solution covering **text detection → orientation classification → text recognition → post-processing**.
It supports not only Chinese and English but also multilingual scenarios, and is widely applicable to various OCR applications like documents, tickets, guide boards, license plates, natural scenes.

This example demonstrates how to perform OCR model inference on the SpacemiT AI computing core, using images or ROS 2 messages as input, and publishing detection results via ROS 2.

## Environment Setup

It is recommended to use the ROS2_LXQT system and make sure the development dependencies are installed.

### Install Dependencies

```bash
sudo apt install python3-opencv ros-humble-cv-bridge ros-humble-camera-info-manager \
ros-humble-image-transport python3-spacemit-ort python3-pyclipper
```

### Load ROS2 Environment

```bash
source /opt/bros/humble/setup.bash
```

## Image Inference

 **Prepare images**

```bash
cp /opt/bros/humble/share/jobot_ppocr_py/data/test.jpg .
```

### Save Inference Results Locally

```bash
ros2 launch rdk_perception ocr_infer_img.launch.py img_path:=/home/bianbu/test.jpg
```

The output result will be saved in the current directory as `ocr_result.jpg`, as shown in the following figure.

![](./images/ocr1.png)

The terminal prints:

![](./images/ocr2.png)

### Parameter Description

**`ocr_infer_img.launch.py` Parameter Description**

| **Parameter Name** | Role                 | Default Value        |
| ------------ | -------------------- | ------------- |
| `img_path`     | inference image path | `data/test.jpg` |

## Inference Service Usage

The inference service accepts raw image messages and returns OCR inference results. You can view the service definition via  `ros2 interface show jobot_interfaces/srv/OCRInfer`.

### Launch the Service

```
ros2 launch rdk_perception ocr_service.launch.py
```

The terminal prints:

![](./images/ocr3.png)

### Client Code

Write client code:

```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from jobot_interfaces.srv import OCRInfer
import cv2
import numpy as np


class OCRClient(Node):
    def __init__(self):
        super().__init__('ocr_client')
        self.client = self.create_client(OCRInfer, 'ocr_infer')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for OCR service...')
        self.get_logger().info('OCR service connected.')

    def send_request(self, img_path):
        req = OCRInfer.Request()

        img = cv2.imread(img_path)
        img_msg = Image()
        img_msg.height, img_msg.width, _ = img.shape
        img_msg.encoding = 'bgr8'
        img_msg.step = img_msg.width * 3
        img_msg.data = img.tobytes()

        req.image = img_msg

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        print(f"Detected {len(resp.result.boxes)} boxes.")
        result_img = self.ros_img_to_cv2(resp.result_img)
        cv2.imwrite('ocr_result_srv.jpg', result_img)
        print("Saved OCR result image to ocr_result_srv.jpg")

    def ros_img_to_cv2(self, ros_img: Image):
        img = np.frombuffer(ros_img.data, dtype=np.uint8).reshape(ros_img.height, ros_img.width, 3)
        return img


def main():
    rclpy.init()
    client = OCRClient()
    client.send_request('test.jpg')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make sure the image path is correct.

### Request Service

Save the client code as `ocr_client.py`.

Then, execute:

```
python3 ocr_client.py
```

The terminal prints:

![](./images/ocr4.png)

The visualized result file is saved in `ocr_result_srv.jpg`.
