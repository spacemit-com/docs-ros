sidebar_position: 5

# YOLOE Object Detection

## Introduction

[YOLOE (Real-Time Seeing Anything)](https://arxiv.org/html/2503.07465v1) is a new advancement in zero-shot, promptable YOLO models. It is designed for "open-vocabulary" detection and segmentation. Unlike previous YOLO models restricted to fixed categories, YOLOE supports prompts from text, images, or an internal vocabulary, enabling real-time detection of arbitrary object classes. Built on YOLOv10 and inspired by [YOLO-World](https://docs.ultralytics.com/zh/models/yolo-world/), YOLOE achieves **state-of-the-art zero-shot performance** with minimal impact on speed and accuracy.

This example demonstrates how to perform inference with the YOLOE model on the SpacemiT AI computing core, using image or video streams as input, and publishing detection results via ROS 2.

## Environment Setup

### Dependency Installation

```bash
sudo apt install python3-venv python3-pip ros-humble-camera-info-manager \
ros-humble-image-transport python3-spacemit-ort
```

### ROS 2 Environment Setup

```bash
source /opt/bros/humble/setup.bash
```

### Virtual Environment Setup

```
python3 -m venv ~/yoloe_env
source ~/yoloe_env/bin/activate
pip install -r /opt/bros/humble/share/jobot_yoloe_py/data/requirements.txt
```

```
export PYTHONPATH="$HOME/yoloe_env/lib/python3.12/site-packages":$PYTHONPATH
```

## Image Inference

### Image Preparation

```bash
cp /opt/bros/humble/share/jobot_yoloe_py/data/bus.jpg .
```

### Save Inference Results Locally

```bash
ros2 launch rdk_perception yoloe_infer_img.launch.py \
img_path:=/home/bianbu/bus.jpg \
text_prompt:="A person wearing off-white clothes"
```

The output result will be saved in `yoloe_result.jpg` in the current directory, as shown in the following figure:

![](images/yoloe1.png)

The terminal prints:

![](./images/yoloe2.png)

### Web Visualization for Inference Results

Launch the inference publishing node (terminal 1):

```bash
ros2 launch rdk_perception yoloe_infer_img.launch.py \
publish_result_img:=true \
img_path:=/home/bianbu/bus.jpg \
text_prompt:="A person wearing off-white clothes"
```

Launch the web visualization service (terminal 2):

```bash
ros2 launch rdk_visualization websocket_cpp.launch.py image_topic:='/result_img'
```

The terminal displays the access address:

```
...
Please visit in your browser: http://<IP>:8080
...
```

Enter `http://<IP>:8080` in your browser to view real-time inference image results.

You can also specify the port number by appending the `port:=xxxx` parameter to avoid a port conflict.

![](images/yoloe3.png)

### Results Subscription

Run `ros2 topic echo /inference_result` to view the inference result topic.

Use the following code to implement a simple topic subscription:

```
from rclpy.node import Node
from std_msgs.msg import Header
from jobot_interfaces.msg import DetectionResultArray, DetectionResult
import rclpy

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_sub')
        self.subscription = self.create_subscription(
            DetectionResultArray,
            '/inference_result',
            self.listener_callback,
            10)

    def listener_callback(self, msg: DetectionResultArray):
        self.get_logger().info(f"Frame: {msg.header.frame_id}")
        for det in msg.results:
            self.get_logger().info(
                f"[{det.label}] ({det.x_min},{det.y_min}) "
                f"{det.width}x{det.height} conf={det.conf:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = DetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

main()
```

### Parameter Description

Parameter Description of **`yoloe_infer_img.launch.py`**

|    **Parameter Name**    |                             Role                             |      Default Value       |
| :----------------: | :----------------------------------------------------------: | :---------------: |
|      `img_path`      |                    Image path used for inference                     |   `data//bus.jpg`   |
| `publish_result_img` |               Whether to publish inference result in the form of image messages                |       `false`       |
|  `result_img_topic`  |   Name of the published rendered image topic; valid only when `publish_result_img` is `true`    |    `/result_img`    |
|    `result_topic`    |                    Name of the published inference result topic                      | `/inference_result` |
|   `conf_threshold`   |       Controls the **minimum confidence threshold for detection results** (filters out low-score boxes) |        `0.2`        |
|   `iou_threshold`    |          Controls the strictness of the **deduplication rule** (handles overlapping  boxes)           |        `0.7`        |
|    `text_prompt`     | Controls the **scope of detection targets** (standard categories or natural language descriptions of objects, separated by `,` for multiple categories) |     "person"      |

## Inference Service Usage

The inference service accepts raw image messages and text prompts and returns YOLOE inference results. You can view the service definition with `ros2 interface show jobot_interfaces/srv/YOLOEInfer`.

### Launch the Service

```
source ~/yoloe_env/bin/activate
export PYTHONPATH="$HOME/yoloe_env/lib/python3.12/site-packages":$PYTHONPATH
```

```
ros2 launch rdk_perception yoloe_service.launch.py
```

The terminal prints:

![](./images/yoloe4.png)

### Client Code

Write client code:

```
import rclpy
from rclpy.node import Node
from jobot_interfaces.srv import YOLOEInfer
from sensor_msgs.msg import Image
import cv2
from jobot_perception_utils_py.cv_bridge import CvBridge

class TestClient(Node):
    def __init__(self):
        super().__init__('yoloe_test_client')
        self.cli = self.create_client(YOLOEInfer, '/yoloe_infer')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service /yoloe_infer...')
        self.req = YOLOEInfer.Request()
        self.bridge = CvBridge()

    def send_request(self, img_path, text_prompt="person"):
        img = cv2.imread(img_path)
        self.req.image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.req.text_prompt = text_prompt
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = TestClient()
    resp = client.send_request('bus.jpg', 'person')
    print(f"Detected {len(resp.detections.results)} objects")

    for box in resp.detections.results:
        print(f'x_min:{box.x_min}, y_min:{box.y_min}, width:{box.width}, height:{box.height}, label:{box.label}, confidence:{box.conf:.2f}')

    # ROS Image -> OpenCV
    img_result = client.bridge.imgmsg_to_cv2(resp.result_img, desired_encoding='bgr8')

    cv2.imwrite('yoloe_service_result.jpg', img_result)
    print("The YOLOE result is saved as: yoloe_service_result.jpg")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

```

**Note:** Make sure the image path is correct.

### Request Service

Save the client code as `yoloe_client.py`.

Then, execute:

```
source ~/yoloe_env/bin/activate
export PYTHONPATH="$HOME/yoloe_env/lib/python3.12/site-packages":$PYTHONPATH
```

```
python3 yoloe_client.py
```

The terminal prints:

![](./images/yoloe5.png)

The visualization result file is saved as `yoloe_service_result.jpg`.

### Parameter Description

**Parameter description of `yoloe_service.launch.py`**

|  **Parameter Name**  |                      Role                      | Default Value |
| :------------: | :--------------------------------------------: | :----: |
| `conf_threshold` | Controls the **minimum confidence threshold for detection results** (filters out low-confidence bounding boxes) |  `0.2`   |
| `iou_threshold`  | Controls the strictness of the **deduplication rule** (handles overlapping bounding boxes)    |  `0.7`   |