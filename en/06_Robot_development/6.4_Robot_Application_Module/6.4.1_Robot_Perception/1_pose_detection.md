sidebar_position: 1

# Human Pose Detection

## YOLOv8-Pose Model Overview

YOLOv8-Pose is a **Pose Estimation Model** from Ultralytics' YOLOv8 series. It detects human keypoints (like head, shoulders, knees) and builds on YOLOv8's architecture for fast, accurate 2D pose estimation. Typical applications include:

- Security surveillance
- Sports analytics
- Human-computer interaction
- Robotics

This guide shows how to:
1. Run YOLOv8-Pose inference on images or video streams using SpacemiT hardware
2. Publish detection results (bounding boxes + human keypoints) via ROS 2

## Environment Setup

### Install Dependencies

```bash
sudo apt install python3-opencv ros-humble-cv-bridge ros-humble-camera-info-manager \
ros-humble-image-transport python3-spacemit-ort
```

### Source ROS2 Environment

```bash
source /opt/bros/humble/setup.bash
```

## Check Available Models

List available pose estimation models:

```bash
ros2 launch br_perception infer_info.launch.py | grep 'pose'
```

Example output:

```
[list-1]   - config/pose/yolov8_pose.yaml
```

For subsequent inference tasks, set the `config_path` parameter to the corresponding `.yaml` file path to use that specific pose estimation model.

## Human Keypoint Definitions

The model detects these keypoints as:

![](./images/keypoints_def.jpg)

Keypoint indices and names:

```
0. Nose
1. Left eye
2. Right eye
3. Left ear
4. Right ear
5. Left shoulder
6. Right shoulder
7. Left elbow
8. Right elbow
9. Left wrist
10. Right wrist
11. Left hip
12. Right hip
13. Left knee
14. Right knee
15. Left ankle
16. Right ankle
```

## Image Inference

**Prepare a Test Image**

```bash
cp /opt/bros/humble/share/jobot_infer_py/data/detection/test.jpg .
```

### Run Inference and Save Results Locally

```bash
ros2 launch br_perception infer_img.launch.py \
  config_path:='config/pose/yolov8_pose.yaml' \
  img_path:='./test.jpg'
```

Results are saved as `pose_result.jpg` in the current directory:

![](images/pose_result.jpg)

Terminal outputs detailed detection information:

```bash
bianbu@bianbu:~$ ros2 launch br_perception infer_img.launch.py   config_path:='config/pose/yolov8_pose.yaml'   img_path:='./test.jpg'
[INFO] [launch]: All log files can be found below /home/bianbu/.ros/log/2025-07-29-11-38-09-728349-bianbu-13624
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [infer_img_node-1]: process started with pid [13631]
[infer_img_node-1] All model files already exist and do not need to be downloaded
[infer_img_node-1] all time cost:0.11103701591491699
[infer_img_node-1]
[infer_img_node-1] Person 0, [xmin:0, ymin:203, width:87, height:151], conf 0.42
[infer_img_node-1]   nose: (49.0, 131.0), vis=1.32
[infer_img_node-1]   left_eye: (55.0, 131.0), vis=1.32
[infer_img_node-1]   right_eye: (49.0, 123.0), vis=1.32
[infer_img_node-1]   left_ear: (63.0, 131.0), vis=1.32
[infer_img_node-1]   right_ear: (49.0, 131.0), vis=0.00
[infer_img_node-1]   left_shoulder: (63.0, 150.0), vis=1.32
[infer_img_node-1]   right_shoulder: (41.0, 150.0), vis=1.32
[infer_img_node-1]   left_elbow: (55.0, 191.0), vis=1.32
[infer_img_node-1]   right_elbow: (16.0, 174.0), vis=1.32
[infer_img_node-1]   left_wrist: (16.0, 183.0), vis=1.32
[infer_img_node-1]   right_wrist: (16.0, 174.0), vis=1.32
[infer_img_node-1]   left_hip: (63.0, 230.0), vis=1.32
[infer_img_node-1]   right_hip: (41.0, 230.0), vis=1.32
[infer_img_node-1]   left_knee: (63.0, 288.0), vis=1.32
[infer_img_node-1]   right_knee: (41.0, 288.0), vis=1.32
[infer_img_node-1]   left_ankle: (80.0, 335.0), vis=1.32
[infer_img_node-1]   right_ankle: (49.0, 335.0), vis=1.32
[infer_img_node-1]
[infer_img_node-1] Person 1, [xmin:1, ymin:112, width:39, height:238], conf 0.85
```

### Web-Based Visualization

**Terminal 1** - Start inference node:

```bash
ros2 launch br_perception infer_img.launch.py \
  config_path:='config/pose/yolov8_pose.yaml' \
  img_path:='./test.jpg' \
  publish_result_img:=true \
  result_img_topic:='result_img' \
  result_topic:='/inference_result'
```

**Terminal 2** - Launch web visualization:

```bash
ros2 launch br_visualization websocket_cpp.launch.py image_topic:='/result_img'
```

The terminal will print the message as:

```
...
Please visit in your browser: http://<IP>:8080
...
```

Open a browser and visit the URL shown in the terminal (e.g., `http://<IP>:8080`) to view results:

![](images/web_pose.png)

### Subscribe to Results

To view the inference results by

```bash
ros2 topic echo /inference_result
```

Or use this Python script to subscribe to inference results:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from jobot_ai_msgs.msg import DetectionPoseResultArray

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            DetectionPoseResultArray,
            '/inference_result',  # Topic name, can be changed
            self.listener_callback,
            10  # QoS
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received pose results: {len(msg.results)} persons")

        for det_id, res in enumerate(msg.results):
            self.get_logger().info(
                f"\nPerson {det_id}, [xmin:{res.x_min}, ymin:{res.y_min}, "
                f"width:{res.width}, height:{res.height}], conf {res.conf:.2f}"
            )

            for i in range(len(res.keypoint_ids)):
                name = f"kp_{res.keypoint_ids[i]}"
                x = res.keypoint_xs[i]
                y = res.keypoint_ys[i]
                conf = res.keypoint_confs[i]
                self.get_logger().info(f"  {name}: ({x:.1f}, {y:.1f}), vis={conf:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameters Description

**`infer_img.launch.py` Parameters**

| Parameter       | Description                                                     | Default                    |
| -------------------- | -------------------------------------------------------------------- | ------------------------------ |
| `config_path`        | Path to the model configuration file used for inference              | `config/detection/yolov6.yaml` |
| `img_path`           | Path to the image file to run inference on                           | `data/detection/test.jpg`      |
| `publish_result_img` | Whether to publish the detection result as an image message          | `false`                        |
| `result_img_topic`   | Topic name for rendered image output (only valid if `publish_result_img = true`) | `/result_img`                  |
| `result_topic`       | Topic name for the inference result message                          | `/inference_result`            |

## Video Stream Inference

### Start USB Camera

```bash
ros2 launch br_sensors usb_cam.launch.py video_device:="/dev/video20"
```

### Run Inference & Publish results

**Terminal 1** - Start inference:

```bash
ros2 launch br_perception infer_video.launch.py \
  config_path:='config/pose/yolov8_pose.yaml' \
  sub_image_topic:='/image_raw' \
  publish_result_img:=true \
  result_topic:='/inference_result'
```

**Terminal 2** - Launch web visualization:

```bash
ros2 launch br_visualization websocket_cpp.launch.py image_topic:='/result_img'
```

The terminal will print the message as:

```
...
Please visit in your browser: http://<IP>:8080
...
```

Open a browser and visit the URL shown in the terminal (e.g., `http://<IP>:8080`) to view results.

**Data-Only Inference (No Visualization)**

If you only want to obtain the model inference results, run the following command:

```bash
ros2 launch br_perception infer_video.launch.py \
  config_path:='config/pose/yolov8_pose.yaml' \
  sub_image_topic:='/image_raw' \
  publish_result_img:=false \
  result_topic:='/inference_result'
```

### Subscribe to Results

To view the inference results by

```bash
ros2 topic echo /inference_result
```

Or use this Python script to subscribe to inference results:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from jobot_ai_msgs.msg import DetectionPoseResultArray

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            DetectionPoseResultArray,
            '/inference_result',  # Topic name, can be changed
            self.listener_callback,
            10  # QoS
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received pose results: {len(msg.results)} persons")

        for det_id, res in enumerate(msg.results):
            self.get_logger().info(
                f"\nPerson {det_id}, [xmin:{res.x_min}, ymin:{res.y_min}, "
                f"width:{res.width}, height:{res.height}], conf {res.conf:.2f}"
            )

            for i in range(len(res.keypoint_ids)):
                name = f"kp_{res.keypoint_ids[i]}"
                x = res.keypoint_xs[i]
                y = res.keypoint_ys[i]
                conf = res.keypoint_confs[i]
                self.get_logger().info(f"  {name}: ({x:.1f}, {y:.1f}), vis={conf:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameters Descriptions

**`infer_video.launch.py` Parameters**

| Parameter Name       | Description                                                  | Default Value                  |
| -------------------- | ------------------------------------------------------------ | ------------------------------ |
| `config_path`        | Path to the model configuration file for inference                         | `config/detection/yolov6.yaml` |
| `sub_image_topic`    | Image topic to subscribe to                    | `/image_raw`                   |
| `publish_result_img` | Whether to publish the rendered inference image              | `false`                        |
| `result_img_topic`   | Image topic to publish (only when `publish_result_img=true`) | `/result_img`                  |
| `result_topic`       | Inference-result topic to publish                            | `/inference_result`            |
