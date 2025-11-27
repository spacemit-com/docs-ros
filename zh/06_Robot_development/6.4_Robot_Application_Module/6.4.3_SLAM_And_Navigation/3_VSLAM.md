

# VSLAM（SVO 系列）

```
最新版本：2025/09/12
```

## 功能简介

本示例基于 SVO-PRO 系列中的 VO_MONO 模块，实现单目视觉里程计（VSLAM）功能，具备实时跟踪、位姿估计与轨迹恢复能力。该模块适用于资源受限设备中的定位建图任务，可接入 EuRoC 数据集进行测试。

## 环境准备

本示例依赖已安装的 ROS2 环境（推荐 ROS 2 Humble），并需安装以下依赖包：

```bash
sudo apt update
sudo apt install ros-dev-tools
sudo apt install ros-humble-pcl-ros
```

## 编译与安装

### 编译 SVO 主模块

```bash
colcon build --packages-up-to svo_ros
```

首次构建耗时较长，编译时间预计 1~3 小时，若设备内存 < 8 GB，请提前配置 SWAP 交换空间。

### 编译本地定位模块

```bash
colcon build --packages-select rdk_localization
```

## 启动使用

### 启动 SVO SLAM 节点

```bash
ros2 launch rdk_localization slam_svo.launch.py
```

### 播放 ROS2 格式的 EuRoC 数据集

使用 `ros2 bag play` 播放转换后的 ROS2 Bag 数据（转换方法见下节）：

```bash
ros2 bag play ~/V2_02_medium
```

请确保 `/cam0/image_raw` （图像）与 `/imu0` （IMU）两个话题已正确发布。

## EuRoC 数据集转换工具（ROS1 → ROS2）

EuRoC 原始数据为图像+IMU CSV 格式，需转换为 ROS2 `.db3` 格式的 Bag 包以供使用。

### 转换脚本 euroc_converter.py

保存如下 Python 脚本为 `euroc_converter.py`：

```python
#!/usr/bin/env python3
# euroc_converter.py
import rclpy
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
import cv2
import os
from cv_bridge import CvBridge
import csv
import shutil
def create_bag(image_folder, timestamp_file, imu_file, output_bag):
    # 初始化ROS2（必要的序列化）
    rclpy.init()

    if os.path.exists(output_bag):
        shutil.rmtree(output_bag)
    # 创建bag写入器
    writer = SequentialWriter()
    storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                       output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    # 创建图像话题
    image_topic = '/cam0/image_raw'
    image_msg_type = 'sensor_msgs/msg/Image'
    writer.create_topic(
        TopicMetadata(
            name=image_topic,
            type=image_msg_type,
            serialization_format='cdr'
        )
    )

    # 创建IMU话题
    imu_topic = '/imu0'
    imu_msg_type = 'sensor_msgs/msg/Imu'
    writer.create_topic(
        TopicMetadata(
            name=imu_topic,
            type=imu_msg_type,
            serialization_format='cdr'
        )
    )

    # 读取图像时间戳文件
    image_timestamps = []
    with open(timestamp_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # 跳过标题行
        for row in reader:
            image_timestamps.append(int(row[0]))

    # 读取IMU数据
    imu_data = []
    with open(imu_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # 跳过标题行
        for row in reader:
            # 假设CSV格式为: timestamp,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z
            imu_data.append({
                'timestamp': int(row[0]),
                'gyro': [float(row[1]), float(row[2]), float(row[3])],
                'acc': [float(row[4]), float(row[5]), float(row[6])]
            })

    # 确保图像文件存在
    image_files = sorted([f for f in os.listdir(image_folder) if f.endswith('.png')])
    if len(image_files) != len(image_timestamps):
        print(f"警告: 图像数量({len(image_files)})和时间戳数量({len(image_timestamps)})不匹配")
        exit()

    bridge = CvBridge()

    # 写入图像数据
    for i, (ts, img_file) in enumerate(zip(image_timestamps, image_files)):
        # 读取图像
        img_path = os.path.join(image_folder, img_file)
        cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if cv_img is None:
            print(f"警告: 无法读取图像 {img_path}")
            exit()
        # 转换为ROS Image消息
        img_msg = bridge.cv2_to_imgmsg(cv_img, encoding='mono8')

        # 设置时间戳
        t = Time()
        t.sec = ts // 10**9
        t.nanosec = ts % 10**9
        img_msg.header.stamp = t
        img_msg.header.frame_id = 'cam0'

        # 写入bag
        writer.write(
            image_topic,
            serialize_message(img_msg),
            ts
        )
        if i % 100 == 0:
            print(f'已处理 {i+1}/{len(image_files)} 张图像')

    # 写入IMU数据
    for i, imu in enumerate(imu_data):
        # 创建IMU消息
        imu_msg = Imu()

        # 设置时间戳
        ts = imu['timestamp']
        t = Time()
        t.sec = ts // 10**9
        t.nanosec = ts % 10**9
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = 'imu0'

        # 设置角速度 (gyro)
        imu_msg.angular_velocity.x = imu['gyro'][0]
        imu_msg.angular_velocity.y = imu['gyro'][1]
        imu_msg.angular_velocity.z = imu['gyro'][2]

        # 设置线性加速度 (acc)
        imu_msg.linear_acceleration.x = imu['acc'][0]
        imu_msg.linear_acceleration.y = imu['acc'][1]
        imu_msg.linear_acceleration.z = imu['acc'][2]

        # 写入bag
        writer.write(
            imu_topic,
            serialize_message(imu_msg),
            ts
        )
        if i % 1000 == 0:
            print(f'已处理 {i+1}/{len(imu_data)} 条IMU数据')

    # 关闭bag
    del writer
    rclpy.shutdown()
    print(f'Bag文件已保存到 {output_bag}')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', required=True)
    args = parser.parse_args()

    folder : str = args.folder
    if folder.endswith('/'):
        folder = folder[:-1]
    bag_name = folder.split('/')[-1]
    image_folder = os.path.join(folder,'mav0', 'cam0', 'data')
    imu_csv = os.path.join(folder, 'mav0', 'imu0', 'data.csv')
    timestamp_csv = os.path.join(folder, 'mav0', 'cam0', 'data.csv')
    create_bag(image_folder, timestamp_csv, imu_csv, bag_name)
```

### 执行转换命令

运行以下命令进行转换：

```bash
python3 euroc_converter.py --folder /path/to/your/euroc_folder
```

示例：

```bash
###示例
python3 euroc_converter.py --folder ~/V2_02_medium/mav0
#converted ros2 bag will be created in the folder you passed in
```

转换成功后，将在当前路径下生成 `V2_02_medium/` 同名的 ROS2 Bag 数据包，包含以下话题：

- `/cam0/image_raw`：灰度图像帧（单目）

- `/imu0`：IMU 原始数据（角速度 + 加速度）