sidebar_position: 3

# VSLAM (SVO Series)

## Overview

This guide uses the **VO\_MONO module** from the **SVO-PRO series** to implement **monocular Visual Odometry (VSLAM)**.

It provides:

- Real-time camera tracking
- Pose estimation
- Trajectory reconstruction

Designed for **localization and mapping on resource-constrained devices**, it can be tested using the **EuRoC dataset**.

## Environment Setup

**Prerequisites:**

- ROS 2 environment installed (ROS 2 Humble recommended)
- Additional packages:

```bash
sudo apt update
sudo apt install ros-dev-tools
sudo apt install ros-humble-pcl-ros
```

## Build and Install

### Compile the SVO Main Module

```bash
colcon build --packages-up-to svo_ros
```

- First build may take **1–3 hours**
- For devices with **less than 8 GB RAM**, set up a **SWAP file** beforehand

### Compile the Local Positioning Module

```bash
colcon build --packages-select br_localization
```

## Launch and Usage

### Launch the SVO SLAM Node

```bash
ros2 launch br_localization slam_svo.launch.py
```

### Play the EuRoC Dataset in ROS 2 Format

Use `ros2 bag play` to replay the converted ROS 2 Bag data (see the next section for the data-conversion steps):

```bash
ros2 bag play ~/V2_02_medium
```

Ensure the following topics are being published:

- `/cam0/image_raw`
- `/imu0`

## EuRoC Dataset Conversion Tool (ROS1 → ROS2)

The EuRoC dataset comes as **image + IMU CSV files** and needs conversion into a **ROS 2 `.db3` Bag**.

### Conversion Script `euroc_converter.py`

Save the following Python script as `euroc_converter.py`:

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
    # Initialize ROS2 (serialization required)
    rclpy.init()

    if os.path.exists(output_bag):
        shutil.rmtree(output_bag)

    # Create bag writer
    writer = SequentialWriter()
    storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                       output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    # Create image topic
    image_topic = '/cam0/image_raw'
    image_msg_type = 'sensor_msgs/msg/Image'
    writer.create_topic(
        TopicMetadata(
            name=image_topic,
            type=image_msg_type,
            serialization_format='cdr'
        )
    )

    # Create IMU topic
    imu_topic = '/imu0'
    imu_msg_type = 'sensor_msgs/msg/Imu'
    writer.create_topic(
        TopicMetadata(
            name=imu_topic,
            type=imu_msg_type,
            serialization_format='cdr'
        )
    )

    # Read image timestamp file
    image_timestamps = []
    with open(timestamp_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            image_timestamps.append(int(row[0]))

    # Read IMU data
    imu_data = []
    with open(imu_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            # CSV expected format: timestamp,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z
            imu_data.append({
                'timestamp': int(row[0]),
                'gyro': [float(row[1]), float(row[2]), float(row[3])],
                'acc': [float(row[4]), float(row[5]), float(row[6])]
            })

    # Ensure all image files exist
    image_files = sorted([f for f in os.listdir(image_folder) if f.endswith('.png')])
    if len(image_files) != len(image_timestamps):
        print(f"Warning: number of images ({len(image_files)}) does not match number of timestamps ({len(image_timestamps)})")
        exit()

    bridge = CvBridge()

    # Write image data
    for i, (ts, img_file) in enumerate(zip(image_timestamps, image_files)):
        # Load image
        img_path = os.path.join(image_folder, img_file)
        cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if cv_img is None:
            print(f"Warning: unable to read image {img_path}")
            exit()

        # Convert to ROS Image message
        img_msg = bridge.cv2_to_imgmsg(cv_img, encoding='mono8')

        # Set timestamp
        t = Time()
        t.sec = ts // 10**9
        t.nanosec = ts % 10**9
        img_msg.header.stamp = t
        img_msg.header.frame_id = 'cam0'

        # Write to bag
        writer.write(
            image_topic,
            serialize_message(img_msg),
            ts
        )
        if i % 100 == 0:
            print(f'Processed {i+1}/{len(image_files)} images')

    # Write IMU data
    for i, imu in enumerate(imu_data):
        # Create IMU message
        imu_msg = Imu()

        # Set timestamp
        ts = imu['timestamp']
        t = Time()
        t.sec = ts // 10**9
        t.nanosec = ts % 10**9
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = 'imu0'

        # Set angular velocity (gyro)
        imu_msg.angular_velocity.x = imu['gyro'][0]
        imu_msg.angular_velocity.y = imu['gyro'][1]
        imu_msg.angular_velocity.z = imu['gyro'][2]

        # Set linear acceleration (acc)
        imu_msg.linear_acceleration.x = imu['acc'][0]
        imu_msg.linear_acceleration.y = imu['acc'][1]
        imu_msg.linear_acceleration.z = imu['acc'][2]

        # Write to bag
        writer.write(
            imu_topic,
            serialize_message(imu_msg),
            ts
        )
        if i % 1000 == 0:
            print(f'Processed {i+1}/{len(imu_data)} IMU samples')

    # Close bag
    del writer
    rclpy.shutdown()
    print(f'Bag file saved to {output_bag}')

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

### Run the Conversion Command

The conversion command is:

```bash
python3 euroc_converter.py --folder /path/to/your/euroc_folder
```

**Example:**

```bash
python3 euroc_converter.py --folder ~/V2_02_medium/mav0
# After conversion, a ROS 2 Bag is created in the folder
```

After a successful conversion, a ROS 2 Bag package named `V2_02_medium` (same as the dataset folder) will be created in the current directory, containing:

- `/cam0/image_raw` Monochrome image frames (monocular)
- `/imu0` Raw IMU data (angular velocity + linear acceleration)
