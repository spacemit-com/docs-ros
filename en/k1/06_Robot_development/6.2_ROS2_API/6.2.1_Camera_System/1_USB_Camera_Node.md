sidebar_position: 1

# USB Camera Node

## Supported Hardware and Protocols

- Compatibility: Any USB 2.0 / USB 3.0 camera that complies with the **UVC (USB Video Class)** standard and supports the Linux `v4l2` driver should work (including common formats such as YUYV, MJPEG, H264, etc.).
- Bandwidth and Port Selection: For high resolutions (≥720p) or high frame rates (≥30fps), the use of a **USB 3.0** camera is strongly recommended; using a **USB 2.0** connection may easily lead to dropped frames or bandwidth contention issues.
- Power Requirements: Certain high-performance cameras or modules equipped with built-in lighting are sensitive to USB power supply fluctuations. It is recommended to ensure the host system has sufficient power delivery capabilities (e.g., 5V/3A or higher, depending on the specific motherboard and total peripherals) or to use a powered USB hub.
- Drivers and Modules: The standard kernel module is `uvcvideo`. If necessary, check the kernel version, firmware, and kernel logs (`dmesg`) to verify that the driver has loaded correctly and that bandwidth negotiation was successful.

## Software and Hardware Environment

Please use the ROS2_LXQT operating system. For hardware, the SpacemiT MUSE Pi Pro is recommended.

Common Testing Tools (during development/debugging):

- `lsusb`, `dmesg | grep uvc`, `v4l2-ctl --list-devices`, `v4l2-ctl --list-formats-ext -d /dev/videoX`
- ROS2：`ros2 topic hz`, `rqt_image_view`

## Sensor Connection Diagram

![](./images/usb_camera_python.jpg)

## Identifying the Device ID

1. Input： `ls /dev/video*`，The output will appear as follows：

   ![](./images/t1.png)

2. Remove the camera, then enter `ls /dev/video*`

   ![](./images/t2.png)

   You can now confirm that the device IDs are `/dev/video20` and `/dev/video21`. For standard USB cameras, the device ID with the lower numerical value is typically the correct one to use; in this specific example, it is `/dev/video20`.

**Or use the v4l2 tool to check**

1. Enter the following command in the terminal：`v4l2-ctl --list-devices`

   The following output will appear:：

   ![](./images/t3.png)

2. Use the command `v4l2-ctl -d /dev/video20 --all` to view detail of `/dev/video20` 

   The output containing the "Format Video Capture" field generally indicates the node used for capturing actual video frames.

   ![](./images/t5.jpg)

3. Use the command `v4l2-ctl -d /dev/video21 --all` to view detail of `/dev/video21` 

   ![](./images/t6.png)

   The output displays：**UVC Payload Header Metadata capture interface** —— which corresponds to the **video metadata capture interface**; this interface is not used for capturing the actual video image frames themselves.

## Launch File

`rdk_sensors/usb_cam.launch.py`



## Software Dependencies

```Shell
sudo apt install ros-humble-cv-bridge ros-humble-camera-info-manager ros-humble-image-transport \
ros-humble-image-transport-plugins python3-pydantic ffmpeg libopencv-dev
```



## Launch Command

```
ros2 launch rdk_sensors usb_cam.launch.py video_device:="/dev/video20"
```

This will publish raw image data to the `/image_raw` topic.



## Parameter Details

|  Parameter Name    |  Type  |Default Value|         Candidate Values/Range        |        Description          |
| :----------------: | :----: | :---------: | :-----------------------------------: | :--------------------------: |
|    camera_name     | string | default_cam |               any string              |           camera name            |
|  camera_info_url   | string |     ""      |               valid URL               | camera Intrinsic File Path (URL format) |
|     framerate      | double |    30.0     | Other frame rates supported by the camera|  Camera Capture Frame Rate     |
|      frame_id      | string | default_cam |          any string            |        Camera Coordinate Frame ID          |
|    image_height    |  int   |     480     |            Positive integer             |     Image Height (Pixels)        |
|    image_width     |  int   |     640     |            Positive integer             |     Image Width (Pixels)       |
|     io_method      | string |    mmap     |      mmap, read, userptr      |    Video Capture I/O Method      |
|    pixel_format    | string |    yuyv     |    mjpeg, yuyv, mjpeg2rgb     |           Pixel Format            |
|  av_device_format  | string |   YUV422P   | YUV422P, other FFmpeg-supported formats |       Underlying AV Device Format     |
|    video_device    | string | /dev/video0 |          /dev/videoX          |      Video Device Path      |
|     brightness     |  int   |     50      |     0 ~ 255 or device-supported range    |            Brightness              |
|      contrast      |  int   |     -1      |         -1 or 0 ~ 255         |   Contrast (-1 for default)      |
|     saturation     |  int   |     -1      |         -1 or 0 ~ 255         |   Saturation (-1 for default)     |
|     sharpness      |  int   |     -1      |         -1 or 0 ~ 255         |    Sharpness (-1 for default)      |
|        gain        |  int   |     -1      |       -1 or device-supported range       |  Gain (-1 for auto/default)    |
| auto_white_balance |  bool  |    true     |          true, false          |     Enable Auto White Balance      |
|   white_balance    |  int   |    4000     |          2000 ~ 7000          |      White Balance Color Temperature (K)        |
|    autoexposure    |  bool  |    true     |          true, false          |      Enable Auto Exposure       |
|      exposure      |  int   |     30      |           40 ~ 1000           |        Exposure Time         |
|     autofocus      |  bool  |    false    |          true, false          |       Enable Auto Focus      |
|       focus        |  int   |     -1      |       -1 or device-supported range      | Focus Parameter (-1 for auto/default)  |



## Subscribed messages

None; data is acquired directly from the **USB** camera.



## Published messages

- pixel_format = yuyv

     Topic Name：/image_raw

​	`ros2 interface show sensor_msgs/msg/Image`

- pixel_format = mjpeg

     Topic Name：/image_raw/compressed

​	`ros2 interface show sensor_msgs/msg/CompressedImage`



## Instructions

1. **Dropped Frames**：This issue is often caused by exhausted USB bandwidth, insufficient CPU decoding power, low memory, or the use of passive USB hubs. Troubleshooting sequence: Check `dmesg` logs, verify stream status via `v4l2-ctl`, monitor system resources using `top`/`htop`, switch to a different physical USB root port, or try lowering the resolution/switching to the MJPG format.
2. **Automatic Parameter Jitter**：When `auto_exposure` or `auto_gain` is enabled, image brightness may fluctuate drastically between frames, negatively impacting object detection and tracking.In production environments, it is recommended to disable these automatic settings and lock in a specific set of parameters.
3. **Unstable Device Nodes**：Connecting/disconnecting devices or changes in connection order may cause the `/dev/videoX` device numbers to shift. Use `udev` rules to create persistent device names based on serial numbers or USB paths (see below for example `udev` rules).
4. **Permission Issues**：If the user running a container or system service lacks membership in the `video` group, they will be unable to access or open the device. Verify and configure `udev` rules, user group memberships, and the `User`/`Group` settings within your systemd service units.
5. **Thermal Management & Physical Mounting**：Under prolonged high-load operation, cameras may overheat, potentially triggering a reduction in the Area of ​​Interest (AOI) or Frame Rate (FPS). Ensure adequate heat dissipation for both the camera and the host motherboard, and provide stable mechanical mounting to minimize vibrations that could compromise camera calibration.
6. **Bandwidth Planning**：When designing multi-camera systems, perform a bandwidth budget analysis in advance (factoring in compression rates, resolutions, and frame rates for each camera). Subsequently, allocate USB ports logically based on the topology of the motherboard's USB controllers.
7. **Firmware & Compliance Updates**：Only use official firmware provided by the manufacturer for device upgrades. Thoroughly test the impact of any firmware update on driver compatibility before deploying it to production devices.



## udev Persistent Naming (Example)

> Use the device's `serial`, `idVendor`, and `idProduct` to establish a stable symlink—for instance, `/dev/video-muse-0`.

```
# /etc/udev/rules.d/80-muse-camera.rules
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="AAAA", ATTRS{idProduct}=="BBBB", \
  ATTRS{serial}=="MUSE123456", SYMLINK+="video-muse-0", MODE="0666", GROUP="video"
```

（Use `lsusb -v` or `udevadm info -a -p /sys/class/video4linux/video0` to obtain the specific attribute values ​​and replace `AAAA/BBBB/MUSE123456`.）



## Common Troubleshooting

- Device not recognized: Check `lsusb` and `dmesg`.
- Desired format not visible in the list: Run `v4l2-ctl -d /dev/videoX --list-formats-ext`.
- No image in ROS: Run `ros2 topic list` to check if `/image_raw` exists; run `ros2 topic echo /image_raw` or `ros2 topic hz /image_raw`; use `rqt_image_view` for visualization.
- Dropped frames or freezing: Check `dmesg` (for USB bandwidth issues or errors); look for CPU/IO bottlenecks; verify whether a passive USB hub is being used; try lowering the resolution or frame rate.
- Inaccurate calibration: Repeat the calibration process, ensuring that the chessboard pattern covers a wide range of angles and distances relative to the camera; record and secure the camera's physical mounting position.
