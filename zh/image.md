---
sidebar_position: 2
---

# 镜像

目前 ROS2_LXQT 提供两种格式的镜像：

- SD 卡 raw 镜像

  以 `*.img.zip` 结尾，可以用 [balenaEtcher](https://etcher.balena.io/) 写入 SD 卡，或者解压后用 `dd` 命令写入 SD 卡。

- 自定义镜像

  以 `.zip` 结尾，可以用 Titan Flasher 刷机，或者解压后用 `fastboot` 刷机。

固件 root 用户的密码：`bianbu`
固件默认 bianbu 用户密码：`bianbu`

## 下载

地址：[ROS2_LXQT Image](https://archive.spacemit.com/ros2/bianbu-ros-images/v1.5/ROS2_LXQT-v1.5rc3-20251107.zip)

## 刷机

Titan Flasher 刷机参考 [刷机工具使用手册](https://www.spacemit.com/community/document/info?lang=zh&nodepath=tools/user_guide/flasher_user_guide.md)。
