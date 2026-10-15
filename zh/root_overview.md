---
sidebar_position: 1
---

# 简介

## ROS 2

平台支持列表：

| 芯片 | 当前支持的 ROS 2 版本 | 规划支持的 ROS 2 版本 | 当前推荐的操作系统 |
| :--: | :------------------: | :------------------: | :----------------: |
| K3 | Humble、Jazzy | Lyrical（计划支持） | Bianbu 4.x（4.0.1 起） |
| K1 | Humble、Jazzy | — | Bianbu 2.3.x |

> “计划支持”表示平台适配规划，尚未作为正式支持版本提供；具体可用时间、配套系统版本和支持范围以 SpacemiT 后续发布说明为准。“—”表示本表暂未列出支持规划。

使用说明：

- **平台适配与安装**：表中支持指 SpacemiT 在对应 Bianbu 系统上的 RISC-V 适配。请使用平台配套的软件源，并按 [K3 安装指南](./k3/06-系统与平台/6.6-ROS2/6.6.1-ROS2-INSTALL.md)或 [K1 安装指南](./k1/06_Robot_development/6.1_OS_Preparation/6.1.2_ROS2_Installation.md)配置环境。
- **版本选择与功能范围**：当前 SDK 示例主要基于 Humble，建议按各章节列出的环境复现。不同 ROS 2 版本的软件包、驱动及功能支持范围可能不同，具体以对应平台文档和发布说明为准。
- **多机通信**：建议板端与开发主机使用相同的 ROS 2 发行版，并按对应示例配置通信中间件、网络和 `ROS_DOMAIN_ID`。ROS 2 上游不保证不同发行版之间的节点通信兼容，详见[官方说明](https://docs.ros.org/en/rolling/Releases.html#cross-distribution-communications)。

## Robot SDK

基于 ROS 2，SpacemiT 发布了定制化 Robot SDK，面向 RISC‑V 机器人平台，适配 SpacemiT K 系列芯片。涵盖系统与方案概述、基础应用（如传感器与多媒体）、进阶开发（模型量化、性能调优等）以及各平台索引与示例。其目标是为客户在 K 系列平台上进行机器人应用开发与集成提供参考。

- [K3](./k3/index.md)
- [K1](./k1/index.md)