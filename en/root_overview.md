---
sidebar_position: 1
---

# Introduction

## ROS 2

Platform support list:

| Chip | Currently Supported ROS 2 Versions | Planned ROS 2 Versions | Recommended OS |
| :--: | :--------------------------------: | :--------------------: | :------------: |
| K3 | Humble, Jazzy | Lyrical (planned) | Bianbu 4.x (from 4.0.1) |
| K1 | Humble, Jazzy | — | Bianbu 2.3.x |

> "Planned" indicates platform adaptation is under development and not yet officially supported; availability, compatible OS versions, and support scope will be announced by SpacemiT in future releases. "—" indicates no planned versions are currently listed.

Usage notes:

- **Platform Adaptation and Installation**: Support in the table refers to SpacemiT's RISC-V adaptation on the corresponding Bianbu system. Use the platform-specific software sources and follow the [K3 Installation Guide](./k3/06-系统与平台/6.6-ROS2/6.6.1-ROS2-INSTALL.md) or [K1 Installation Guide](./k1/06_Robot_development/6.1_OS_Preparation/6.1.2_ROS2_Installation.md) to configure the environment.
- **Version Selection and Feature Coverage**: Current SDK examples are primarily based on Humble; it is recommended to replicate the environment as specified in each chapter. Package availability, drivers, and feature support may vary across ROS 2 versions; refer to the corresponding platform documentation and release notes for details.
- **Multi-Machine Communication**: It is recommended to use the same ROS 2 distribution on both the board and development host, and configure the communication middleware, network, and `ROS_DOMAIN_ID` according to the examples. ROS 2 upstream does not guarantee node communication compatibility between different distributions; see the [official documentation](https://docs.ros.org/en/rolling/Releases.html#cross-distribution-communications).

## Robot SDK

Based on ROS 2, SpacemiT has released a customized Robot SDK for RISC-V robotic platforms, supporting SpacemiT K series chips. It covers system and solution overviews, basic applications (such as sensors and multimedia), advanced development topics (model quantization, performance optimization, etc.), and platform indexes and examples. Its goal is to provide reference materials for robot application development and integration on K series platforms.

- [K3](./k3/index.md)
- [K1](./k1/index.md)