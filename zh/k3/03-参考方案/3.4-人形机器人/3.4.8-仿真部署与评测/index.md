---
sidebar_position: 8
slug: /k3/reference-solutions/humanoid/simulation
---

# 仿真部署与评测

人形方案支持 PC 单机、PC + K3 跨机和 K3 本机 MuJoCo 仿真。以下已整理的三条流程复用相同的机型配置、策略模型、仿真资源和运行时，区别在于进程所在设备与控制端是否包含 FSM。

| 流程 | 设备 | 控制端 | 通信 | 适用场景 |
| --- | --- | --- | --- | --- |
| [PC 单机仿真](3.4.8.2-PC单机仿真.md) | x86_64 PC | FSM 或 Sim2Sim | SHM / 本机 UDP | 初次体验、开发调试和快速迭代 |
| [K3 跨机 FSM 仿真](3.4.8.3-K3跨机FSM仿真.md) | x86_64 PC + K3 | `control_runtime` + `hmi_runtime` | UDP | 验证 K3 上的完整控制链路和策略切换 |
| [K3 跨机 Sim2Sim](3.4.8.4-K3跨机Sim2Sim.md) | x86_64 PC + K3 | `control_sim2sim_runtime` | UDP | 跳过 FSM，快速验证新 RL 策略的 K3 部署效果 |

K3 本机仿真由 K3 同时运行 MuJoCo 和控制进程，图形显示采用 gl4es + PowerVR GLES；对应操作页仍在整理。

ROS 2 联合仿真另设占位页，整理 `humanoid_mujoco` 与 Native 控制端、导航桥接及传感器链路；操作文档暂未发布。

开始前先完成[安装与构建](../3.4.2-开发基础/3.4.2.1-安装与构建.md)，并准备对应机型的 RL 策略模型。进程职责和通信方式见 [SDK 架构](../3.4.2-开发基础/3.4.2.2-SDK架构.md)。

延迟测量、日志和验收分层见[性能基准与验收](3.4.8.7-性能基准与验收.md)。本章展示的 MuJoCo 闭环结果不等同于人形实机验收结果。
