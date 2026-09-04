---
sidebar_position: 3
slug: /k3/ai/rl
---

# 4.3 强化学习

## 功能定位

RL 组件是 SpaceMIT Robot SDK 内的通用策略推理运行时，用于将已导出的 ONNX 策略接入机器人控制链路。它负责解析 YAML 配置、组装观测、执行 ONNX 推理，并将策略输出映射为关节目标位置。

本章讲解的是策略如何在 SDK 中运行、验证和评测，不包含训练环境搭建、训练算法原理和数据制作。

## 当前能力

- 每个策略通过 `model_io` 显式声明 ONNX 张量的名称和用途，不根据 MLP、LSTM 等网络类别选择执行路径。
- 支持 `observation`、`observation_history`、`feedback`、`constant` 和 `external` 输入，以及 `action`、`expose` 和 `ignore` 输出目标。
- 支持无历史、`flat_history` 和 `frame_history` 三种观测组装方式，以及自定义标量、自定义数组和独立模型输入输出。
- 支持在同一 YAML 中声明多个策略并按名称加载，以及观测/动作裁剪、scale、blend、default position 和关节索引映射。
- 当前推理后端为 ONNX Runtime，支持 x86_64 开发验证和 K3 板端运行。

## 文档导航

| 文档 | 解决的问题 |
| --- | --- |
| [快速开始](4.3.1-快速开始.md) | 安装依赖、编译组件，并完成一次配置加载、观测组装、推理和动作映射验证 |
| [运行时架构与接口](4.3.2-运行时架构与接口.md) | 理解组件边界、C++ API、推理后端和执行器状态生命周期 |
| [策略配置与模型契约](4.3.3-策略配置与模型契约.md) | 配置 `model_io`、观测历史、动作映射及上层控制参数 |
| [模型验证与性能评测](4.3.4-模型验证与性能评测.md) | 检查 ONNX 模型、批量扫描策略，区分纯推理与组件链路 benchmark |
| [故障排查](4.3.5-故障排查.md) | 按模型、配置、观测、推理和系统效果分层定位问题 |

## 使用边界

- ONNX 模型能加载、能输出数值，只证明模型推理链路可用，不代表训练端与 SDK 端数值已对齐。
- `test_policy_executor` 通过只验证 RL 组件内的配置、观测、推理和映射流程，不代表 MuJoCo 仿真、K3 推理性能或实机闭环已验证。
- 组件 benchmark 不包含真实传感器采样、跨进程通信和电机下发时间；即使周期测试无超时，也不等同于整机实时性验收。
- 不支持 PyTorch 原生推理；策略需要先导出为 ONNX。当前输出语义为单个确定性关节位置 action，不直接支持多 action head、随机分布采样或力矩 action。

策略类型、机型和已验证平台适合在模型库或兼容性矩阵中维护，不按基础运控、动作模仿、感知增强或动作跟踪拆分运行时文档。

## 代码与详细接口

- [RL 策略推理组件](https://github.com/spacemit-com/model_zoo_rl)
- [组件 README](https://github.com/spacemit-com/model_zoo_rl/blob/main/README.md)
- [C++ 对外头文件](https://github.com/spacemit-com/model_zoo_rl/blob/main/include/rl_service.h)
- [完整调用示例](https://github.com/spacemit-com/model_zoo_rl/blob/main/example/test_policy_executor.cpp)
