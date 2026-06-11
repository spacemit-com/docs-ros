---
sidebar_position: 3
slug: /k3/robot-dev/sensors/5-6-3-电机
---

# 基础传感器 · 电机

## 1. 模块概述

- **主要功能**：`motor_node` 是 ROS 2 电机硬件抽象层节点，属于 `peripherals` 包。它将 ROS 2 话题/服务消息统一转换为多种总线协议（CAN、UART、EtherCAT）的电机控制指令，并以标准话题反馈电机状态，使上层控制算法无需关心底层硬件差异。
- **规格与特性**：
  - 支持接口协议：EtherCAT（CiA402）、CAN（达妙 MIT 协议）、UART（飞特 Feetech SCS/STS 协议）
  - 控制模式：IDLE / POS / VEL / TRQ / HYBRID(MIT) / CSP / CSV / CST / HM 共 9 种
  - EtherCAT 总线周期：2–5 ms（可配置）
  - 状态反馈频率：最高 100 Hz（可配置）
  - 实时调度：EtherCAT 线程 SCHED_FIFO 优先级 80
- **相关目录结构**：

| 路径 | 职责 |
| --- | --- |
| `components/peripherals/motor/include/motor.h` | C 层统一电机 API 头文件（模式枚举、cmd/state 结构体） |
| `components/peripherals/motor/src/motor_core.c` | 电机核心调度逻辑 |
| `components/peripherals/motor/src/drivers/` | 各协议驱动实现（`drv_ethercat_jmc`、`drv_can_dm`、`drv_uart_feetech` 等） |
| `middleware/ros2/peripherals/motor/src/motor_node.cpp` | ROS 2 节点封装（话题、服务、参数管理） |
| `middleware/ros2/peripherals/motor/config/` | YAML 参数配置文件（`ecat_motor_params.yaml`、`can_motor_params.yaml`、`uart_motor_params.yaml`） |

## 2. 环境准备

### 前置条件

- **运行环境**：Linux（推荐 Ubuntu 22.04）；ROS 2 Humble；CMake ≥ 3.10；GCC/G++ 支持 C99 / C++17
- **依赖与外部资源**：
  - ROS 2 Humble 基础包（`rclcpp`、`std_srvs`、`std_msgs`）
  - `motor` 组件库（`components/peripherals/motor`，编译后生成 `libmotor.so`）
  - EtherCAT 驱动依赖：SOEM 或 IgH EtherCAT Master（系统级安装）
  - CAN 工具：`can-utils`（`sudo apt install can-utils`）
- **环境变量与初始化**：
  ```bash
  cd spacemit_robot # SDK 根目录
  source build/envsetup.sh
  ```
- **硬件与连接**：
  - EtherCAT：K3 COM260 开发板 + JMC IHSS42-EC 步进伺服，RJ45 网线连接 `eth0`
  - CAN：K3 COM260 + 达妙电机，CAN 收发器连接 `can0`
  - UART：K3 COM260 + 飞特 Feetech SCS/STS 舵机，USB-TTL 连接 `/dev/ttyACM0`
  - 确保电机供电正常、散热良好
- **工具与权限**：
 
  - CAN 需要 `sudo ip link set can0 up type can bitrate 1000000`
  - UART 需要当前用户在 `dialout` 组或具备 `/dev/ttyACM0` 读写权限

### 构建编译

- **获取代码**：
  
  具体步骤可参考《快速入门》中「配置编译」章节。

- **本模块编译**：

    **注意： 在编译前请确定当前方案已经启用将要使用的电机驱动**，如
    ```
    {
    "version": "1.0",
    "board": "k3-com260",
    "product": "minimal",
    "description": "K3 COM260 board - minimal build configuration",
    "enabled_packages": [
        "components/peripherals/motor", 
        "middleware/ros2/peripherals/motor"
    ],
    "enabled_package_options": {
        "components/peripherals/motor": { "enabled_drivers": ["drv_can_dm", "drv_uart_feetech", "drv_ethercat_jmc"] },  # 启用电机驱动
        "components/peripherals/pm": { "enabled_drivers": [] }
    },


    "options": {
        "parallel_jobs": 4,
        "auto_resolve_dependencies": true
    }
    }
   ```

**编译方式**：

  ```bash
  # 方法 1：自动编译依赖（推荐）
  cd middleware/ros2/peripherals/motor
  mm --with-deps  # 自动先编译 motor 组件，再编译 motor_node

  # 方法 2：手动分步编译
  cd components/peripherals/motor
  mm  # 先编译底层 motor 组件库

  cd middleware/ros2/peripherals/motor
  mm  # 再编译 ROS 2 节点
  ```

  编译成功后，导入环境变量：
  ```bash
  sros2_setup  # 已在 SDK 根目录 source build/envsetup.sh
  ```

  产物：`/root/robot/output/build/ros2/middleware/peripherals/motor_node`


### 配置文件注意事项

motor_node 的运行参数由 YAML 配置文件定义。使用配置文件时需注意：

- **参数数组大小必须与电机数量匹配**：
  - `profile_vel`、`profile_acc`、`profile_dec` 等数组参数的元素个数必须等于 `motor_ids` 的长度
  - 例如：`motor_ids: [1]`（1 个电机）则 `profile_vel: [10000.0]`（1 个值）
  - 若有多个电机 `motor_ids: [1, 2]`，则 `profile_vel: [10000.0, 10000.0]`（2 个值）
  - **异常提示**：如果数组大小不匹配，启动时会报 `Parameter 'profile_vel' size mismatch` 错误

- **参数扩展规则**：
  - 若某数组参数只提供 **1 个值**，会自动扩展为与电机数量相同的副本
  - 例如：`motor_ids: [1, 2, 3]` + `profile_vel: [10000.0]` 会自动扩展为 `[10000.0, 10000.0, 10000.0]`

- **各协议推荐配置**：
  - **EtherCAT**（`ecat_motor_params.yaml`）：
    - `ecat_cycle_ms: 2~5`（建议 2 ms 用于实时性强的应用，5 ms 用于非 RT 内核）
    - `profile_vel/acc/dec`：典型值 10000～100000（单位 rad/s²，由电机型号决定）
  - **CAN**（`can_motor_params.yaml`）：
    - `pid_kp/ki/kd`：达妙电机推荐 Kp=25～30，Ki=0，Kd=1～2
    - `max_velocity`、`max_torque`：根据电机规格设置安全限值
  - **UART**（`uart_motor_params.yaml`）：
    - `baud: 1000000`（飞特舵机通常为 1 Mbps）
    - `motor_iface`：串口设备路径（如 `/dev/ttyACM0` 或 `/dev/ttyUSB0`）

## 3. 示例使用（从 0 跑通）

### 3.1 EtherCAT 电机控制（JMC IHSS42-EC，CSP 模式）

**前置**：见 §2，确保 EtherCAT 网线已连接 `eth0`，电机已上电，以 `root` 运行。

**步骤 1**：加载环境
```bash
sros2_setup  # 已在 SDK 根目录 source build/envsetup.sh

```

**步骤 2**：启动 motor_node
```bash
ros2 run peripherals motor_node --ros-args --params-file config/ecat_motor_params.yaml
```
预期现象：终端输出 `[motor_node] EtherCAT slaves ready, entering RUNNING state`，无 ERROR 日志。

**步骤 3**：发送位置指令（CSP 模式，移动到 3.14 rad）
```bash
ros2 topic pub /cmd/motor peripherals/msg/MotorCommandArray \
  "{commands: [{id: 1, mode: 5, pos_des: 3.14}]}" --rate 100
```
预期现象：电机平滑转动到目标位置；通过 `ros2 topic echo /motor/state` 可观察 `pos` 逐渐趋近 3.14。

**步骤 4**：查看状态反馈
```bash
ros2 topic echo /motor/state
```
预期输出包含 `pos`、`vel`、`trq`、`error_flags` 字段。

### 3.2 CAN 电机控制（达妙 Damiao，速度模式）

**前置**：见 §2，确保 CAN 总线已启用：
```bash
sudo ip link set can0 up type can bitrate 1000000
```

**步骤 1**：加载环境
```bash
sros2_setup  # 已在 SDK 根目录 source build/envsetup.sh
```

**步骤 2**：启动 motor_node
```bash
ros2 run peripherals motor_node --ros-args --params-file config/can_motor_params.yaml
```
预期现象：终端输出 `[motor_node] CAN interface can0 opened`。

**步骤 3**：使能电机（必要步骤）
```bash
ros2 service call /motor/enable std_srvs/srv/SetBool "{data: true}"
```
预期现象：返回 `success: true`。

**步骤 4**：发送速度指令（VEL 模式，3.0 rad/s）
```bash
ros2 topic pub /cmd/motor peripherals/msg/MotorCommandArray \
  "{commands: [{id: 2, mode: 2, vel_des: 3.0}]}" --rate 10
```
预期现象：电机以约 3.0 rad/s 匀速旋转。

### 3.3 UART 电机控制（飞特 Feetech，位置模式）

**前置**：见 §2，确保 USB-TTL 已连接，`/dev/ttyACM0` 可访问。

**步骤 1**：加载环境
```bash
sros2_setup  # 已在 SDK 根目录 source build/envsetup.sh
```

**步骤 2**：启动 motor_node
```bash
ros2 run peripherals motor_node --ros-args --params-file config/uart_motor_params.yaml
```

**步骤 3**：使能电机（必要步骤）
```bash
ros2 service call /motor/enable std_srvs/srv/SetBool "{data: true}"
```

**步骤 4**：发送位置指令（POS 模式，移动到 1.57 rad）
```bash
ros2 topic pub /cmd/motor peripherals/msg/MotorCommandArray \
  "{commands: [{id: 1, mode: 1, pos_des: 1.57}]}" --rate 10
```
预期现象：舵机转动到约 90° 位置。

## 4. 应用开发

- **对外 API 与接口形态**：
  - ROS 2 话题：
    - `/cmd/motor`（订阅）：`peripherals/msg/MotorCommandArray` — 下发控制指令
    - `/motor/state`（发布）：`peripherals/msg/MotorStateArray` — 电机状态反馈
  - ROS 2 服务：
    - `/motor/enable`：`std_srvs/srv/SetBool` — 使能/失能电机（CAN、UART 电机必须先调用）
    - `/motor/set_param`：`peripherals/srv/MotorSetParam` — 写入电机寄存器参数
    - `/motor/get_param`：`peripherals/srv/MotorGetParam` — 读取电机寄存器参数
  - C 层 API（`motor.h`）：`motor_init`、`motor_set_cmds`、`motor_get_states`、`motor_free`

- **控制模式映射**：

  | mode 值 | 枚举 | 说明 | 支持硬件 |
  | :--- | :--- | :--- | :--- |
  | 0 | `IDLE` | 停止输出，电机待机/释放 | 全部 |
  | 1 | `POS` | 位置闭环 (rad) | 全部 |
  | 2 | `VEL` | 速度闭环 (rad/s) | 全部 |
  | 3 | `TRQ` | 力矩闭环 (Nm) | CAN, EtherCAT |
  | 4 | `HYBRID` | MIT 混合模式 (P/V/T/Kp/Kd) | 达妙 CAN |
  | 5 | `CSP` | 周期同步位置 | EtherCAT |
  | 6 | `CSV` | 周期同步速度 | EtherCAT |
  | 7 | `CST` | 周期同步力矩 | EtherCAT |
  | 8 | `HM` | 回零模式 | EtherCAT |

- **调用注意点**：
  - EtherCAT 驱动线程为 SCHED_FIFO（优先级 80），需 `root` 或 `CAP_SYS_NICE` 权限
  - EtherCAT 首条位置指令会触发零位锚定（防止使能跳变），后续指令相对该锚点运动
  - CAN/UART 电机必须先调用 `/motor/enable` 服务才能响应指令
  - 可通过 `ros2 param set` 动态修改 `max_velocity`、`profile_acc`、`profile_dec` 等参数

- **参考 demo 与配置路径**：
  - 配置文件：`middleware/ros2/peripherals/motor/config/`
  - 节点源码：`middleware/ros2/peripherals/motor/src/motor_node.cpp`
  - 底层驱动：`components/peripherals/motor/src/drivers/`

## 5. 调试指南

- **详细日志**：启动时添加 `--log-level debug` 查看每帧指令解析与映射过程：
  ```bash
  ros2 run peripherals motor_node --ros-args --params-file config/ecat_motor_params.yaml --log-level debug
  ```
- **状态监听**：
  ```bash
  ros2 topic echo /motor/state
  ```
  关注字段：`pos`（当前位置 rad）、`vel`（速度 rad/s）、`error_flags`（CiA402 Statusword，如 `0x0637` = OPERATION_ENABLED）
- **运行时参数调整**：
  ```bash
  ros2 param set /motor_node max_velocity 5.0
  ros2 param set /motor_node profile_acc "[50000.0, 50000.0]"
  ```
- **EtherCAT 诊断**：
  - 检查从站状态：`ethercat slaves`（需 IgH 工具）
  - 抓包：`tcpdump -i eth0 ether proto 0x88a4 -w ecat.pcap`
- **CAN 诊断**：
  ```bash
  candump can0       # 查看总线原始帧
  cansend can0 001#0102030405060708  # 手动发送测试帧
  ```
- **信息收集清单**（与硬件/内核同事协作时）：
  - `dmesg | grep -i ecat` 或 `dmesg | grep -i can`
  - `uname -r`（内核版本，是否为 RT 内核）
  - `cat /proc/sys/kernel/sched_rt_runtime_us`
  - motor_node 完整日志（debug 级别）

## 6. 常见问题

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 启动时报 `Parameter 'profile_vel' size mismatch` 错误 | 配置文件中数组参数大小与 `motor_ids` 不匹配 | 确保 `profile_vel`、`profile_acc`、`profile_dec` 等数组的元素个数等于 `motor_ids` 的长度；或只保留 1 个值让系统自动扩展 |
| 电机不响应位置指令，tpos 始终等于 apos | EtherCAT 零位锚定逻辑：首条指令被视为当前位置 | 发送一个**变化**的 pos_des 值（如从 0.0 改为 0.1），触发实际运动 |
| EtherCAT 报 `UNMATCHED` 警告 | `ecat_cycle_ms` 过小，非实时内核无法满足 | 将 `ecat_cycle_ms` 设为 5 以上，ROS 2 发布频率与之匹配（如 200 Hz） |
| `[Adapter] WARNING: EtherCAT loop overrun detected` | 系统调度抖动过大 | 降低发布频率、减少系统负载，或使用 RT 内核 |
| CAN 电机发指令无反应 | 未调用使能服务 | 先执行 `ros2 service call /motor/enable std_srvs/srv/SetBool "{data: true}"` |
| UART 电机无响应 | 串口权限不足或波特率不匹配 | 确认用户在 `dialout` 组，检查 `baud` 参数与电机实际波特率一致 |

## 附录：性能与测试数据（可选）

| 指标 | 数值 | 测试条件 |
| --- | --- | --- |
| EtherCAT 控制周期 | 2 ms | K3 COM260，单从站 |
| 状态反馈延迟 | < 5 ms | EtherCAT CSP 模式，100 Hz 发布 |
| CAN 指令响应时间 | < 10 ms | 达妙电机，1 Mbps CAN 总线 |
| UART 位置精度 | ±0.02 rad | 飞特 STS 舵机，1 Mbps 波特率 |

**测试方法**：K3 COM260 开发板，ROS 2 Humble，motor_node 单节点运行，使用 `ros2 topic delay` 和 `ros2 topic hz` 统计延迟与频率。
