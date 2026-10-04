# RoboMaster 轮腿步兵机器人固件仓库

RoboMaster 高校联盟赛轮腿步兵机器人嵌入式软件工程，基于 STM32H723VGT6 主控，采用 FreeRTOS 实时操作系统，C/C++ 混合开发。

本仓库包含三个子工程：**底盘控制固件（V2.1.4）**、**云台控制固件（Gimbal V1.0）** 以及 **MATLAB 控制仿真（Simulation）**，另附配套项目文档。

> 文档更新时间：2026-08-30。当前 `main` 已连接 GitHub 远程 `origin`；此前提交中的改动均已提交入库，工作区干净。

## 近期本地改动

- **控制任务**：数据采集与状态计算继续收拢到 `Caculate` 任务；传感器数据超时时清零控制输出，并增加任务心跳日志。
- **状态估计**：新增带符号的前向加速度 `accel_x`，轮速 Kalman 不再使用水平加速度幅值；Kalman 校正后统一更新位置、速度和速度误差。
- **轮腿控制**：腿长控制改为独立 PID，加入目标腿长切换时积分清零、共用起身状态机和 roll 差动力补偿；LQR 增益计算抽为公共函数，150/250 档增益已更新。
- **电机与通信**：恢复 `ControlTask` 的 DM 电机 MIT/速度指令输出；DM 电机增加失能检测计数状态，`app_main` 启动时初始化 UART DMA 打印同步。
- **仿真参数**：`Simmulation/get_K_jiao_LQR.m` 调整腿角、姿态和位置相关的 LQR 权重。

上述改动已写入 `TASK_LOG.md`，底盘和云台工程均已通过 CMake Debug 编译；尚未在实车上完成闭环验收。

## 目录结构

```
RoboMaster_Wheeleg_Infantory/
├── RoboMaster_InfantoryV2.1.4/       # 轮腿步兵底盘控制固件（主工程）
├── RoboMaster_Infantory_GimbalV1.0/  # 云台控制固件
├── Common/                           # 底盘/云台共享板间通信协议
├── Simmulation/                      # 腿长 LQR 求解 + 腿控仿真
├── TASK_LOG.md                       # 任务日志 + 分阶段实施方案
├── ARCHITECTURE.md                   # 当前任务架构梳理
├── REQUIREMENTS.md                   # 项目需求表（按 RoboMaster 平衡步兵）
└── README.md
```

## 项目文档索引

| 文档 | 内容 |
|---|---|
| `TASK_LOG.md` | 对标完整版差距分析、任务清单（T1-T9）、Phase 0-5 实施方案 |
| `ARCHITECTURE.md` | 双板 FreeRTOS 任务架构、IPC、1ms 事件链数据流 |
| `REQUIREMENTS.md` | 44 项需求表（底盘/云台/射击/裁判/通信/安全/工程），含验收指标与状态跟踪 |

## 子工程说明

### 1. RoboMaster_InfantoryV2.1.4 — 底盘控制固件

轮腿步兵底盘主控固件，负责全车姿态解算、轮腿平衡控制与电机驱动。

- **主控芯片**：STM32H723VGT6（Cortex-M7 @ 550MHz）
- **工程配置**：由 STM32CubeMX（`RoboMaster_Infantry.ioc`）生成，FreeRTOS 内核
- **构建方式**：CMake + Ninja + arm-none-eabi-gcc（也可使用 Keil MDK）
- **电机**：DM8009 无刷电机（4 个轮腿驱动/舵机）、DM3519L/R 动量轮电机，FDCAN 通信
- **控制周期**：1ms 实时控制（Calculate → Control 事件链）

| 目录 | 说明 |
|------|------|
| `Core/` | CubeMX 生成的外设驱动、中断、FreeRTOS 配置 |
| `BSP/` | 板级外设封装（FDCAN、CAN、UART） |
| `HardWare/` | 硬件驱动（BMI088、DMIMU、Hipnuc hi14 惯导、SBUS 遥控、PID、WS2812、蜂鸣器、电源） |
| `APP/` | 应用层：任务（INS / Remote / Caculate / Control；Data 处理已并入 Caculate）与算法（四元数 EKF、轮速卡尔曼、离地检测、VMC 虚拟力控制、腿长 LQR/PID 控制） |
| `Middlewares/` | FreeRTOS、ARM CMSIS-DSP 等第三方中间件 |
| `Drivers/` | STM32H7 HAL 驱动库、CMSIS |

**控制任务框架**（`APP/APP_Task/src/app_Task.cpp`）：

| 任务 | 优先级 | 功能 |
|------|--------|------|
| Caculate | 9 | 电机/惯导数据处理、状态估计、LQR/PID/VMC 控制量计算 |
| Remote | 8 | SBUS 遥控器数据处理 |
| Control | 8 | 电机 MIT 力矩输出 |
| INS | 7 | 惯导数据采集与融合（IMU 信号量触发） |
| BoardLink | 4 | UART3 RS485 板间通信（10 ms 双向收发） |
| Defalut | 2 | WS2812 灯效 + 任务栈水位监控 |

> 详细任务架构（IPC、事件链数据流）见 `ARCHITECTURE.md`。

### 2. RoboMaster_Infantory_GimbalV1.0 — 云台控制固件

云台独立控制板固件，与上位机（视觉/主控板）通过 **USB CDC 虚拟串口** 通信。

- 自定义帧协议：帧头 `0x51 0x59` + 姿态数据 + CRC16 校验
- 接收上位机云台指令帧（`CDC_Receive_HS` 回调解包），发送云台 Roll / Pitch / Yaw 姿态
- 调试统计：收包次数、解包成功/失败计数、失败原因（长度/帧头/CRC）
- 运行状态：已启用 FreeRTOS，新增 `BoardLinkTask` 处理底盘板 UART3 RS485 通信；云台 Pitch + 射击电机控制仍按 `TASK_LOG.md` Phase 0-1 规划推进

### 3. Simmulation — 控制仿真

MATLAB 腿控仿真，用于生成底盘主控中使用的控制增益。

- `get_K_jiao_LQR.m`：基于动力学方程组（参考上交轮腿电控开源符号体系）符号求解 A/B 矩阵，计算腿长 LQR 反馈矩阵 K（支持定腿长模式）
- `Shangjiao_Leg.slx`：上身-腿动力学 Simulink 仿真模型

## 环境要求

- **IDE / 工具链**：Keil MDK（.uvprojx）或 CLion/VSCode + CMake + Ninja + arm-none-eabi-gcc（见 `CMakePresets.json`）
- **CubeMX**：STM32CubeMX 6.x（重新生成外设代码时使用 `.ioc`）
- **MATLAB**：Symbolic Math Toolbox + Simulink（仿真用，非编译必需）

## 构建方法

### CMake 方式（推荐）

```bash
cmake --preset Debug
cmake --build build/Debug
```

固件产物位于 `build/Debug/RoboMaster_Infantry.elf`。

### Keil 方式

打开 `.code-workspace` 中引用的 Keil 工程文件，选择目标芯片 STM32H723VGTx 后编译烧录。

## 控制方案概述

### 底盘 ↔ 云台板间通信

- **物理链路**：两块板的 `USART3` RS485，`PD8/PD9` 为 TX/RX，DE 由外设硬件控制，波特率 **2,000,000，8N1**
- **任务模型**：两端 `BoardLinkTask` 每 10 ms 发送一帧，同时通过 DMA + IDLE 接收；ISR 只复制入队，CRC 和业务解包在任务上下文执行
- **协议**：共享头文件 `Common/board_link_protocol.h`，帧头 `0xA5 0x5A`、版本、发送端、消息类型、负载长度、序号、负载和 CRC16
- **数据方向**：底盘发送姿态/腿长/轮速/目标速度；云台发送 HiPNUC 姿态、目标 Pitch 和状态字。两端保留最近有效帧及收发错误计数
- **当前状态**：两个工程已通过 CMake Debug 编译，尚未完成 RS485 实线、收发计数和掉线恢复联调

- **轮腿平衡**：轮腿（DM8009 × 4）支撑 + 动量轮（DM3519 × 2）姿态稳定，状态估计采用四元数 EKF 融合 BMI088 / DMIMU 数据
- **腿长控制**：LQR 负责平衡反馈，独立腿长 PID 负责 0.15/0.25/0.35 m 目标腿长，起身状态机控制支撑力偏置
- **地面检测**：轮速卡尔曼滤波 + 离地检测算法（`Ground_clearance_detection.c`）判断车轮离地状态
- **失效保护**：遥控通道 10 触发紧急停车（Emergency Task），电源管理带软开关控制（POWER）

## 开发规范

- **Bug 记录**：`RoboMaster_InfantoryV2.1.4/Bug.MD/` 按 BUG-XXX 编号记录（含模板）
- **任务跟踪**：完成后勾选 `TASK_LOG.md` / `REQUIREMENTS.md` 对应 checkbox
- **提交规范**：提交说明写明改动内容（如 `修复FDCAN滤波器配置`、`新增裁判系统移植`）
