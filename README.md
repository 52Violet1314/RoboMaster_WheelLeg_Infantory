# RoboMaster 轮腿步兵机器人固件仓库

RoboMaster 高校联盟赛轮腿步兵机器人嵌入式软件工程，基于 STM32H723VGT6 主控，采用 FreeRTOS 实时操作系统，C/C++ 混合开发。

本仓库包含三个子工程：**底盘控制固件（V2.1.4）**、**云台控制固件（Gimbal V1.0）** 以及 **MATLAB 控制仿真（Simulation）**。

## 目录结构

```
RoboMaster_Wheeleg_Infantory/
├── RoboMaster_InfantoryV2.1.4/       # 轮腿步兵底盘控制固件（主工程）
├── RoboMaster_Infantory_GimbalV1.0/  # 云台控制固件
├── Simmulation/                      # 腿长 LQR 求解 + 腿控仿真
└── README.md
```

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
| `APP/` | 应用层：任务（INS / Remote / Calculate / Control / Emergency / Data）与算法（四元数 EKF、轮速卡尔曼、离地检测、VMC 虚拟力控制、腿长 LQR 控制） |
| `Middlewares/` | FreeRTOS、ARM CMSIS-DSP 等第三方中间件 |
| `Drivers/` | STM32H7 HAL 驱动库、CMSIS |

**控制任务框架**（`APP/APP_Task/src/app_Task.cpp`）：

| 任务 | 优先级 | 功能 |
|------|--------|------|
| Caculate | 9 | 姿态解算、状态估计、LQR/PD 控制量计算 |
| Remote | 8 | SBUS 遥控器数据处理 |
| Control | 8 | 电机 MIT 力矩输出 |
| INS | 7 | 惯导数据采集与融合（IMU 信号量触发） |
| Defalut | 2 | WS2812 灯效 + 任务栈水位监控 |

### 2. RoboMaster_Infantory_GimbalV1.0 — 云台控制固件

云台独立控制板固件，与上位机（视觉/主控板）通过 **USB CDC 虚拟串口** 通信。

- 自定义帧协议：帧头 `0x51 0x59` + 姿态数据 + CRC16 校验
- 接收上位机云台指令帧（`CDC_Receive_HS` 回调解包），发送云台 Roll / Pitch / Yaw 姿态
- 调试统计：收包次数、解包成功/失败计数、失败原因（长度/帧头/CRC）

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

- **轮腿平衡**：轮腿（DM8009 × 4）支撑 + 动量轮（DM3519 × 2）姿态稳定，状态估计采用四元数 EKF 融合 BMI088 / DMIMU 数据
- **腿长控制**：基于 MATLAB 求解的 LQR 增益实现定腿长与变腿长状态机（`LegLengthState_t`）
- **地面检测**：轮速卡尔曼滤波 + 离地检测算法（`Ground_clearance_detection.c`）判断车轮离地状态
- **失效保护**：遥控通道 10 触发紧急停车（Emergency Task），电源管理带软开关控制（POWER）
