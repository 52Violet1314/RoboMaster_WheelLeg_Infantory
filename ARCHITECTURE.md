# 当前任务架构梳理

> 仓库：RoboMaster_Wheeleg_Infantory（STM32H723 双板：底盘 V2.1.4 + 云台 V1.0）
> 整理时间：2026-08-16

---

## 一、底盘板 V2.1.4（FreeRTOS，5 任务）

### 任务列表

| 优先级 | 任务 | 栈深 | 功能 |
|---|---|---|---|
| 9 | Caculate | 256×6 | 姿态解算、状态估计、LQR/PD 控制量计算 |
| 8 | Control | 256×4 | DM 电机 MIT 力矩输出（FDCAN1） |
| 8 | Remote | 256×4 | SBUS 遥控数据处理 |
| 7 | INS | 256×4 | IMU 数据采集与融合 |
| 2 | Defalut | 256×2 | WS2812 灯效 + 任务栈水位监控 |

### IPC 资源（app_Task.cpp）

| 类型 | 名称 | 用途 | 状态 |
|---|---|---|---|
| 队列 | SBUSQueue | SBUS 数据（16 帧 × 16ch） | 使用中 |
| 信号量 | IMUSemaphore | 二值信号量（声明） | 创建未使用 |
| 事件组 | ControlEventGroup | Caculate ↔ Control 握手 | 使用中 |
| 事件组 | DataGroup | IMU/SBUS 数据就绪位 | 使用中 |
| 事件组 | EnmegencyEventGroup | 紧急事件 | 创建未使用 |

### 核心数据流（1ms 事件链）

```
USART SBUS (DMA+IDLE 中断)
        │ SBUSQueue
        ▼
Remote ──SBUS_DATA_READY_BIT──► DataGroup
                                    │
SPI BMI088 ──► INS (1ms) ──IMU_DATA_READY_BIT──► DataGroup
                                    │
          ┌─────────────────────────┘
          ▼
Caculate ── xEventGroupWaitBits(等待两数据就绪)
          │  状态估计(EKF/卡尔曼) → LQR/PD → 控制量写入 Classic_Data
          ▼
  Calculate_OK_BIT ──► ControlEventGroup
          ▼
Control ── xEventGroupWaitBits(Calculate_OK_BIT)
          │  读控制量 → DM_Motor_MIT_Control → FDCAN1 输出
          ▼
  Control_OK_BIT ──► 下一周期 Caculate 继续
```

### 遗留问题

1. `EnmegencyTask` 已声明未创建（app_Task.cpp:24），`app_Emergency_Task.cpp` / `app_Classic_Detect_Task.cpp` 为空壳
2. `IMUSemaphore` 创建后未使用（INS 实际走 1ms 轮询 + 事件位）
3. `app_Data_Task` 已合并进 Calculate Task（未单独创建）
4. FDCAN3 上挂的 DM3519 pitch / 摩擦轮电机无控制逻辑（仅初始化，方案中迁往云台板）

---

## 二、云台板 V1.0（无 RTOS，裸机）

### 主循环数据流

```
USART2 HiPNUC HI14 (DMA+IDLE 中断)
        │ 逐字节喂 hipnuc_input()
        ▼
main() while(1) ──► 解包成功
        │           ├─► printf 日志 (USART1 调试串口)
        │           └─► USB CDC 姿态帧 (0x51 0x59 + CRC16)
        ▲
CDC_Receive_HS 回调 ── 上位机命令帧解包（帧头+CRC16 校验）
```

- `USE_CMSIS_OS = 0`：FreeRTOS 已由 CubeMX 配置，但调度器未启动（裸机循环）
- 角色：纯 IMU 姿态中继板，无电机控制

### 硬件资源现状（main.c 已全部初始化）

| 外设 | 状态 |
|---|---|
| FDCAN1 / FDCAN2 / FDCAN3 | 已初始化，未用（可接云台/射击电机） |
| USART1 | 调试 printf |
| USART2 | HiPNUC DMA+IDLE 接收 |
| USART3 / UART5 / UART7 | 已初始化，空闲（可做板间通信/裁判系统） |
| SPI2 / SPI6 | 已初始化（BMI088 备用） |
| TIM3 / TIM5 / TIM12 | 已初始化 |
| USB HS CDC | 与上位机通信 |

---

## 三、对标 SENTRY 的任务视角差距

| 维度 | SENTRY 完整版 | 当前工程 |
|---|---|---|
| 底盘任务数 | 6（含 Board_Can / Ref / Super_Cap / Check） | 5（缺板间通信、裁判、超电、链路检测） |
| 云台任务数 | 6（Gimbal / Nuc / Limit / Board_Can / Check / Ins） | 0（裸机循环） |
| 1ms 事件同步链 | 有 | 有（设计一致，可直接沿用） |
| IPC 机制 | 队列 + 事件组 | 同款（EnmegencyEventGroup 待启用） |

## 四、结论

1. **底盘板**：1ms 事件链架构与 SENTRY 同思路，扩展新任务（Ref / Super_Cap / Board_UART / Check）直接挂在现有 IPC 上即可，无需重构
2. **云台板**：需先补 FreeRTOS 任务框架（Phase 0，`USE_CMSIS_OS=1`），再挂 Gimbal / Shoot / Limit / Board_UART 任务
3. **硬件**：云台板外设富余（3 路 FDCAN + 多路 UART 空闲），Phase 0 不用改 CubeMX 配置
