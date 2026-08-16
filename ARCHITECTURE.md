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

---

## 五、任务架构重构方案（2026-08-16 定稿）

### 5.1 现状问题清单

| # | 问题 | 位置 |
|---|---|---|
| 1 | Remote 任务职责过载：电机初始化 + SBUS 解析 + 急停 + 云台/射击控制全在一起，裸 `vTaskDelay` 凑节拍 | `app_Remote_Task.cpp` |
| 2 | Calculate 任务内缝合 Data 任务逻辑（电机数据搬运、状态变量、UART5 DMA 初始化） | `CalculateTask.cpp` |
| 3 | 急停内联在 Remote（优先级 8），被抢占时卸力延迟，非真正急停 | `app_Remote_Task.cpp:44` |
| 4 | `EnmegencyEventGroup` / `IMUSemaphore` / `SBUSQueue` 创建后未使用，死代码 | `app_Task.cpp` |
| 5 | INS 优先级 7 低于 Remote 8，IMU 采集优先级不合理 | `app_Task.cpp:108` |
| 6 | Emergency / Classic_Detect 任务为空壳 | `APP\APP_Task\src\` |

### 5.2 目标任务架构

**底盘板（7 任务）**

| 任务 | 优先级 | 周期 | 职责 | IPC |
|---|---|---|---|---|
| Emergency | 10 | 事件触发 | 全电机卸力（唯一职责） | 任务通知 |
| Caculate | 9 | 1ms | 纯计算：EKF/LQR/VMC → 通知 Control | 任务通知 |
| Control | 8 | 1ms | DM 电机 MIT 输出 → 通知回 Calculate | 任务通知 |
| INS | 8 | 1ms | IMU 采集+融合（内置 BMI088/DMIMU） | DataGroup 事件位 |
| Data | 7 | 1ms | SBUS 解析 + 电机数据 + 裁判 + 板间收，数据齐置就绪位 | DataGroup 事件位 |
| Check | 3 | 100ms | 链路心跳检查 + 模式状态机（ERO/STOP/RC/PC/AUTO）+ 蜂鸣器 | 全局时间戳 |
| Defalut | 2 | 500ms | WS2812 灯效 + 栈水位监控 | — |

**云台板（Phase 0 后，5 任务）**

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| Gimbal | 9 | 1ms | Pitch PID 闭环（HiPNUC 姿态源）+ 限位 + 重力补偿 |
| Shoot | 8 | 1ms | 摩擦轮稳速 + 拨弹 + 热量限制 + 堵转检测 |
| Nuc | 8 | 事件 | USB CDC 上位机/自瞄协议 |
| Board_UART | 7 | 1ms | 板间 UART 收发（DMA+IDLE） |
| Check | 3 | 100ms | 链路检查 + 模式状态机 |

### 5.3 关键设计决策

1. **计算↔控制改任务通知**：`ControlEventGroup` 两个 bit 替换为 `xTaskNotifyGive / ulTaskNotifyTake`（1 对 1 同步更快更省）；事件组只保留一对多场景（Data 就绪位可同时唤醒 Calculate 与 Check）
2. **急停机制**：Remote/Data 检测到通道 10 → `xTaskNotify` → Emergency（优先级 10）立即卸力，响应时间只受抢占延迟限制
3. **IMU 源抽象**：新增 `imu_source` 接口层，INS 任务代码双板共用；底盘选内置 BMI088（+DMIMU 融合，平衡不需要绝对航向），云台选外置 HiPNUC（绝对 yaw 用于指向）；切换数据源只改配置不改任务
4. **电机初始化集中化**：全部 `DM_Motor_Init` 抽到 `Board_Init()`，任务体内不再出现初始化代码
5. **射击控制迁云台板**：拨弹/摩擦轮随接线迁至云台板 FDCAN，底盘板 Remote 中的云台/射击控制段删除
6. **清理死代码**：`EnmegencyEventGroup`（启用或删除）、`IMUSemaphore`、`SBUSQueue` 按新设计归宿

### 5.4 实施步骤（依赖 TASK_LOG Phase 0-3）

| 步骤 | 内容 | 影响文件 |
|---|---|---|
| 1 | 拆 Remote：SBUS 解析保留，电机初始化抽入 `Board_Init()`，删除云台/射击控制段 | `app_Remote_Task.cpp`、新增 `Board_Init` |
| 2 | 恢复独立 Data 任务：从 Calculate 挖出"原 Data Task"段落 | `CalculateTask.cpp`、`app_Data_Task.cpp` |
| 3 | Emergency 任务实体化：通道 10 → 任务通知 → 优先级 10 卸力 | `app_Emergency_Task.cpp`、`app_Task.cpp` |
| 4 | Calculate↔Control 改任务通知，替换 `ControlEventGroup` | `CalculateTask.cpp`、`ControlTask.cpp` |
| 5 | Check 任务实体化：链路时间戳 + 状态机 | `app_Classic_Detect_Task.cpp` |
| 6 | INS 源抽象 + 优先级调整（INS 8） | `app_INSTask.cpp`、`HardWare/` |
| 7 | 云台板 Phase 0：启用 FreeRTOS + 移植 DM 电机驱动 + 接 FDCAN | GimbalV1.0 工程 |
| 8 | 云台板挂 Gimbal / Shoot 任务，射击控制迁入 | GimbalV1.0 工程 |
