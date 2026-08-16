# 任务日志（对标 SENTRY_LEG 完整版）

> 基准工程：`C:\Users\14420\Downloads\UTF-8__SENTRY_LEG\SENTRY_LEG`（哨兵完整版，F407 双板）
> 本仓库：STM32H723 双板（底盘 V2.1.4 + 云台 V1.0）
> 更新时间：2026-08-16

## 状态约定

- [ ] 未开始
- [~] 进行中
- [x] 已完成

---

## P0 — 云台板：电机控制 + 射击（当前最缺）

### T1. 云台电机控制（GimbalV1.0）
- [ ] 移植 DM 电机驱动到云台板（pitch DM3519，FDCAN），参考 SENTRY `Gimbal_Task.c` 的 PID 结构
- [ ] Pitch PID 闭环、重力补偿、限位（±30°/-24°）；yaw 电机硬件到位后补充
- [ ] 零位回中、遥控/上位机双目标源
- [ ] 启用 FreeRTOS 调度（当前裸机 while 循环，`USE_CMSIS_OS = 0`）
- 验收：云台能稳定跟随目标角度，限位不失控

### T2. 射击控制
- [ ] 摩擦轮双电机（DM3519 Shoot L/R）+ 拨弹轮控制
- [ ] 移植 `Shoot_Control()`：摩擦轮稳速 ~5500rpm、拨弹 ~6000rpm
- [ ] 堵转检测（卡弹处理）
- 验收：拨弹顺畅，卡弹能自动处理

### T3. 板间 UART 通信（底盘 ↔ 云台）
- [ ] 云台板新增 UART（与下板通信），DMA+IDLE 双缓冲，帧头+长度+CRC16
- [ ] 协议字段对齐 SENTRY `Board_Can_Task` 的 `Send_Message`：底盘→云台（车体姿态/模式/遥控/裁判数据），云台→底盘（云台角度/射击状态/上位机回传）
- 验收：两板互发数据帧 1kHz 无丢包

---

## P1 — 底盘板：裁判系统 + 安全功能

### T4. 裁判系统（V2.1.4）
- [ ] 移植 `Judge\` 全目录（protocol.h / Referee / fifo / CRC）
- [ ] USART6 接裁判系统，`Ref_Task` 解包：血量/热量/弹速/状态
- [ ] UI 静态/闪烁显示
- 验收：客户端能看到实时数据，电量耗尽自动断电逻辑生效

### T5. 紧急停车 + 状态机（补齐空壳）
- [ ] 实现 `app_Emergency_Task.cpp`（遥控通道 10 触发，创建任务）
- [ ] 实现 `app_Classic_Detect_Task.cpp`（对标 `Check_Task`：ERO/STOP/RC/PC/AUTO 模式机、外设断链检测）
- 验收：通道 10 立即卸力；断链自动进 STOP

### T6. 倒地自救援（Self_Rescue）
- [ ] 移植 `Self_Rescue.c`：俯/仰/侧倒姿态判断 + 起立动作序列
- 验收：任意方向倒地 5s 内自主站立

---

## P2 — 增强功能

### T7. 打滑 / 卡腿 / 越障检测
- [ ] 打滑检测（轮速卡尔曼 + 地面反力 Fn 判断）
- [ ] 卡腿处理、bump 越障模式（对照 SENTRY `Chassis_Task`）
- 验收：湿滑路面不失控；上台阶模式可越障

### T8. 超电控制（Super_Cap）
- [ ] 移植 `Super_Cap.c` + `Super_Cap_Task`（CAN 注册读写、功率限制）
- 验收：功率限制生效，无超功率扣血

### T9. 调试与遥控扩展
- [ ] VOFA 遥测（USB CDC 出姿态/电机/控制量）
- [ ] VT03 遥控 / PC 鼠标键盘控制（可选）
- [ ] 参数树集中管理 `Parameter.h`

---

## 备查：SENTRY 参考文件对照表

| 需求 | SENTRY 源文件 |
|---|---|
| 裁判系统 | `Chassis\Judge\*`（两板相同） |
| 云台控制 | `Gimbal\Task\Gimbal_Task.c`、`Gimbal\Controller\PID.c` |
| 射击/热量 | `Gimbal\Task\Limit_Task.c`、`Gimbal_Task.c` |
| 板间通信 | `Chassis\Task\Board_Can_Task.c`、`Gimbal\Task\Board_Can_Task.c` |
| 自救援 | `Chassis\APP\Self_Rescue.c`、`Chassis\Task\Chassis_Task.c` |
| 超电 | `Chassis\APP\Super_Cap.c`、`Chassis\Task\Super_Cap_Task.c` |
| 状态机/断链 | `Chassis\Task\Check_Task.c`（Gimbal 同名） |
| VOFA/VT03/鼠标 | `Chassis\APP\Vofa.c / VT03.c / Remote_Control.c` |

## 执行顺序

T1 → T2 → T3 → T4/T5 → T6 → T7/T8 → T9

---

# 实施方案（2026-08-16 定稿）

> 架构决策：**云台板接管云台+射击（SENTRY 式）**；板间通信用 **UART**（非 CAN）；当前无 yaw 电机，先做单轴 pitch + 射击，yaw 待硬件补充。

## 总体架构

| 板卡 | 现状 | 目标 |
|---|---|---|
| 底盘板 V2.1.4 | 腿+动量轮平衡；FDCAN3 挂 pitch/摩擦轮 | 只保留腿+动量轮；FDCAN3 腾出；接裁判系统 UART；新增板间 UART |
| 云台板 V1.0 | 裸机 IMU→USB 中继 | 启用 FreeRTOS；FDCAN 接管 pitch+2×摩擦轮；HiPNUC 继续做姿态源；USB 对接上位机/自瞄；板间 UART |

与 SENTRY 的两处刻意偏离：
1. 板间通信 UART（用户硬件要求）替代 SENTRY 板间 CAN
2. 单轴 pitch + 射击起步，yaw 后续补电机（车体转向暂代）

## Phase 0 — 云台板基础设施（先做）

- [ ] CubeMX 打开 `USE_CMSIS_OS=1`，重写 `freertos.c` 任务表：Gimbal / Shoot / Limit / Board_UART / Check（对标 SENTRY Gimbal 6 线程）
- [ ] 移植底盘板 `bsp_fdcan.c/h` 到云台板，波特率对齐 DM3519
- [ ] 移植底盘板 `app_motor.c` 的 DM 电机部分（`DM_Motor_Init` / MIT / Speed 函数族），初始化 pitch(0x011) + Shoot L/R(0x012/0x013)
- [ ] 改接线：3 个电机从底盘 FDCAN3 → 云台板 FDCAN；删除底盘板 `app_Remote_Task.cpp:21-25` 三行初始化
- [ ] 云台板 CMakeLists 加入新源文件 glob

## Phase 1 — 云台+射击闭环（T1/T2）

- [ ] Pitch 闭环：HiPNUC 姿态 → pitch PID → DM3519 速度环；移植 SENTRY `Gimbal_Task.c` PID 结构 + 限位（±30°/-24°）+ 重力补偿；PID 库复用底盘板 `HardWare/PID.c`
- [ ] 摩擦轮：DM3519 SPEED 模式，稳速 5500rpm 量级（`DM_Motor_Speed_Control` 现成）
- [ ] 堵转检测：指令速度 vs 反馈偏差超阈值×时间 → 反转清卡（参考 SENTRY jam 逻辑）
- [ ] USB 协议扩展：`0x51 0x59` 命令帧加 pitch 目标角、射击开关字段
- [ ] Limit_Task 雏形：内部热量模型，裁判接入后换真实数据

## Phase 2 — 板间 UART 通信（T3）

- [ ] 云台板新增 UART（115200~921600 bps，DMA+IDLE 双缓冲；**缓冲区放 D3 域，勿放 TCM**，见 Bug.MD BUG-005）
- [ ] 帧协议：帧头+长度+CRC16+消息类型；字段对齐 SENTRY `Send_Message`
- [ ] 1kHz 双向，队列+事件组解耦（沿用 `app_Task.cpp` IPC 风格）

## Phase 3 — 裁判系统（T4）

- [ ] 裁判串口接底盘板；移植 `Judge\` 全目录（纯 C 平台无关，仅 USART 收发适配 H7 HAL）
- [ ] `Ref_Task` 5ms 解包 → 血量/热量/状态入 `Classic_Data` → 板间 UART 转发云台板
- [ ] 云台板 Limit_Task 换真实热量数据

## Phase 4 — 安全（T5/T6）

- [ ] `app_Emergency_Task.cpp`：通道 10 → 事件组 → 全电机卸力
- [ ] `app_Classic_Detect_Task.cpp`：对标 `Check_Task` 状态机（ERO/STOP/RC/AUTO + 断链检测）
- [ ] `Self_Rescue` 移植到底盘板（CalculateTask 外挂状态机）

## Phase 5 — 增强（T7-T9）

- [ ] 打滑/卡腿/越障（对照 SENTRY `Chassis_Task`）
- [ ] 超电控制（需硬件到位）
- [ ] VOFA 遥测 / VT03 / 鼠标键盘 / 参数树

## 关键风险

1. **H7 移植坑**：DMA 缓冲区区域（BUG-005）、FDCAN 滤波器（BUG-001 前科）、DWT 使能
2. **板间 UART 带宽**：1kHz 全姿态帧建议 ≥921600 bps
3. **接线变更**：FDCAN3 空置后先不删 CubeMX 配置，避免返工
