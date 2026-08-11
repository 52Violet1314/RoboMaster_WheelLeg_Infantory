---
id: "BUG-001"
title: "FDCAN1 过滤器配置错误导致无法接收电机反馈数据"
status: fixed
severity: critical
date: "2026-05-30"
author: ""
tags: [FDCAN, CAN, STM32H7, 过滤器, 电机通信]
related_files:
  - BSP/bsp_fdcan.c
  - Core/Src/fdcan.c
  - Core/Inc/fdcan.h
related_bugs: []
---

# BUG-001: FDCAN1 过滤器配置错误导致无法接收电机反馈数据

## 1. 现象 (Symptoms)

- FDCAN1 总线上连接了 6 个达妙电机（ID: 0x091 ~ 0x096），但只能收到 ID 最低的电机数据
- 其余 5 个电机的反馈数据全部丢失
- 发送控制命令（0x51 ~ 0x56）正常，电机能响应，但反馈收不到

## 2. 复现条件 (Reproduction)

1. 将 6 个达妙电机连接到 FDCAN1 总线
2. 使用 CubeMX 默认生成的 FDCAN 过滤器配置（Mask 模式，FilterID1/FilterID2 均为 0x00）
3. 上电运行，观察电机反馈数据
4. 只会收到一个电机的数据

## 3. 根因分析 (Root Cause)

原过滤器配置使用了 `FDCAN_FILTER_MASK` 模式：

```c
fdcan_filter.FilterType = FDCAN_FILTER_MASK;
fdcan_filter.FilterID1 = 0x00;
fdcan_filter.FilterID2 = 0x00;
```

在 STM32H7 FDCAN 的 MASK 模式下，接受的 ID 必须满足：

```
(received_id & ~Mask) == (FilterID1 & ~Mask)
```

当 Mask（FilterID2）= 0x00 时，`~Mask = 0x7FF`（全 1），只有 `received_id == 0x00` 的消息被接受。电机反馈 ID 为 0x091~0x096，全部被硬件过滤器丢弃。

## 4. 解决方案 (Solution)

将 FDCAN1 过滤器从 `FDCAN_FILTER_MASK` 改为 `FDCAN_FILTER_RANGE`，覆盖电机 ID 范围：

```c
fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
fdcan_filter.FilterID1 = 0x00;    // 起始ID
fdcan_filter.FilterID2 = 0x7FF;   // 结束ID，覆盖所有标准ID
```

配合全局过滤器：

```c
HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                             FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
```

**实际采用方案**：接受全部标准 ID（0x000 ~ 0x7FF），在软件层通过 `fdcan1_rx_callback()` 中的 `switch(rec_id)` 做二次过滤。这样即使在调试期间更换电机或调整 ID，也不需要重新配置硬件过滤器。

## 5. 验证方法 (Verification)

- [ ] 确认 `can_filter_init()` 中 FDCAN1 的 `FilterType` 为 `FDCAN_FILTER_RANGE`
- [ ] 确认 `FilterID2` 不小于 0x096（最大电机 ID）
- [ ] 上电后检查 `Motor_Data_ReadyBit` 是否能达到 0x3F（6 个电机数据全部就绪）
- [ ] 通过调试器在 `fdcan1_rx_callback()` 中打断点，确认每个电机 ID 都能进入对应 case

## 6. 经验教训 (Lessons Learned)

1. **FDCAN Mask vs Range 模式的区别**：Mask 模式下 FilterID2 是掩码（位到位比较），不是上限值；Range 模式下才是 [FilterID1, FilterID2] 的闭区间
2. **CubeMX 生成代码的陷阱**：CubeMX 默认生成的过滤器参数通常需要手动调整，不能直接使用
3. **硬件过滤 + 软件过滤双层策略**：硬件过滤器做粗筛（接受整个 ID 范围），软件层做精确匹配，兼顾灵活性和性能
4. **可以提炼为 Skill 的检查项**：STM32H7 FDCAN 初始化后，检查所有 `can_filter_init()` 中的 FilterType 和 FilterID 值是否与总线设备 ID 匹配
