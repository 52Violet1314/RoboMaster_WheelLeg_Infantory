---
id: "BUG-005"
title: "UART DMA 打印失效 — DTCM 缓冲区 DMA 不可达 + 回调空指针"
status: fixed
severity: high
date: "2026-05-31"
author: ""
tags: [DMA, UART, DTCM, DCache, FreeRTOS, STM32H7, 串口]
related_files:
  - BSP/bsp_UART.c
  - BSP/bsp_UART.h
  - STM32H723XG_FLASH.ld
  - Core/Src/usart.c
  - Core/Src/main.c
related_bugs: []
---

# BUG-005: UART DMA 打印失效

## 1. 现象 (Symptoms)

- `uart_print()` 改用 `HAL_UART_Transmit_DMA` 后完全无输出
- 串口助手上看不到任何打印内容
- 原阻塞模式 `HAL_UART_Transmit` 正常工作

## 2. 复现条件 (Reproduction)

1. 将 `uart_print` 实现改为 DMA 模式
2. 编译下载固件到 STM32H723
3. 串口助手观察 — 无任何输出

## 3. 根因分析 (Root Cause)

共三个相互独立的问题，任意一个都足以导致失效。

### 3.1 DTCM 缓冲区 DMA 不可达（主因）

```c
// bsp_UART.c — 修复前
static char uart_dma_buf[2][UART_BUFFER_SIZE] __attribute__((aligned(32)));
```

链接器脚本将 `.bss` 放置在 **DTCMRAM** (0x20000000, 128K)：

```ld
.bss (NOLOAD) : ALIGN(4)
{
    *(.bss)
    *(.bss*)
    *(COMMON)
} >DTCMRAM
```

STM32H7 的 DTCM (Tightly Coupled Memory) 是 CPU 专用总线，**不挂在 AXI/AHB 总线矩阵上**，DMA 控制器无法访问该地址空间。DMA 从 DTCM 读数据会读到全 0 或未定义值。

对比：SBUS 接收的 `rx_buff` 特意使用 `.sram4` 段（放在 RAM_D1 @ 0x24000000），DMA 可达：

```c
// main.c
__attribute__((section(".sram4"))) uint8_t rx_buff[BUFF_SIZE];
```

### 3.2 回调空指针死锁

```c
// bsp_UART.c — 修复前
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // uart_dma_done 在 bsp_UART_Init() 中才创建
        // 但 main.c 初始化阶段已调用 uart_print，触发 DMA → 回调
        xSemaphoreGiveFromISR(uart_dma_done, &woken);  // uart_dma_done == NULL!
    }
}
```

调用时序：
1. `main()` → `uart_print("初始化开始")` → `HAL_UART_Transmit_DMA`
2. DMA 完成 → `USART1_IRQHandler` → `HAL_UART_TxCpltCallback`
3. 此时 `bsp_UART_Init()` 尚未被调用 → `uart_dma_done == NULL`
4. `xSemaphoreGiveFromISR(NULL, ...)` → FreeRTOS `configASSERT` 触发 → 死循环

### 3.3 DCache 一致性问题

H7 上电后 `SCB_EnableDCache()` 启用了 L1-DCache。CPU 写入缓冲区的数据暂存在 DCache 中，未写回到物理 RAM。DMA 只能读物理 RAM，因此看到的是旧数据。

```c
// 修复：启动 DMA 前必须 Clean DCache
SCB_CleanDCache_by_Addr(buf, aligned_size);
HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
```

## 4. 解决方案 (Solution)

### 4.1 缓冲区移到 DMA 可达区域

```c
// bsp_UART.c
static char uart_dma_buf[2][UART_BUFFER_SIZE]
    __attribute__((aligned(32), section(".sram4")));
// .sram4 → RAM_D1 @ 0x24000000，DMA 可达
```

### 4.2 回调中空指针保护

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1 && uart_dma_done != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(uart_dma_done, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

### 4.3 DCache 维护

```c
// TX 方向：CPU 写 → Clean → DMA 读
SCB_CleanDCache_by_Addr(buf, (len + 31) & ~31);
HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
```

### 4.4 完整方案架构

```
uart_print()                      HAL_UART_TxCpltCallback() [ISR]
  │                                     ▲
  ├─ xSemaphoreTake(sem, 100ms)         │ UART TC 中断
  ├─ buf_idx ^= 1（双缓冲切换）         │ → xSemaphoreGiveFromISR(sem)
  ├─ vsnprintf → 缓冲区                │
  ├─ SCB_CleanDCache_by_Addr           │
  └─ HAL_UART_Transmit_DMA → 立即返回  │
     (DMA 后台异步发送)                 │
```

- Binary Semaphore 初始计数 1（available），DMA 完成 ISR 中 Give，发送前 Take
- 双缓冲：buf[0] 被 DMA 发送时，buf[1] 可安全填充
- 调度器启动前兼容：`uart_dma_done==NULL` 时跳过信号量操作

## 5. 验证方法 (Verification)

1. 编译下载，串口助手观察输出
2. 应看到 `初始化开始` → `初始化完毕` → `Running to app_main` 等正常打印
3. 多任务并发打印无乱码（双缓冲 + 信号量保护）
4. 调度器启动前的初始化打印正常（NULL 保护）

## 6. 经验教训 (Lessons Learned)

1. **STM32H7 DTCM ≠ 通用 SRAM**：DTCM 是 CPU 专用总线，任何 DMA/总线矩阵访问的缓冲区必须放在 D1/D2/D3 SRAM 中
2. **链接器脚本意识**：`static` 变量的物理位置由链接器决定，在 H7 上默认就是 DTCM。需要 DMA 的缓冲必须显式指定段
3. **DMA + DCache 是 H7 的经典陷阱**：TX = Clean（写回），RX = Invalidate（作废），缺一不可
4. **ISR 回调中的空指针**：初始化阶段的异步回调可能在资源创建之前触发，必须加 NULL 检查
5. **参考已有代码**：SBUS 的 DMA 接收实现已经示范了正确的 buffer 位置 + DCache 维护模式
