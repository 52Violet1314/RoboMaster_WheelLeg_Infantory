---
id: "BUG-002"
title: "FPU 未启用 + FreeRTOS 配置不一致导致 HardFault"
status: fixed
severity: critical
date: "2026-05-30"
author: ""
tags: [HardFault, FPU, FreeRTOS, CMSIS-DSP, STM32H7, Cortex-M7, 启动文件]
related_files:
  - Core/Inc/FreeRTOSConfig.h
  - startup_stm32h723xx.s
  - CMakeLists.txt
  - Core/Src/system_stm32h7xx.c
  - Core/Src/stm32h7xx_it.c
related_bugs:
  - BUG-003
---

# BUG-002: FPU 未启用 + FreeRTOS 配置不一致导致 HardFault

## 1. 现象 (Symptoms)

- DAPLINK Debug 时卡在 `HardFault_Handler` 的死循环中
- 所有 Fault Handler（NMI、HardFault、MemManage、BusFault）均为空实现，无法输出任何诊断信息
- 无法定位崩溃发生的位置和原因

## 2. 复现条件 (Reproduction)

1. 编译下载当前固件到 STM32H723
2. DAPLINK 连接调试
3. 程序执行到 `app_main()` → FreeRTOS 调度器启动后 → 某个任务中调用 CMSIS-DSP 函数（如 `arm_sqrt_f32()`）时触发 HardFault

## 3. 根因分析 (Root Cause)

STM32H723 的 Cortex-M7 内核包含硬件 FPU（`__FPU_PRESENT = 1`），但存在三处配置不一致：

### 3.1 CMakeLists.txt 缺少 FPU 编译标志

```cmake
# 当前：未设置任何 FPU 相关标志
# 编译器默认不生成硬件浮点指令
```

缺少 `-mfloat-abi=hard -mfpu=fpv5-sp-d16`。

### 3.2 但链接了硬件 FPU 版本的 CMSIS-DSP 库

```cmake
# CMakeLists.txt 第 133 行
target_link_libraries(${CMAKE_PROJECT_NAME}
    ${CMSIS_DSP_LIB}/libarm_cortexM7lfsp_math.a  # "lf" = Little-endian + FPU
)
```

即使编译器未生成硬件 FPU 指令，链接的库函数包含 `VLDR`、`VMUL.F32` 等硬件 FPU 指令。

### 3.3 SystemInit() 中 FPU 启用是条件编译

```c
// system_stm32h7xx.c 第 187 行
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));
#endif
```

`__FPU_USED` 由 CMSIS 头文件根据编译器是否定义了 `__FPU_PRESENT` 和浮点宏自动推导。当编译器未设置 `-mfpu` 时 → `__FPU_USED = 0` → `SCB->CPACR` 不被设置 → CP10/CP11 协处理器保持禁用状态。

### 3.4 执行路径

```
app_main() → Task 调用 arm_sqrt_f32() 
  → 跳转到 libarm_cortexM7lfsp_math.a 中的硬件 FPU 指令
    → CP10 未启用 → NOCP UsageFault → HardFault
```

### 3.5 其他相关配置问题

| 配置 | 当前值 | 问题 |
|---|---|---|
| `FreeRTOSConfig.h` `configENABLE_FPU` | 0 | FreeRTOS 上下文切换不保存 FPU 寄存器 S0-S31 |
| `startup_stm32h723xx.s` `.fpu` | `softvfp` | 汇编代码不假设 FPU 存在 |
| FreeRTOS 移植层 | `ARM_CM4F` (`port.c`) | 应为 `ARM_CM7`（Cortex-M7 有额外的 FPU 特性） |

## 4. 解决方案 (Solution)

### 4.1 启用硬件 FPU（核心修复）

在 `CMakeLists.txt` 中添加 FPU 编译标志：

```cmake
# 在 target_compile_options 之前添加
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard -mfpu=fpv5-sp-d16")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=hard -mfpu=fpv5-sp-d16")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mfloat-abi=hard -mfpu=fpv5-sp-d16")
```

### 4.2 修改 FreeRTOS 配置

```c
// FreeRTOSConfig.h
#define configENABLE_FPU    1   // 改为 1
```

### 4.3 修复启动文件

```asm
; startup_stm32h723xx.s 第 29 行
.fpu fpv5-sp-d16    ; 替换 softvfp
```

### 4.4 FreeRTOS 移植层说明

FreeRTOS v10.3.1 发行版中 Cortex-M4F 和 Cortex-M7 共用 `ARM_CM4F` 移植层，
不存在独立的 `ARM_CM7` 目录。`ARM_CM4F/port.c` 同时适用于 M4F 和 M7 内核，
FPU 上下文保存/恢复逻辑相同。**无需修改此路径。**

### 4.5 可选：添加栈溢出检测

```c
// FreeRTOSConfig.h
#define configCHECK_FOR_STACK_OVERFLOW    2   // 方法 2：在栈底放置 canary
```

## 5. 验证方法 (Verification)

1. **GDB 寄存器验证**（修复前确认）：
   ```gdb
   # 在 HardFault 处
   (gdb) p/x (*(uint32_t*)0xE000ED28)   # SCB->CFSR
   # bit 18-19 (NOCP) == 1 → FPU 问题确认
   (gdb) p/x (*(uint32_t*)0xE000ED2C)   # SCB->HFSR
   # bit 30 (FORCED) == 1 → 由 UsageFault 升级
   ```

2. **修复后验证**：
   - [x] 确认 `configENABLE_FPU = 1`（FreeRTOSConfig.h 第 59 行）
   - [x] 确认 `startup_stm32h723xx.s` 第 29 行为 `.fpu fpv5-sp-d16`
   - [x] 确认 CMakeLists.txt 中 FPU 编译标志已添加
   - [x] 确认 FreeRTOS 移植层 `ARM_CM4F` 适用于 Cortex-M7（无需改为 ARM_CM7）

## 6. 经验教训 (Lessons Learned)

1. **FPU 配置必须全链路一致**：编译器标志 → 启动文件 → FreeRTOS → 链接库 → 系统初始化，任意一处断开就 HardFault
2. **CMSIS-DSP 的 `lf` 后缀**：链接前必须确认是 `lf`（硬件 FPU）还是 `l`（软浮点）版本，必须与编译器 FPU 标志匹配
3. **STM32H7 的 FreeRTOS 移植层应是 `ARM_CM7`** 而非 `ARM_CM4F`，Cortex-M7 的双精度 FPU 有不同的保存/恢复逻辑
4. **Fault Handler 必须有诊断能力**：至少输出 SCB 寄存器值到串口，否则每次 HardFault 都是盲调
5. **CubeMX 默认配置的陷阱**：生成的 `FreeRTOSConfig.h` 中 `configENABLE_FPU=0` 是 CubeMX H7 生成代码的已知问题
