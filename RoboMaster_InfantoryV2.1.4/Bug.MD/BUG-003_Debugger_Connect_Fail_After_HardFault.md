---
id: "BUG-003"
title: "HardFault 后调试器无法连接 MCU / 无线 DAPLink 无法识别"
status: fixed
severity: critical
date: "2026-05-30"
author: ""
tags: [HardFault, Debug, OpenOCD, CMSIS-DAP, DAPLink, ST-Link, SWD, 调试器连接]
related_files:
  - startup_stm32h723xx.s
  - .vscode/launch.json
  - CMakeLists.txt
  - Core/Inc/FreeRTOSConfig.h
related_bugs:
  - BUG-002
---

# BUG-003: HardFault 后调试器无法连接 MCU / 无线 DAPLink 无法识别

## 1. 现象 (Symptoms)

- 固件进入 HardFault 后，调试器（DAPLink / ST-Link）无法通过 SWD 连接到 STM32H723
- 换用不同烧录器仍然无法连接
- 无线 DAPLink 报错：`CMSIS-DAP command 0x0 not implemented` / `CMD_INFO failed`
- Cortex-Debug 插件显示：`OpenOCD: GDB Server Quit Unexpectedly`

## 2. 复现条件 (Reproduction)

1. 固件因 FPU 配置不一致进入 HardFault（参见 BUG-002）
2. HardFault Handler 为死循环（`Default_Handler: b Infinite_Loop`）
3. MCU 上电后立即崩溃，调试器尝试连接时 MCU 已处于不可控状态
4. 使用无线 DAPLink 时，PC 端发射器未被 Windows 识别为 CMSIS-DAP 设备

## 3. 根因分析 (Root Cause)

存在两层独立的问题：

### 3.1 软件层：调试器连接策略不当

`launch.json` 中 OpenOCD 配置使用了 `connect_deassert_srst`：

```
reset_config srst_only srst_nogate connect_deassert_srst
```

此配置在连接期间**释放** MCU 复位，意味着固件在调试器接管前就已开始执行。如果固件一上电就进入 HardFault 死循环（或重映射 SWD 引脚、关闭调试时钟），调试器将无法在 SWD 层面与内核建立通信。

**解决**：改为 `connect_assert_srst`，连接期间保持 MCU 复位，待 GDB 完全接管后再释放。

### 3.2 硬件层：无线 DAPLink 未被 Windows 识别

- Windows 设备管理器中没有 CMSIS-DAP 设备
- OpenOCD 使用的 `interface/cmsis-dap.cfg` 默认后端为 USB（`auto | usb_bulk | hid`）
- 该版本的 OpenOCD（0.12.0 msys64 构建）不支持 `cmsis_dap_backend tcp`，无法通过 WiFi 连接无线 DAPLink
- 无线 DAPLink 的 PC 端发射器需要专门的驱动或配套软件才能暴露为 CMSIS-DAP USB 设备

**临时方案**：改用有线 ST-Link（Windows 已正确识别 `VID_0483 PID_3748/374B`）。

### 3.3 附加因素：启动文件 FPU 配置不一致

`startup_stm32h723xx.s` 第 29 行仍为 `.fpu softvfp`，而 CMakeLists.txt 已设置了 `-mfloat-abi=hard -mfpu=fpv5-sp-d16`。此不一致虽不直接阻止调试器连接，但加重了 HardFault 的严重性，使 MCU 更快进入不可恢复状态。

## 4. 解决方案 (Solution)

### 4.1 修改 launch.json 为 connect_assert_srst（两处 OpenOCD 配置）

```json
"openOCDLaunchCommands": [
    "transport select swd",
    "adapter speed 100",
    "reset_config srst_only srst_nogate connect_assert_srst"
],
"preLaunchCommands": [
    "monitor halt"
]
```

- 去掉 `monitor reset init`，避免覆盖 connect_assert_srst 的复位保持效果
- 连接流程：断言 NRST → SWD 握手 → GDB 接管 → 释放 NRST → halt

### 4.2 修复 startup_stm32h723xx.s

```asm
; 第 29 行
.fpu fpv5-sp-d16    ; 原为 softvfp
```

### 4.3 无线 DAPLink 的替代方案

在本机 OpenOCD 不支持 TCP 后端的情况下，优先使用 ST-Link：

- `"STLink (ST-Link GDB Server)"` — 使用 ST 官方 GDB Server
- `"STLink (OpenOCD)"` — 使用 OpenOCD + stlink 驱动

无线 DAPLink 需待后续升级 OpenOCD 或安装配套驱动后再启用。

## 5. 验证方法 (Verification)

- [x] startup `.fpu` 改为 `fpv5-sp-d16`
- [x] launch.json 中 `connect_deassert_srst` → `connect_assert_srst`
- [x] launch.json 中移除多余的 `monitor reset init`
- [ ] 重新编译固件
- [ ] 使用 ST-Link (OpenOCD) 配置启动调试，确认能正常连接并停在 `main`
- [ ] 确认 HardFault 不再发生（参见 BUG-002 验证列表）
- [ ] 后续可选：解决无线 DAPLink 驱动问题，恢复无线调试能力

## 6. 经验教训 (Lessons Learned)

1. **`connect_assert_srst` 是连接"已崩溃 MCU"的关键**：当 MCU 处于不可控状态时，只有保持复位才能让调试器强行接管；`connect_deassert_srst` 假定固件是正常的
2. **调试器问题 ≠ 代码问题**：CMSIS-DAP CMD_INFO 失败是 USB 层面的故障，与固件源码无关；排查时应先确认 Windows 设备管理器中存在对应设备
3. **无线调试器增加了一层不确定性**：有线调试器的连接问题是"通或不通"，无线调试器引入了驱动、配对、网络栈等多个额外故障点
4. **始终保留一个可靠的有线调试方案作为后备**：ST-Link 是 STM32 生态中最成熟稳定的调试器，在排查无线方案问题时应优先用它验证芯片本身是否正常
5. **Fault Handler 应至少输出 SCB 寄存器到串口**：否则每次 HardFault 都是盲调（已在 BUG-002 中记录，此处再次强调）
