---
id: "BUG-004"
title: "无线 DAPLink 烧录失败：多种无线调试器无法完成固件烧录"
status: fixed
severity: major
date: "2026-05-31"
author: ""
tags: [Wireless, DAPLink, CMSIS-DAP, pyOCD, SWD, Flash, HardFault, Bluetooth, WiFi, DAP, nRESET]
related_files:
  - build/Debug/RoboMaster_Infantry.elf
related_bugs:
  - BUG-003
---

# BUG-004: 无线 DAPLink 烧录失败

## 1. 现象 (Symptoms)

测试了 4 种无线 CMSIS-DAP 调试器，烧录 STM32H723 固件时均遇到不同程度的失败：

### 1.1 Horco CMSIS-DAP（蓝牙）

| 步骤 | 结果 |
|------|------|
| `pyocd list --probes` | ✅ 识别为 `Horco Horco CMSIS-DAP` |
| `pyocd commander ... status` | ✅ 连接成功，Core 0: Running |
| `pyocd commander ... halt` | ✅ 成功 halt |
| `pyocd flash`（默认参数） | ❌ `AssertionError: assert self.dp.is_reset_asserted()` |
| `pyocd flash -O connect_mode=attach` | ❌ `target was not halted as expected after calling flash algorithm routine (IPSR=3)` |
| `pyocd commander ... load`（attach 模式） | ❌ 同上，进度条走到一半 HardFault |
| 芯片状态 | ⚠️ 被半擦除，原有程序丢失 |

### 1.2 正点原子 ATK-HS-V3 CMSIS-DAP（蓝牙）

| 步骤 | 结果 |
|------|------|
| `pyocd list --probes` | ✅ 识别为 `ATK ATK-HS-V3-CMSIS-DAP` |
| `pyocd commander ... status`（默认） | ❌ `SWD/JTAG communication failure (No ACK)` |
| `pyocd commander -M under-reset` | ❌ 同上 |
| `pyocd commander -M pre-reset` | ❌ 同上 |
| `pyocd commander -M attach -f 100000` | ❌ 同上 |
| 根因 | SWD 物理链路无应答，DAP 端 SWD 排针与芯片未正确连接 |

### 1.3 TaiOuSi Slim CMSIS-DAP（WiFi USB）

| 步骤 | 结果 |
|------|------|
| `pyocd list --probes` | ⚠️ 有时能识别 `TaiOuSi Slim CMSIS-DAP`，有时超时 |
| `pyocd commander ... status` | ❌ `Timeout reading from probe` |
| `pyocd commander ... status`（重试） | ❌ `Probe not found` |
| 根因 | WiFi 虚拟 USB 延迟过高，pyOCD 的 `pyusb_v2_backend.open()` 超时 |

### 1.4 Horco CMSIS-DAP（第二个连接实例，ID 不同）

| 步骤 | 结果 |
|------|------|
| `pyocd list --probes` | ✅ 识别（Unique ID 与第一次不同：`102021325370`） |
| `pyocd commander ... halt` | ✅ 成功 halt |
| `pyocd flash -O connect_mode=attach -O reset_type=default` | ✅ **成功！** |

## 2. 复现条件 (Reproduction)

1. 使用任意无线 CMSIS-DAP 调试器（蓝牙或 WiFi）
2. SWD 线正确连接（SWCLK → PA14，SWDIO → PA13，GND → GND，可选 3V3）
3. 执行 `pyocd flash -t stm32h723xx -u <PROBE_ID> <ELF>`
4. 观察到以下至少一种错误：
   - `AssertionError`（`safe_reset_and_halt` 中 `is_reset_asserted()` 返回 False）
   - `target was not halted ... IPSR=3`（flash 算法 HardFault）
   - `SWD/JTAG communication failure (No ACK)`
   - `Timeout reading from probe`

## 3. 根因分析 (Root Cause)

### 3.1 核心问题：无线 DAP 普遍缺少 nRESET 信号线

pyOCD 的 `stm32h723xx` 目标内置了 `safe_reset_and_halt` 初始化序列：

```python
# target_STM32H723xx.py 第 151 行
def safe_reset_and_halt(self):
    assert self.dp.is_reset_asserted()  # 断言 nRESET 已断言
    ...
```

无线 DAP 的 SWD 排针通常只有 4 根（SWCLK / SWDIO / GND / 3V3），**未引出 nRESET 线**。PC 端无法通过调试器控制芯片硬件复位，导致 `is_reset_asserted()` 始终返回 `False`，断言失败。

### 3.2 Flash 算法 HardFault（IPSR=3）的原因

蓝牙 HID 带宽极低（~10-15 KB/s），而 STM32H723 的 Flash 写入需要严格的时序：

- Flash 算法被下载到 SRAM 中执行
- 算法通过 SWD 轮询 Flash 状态寄存器
- 蓝牙 HID 每次 64 字节的报文 + 往返延迟（> 10ms），导致轮询间隔过长
- Flash 某些操作（如 sector erase）的超时窗口内未完成状态检查，触发 HardFault

在 `connect_mode=attach` + `reset_type=default` 下，pyOCD 采用了更宽松的初始化流程，绕过了部分对 nRESET 的依赖，使 Flash 算法能在较高容错下运行——最终以 9.95 KB/s 的速度完成烧录。

### 3.3 SWD No ACK（正点原子 ATK-HS-V3）

日志显示 pyOCD 与 DAP 固件的 HID 通信完全正常（`open`、`set_clock`、`swj_sequence` 均成功），但在 SWD `DP IDCODE` 读取阶段收到 No ACK。这完全是 DAP 端 SWD 物理连接问题（接线或电平不匹配），与无线传输无关。

### 3.4 WiFi USB 超时（TaiOuSi Slim）

TaiOuSi 通过 WiFi 将 CMSIS-DAP 封装为虚拟 USB 设备。pyOCD 使用 `pyusb_v2_backend` 打开设备时，WiFi 延迟导致 USB 枚举和首次通信超时，设备在 `open()` 阶段就失败了。

## 4. 解决方案 (Solution)

### 最终成功的命令

```bash
pyocd flash \
  -u <PROBE_UNIQUE_ID> \
  -t stm32h723xx \
  -O connect_mode=attach \
  -O reset_type=default \
  D:/STM32WorkSpace/RoboMaster_InfantoryV2.1.2/build/Debug/RoboMaster_Infantry.elf
```

关键参数说明：

| 参数 | 作用 |
|------|------|
| `-O connect_mode=attach` | 绕过 `safe_reset_and_halt` 中的 nRESET 断言，以 attach 模式连接已运行的芯片 |
| `-O reset_type=default` | 禁用 default reset 流程中的硬件复位断言步骤 |

### 备用方案：串口 Bootloader

当所有无线 DAP 均不可用时，可利用 STM32H723 内置 System Bootloader：

1. BOOT0 拉高，复位 → 芯片进入 bootloader
2. PC 端通过蓝牙串口模块连接 USART1
3. 使用 `stm32flash` 烧录：
   ```bash
   stm32flash -w build/Debug/RoboMaster_Infantry.elf -v -g 0 COMx
   ```
4. BOOT0 拉低，复位 → 从 Flash 正常启动

## 5. 验证方法 (Verification)

- [x] Horco CMSIS-DAP 使用 `connect_mode=attach` + `reset_type=default` 成功烧录
- [x] 烧录完成后 `pyocd commander status` 显示 `Core 0: Running`
- [ ] 其他无线 DAP（正点原子 ATK-HS-V3、TaiOuSi Slim）在 SWD 接线修正后，使用相同参数验证
- [ ] 有线 ST-Link 交叉验证芯片功能正常

## 6. 经验教训 (Lessons Learned)

### 6.1 无线 DAP 烧录的通用规则

- **必须使用 attach 模式**：`-O connect_mode=attach` 是无线 DAP 烧录的前提，因为几乎所有无线 DAP 都不接 nRESET
- **pyOCD 比 OpenOCD 更适合无线场景**：pyOCD 的 session option 机制提供了精细的控制粒度（`connect_mode`、`reset_type` 等），OpenOCD 的脚本化配置无法做到同等程度的灵活适配
- **蓝牙带宽勉强够用**：9.95 KB/s 的烧录速度虽慢（有线可达 40+ KB/s），但在 100-200KB 的固件范围内可接受

### 6.2 无线调试器的选型建议

| 连接方式 | 稳定性 | 带宽 | 推荐度 |
|----------|--------|------|--------|
| WiFi TCP CMSIS-DAP | ★★★★ | ★★★★ | 首选 |
| 蓝牙 BLE HID (CMSIS-DAP v2) | ★★★ | ★★ | 可用 |
| 蓝牙 Classic SPP + 串口 Bootloader | ★★★ | ★★ | 兜底 |
| WiFi USB 虚拟化 | ★★ | ★★★ | 不推荐 |

### 6.3 流程改进

1. **无线烧录前必做**：`pyocd commander status` 确认 SWD 连接正常后再执行 flash
2. **半擦恢复**：如果芯片被半擦导致原有程序丢失，直接用串口 Bootloader 兜底，不要反复尝试 SWD
3. **始终保留一块有线 ST-Link 在手边**：当无线调试链路出现不可复现的问题时，有线 ST-Link 是唯一可靠的参照基准
