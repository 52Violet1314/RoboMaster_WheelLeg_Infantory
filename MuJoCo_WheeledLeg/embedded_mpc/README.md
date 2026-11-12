# 离线显式 MPC 与单片机接口

`explicit_mpc.c` + `explicit_mpc.h` + `mpc_tables.h` 可独立加入 STM32 工程。
运行时使用 `float`、固定大小栈数组和 `const` 表，没有 Python、SciPy、堆分配、
矩阵求逆或 QP 求解器。`design.json` 记录生成参数。
当前仓库底盘 MCU 为 STM32H723，已用 Cortex-M7 硬浮点选项交叉编译此模块。
尚未接管实际固件控制任务，也未测量板上的最坏执行时间。

## 设计和离线生成

在 `MuJoCo_WheeledLeg` 下运行：

```powershell
python -B generate_mpc.py --dt 0.001 --horizon 3
```

脚本同时生成 Python 使用的 `mpc_tables.npz` 和 C 使用的 `mpc_tables.h`，二者应一起更新。
`generate_mpc.py` 的 `BODY`、`Q`、`R` 为离线设计参数，与当前 MuJoCo 平衡模型一致。
改变质量、轮半径、控制周期或权重后应重新生成并验证，不能沿用旧表。
Python 启动会检查表的周期和权重。

离散预测模型为 `x[k+1] = Ad*x[k] + Bd*u[k]`，`Ad/Bd` 由解析模型作零阶保持离散化。
每次预测令 `u[0]=...=u[N-1]=u`，优化：

```text
J = sum(x[k]' Q x[k], k=1..N-1) + x[N]' P x[N] + N u' R u
|u[i]| <= limit[i]
P = DARE(Ad, Bd, Q, R)
```

这是预测时域 N=3、控制时域为 1 的常值输入 MPC。每次仅执行当前 1 ms 指令，
下一周期重新测量并计算。终端代价借助 LQR 离线计算；MPC 运行时不会调用 LQR 求解。
没有加状态硬约束、输入变化率约束或终端不变集。
当前使用较短预测时域来维持倒地自起接管所需的快速腿姿态反馈；
常值输入参数化在 N=10 时会减弱该反馈，120° 自起测试未能回正腿姿态。
修改预测时域后应重新验证这类大姿态接管，不能只检查直立模型的特征值。
离线脚本检查每一档模型的无约束局部闭环特征值，不构成饱和、模型切换及非线性系统的稳定性证明。

将代价凝聚为 `.5*(u-v)'H*(u-v)`，其中 `v=-gain*x`。
可选 `offset` 把无约束期望改为 `v=-gain*x+offset`，用于复用原位置积分和可选转向补偿；
此时相当于给原 QP 增加线性代价项，并非新算出的参考轨迹。
四路输入各有“自由／下限／上限”三种状态，完整枚举得到 81 个候选区域。
对固定自由集 F 和饱和集 A，离线保存：

```text
T_FA = -inverse(H_FF) * H_FA
u_A  = +/-limit_A
u_F  = v_F + T_FA * (u_A - v_A)
```

运行时只计算这些仿射式，并检查其 KKT 区域：自由输入在限幅内，
上限输入的代价梯度不大于零，下限输入的梯度不小于零。
区域条件本身也是关于状态和限幅的线性不等式。
这是分解存储的显式控制律，在线不会逐次修改活动集、求解线性系统或迭代逼近 QP。
对正定 H、正限幅，该输入盒约束 QP 始终有唯一最优解；浮点实现设置小容差。

所有模型共有 39 档（90～450 mm），常规间隔 10 mm，另含 133.9 和 135 mm。
选择最近模型，不对区域或增益插值。39 档的系数、区域状态、长度和掩码合计
49,233 字节；当前 `arm-none-eabi-gcc -Os` 的 Cortex-M7 目标 `text=50,216` 字节，
`data=0, bss=0`。上下文通常为 4 字节，另有固定栈空间。
每周期先检查上次区域，最坏总共检查 81 个区域。

显式 MPC 的基本定义可参考 [MathWorks 官方说明](https://www.mathworks.com/help/mpc/ug/explicit-mpc.html)。

## C 调用示例

```c
#include "explicit_mpc.h"

static WheelLegMPC balance;

void balance_init(void) {
    wheelleg_mpc_init(&balance);
    /* Handle a 0 return as an invalid length. */
    wheelleg_mpc_select_length(&balance, 0.150f);
}

/* Call every 0.001 s. x must already be estimated and reference-subtracted. */
int balance_step(const float x[10], float u[4], int compliant) {
    const float hip = compliant ? 2.0f : 10.0f;
    const float limit[4] = {7.0f, 7.0f, hip, hip};
    return wheelleg_mpc_evaluate(&balance, x, limit, 0, u);
}
```

必须检查返回值。输入非有限、长度越界、上下文无效或区域匹配失败时返回 0；
`evaluate` 会把输出清零，由调用方采取适合机器人的故障处理。
长度选择失败会保留原模型；复位后需要选择实际使用的腿长。
不能让输出数组与输入／限幅／偏置数组重叠。

状态顺序（SI 单位；角度为 rad）：

```text
x = [位置误差, 速度误差, 航向误差, 航向角速度,
     左虚拟腿角误差, 左腿角速度, 右虚拟腿角误差, 右腿角速度,
     机体俯仰, 俯仰角速度]
u = [左轮力矩, 右轮力矩, 左髋虚拟力矩, 右髋虚拟力矩]  // N*m
```

应沿用 `sim_lqr.py::state_vector()` 的符号、腿长配平角和航向坐标系：
仿真虚拟腿角取测量误差的负号，俯仰正号；航向误差 wrap 到 ±pi。
固件原 K 表的轮输入以“力”存储，不能直接把此处 N·m 当作轮力。
本接口输出也未包含轮反作用前馈、anti-crash、制动、VMC 机构映射和最终电机限幅。
当前仿真约束的是这四路平衡输入，后续补偿会改变实际执行力矩，不能视为完整执行器预测约束。

这些表基于 MuJoCo 的质量／惯量和对称虚拟腿模型。实际机器人接入前需要用实际参数
重新生成表，核对传感器与电机符号、轮力／轮力矩换算、状态估计、控制周期，以及模型切换表现。
在 H723 上用 DWT 周期计数器测量所有区域路径，验证总控制任务能满足 1 ms 周期；
交叉编译成功本身不代表已完成板上时序验证。

## 复现验证

```powershell
python -B -m unittest test_mpc -v
arm-none-eabi-gcc -std=c99 -Os -Wall -Wextra -Werror -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -c embedded_mpc/explicit_mpc.c -o embedded_mpc/mpc_cortex_m7.o
arm-none-eabi-size embedded_mpc/mpc_cortex_m7.o
```

`test_mpc.py` 用独立 SLSQP 求解器对照显式控制律；若安装主机 GCC，还会临时编译 C 库，
跨全部腿长和不同限幅检查 Python/C 一致性，再运行推力、变腿长、驾驶、制动、转向、
持续站立、跳跃及倒地自起接管仿真。

### 本机验证结果（2026-11-03）

默认 N=3 离线表，`python -B -m unittest test_mpc -v`：7 项全部通过。

| 检查 | 结果 |
|---|---|
| 120 组独立有界 QP 对照 | 最大力矩误差约 8.7e-6 N·m |
| 全部 39 档腿长，共 1,560 组 C/Python 对照 | 最大差约 7.4e-6 N·m，最多检查 81 区域 |
| 40 N 推力、持续 0.2 s | 俯仰峰值约 1.27°，恢复平衡 |
| 腿长 150 / 270 / 350 / 133.9 mm 切换 | 保持平衡，腿长跟踪通过 |
| 0.35 m/s 驾驶及停车 | 行驶距离、停车速度和电机限幅检查通过 |
| 0.5 rad/s 转向 | 实测约 28.60°/s，停止后航向误差约 -0.011° |
| 持续站立 30 s | 俯仰峰值约 0.126° |
| 跳跃 | 高度约 0.135 m，落地后恢复平衡 |
| 120° 倒地自起 | 通过原有动作阶段并接管平衡，左右腿回正 |
| Cortex-M7 硬浮点交叉编译 | `-Wall -Wextra -Werror` 通过；代码及只读数据 50,216 字节 |

另运行了原有 `test_regressions`：37 项中 25 项通过、2 项断言失败、10 项错误。
该套测试仍使用默认 LQR，当前工作区的场地已更换为 RMUL 2027：旧测试引用的 `step`
和部分旧场地几何名称已不存在，另有旧碰撞位置及网格包围盒断言未同步。
本次未修改场地、几何文件或这些旧断言；这组遗留测试仍需随当前场地单独更新。
