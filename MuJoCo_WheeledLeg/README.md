# wheelbipeV14_2 — MuJoCo 轮腿机器人 VMC + MPC / LQR 仿真

## 离线显式 MPC（新增，GUI 默认）

```powershell
python -B view_gui.py                         # 默认：VMC + 显式 MPC
python -B sim_mpc.py --seconds 10             # 无界面 MPC 仿真
python -B view_gui.py --controller lqr        # 切回原 LQR 对照
python -B -m unittest test_mpc -v             # MPC 数值 / C / 动力学验证
python -B generate_mpc.py                     # 修改设计后，在电脑上离线重新生成表
```

这是**在线不求解 QP 的显式 MPC**：1 ms 控制周期、3 步（3 ms）预测、
1 个自由控制量块（预测期间四路力矩保持常值，每 1 ms 根据新状态重新选择）。
10 维状态、4 路输入与原 LQR 一致；使用原解析 A/B 的零阶保持离散模型及离散
Riccati 终端代价。每个模型有最多 81 个仿射控制律，通过共享 16 个自由集矩阵压缩存储。
39 档离线腿长模型覆盖 90～450 mm（常规 10 mm 间隔，额外含 133.9/135 mm），
运行时选最近一档，系数共 49,233 字节，Cortex-M7 `-Os` 编译目标的代码及只读数据约 49 KiB。

腿长 PID、roll 补偿、制动、轮反作用前馈、跳跃及自起状态机继续复用。
为兼容原脚本，`WheelLegLQR()` 的 Python 默认仍为 LQR；
`WheelLegMPC()` 或 `WheelLegLQR(controller="mpc")` 使用 MPC，
命令行 `sim_lqr.py` 和 `view_gui.py` 默认使用 MPC。下文原 LQR 验证数据仍指原 LQR。

当前 MPC 仅约束平衡控制输入（轮 ±7 N·m、髋 ±10 N·m，按住 0 时髋 ±2 N·m），
不包含状态硬约束或终端不变集；后续前馈与制动叠加仍由最终电机限幅保护。
因此不能据此宣称全局稳定、全预测时域内实际电机力矩可行，或性能必然优于 LQR。
纯 C 移植接口、数学定义和硬件适配说明见 [embedded_mpc/README.md](embedded_mpc/README.md)。
新增 MPC 测试 7 项全部通过（含 30 秒站立、抗扰、转向、跳跃、120° 自起接管）；
旧 LQR 场地回归另有 12 项未通过，原因及测量结果也记录在该说明中。

---

用开源仓库 `D:\wheeled-legged_RL` 里的模型（`wheelbipeV14_2`），在 MuJoCo 上做**传统 VMC + 矩阵 LQR** 的轮腿平衡仿真。**不涉及任何强化学习**（策略网络完全没有使用），只把该仓库当作模型来源。
源模型与派生网格的 MIT 许可保存在 `LICENSE.wheeled-legged_RL`。

快速上手：

```bash
python view_gui.py                 # 键盘操控 GUI（8/5 前后 / 4/6 转向 / 1/2/3 腿长）
python verify.py                   # 全套验证（站立 / 抗扰 / 腿长 / K 重算）
python view_model.py --full        # 看全保真 37 连杆模型（静态装配姿态）
```

重新从开源仓库生成模型：

```bash
python rebuild_models.py
```

默认读取 `D:\wheeled-legged_RL\...\wheelbipeV14_2_1`。如果源码在别处，先设置
`WHEELBIPE_USD_DIR` 为 `wheelbipeV14_2_1` 目录。生成链会直接读取源 USD 的刚体位姿、
质量、惯量、关节、视觉网格和材质颜色。

---

## 1. 模型来源与转换链

`wheeled-legged_RL` 仓库里**没有 URDF**，只有 Isaac Sim 导出的 USD：

```
source/agent_world/agent_world/assets/usd_files/wheelbipeV14_2_1/
    wheelbipeV14_2.usd                      主文件（37 个刚体、36 个树关节、6 个球铰闭链）
    configuration/wheelbipeV14_2_base.usd   STL 网格（几何）
    configuration/wheelbipeV14_2_physics.usd 质量/惯量/关节/限位
```

所以走的是 **USD → URDF → MJCF** 的自动转换链（脚本用 `usd-core` 直接解析 USD 的
`UsdPhysics` schema，把 STL 几何、质量、质心、惯量、关节全部提取出来）。

视觉网格严格保留 USD 的两层局部变换：`/meshes/<link>` 的 CAD 节点变换和
`/visuals/<link>` 的装配变换。导出的 STL 坐标属于各自刚体，不再重复减去刚体世界坐标；
简化控制模型在合并刚体时也会补上每个源连杆相对合并坐标系的位置和四元数，并保留源材质颜色。

### 1.1 生成的 URDF

`wheelbipeV14_2.urdf` — 37 link / 36 树关节，几何用真实 STL。

```python
import mujoco
m = mujoco.MjModel.from_xml_path("wheelbipeV14_2.urdf")
# nq 36  nv 36  nbody 37  njnt 36
```

> **注意**：URDF 文件本身是完整的（37 link 全部带 `<inertial>`，质量合计 **23.302 kg**），
> 但 MuJoCo 的 URDF 导入器会把**根连杆（base_link, 15.963 kg）并入 world body**，
> 所以 `sum(body_mass)` 只有 7.34 kg。用 MJCF 版本或 Pinocchio / PyBullet 加载就没有这个问题。
> URDF 里加了 `<mujoco><compiler meshdir="meshes"/></mujoco>` 扩展块，让 MuJoCo 能找到网格。

### 1.2 生成的 MJCF（全保真）

`wheelbipeV14_2.xml` — 保留全部 37 个刚体，6 个球铰闭链用 `<equality><connect>` 表达。

```python
m = mujoco.MjModel.from_xml_path("wheelbipeV14_2.xml")
# nq 43  nv 42  nbody 38  neq 6  nu 8   mass 23.30 kg
```

> 说明：USD 里关节限位是**角度制**，转换时已换成弧度。

---

## 2. 控制用的简化模型

`wheelbipe_lqr.xml`（由 `gen_model.py` 生成）

LQR 推导（`Simmulation/get_K_jiao_LQR.m`）把机器人建模为：

```
机体  +  两条刚性摆杆腿（髋关节力矩）  +  两个驱动轮（轮力矩）
```

USD 的腿是 6 球铰闭链机构，直接拿来做平衡控制会非常脆，而且 VMC/LQR 抽象里腿就是一根摆杆。
所以简化模型的做法是：

| 项目 | 处理 |
|---|---|
| 连杆质量 / 质心 / 惯量 | **全部取自 USD**（底盘、云台和固定导向件合并 = 20.165 kg） |
| 髋关节位置 | **取自 USD**（base 坐标系 ±0.174 m） |
| 轮子位置 / 半径 / 惯量 | **取自 USD**（轮心在髋前 6.6 mm、下 124.8 mm，R = 60 mm） |
| 腿 | 合并成一根刚性腿，**加一条腿长滑移轴**（对应 USD 的 `spring2_joint` 弹簧） |
| 腿关节摩擦 | 髋关节 `frictionloss = 0.5 N·m`（+ 原有 `damping = 0.05`），滑移轴沿用 `damping = 40` |
| 云台 | 焊死并入机体（LQR 不控制云台） |
| 外部碰撞 | 轮子使用圆柱、机身使用简化盒；大腿、底部导杆及前后连杆使用各零件网格的凸包参与环境碰撞 |
| 自碰撞 | 关闭（与 Isaac 配置 `enabled_self_collisions=False` 一致） |

控制动力学仍采用虚拟刚性腿，但外观不再刚性锁死：`linkage_kinematics.py` 使用源 USD
的真实铰点尺寸，同时满足前四连杆、后两连杆和弹簧支链的闭环约束。每个物理周期把
实际腿长映射到连杆几何，接触力通过简化腿刚体传回动力学。关闭静态局部碰撞 BVH，
避免连杆几何更新后仍使用旧包围盒。各零件使用凸包近似，未建立真实六连杆的独立动力学。

结果：**腿长 133.9 mm，总质量 23.30 kg，站姿基座高度 184.8 mm**。

场地已换为 RMUL 2027 官方 STEP 的审计网格。`arena_source/step_geometry.json.gz`
保留从 STEP 导出的米制三角面与 CAD 颜色，`arena_source/official_top.png` 是配套顶视材质。
运行 `python tools/convert_rmul2027.py` 会生成各材质 OBJ、贴图和碰撞用 STL，
再运行 `python tools/arena_collision.py` 更新碰撞盒。场地坐标的比赛区域为
`x∈[0,12]`、`y∈[0,8]`，地面 `z=0`；外沿最大范围约 `12.6 × 8.6 m`。

| 结构 | 位置/尺寸 |
|---|---|
| 行驶地面 | `z = 0`，官方贴图覆盖比赛区域 |
| 场地围挡 | CAD 顶部最高 `z = 0.42 m` |
| 高地 | CAD 高度约 `z = 0.20 m`，含控制区及连接结构 |

- 网格按 CAD 颜色拆分；顶面使用官方贴图及原 UV 坐标。网格只负责显示，
  碰撞由地面平面和 `arena_collision.xml` 中 305 个高度图盒体提供（2.5 cm 栅格）。
  这是保守的碰撞近似：有顶棚的通道也可能被当作实心障碍，不能用于精确净空验证。
- 出生点使用场地包的 `(1.25, 6.75)`，基座初始高度为腿长加轮半径。

```
world ─ base_link (freejoint, 20.165 kg)
          ├ left_leg  ── hip_joint(hinge,y) ─┬ left_leg_ext ── legslide_joint(slide) ── left_wheel ── wheel_joint(hinge,y)
          └ right_leg ── hip_joint(hinge,y) ─┴ right_leg_ext ─ legslide_joint(slide) ── right_wheel ── wheel_joint(hinge,y)
```

---

## 3. K 值计算（`lqr_k.py`）

`Simmulation/get_K_jiao_LQR.m` 的 Python 移植，**不是硬编码**，而是从模型参数重新推导：

**状态（10 维）**

| # | 符号 | 含义 |
|---|---|---|
| 1 | `s` | 自然坐标系水平位移 |
| 2 | `ds` | 水平速度 |
| 3 | `phi` | 偏航角 |
| 4 | `dphi` | 偏航角速度 |
| 5 | `theta_ll` | 左腿摆杆与竖直方向夹角 |
| 6 | `dtheta_ll` | 左腿角速度 |
| 7 | `theta_lr` | 右腿摆杆与竖直方向夹角 |
| 8 | `dtheta_lr` | 右腿角速度 |
| 9 | `theta_b` | 机体俯仰角 |
| 10 | `dtheta_b` | 机体俯仰角速度 |

**输入（4 维）**：`[T_wl, T_wr, T_bl, T_br]` = 左轮力矩、右轮力矩、左髋力矩、右髋力矩

**方法**：5 个刚体方程 (3.11)–(3.15) 对加速度是线性的，写成 `M·q̈ + N·x + P·u = 0`，
于是 `∂q̈/∂x = -M⁻¹N`、`∂q̈/∂u = -M⁻¹P` 就是精确的线性化；再组装 A(10×10)、B(10×4)，
最后解连续时间 Riccati 方程 `K = R⁻¹BᵀP`（等价于 MATLAB 的 `icare`/`lqr`）。

```bash
python lqr_k.py        # 打印 150/252/350 mm 腿长对应的 K 和闭环特征值
```

**与固件的交叉验证**：用 `app_K_value.cpp` 里 `K_Fixed_Leg150` 的物理参数
（R_w=0.0525, R_l=0.20, m_b=15.75 …）重算，四行的方向余弦均为 **−0.97**，
即与固件表同构，只差一个符号约定和 `diag(R_w,R_w,1,1)` 的单位换算（固件存的是轮**力**）。

---

## 4. 控制回路（`sim_lqr.py`）

完全对标 `RoboMaster_InfantoryV2.1.4/APP/APP_Task/src/CalculateTask.cpp`：

```
① VMC 腿部运动学
     由髋/轮几何算虚拟腿长 L0、腿角 theta 及其变化率
② 腿长 PID
     F0 = F0_ff + PID(L_ref - L0)          → 虚拟腿支撑力
     F0_ff 由模型静力分析给出（约 109.3 N/腿）
③ roll 差动补偿
     F0_L -= roll_comp,  F0_R += roll_comp
④ 矩阵 LQR
     u = -K x
     x = [s, ds, yaw, dyaw, thL, dthL, thR, dthR, pitch, dpitch]
     u = [TwL, TwR, TbL, TbR]
⑤ 腿角微分环（固件 anti_crash_）
     err = theta_L - theta_R,  Tb_L += out,  Tb_R -= out
⑥ 虚拟模型力矩分配
     F0 → 腿长滑移轴力；Tb → 髋关节力矩；Tw → 轮力矩
```

转向有两种实现（`yaw_pid_enable` 切换）：

| 模式 | 做法 | 说明 |
|---|---|---|
| `False`（**默认**） | 把 `target_yaw` 按指令速率积分，交给 LQR 偏航列跟踪 | 在转换后的模型上更稳，可撑 30 s 以上 |
| `True` | 独立 yaw 速率 PID，差动叠加到轮力矩（固件做法） | 会反作用到髋关节让腿张开，见第 7 节 |

**偏航误差确实在 LQR 里**：`x[2] = wrap(yaw − target_yaw)`，对应增益列
`K[:,2]`，同时驱动轮差动和髋差动（`u = -K x`）。但两处标定不对时现象会像"没接进去"：

- **偏航权重太软**：`q_yaw = 400` 时差动轮力矩只有 **7.6 N·m/rad**，远低于轮胎侧向
  挣脱所需，于是扰动后航向回不来。实测：机体上打 40 N·m / 0.3 s 偏航脉冲，残余
  **98°**；持续 30 N·m 5 s，残余 **116°**；目标阶跃 30° 停差 **7.3°**。
  现在 `q_yaw = 4000`（差动 **24 N·m/rad**）：上面三种情况残余 **0.00° / −0.01° /
  1.8°**。
- **腿角测量把偏航混了进来**：轮子比髋外偏 48 mm，世界系"髋→轮"向量在机体偏航 ψ 后
  纵向投影变成 `∓0.048·sinψ` —— 腿关节一动不动，机体转 10° 就量成 `∓3.8°`。
  转向时这个假"劈叉"随航向增大，LQR 腿角列（153 N·m/rad）与它死顶，腿张开 ~4.7°、
  偏航误差冻在 **9°** 不动，看着就像"偏航根本没闭环"。修正后（`measure()` 先按
  `R_z(-yaw)` 把腿向量和角速度转回航向坐标系）实测：

  | 4/6 指令 | 0.1 | 0.25 | 0.5 | 1.0 | 2.5 rad/s |
  |---|---|---|---|---|---|
  | 实际偏航速率 | 5.7 | 14.3 | 28.7 | 56.8 | 56.8 °/s（牵引饱和） |
  | 腿摆角差 | 0.00 | 0.05 | 0.09 | 0.20 | 0.20 ° |
  | 松键后残余航向误差 | 0.03 | −0.01 | −0.02 | 0.01 | 0.02 ° |

  即指令速率被**等值跟踪**（1.0 rad/s = 57.3°/s）。`YAW_TARGET_LEAD_LIMIT = 0.16 rad`
  限定转向时的航向 lead，从而限定差动轮力矩，避免饱和后把腿顶开。
  键盘 4/6 的斜坡为 `TURN_ACCEL = 25 rad/s²`、上限 `TURN_MAX = 2.5 rad/s`（5 倍灵敏度）：
  按住 0.5 s 到 44.8°/s、3 s 到 57.3°/s，点一下 0.1 s 约 12.6°/s；松键后目标冻结，
  机器人继续转到该目标（≤0.16 rad ≈ 9.2°）后停住，残余航向误差 ~0.02°。
  回归测试：`test_yaw_error_is_a_lqr_state_that_moves_the_wheels`、
  `test_heading_hold_rejects_a_yaw_disturbance`、
  `test_turn_command_tracks_the_requested_yaw_rate`、
  `test_leg_angle_measurement_is_yaw_invariant`、
  `test_turn_keys_ramp_five_times_faster`。

### 运行

```bash
# 站立（默认 5 s，打印状态）
python sim_lqr.py --seconds 10

# 指定腿长
python sim_lqr.py --leg 0.180
```

```python
from sim_lqr import WheelLegLQR

sim = WheelLegLQR(leg_length=0.134)
sim.reset()
for _ in range(5000):
    info = sim.step()
print(info["pitch"], info["L0"], info["TwL"], info["TbL"])
```

### 验证结果（`python verify.py`）

| 测试 | 结果 |
|---|---|
| 站立 60 s | z=0.1846 m，pitch +0.82°、roll +0.01°、速度约 0 → **稳定、水平、静止** |
| 抗扰（60 N 推力 0.4 s） | 峰值 pitch **0.91°**、roll 0.02°，之后恢复 |
| 腿长阶跃 134 → 180 mm | L0 跟到 180.1 mm，z 到 0.2330 m，全程保持平衡 |
| K 随腿长重算 | 120/134/180/250/350 mm 五档闭环全部稳定 |
| 航向抗扰（20 N·m 偏航脉冲 0.3 s） | 残余航向误差 **0.01°**（= 2 倍髋限幅仍能恢复） |
| 髋力矩限幅 | 常规 `|Tp| ≤ 10 N·m`（1.0 m/s 加速时正好打到 10）；按住 `0` 时 `|Tp| ≤ 2 N·m`（40 N 推力下俯仰 3.15°） |
| 腿长预设 150 / 270 / 350 mm | 站姿 z = 0.2217 / 0.3455 / 0.4267 m，pitch 均 < 0.3°，K 闭环稳定，连杆网格完全跟随 |
| 按 `4/6` 转向（1.0 rad/s 指令） | 实际 **56.8°/s**（=指令），腿摆角差 0.20°、俯仰 0.25°，松键残余 0.01° |
| 站在场地底板上（30 s） | 基座 z=**0.2044 m**（底板 0.02 + 站姿 0.184），pitch 0.05°、腿摆角误差 0.18° |
| 台面行驶 0.6 m/s | 6 s 走 3.3 m，vx 0.897 m/s、pitch 1.12° |
| 台面转向 1.0 rad/s | **57.4°/s**，腿摆角差 0.25° |
| 台面跳跃 | 峰值 13.5 cm，落地后恢复站姿 |
| 撞挡墙 / 220 mm 高台 | 分别停在 `y=8.192`、`x=2.692`、`x=0.708`，无穿模 |
| 髋关节摩擦 0.5 N·m | 站立/直行不受影响（0.5 m/s 指令仍是 0.711 m/s）；跳跃落地动量略降（0.32 → 0.12 m/s） |

---

## 5. GUI 与键盘操控

### 5.1 键盘驾驶（`view_gui.py`）

```bash
python view_gui.py                 # 站立启动
python view_gui.py --speed 0.3     # 带速度指令启动
python view_gui.py --leg 0.18      # 带腿长启动
python view_gui.py --realtime 0.5  # 0.5 倍速（默认 1.0 实时，0 = 响应优先的最高速）
```

| 按键 | 功能 |
|---|---|
| 按住 `8` / `5` | 前进 / 后退，按 5.0 m/s² 积分到 ±2.5 m/s；松开立即归零 |
| 按住 `4` / `6` | 左转 / 右转，按 25.0 rad/s² 积分到 ±2.5 rad/s；松开立即归零（5 倍灵敏度） |
| `1` / `2` / `3` | 腿长预设 150 / 270 / 350 mm（数字行或小键盘，目标值立即生效） |
| `7` | 失能：清零所有电机指令、模拟气弹簧力及外加力；重力和场地碰撞继续运行 |
| `9` | 倒地自起：先翻正倒置机身，再逐侧判断前向腿并抬起、转向后方、整理支撑角，最后以 150 mm 腿长接管平衡 |
| 按住 `0` | 髋关节力矩 `Tp` 限幅到 **2 N·m**（腿部变软、可被推动）；松开恢复常规 10 N·m |
| `Space` | 跳跃（保留当前速度/转向指令；150 mm 下蹲、350 mm 蹬伸、离地后立即收至 135 mm） |
| `X` | 急停（所有力矩归零）；再按时以直立、停车状态复位 |
| `Z` / `R` | 复位/重新加载当前预设腿长 |
| `C` | 切换视角（free / side / front / top） |
| `F` | 开关相机跟随（默认开启，关闭后可自由平移观察） |
| `V` | 接触力显示开关 |
| `T` | 屏幕 HUD 开关 |
| `H` | 控制台打印当前状态 |
| `Esc` | 退出 |

鼠标：左键拖拽旋转 / 右键拖拽平移 / 滚轮缩放 / 双击选体。
左下角 HUD 实时显示速度指令与实际值、偏航、腿长（含左右腿摆角相对配平角的误差
`swing err`）、俯仰/横滚、机身高度，以及
VMC 轴向力 `F0`、虚拟髋力矩 `Tb`、实际髋关节电机力矩和轮毂电机力矩。按 `H`
也会把同一组控制量及连杆角度、腿摆角误差打印到控制台；HUD 中的 `link q` 显示左右腿
`front1/rear1` 的运动学角度；`zero-cmd brake` 是速度指令归零后附加的停车力矩。

键盘回调只把按键放入有界队列，所有控制状态都由仿真主线程更新；按键自动重复即使
瞬间堆积也不会阻塞 MuJoCo 的 GUI 线程。实时循环最多补算 50 ms，窗口拖动或系统卡顿后
不会为了追赶墙钟时间而长时间霸占主线程。松开 `8/5` 时，界面速度指令立即归零；
控制器用 5 m/s² 的短目标斜坡抑制轮毂力矩尖峰，并启用带限幅和滤波的
零速 PI 制动，避免平衡 LQR 的位置项与速度项抵消后继续滑行或加速。
行驶时机身位置随速度自然积分；松键后位置环把释放点设为新的局部原点并保持，
不会再把机器人拉回世界坐标原点。`Z/R` 复位会同步当前腿长预设。
简化模型使用随腿长插值的虚拟腿静态配平角（134/150/200/250/300/350/450 mm 对应
0.040/0.037/0.027/0.0205/0.017/0.0145/0.012 rad），以匹配原模型质心分布；否则长腿在
`cmd=0` 时会自行后滚。
急停后机器人通常已经倒下，因此再次按 `X` 会先清零移动指令并复位直立姿态，避免在倒地
状态直接恢复控制产生过大约束冲击。
`7` 后按 `9` 启动独立自起控制器。恢复用的完整 Pitch 采用 `atan2`，可以识别超过 ±90°
的倒置姿态。状态顺序为 `body → body_settle → lift → sweep → stand → balance → idle`，
未倒置时跳过前两阶段；前向腿经上方转向后方，原本在后方的腿走最短整理路径。
左右腿分别保存角度目标，目标角速度限制为 0.6 rad/s，实际超速时主动制动；髋力矩
限制为 20 N·m。动作只使用原有电机执行器，不修改基座位置/姿态或施加外部翻身力矩。
恢复阶段移动与跳跃指令被屏蔽，45 s 未完成进入失能故障，可按 `9` 重试、`7` 中止。
支撑整理阶段会临时收腿以免脚轮顶翻机身，最终站立目标固定为 150 mm。
`python -B test_self_right.py --angle 120` 可运行物理自起回归测试，`--asymmetric` 设置不同的左右腿初态。

### 5.2 跳跃与 15 N·m 气弹簧

30 cm 腾空高度对应的理想需求为 `h=0.30 m`，20 kg 整车忽略损耗时需要的离地速度、能量和冲量为：

```text
v_takeoff = sqrt(2 g h) = 2.426 m/s
E_air     = m g h       = 58.86 J
J         = m v_takeoff = 48.5 N·s
```

从 150 mm 蹬伸到 350 mm，做功行程 `s=0.20 m`。蹬伸时还要克服重力，因此两腿所需
平均轴向总力不是简单的 `mgh/s`，而是：

```text
F_total,avg = m g + m g h / s = 490.50 N
```

`15 N·m` 是转矩而不是可直接相加的直线力。仿真假定左右腿各有一只 15 N·m 气弹簧，
通过后连杆运动学雅可比映射到腿轴：

```text
F_gas(L) = tau_gas * |d theta_rear2 / dL|
E_gas    = 2 * tau_gas * |Delta theta_rear2|
F_motor,total,avg = m g + (m g h - E_gas) / s
```

该机构在 150–350 mm 内 `|Delta theta_rear2|=1.064 rad`，所以两只气弹簧提供约
`31.9 J`，平均约 `159.6 N`（每腿约 79.8 N）。对 20 kg 理想系统，电机至少还需
平均约 `331 N`，即 **165 N/腿**；气弹簧与电机合计约 **245 N/腿**。模型已把开源模型
原本 23.30 kg 的总质量校准为 **20.00 kg**（保留轮腿质量，用 16.862 kg 机身补足，并
同比缩放机身惯量）。考虑接触、姿态控制、有限蹬伸时间和执行器损失，蹬伸阶段采用
VMC 采用硬件限幅：常规状态 `Tp=±10 N·m`、`Fn=±200 N`、轮毂电机 `±7 N·m`
（按住 `0` 时 `Tp` 收紧到 `±2 N·m`），并在控制器和 MuJoCo 执行器两层同时生效。
仅在跳跃 `thrust` 蹬伸阶段，Fn 上限提高到
**200 N/腿**，腿部执行器也只在该阶段允许到 +200 N；Tp 和轮毂限制不变。实际蹬伸
总力设为 **200 N/腿**；无台阶回归的净跳高约为 **13.4 cm**。若需恢复 30 cm，需要把
该值调回约 279 N/腿，随后空中收腿并稳定落地。

跳跃状态机依次为 `crouch -> thrust -> flight -> landing -> recover -> idle`。腿伸到 350 mm
并完成离地后，空中 VMC 立即把腿收到 135 mm；到达目标后电机抵消被动气弹簧，避免再次
伸长。触地后渐进恢复轮毂/髋关节 LQR；轮毂关节加入
0.05 N·m·s/rad 等效电机阻尼，吸收触地瞬间的轮速冲击。HUD 和 `H` 输出会显示当前
阶段、跳高、气弹簧轴向力、VMC 总力以及腿/髋/轮毂电机输出。

腿长目标不做速度斜坡：普通预设、跳跃下蹲以及空中收腿都会立即写入目标值，实际伸缩
速度由 VMC 力限幅、气弹簧和机构动力学决定。跳跃不再检查或清零水平移动速度，Space
会保留当前行驶指令；落地稳定后才恢复跳跃前的腿长目标。

按住 `0` 时，GUI 从真实按键按下/松开状态（而不是键盘自动重复事件）把髋关节力矩
`Tp` 限幅到 **2 N·m**：LQR、偏航环、VMC 腿长环、轮反作用前馈和制动全部照常运行，
只是髋关节"没力气"，腿可以被推动（实测站立时常规 `Tp` 只有 0.46 N·m，所以松开/按住
站姿几乎不变；40 N 推力下机身俯仰放宽到 3.15°）。松开 `0` 后立即恢复常规 10 N·m 限幅。
`0` 与行驶键不再互斥（旧的"直接轮毂模式"已删除）。

简化虚拟腿的滑移轴覆盖 90–450 mm 行程；其中源六连杆可严格闭环的可视化范围是
90–**359 mm**（再往外连杆自身撞到限位，轮心目标追不上，闭环残差随超长线性增大，
例如 370 mm 处 9 mm、420 mm 处 59 mm）。最高档 350 mm 预设在该范围内，连杆网格与实际
腿长**完全一致**；只有实验性的超长命令（>359 mm）会让**网格**停在该端点，而轮子、碰撞
和控制始终使用完整的虚拟腿长。简化模型不再在滑移关节上重复添加被动刚度；腿长
刚度与主要阻尼由 VMC PID 提供，关节只保留少量耗能阻尼。每次腿长命令变化时会按新
腿长重算矩阵 LQR 增益。

机身底部的碰撞盒与四个被动滑块采用 `mu=0.001` 的近零滑动摩擦；它们与地面、台阶
仍保留法向碰撞，因此底盘碰到 200 mm 台阶时会沿其表面滑动，而不会像高摩擦底板一样
被卡住。

GUI 以 60 Hz 刷新，并限制单帧补算量；`--realtime 1` 冒烟测试 3 秒完成约 3000 个
物理步（模型步长 1 ms），窗口可正常创建和关闭。

### 5.3 查看全保真模型（`view_model.py`）

```bash
python view_model.py           # 控制模型（VMC + LQR 运行中）
python view_model.py --full    # 全保真 37 连杆 MJCF（含 6 个球铰闭链，默认静态）
python view_model.py --urdf    # 导出的 URDF
python view_model.py --full --simulate  # 明确要求时才做无控制重力仿真
```

全模型默认保持 USD 的装配姿态，便于检查连杆、闭链和关节；它没有平衡控制器，开启
`--simulate` 后会自然倒下。转换器会保留 `spring2` 连杆的静态四元数，使六个闭链连接点
初始误差均小于 0.1 mm。

### 5.3 录制视频（`render_video.py`）

```bash
python render_video.py --track                    # demo.mp4, 1280x720 @ 50 fps
python render_video.py --width 1920 --height 1080
```

38 s 演示：站立 → 60 N 推力扰动 → 腿长 134→180→134 mm → 速度 0.5 m/s → 转向 0.2 rad/s → 停车。

---

## 6. 符号约定标定

解析 K 是在"机体坐标系"里推的，MuJoCo 里要用实测状态。二者的映射通过
`calibrate_signs.py` 扫描全部 2⁶ 种组合、取能站住的那一组确定：

```python
DEFAULT_SIGNS = dict(s=+1, ds=+1, yaw=+1, dyaw=+1,
                     th=-1, dth=-1,          # 腿角：解析模型正值 = 机体前倾
                     pitch=+1, dpitch=+1,    # 俯仰：低头为正（asin(-R[2,0])）
                     Tw=+1, Tb=+1)           # 轮/髋力矩直接映射到 MuJoCo 电机
```

单独实测确认：轮子 +2 Nm → 前进（+0.66 m/s），与 `Tw=+1` 一致。

### 一个容易踩的坑：freejoint 角速度是机体系

MuJoCo 里 freejoint 的 `qvel[3:6]` 是**机体系**角速度，不是世界系
（用 `tools/probe_frame.py` 实测确认）。修正前把它当世界系用，导致俯仰阻尼和
偏航阻尼都作用在错误的量上，速度跟踪只有 0.13 m/s；改成 `omega = R @ omega_body`
之后升到 0.62 m/s。

### 6.1 第二个坑：航向角不能污染腿角与俯仰率

LQR 的 `theta`（腿摆角）和 `dpitch`（俯仰角速度）必须定义在**机器人自己的纵向平面 /
航向轴**上，但 MuJoCo 给的是世界系量：

- **腿角**：`theta = atan2(v_x, -v_z)`，`v` = 髋→轮向量。轮子比髋**外偏 48 mm**，
  所以 `v` 自带侧向分量；机体偏航 ψ 后它在世界系的纵向投影变成 `∓0.048·sinψ`。
  实测（腿关节完全不动，只转机体）：10°/30°/90° → `∓3.8°/∓10.9°/∓21.0°`（左右反号）。
  这个假"劈叉"随航向单调增大，转向时 LQR 腿角列一直追不上 → 腿张开、偏航误差冻住。
- **俯仰/横滚角速度**：世界系 `omega[1] = q·cosψ − p·sinψ`，转过 90° 后符号反转，
  等于把俯仰阻尼列变成正反馈（连续转向到 ~145° 会出 ±17° 俯仰极限环）。

修正：`measure()` 里先按 `R_z(-yaw)` 把两个量转回航向坐标系（`omega_heading`）。
`yaw = 0` 时两套坐标完全一致，站立/直行行为不变；转向时腿角差 ≤0.2°、俯仰 ≤0.25°。

---

## 7. 已知限制

1. **位置 / 速度环偏弱（模型失配）**：解析推导假设腿是**匀质摆杆**（质心在腿长
   中点），但 USD 里腿部连杆的合并质心在**髋后方约 95 mm**（rear1/rear2 是向后
   伸的长臂）。后果：
   - 站立时会缓慢漂移约 3 m 后停在新的平衡点（`pos_ki` 位置积分把它限制住）；
   - 速度指令只影响**行进距离**，不能维持恒定速度（cmd 0.8 → 30 s 走 6.6 m）。
   姿态环（腿角 + 俯仰）工作得很好（0.01° 级）。
   `gen_model.py` 里的 `SYMMETRIC_LEG = True` 会把腿质心放回腿长中点以匹配抽象。

2. **转向能力**：参考固件里转向是**独立的 yaw 速率 PID** 差动叠加到轮力矩上
   （`Chassis::SynthesizeMotion()`）；本仿真默认改用**纯 LQR 航向设定值跟随**
   （`yaw_pid_enable=False`），因为差动轮力矩会**反作用到髋关节**，让两条腿前后张开。
   - 修好偏航坐标与权重后（见第 4 节），键盘 4/6 能**等值跟踪**指令速率：
     0.5 rad/s → 28.7°/s、1.0 rad/s → 56.8°/s，腿摆角差 ≤0.2°、俯仰 ≤0.25°。
   - 上限来自**轮胎侧向摩擦**：原地转向要轮子侧向擦地，再大的差动轮力矩也只能到
     ~57°/s，硬加力矩只会让轮子打滑。
   - `gen_model.py` 给髋关节加了 ±35° 机械限位（USD 里 rear1/front1 是无限制的），
     否则张开会让仿真直接崩掉。

3. **腿滑移轴取纵向投影**：轮子比髋外偏 48 mm，如果滑移轴直接沿"髋→轮"向量，
   轴会侧倾 21°，腿推力永远带一个侧向分量（持续扰动）。已改为投影到纵向平面。

4. **`numeric_lqr.py` 未完成**：思路是从 MuJoCo 模型本身做数值线性化再解 Riccati
   （与 `get_K_jiao_LQR.m` 同样"从模型算 K"），目前状态映射（尤其
   `dθ/dt`、`d²θ/dt²` 的有限差分）还需完善。要根治上面的失配问题应该走这条路。

5. **云台焊死**：简化模型里 gimbal 并入机体，未建 yaw/pitch 自由度。

6. **全保真 MJCF 未做控制**：`wheelbipeV14_2.xml` 只用于几何/质量核对与可视化。

---

## 7. 文件清单

### 模型
| 文件 | 说明 |
|---|---|
| `wheelbipeV14_2.urdf` | 全保真 URDF（37 link / 36 关节），可直接被 MuJoCo / PyBullet / Pinocchio 加载 |
| `wheelbipeV14_2.xml` | 全保真 MJCF（含 6 个球铰闭链 equality 约束） |
| `meshes/*.stl` | 全保真模型的 37 个连杆网格 |
| `wheelbipe_lqr.xml` | **控制用简化模型**（2 髋 + 2 腿长轴 + 2 轮） |
| `meshes_lqr/*.stl` | 简化模型使用的网格 |
| `arena_source/` | STEP 审计三角面及官方顶视材质来源 |
| `meshes_lqr/arena_*.obj` | 按颜色与贴图拆分的 RMUL 2027 场地显示网格 |
| `meshes_lqr/arena.stl` | RMUL 2027 场地碰撞生成用网格 |
| `arena_collision.xml` | 场地高度图碰撞盒，由 `tools/arena_collision.py` 生成 |

### 代码
| 文件 | 说明 |
|---|---|
| `sim_lqr.py` | **主仿真 + 控制器**：VMC + 腿长 PID + roll 补偿 + 矩阵 LQR + 偏航环 |
| `lqr_k.py` | `get_K_jiao_LQR.m` 的 Python 移植：A/B 推导 + Riccati 求解 |
| `linkage_kinematics.py` | 虚拟腿长 → 原六连杆角度的闭链逆运动学与可视化映射 |
| `view_gui.py` | **键盘操控 GUI**（8/5 前后 / 4/6 转向 / 1/2/3 腿长 / HUD / 力矩显示） |
| `view_model.py` | 查看三种模型（控制 / 全保真 MJCF / URDF） |
| `render_video.py` | 录制演示 MP4 |
| `render_frames.py` | 离屏渲染单帧 PNG |
| `verify.py` | 综合验证（站立 / 抗扰 / 腿长阶跃 / K 重算 / 固件交叉验证） |
| `validate_lqr.py` | `lqr_k.py` 与固件 `K_Fixed_Leg*` 表的对比 |
| `calibrate_signs*.py` | 符号约定扫描标定（2⁶ 组合） |
| `tune_q.py` / `tune_steer*.py` | Q 权重与转向环调参 |
| `test_behavior.py` / `test_drive.py` / `test_envelope.py` | 行为 / 驾驶 / 包线测试 |
| `test_gui.py` | GUI 可用性冒烟测试 |
| `numeric_lqr.py` | 数值线性化求 K（**未完成**） |
| `probe_signs.py` / `probe_B.py` / `probe_yaw.py` / `probe_roll.py` | 力矩方向、B 矩阵、偏航/横滚响应探针 |
| `diag*.py` / `isolate.py` / `run_long.py` / `run_start.py` | 诊断脚本 |

### 生成脚本（`tools/`）
| 文件 | 说明 |
|---|---|
| `usd_dump_full.py` | USD → JSON（几何/质量/惯量/关节） |
| `urdf_builder.py` | 公共工具（四元数、STL 写出、坐标变换） |
| `gen_urdf_mjcf.py` | JSON → URDF |
| `gen_mjcf2.py` | JSON → 全保真 MJCF |
| `gen_model.py` | JSON → **控制用简化模型** `wheelbipe_lqr.xml` |
| `convert_rmul2027.py` | 审计 STEP 网格与官方贴图 → MuJoCo 场地资源 |
| `convert_arena_stl.py` | 旧 STL 场地转换工具；当前场地不使用 |
| `arena_collision.py` | 场地网格 → `arena_collision.xml`（按高度层生成碰撞盒，避免凸包填实） |
| `test_arena_collision.py` | 检查 CAD 顶点、三角面中心和边中点是否被碰撞盒覆盖 |
| `probe_frame.py` | 验证 freejoint 角速度所在坐标系 |

### 产物
`demo.mp4`（38 s 演示视频）、`frames/*.png`（离屏渲染帧）
