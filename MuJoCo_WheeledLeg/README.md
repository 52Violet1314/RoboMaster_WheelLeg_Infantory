# wheelbipeV14_2 — MuJoCo 轮腿机器人 VMC + LQR 仿真

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
| 云台 | 焊死并入机体（LQR 不控制云台） |
| 外部碰撞 | 轮子使用圆柱；机身使用 `500×540×170 mm` 简化盒；视觉连杆不参与碰撞 |
| 自碰撞 | 关闭（与 Isaac 配置 `enabled_self_collisions=False` 一致） |

控制动力学仍采用虚拟刚性腿，但外观不再刚性锁死：`linkage_kinematics.py` 使用源 USD
的真实铰点尺寸，同时满足前四连杆、后两连杆和弹簧支链的闭环约束，再以 100 Hz 把
实际腿长映射到各视觉网格的角度。映射只修改无碰撞视觉几何，不会向 VMC/LQR 注入额外
闭链约束力。

结果：**腿长 133.9 mm，总质量 23.30 kg，站姿基座高度 184.8 mm**。

场景在机器人正前方 `+X` 方向放置一块可碰撞台阶：高 200 mm、沿行驶方向长
2000 mm、宽 1000 mm；前沿距初始轮心约 800 mm（`x=0.8…2.8 m`）。

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
| 按住 `4` / `6` | 左转 / 右转，按 5.0 rad/s² 积分到 ±0.5 rad/s；松开立即归零 |
| `1` / `2` / `3` | 腿长预设 150 / 200 / 300 mm（数字行或小键盘，目标值立即生效） |
| 按住 `0` | 直接轮毂模式：Tp=0、不运行 LQR、腿角/腿长目标保持不变，左右轮各输出 +2 N·m；松开后恢复 LQR |
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
左下角 HUD 实时显示速度指令与实际值、偏航、腿长、俯仰/横滚、机身高度，以及
VMC 轴向力 `F0`、虚拟髋力矩 `Tb`、实际髋关节电机力矩和轮毂电机力矩。按 `H`
也会把同一组控制量及连杆角度打印到控制台；HUD 中的 `link q` 显示左右腿
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
VMC 现采用硬件限幅：常规状态 `Tp=±20 N·m`、`Fn=±200 N`、轮毂电机 `±7 N·m`，并在
控制器和 MuJoCo 执行器两层同时生效。仅在跳跃 `thrust` 蹬伸阶段，Fn 上限提高到
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

按住 `0` 时，GUI 从真实按键按下/松开状态持续驱动直接轮毂模式，而不是依赖键盘
自动重复事件。该模式把虚拟髋力矩 `Tp` 置为 0，明确跳过 LQR 计算和增益调度，
轮子位置误差/积分固定为 0，偏航环不运行；腿角和腿长目标**不作任何改写**，仍由普通
腿长 VMC 保持当前命令。左右轮毂各输出固定 `+2 N·m`。松开 `0` 后，固定轮毂力矩立即
撤销，由正常 LQR 的实时计算值接管。

简化虚拟腿的滑移轴覆盖 90–450 mm 行程；其中源六连杆可严格闭环的可视化范围仍为
90–350 mm，因此其他 450 mm 虚拟腿命令会让连杆网格保持在该端点，轮子、碰撞和控制仍使用
完整的 450 mm 虚拟腿。简化模型不再在滑移关节上重复添加被动刚度；腿长
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

---

## 7. 已知限制

1. **位置 / 速度环偏弱（模型失配）**：解析推导假设腿是**匀质摆杆**（质心在腿长
   中点），但 USD 里腿部连杆的合并质心在**髋后方约 95 mm**（rear1/rear2 是向后
   伸的长臂）。后果：
   - 站立时会缓慢漂移约 3 m 后停在新的平衡点（`pos_ki` 位置积分把它限制住）；
   - 速度指令只影响**行进距离**，不能维持恒定速度（cmd 0.8 → 30 s 走 6.6 m）。
   姿态环（腿角 + 俯仰）工作得很好（0.01° 级）。
   `gen_model.py` 里的 `SYMMETRIC_LEG = True` 会把腿质心放回腿长中点以匹配抽象。

2. **转向能力有限**：参考固件里转向是**独立的 yaw 速率 PID** 差动叠加到轮力矩上
   （`Chassis::SynthesizeMotion()`），我也照此实现了，但在转换后的模型上：
   - 差动轮力矩会**反作用到髋关节**，让两条腿前后张开（"劈叉"），机身下沉；
   - 因此本仿真默认改用**纯 LQR 航向设定值跟随**（`yaw_pid_enable=False`），
   GUI 把持续转向速率限制在 **0.05 rad/s**；0.05 rad/s 与 0.8 m/s 前进组合已连续验证
   30 s。更高转向率只保留给离线调参脚本，长时间运行会让腿张开并最终发散。
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
| `probe_frame.py` | 验证 freejoint 角速度所在坐标系 |

### 产物
`demo.mp4`（38 s 演示视频）、`frames/*.png`（离屏渲染帧）
