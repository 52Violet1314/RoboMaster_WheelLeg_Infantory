# 等效直杆腿 LQR：使用说明

本文件夹把原先的 `simulation1`（非线性动力学、平衡点、线性化）和 `simulation2`（LQR）合并为一个模块。

模型与旧 `get_K_jiao_LQR.m` 一致：左右腿都视作等效直杆，`leg_length` 是唯一随工况改变的参数，当前 `phi_c=0`。

## 快速开始

在 MATLAB 执行：

```matlab
addpath('E:\Users\13302\Documents\串腿开源\wheelleg_lqr')
wheelleg_lqr('help')
```

最常用的三条命令：

```matlab
wheelleg_lqr;                         % 默认腿长 0.20 m，只输出 4x10 K
result = wheelleg_lqr('solve',0.20);  % 只计算，返回全部结果结构体
wheelleg_lqr('all',0.20);             % 打印全部计算过程
wheelleg_lqr('checks',0.20);          % 只打印残差、秩、稳定性检查
```

生成给 Simulink 实时插值用的整张增益表：

```matlab
schedule = build_lqr_schedule;
```

该命令用与原版相同的 Q/R，生成 `0.13:0.01:0.46 m` 的 34 套 K，
并保存为 `lqr_gain_schedule.mat`。低于 0.13 m 时十阶模型不可完全控制，
不生成 K；Simulink 的查表输入会夹紧到表范围，但不会修改腿长控制指令。

长度单位是 m；`0.20` 即 200 mm。

## 命令总表

统一格式：

```matlab
wheelleg_lqr('命令',leg_length)
```

| 命令 | 输出 |
|---|---|
| `help` | 命令帮助 |
| `solve` | 返回计算结构体，不打印 |
| `parameters` | 所有物理量的符号、数值、单位、含义 |
| `equilibrium` | 平衡点 `X*`、`U*`、所有角度、残差 |
| `symbolic` | 五条化简后方程的完整符号公式 |
| `five` | 五条化简方程的数值形式；完整 `Mred/Btau/Q_tau/G/C` |
| `state` | 十阶状态方程；完整 `A/B/Q/R` |
| `k` | 完整 `P/K`、开环和闭环极点 |
| `checks` | 静态残差、矩阵秩、条件数、可控性、CARE 残差和稳定性 |
| `all` | 以上全部内容 |

示例：

```matlab
wheelleg_lqr('parameters',0.15)
wheelleg_lqr('equilibrium',0.20)
wheelleg_lqr('five',0.20)
wheelleg_lqr('state',0.25)
wheelleg_lqr('k',0.20)
```

## 结果和控制律

```matlab
result.p                 % 当前腿长对应的全部物理参数
result.equilibrium.X     % 十阶参考状态 X*
result.equilibrium.U     % 四维参考力矩 U*
result.A, result.B       % 平衡点处线性模型
result.Q, result.R       % LQR 权重
result.P                 % Riccati 方程解
result.K                 % 4x10 LQR 增益
```

状态、输入顺序固定为：

```matlab
X = [s; ds; phi; dphi; theta_l_l; dtheta_l_l; ...
     theta_l_r; dtheta_l_r; theta_b; dtheta_b];

U = [Twl_l; Twl_r; Tbl_l; Tbl_r];
```

## 坐标、角度与力矩正方向

这一项已按 `D:\code\chuanlian_leg\get_K_jiao_LQR.m` 的状态和输入顺序统一，部署时**不再额外取反**：

- 车体系：`+x` 向右、`+y` 向前、`+z` 向上；`phi>0` 是绕 `+z` 左转。
- Simulink 腿二维计算：`+x` 向后、`+y` 向下；`theta_l_l/theta_l_r>0` 是腿前摆。
- `theta_b`、两个腿角和它们的角速度均须直接使用与旧 `get_K_jiao_LQR.m` 相同的控制器信号。旧脚本没有单独定义“车头抬/低”的文字符号，因此不能仅凭变量名再额外人为取反；应保持现有控制器该信号的原样接入。
- 输入顺序始终为 `[左轮, 右轮, 左腿等效力矩, 右腿等效力矩]`。控制律是 `U=U* - K(X-X*)`。

质心在这个二维等效模型中不是完整三维坐标：`l_w_l/l_b_l` 是腿质心相对轮心、髋点沿等效腿的距离，`l_c` 是车身质心相对左右髋中点的**有符号**平面偏距，`phi_c` 是该偏距的角偏置。当前 `phi_c=0`；腿被假定为均匀等效直杆，所以质心在腿中点。

部署时必须使用：

```matlab
U_cmd = result.equilibrium.U ...
      - result.K * (X - result.equilibrium.X);
```

不要只写 `-K*X`；否则会丢失平衡点偏置。`Tbl_l/Tbl_r` 是左右等效虚拟腿力矩，仍需经过现有 VMC 映射为每侧 A 点的两个实际电机力矩。

## 文件说明

| 文件 | 作用 | 是否需要直接运行 |
|---|---|---|
| `wheelleg_lqr.m` | 统一命令入口和全部报告 | 是，日常只运行它 |
| `zju_parameters_template.m` | 集中定义等效腿物理参数 | 改参数时查看 |
| `zju_nonlinear_eliminate.m` | 五自由度完整数值非线性动力学，消元后求 `Mred/Btau/G/C` | 否 |
| `zju_find_static_equilibrium.m` | 搜索 `X*`、`U*`，并在该点数值线性化得到 `A/B` | 否 |
| `zju_lqr_gain.m` | 调用平衡点模型并使用 `icare` 求 `P/K` | 否 |
| `build_lqr_schedule.m` | 按 1 cm 生成并校验实时 LQR 增益表 | 需要重新生成表时运行 |
| `lqr_gain_schedule.mat` | 已校验的 34 套 K 表，供审查或复现 | 否 |
| `zju_symbolic_eliminate.m` | 符号消元，生成五条符号方程和矩阵元素 | 通常否 |
| `zju_非线性符号方程.md` | 完整符号公式文档 | 可直接打开阅读 |

## 当前物理参数

这些参数直接复刻旧 `get_K_jiao_LQR.m`：

```matlab
R_w = 0.0525;       R_l = 0.20;
m_w = 0.30;         m_l = m_r = 0.80;
m_b = 15.75;        l_c = -0.03;
phi_c = 0;          I_zz = 0.226;

l_w_l = l_w_r = leg_length/2;
l_b_l = l_b_r = leg_length/2;
I_l = I_r = 0.80*(leg_length^2 + 0.048^2)/12;
```

改腿长：调用时传入新的 `leg_length`。改质量、轮半径、轮距、车身惯量等固定参数：编辑 `zju_parameters_template.m` 后重新运行。

## 如何判断结果有效

```matlab
wheelleg_lqr('checks',0.20)
```

需要满足：

- `static five-equation residual norm` 足够小；
- `state derivative norm at (X*,U*)` 足够小；
- `Mred rank` 为 5；
- `controllability rank [A B]` 为 10；
- `CARE residual` 足够小；
- `max real closed-loop pole` 小于 0。

若平衡点搜索失败，不应使用该腿长的 K；先检查腿长、物理参数、角度零位和正方向。
