# C 版 WheelLeg 仿真

`wheelleg_c_sim` 是不依赖 Python 运行时的 MuJoCo C 控制循环。它读取原来的
`wheelbipe_lqr.xml`，并直接调用仓库已有的 `embedded_mpc/explicit_mpc.c`。

当前已迁入 C 的运行路径：状态测量与坐标变换、10 维状态组装、腿长 PID、VMC
支持力、轮反力前馈、双腿防劈腿 PID、位置积分、输入限幅和显式 MPC。

从仓库根目录构建：

```powershell
cmake -S MuJoCo_WheeledLeg/c_sim -B MuJoCo_WheeledLeg/c_sim/build -G "MinGW Makefiles"
cmake --build MuJoCo_WheeledLeg/c_sim/build --parallel
```

在 `MuJoCo_WheeledLeg` 目录运行：

```powershell
./c_sim/build/wheelleg_c_sim.exe ./wheelbipe_lqr.xml 5.0 0.1339 0.0
```

五个可选位置参数依次为：模型 XML、时长（s）、目标腿长（m）、目标速度（m/s）、
离地测试开关（`0/1`）。最后一项为 `1` 时，程序在 1 秒处给车身一个向上的测试脉冲：

```powershell
./c_sim/build/wheelleg_c_sim.exe ./wheelbipe_lqr.xml 4.0 0.1339 0.0 1
```

程序打印的 `TwL/TwR/TbL/TbR` 是每个 1 ms 控制周期中 C 显式 MPC 的虚拟输入；
控制并不调用 Python。

启动原生 C GUI：

```powershell
./c_sim/build/wheelleg_c_sim.exe --gui ./wheelbipe_lqr.xml 0.1339 0.0
```

GUI 的左侧是 MuJoCo 三维视图；右侧从上到下分别画 `x` 的车身状态、双腿状态、
显式 MPC 输出 `u=[TwL,TwR,TbL,TbR]` 和支持力。支持力图同时显示从 STM32
离地算法移植来的 `Fn est L/R`、MuJoCo 轮地接触真值 `Fn contact L/R`，以及
15 N 离地、22.5 N 接地阈值。曲线独立按 100 Hz 采样，保留最近 5 秒，
控制和物理步进仍为 1 kHz。

C 动力学使用与 Python `sim_lqr.py` 相同的连杆角查表、0.5 mm 中心差分和
15 N·m 气弹簧力矩计算被动推力；电机只补足总 VMC 目标与被动推力之差。
离地观测器再按固件做法还原轮端净 `F0`，因此支持力对比也覆盖气弹簧项。

C 控制路径现与 Python GUI 默认的显式 MPC 路径保持一致，包括：1 m/s² 内层
速度斜坡、位置轨迹启停和松键驻车保持、停车 PI 制动、轮反力前馈、双腿防劈腿、
横滚支撑力补偿、按轮地接触/姿态切换的跳跃状态机，以及 `9` 键物理自起身状态机。

| 操作 | 功能 |
| --- | --- |
| 按住 `8/5` 或 `W/S` | 以 5 m/s² 加/减目标速度，限幅 ±2.5 m/s；松开归零 |
| 按住 `4/6` 或 `A/D` | 以 25 rad/s² 加/减转向速度，限幅 ±2.5 rad/s；松开停止并锁定当前航向 |
| `1/2/3` | 腿长目标 150 / 270 / 350 mm |
| 按住 `0` 或小键盘 `0` | 髋部力矩限幅降至 ±2 N·m；松开恢复 ±10 N·m |
| `F` | 开关相机跟随车体 |
| `J` | 给车身向上测试脉冲，观察离地—落地判定 |
| `V` | 开关 MuJoCo 接触力箭头 |
| `Space` | 执行下蹲—伸腿—收腿—落地恢复跳跃流程 |
| `7` | 禁用全部控制力 |
| `X` | 急停；再次按下会以当前腿长直立复位后释放 |
| `9` | 执行与 Python 相同的物理自起身流程，最后回到 150 mm 平衡 |
| 鼠标左键拖动 | 自由旋转视角 |
| 鼠标右键拖动或滚轮 | 缩放 |
| `C` | 恢复默认相机角度 |
| `Z` / `R`、`Esc` | 复位、退出 |
