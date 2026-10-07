import numpy as np
import sim_lqr as S

sim = S.WheelLegLQR()
sim.reset()
print(f"{'t':>6}{'x':>9}{'vx':>9}{'z':>8}{'pitch':>10}{'L0':>8}{'theta':>9}"
      f"{'TwL':>8}{'TbL':>8}{'F0':>8}{'s_int':>8}")
for i in range(int(2.0 / sim.dt)):
    info = sim.step()
    if i % int(0.05 / sim.dt) == 0:
        print(f"{info['t']:6.2f}{info['x']:9.4f}{info['vx']:9.4f}{info['z']:8.4f}"
              f"{info['pitch']:10.5f}{info['L0']:8.4f}{info['theta']:9.4f}"
              f"{info['TwL']:8.3f}{info['TbL']:8.3f}{info['legF']:8.2f}"
              f"{sim.s_int:8.4f}")
