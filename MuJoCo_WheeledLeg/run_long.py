import numpy as np
import sim_lqr as S

sim = S.WheelLegLQR()
sim.reset()
hdr = ("t", "x", "z", "pitch", "roll", "L0", "theta", "vx", "TwL", "TbL")
print(f"{hdr[0]:>7}{hdr[1]:>10}{hdr[2]:>9}{hdr[3]:>10}{hdr[4]:>10}"
      f"{hdr[5]:>9}{hdr[6]:>9}{hdr[7]:>9}{hdr[8]:>8}{hdr[9]:>8}")
for i in range(int(40 / sim.dt)):
    info = sim.step()
    if i % int(2.0 / sim.dt) == 0:
        print(f"{info['t']:7.1f}{info['x']:10.4f}{info['z']:9.4f}"
              f"{info['pitch']:10.5f}{info['roll']:10.5f}{info['L0']:9.4f}"
              f"{info['theta']:9.4f}{info['vx']:9.4f}{info['TwL']:8.3f}"
              f"{info['TbL']:8.3f}")
print("final:", {k: round(float(v), 5) for k, v in info.items()})
