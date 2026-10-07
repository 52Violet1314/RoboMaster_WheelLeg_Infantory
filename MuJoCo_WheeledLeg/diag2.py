import numpy as np
import sim_lqr as S

for yaw_en in (True, False):
    sim = S.WheelLegLQR()
    sim.yaw_pid_enable = yaw_en
    sim.reset()
    for i in range(int(20 / sim.dt)):
        info = sim.step()
    meas = sim.measure()
    x, vx = sim.state_vector(meas)
    print(f"yaw_pid_enable={yaw_en}")
    print("  omega world =", np.round(meas["omega"], 6))
    print("  state x     =", np.round(x, 6))
    print("  u = -Kx     =", np.round(-sim.K @ x, 4))
    print(f"  wheel torques {info['TwL']:+.4f} / {info['TwR']:+.4f}"
          f"   hip {info['TbL']:+.4f} / {info['TbR']:+.4f}"
          f"   turn {info['turn']:+.4f}")
    print(f"  z={info['z']:.4f} pitch={np.degrees(meas['pitch']):+.4f} deg")
    print()
