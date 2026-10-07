"""Check whether the LQR yaw channel (target_yaw) can actually turn the robot."""
import numpy as np
import sim_lqr as S

for yaw_cmd in (0.0, 0.3, -0.3):
    sim = S.WheelLegLQR(target_yaw=yaw_cmd)
    sim.reset()
    for i in range(int(12 / sim.dt)):
        info = sim.step()
    meas = sim.measure()
    print(f"yaw_cmd={yaw_cmd:+.2f} rad -> yaw={meas['yaw']:+.5f} rad  "
          f"wy={meas['omega'][2]:+.5f} rad/s  pitch={np.degrees(meas['pitch']):+.3f} deg  "
          f"TwL={info['TwL']:+.3f} TwR={info['TwR']:+.3f}")
