"""Steering PID sweep with a realistic differential-torque limit."""
import numpy as np
import sim_lqr as S


def run(kp, ki, kd, lim, turn_cmd, seconds=20.0):
    sim = S.WheelLegLQR(speed_ramp=2.0)
    sim.yaw_pid = S.LegPID(kp, ki, kd, out_limit=lim, int_limit=0.5)
    sim.turn_limit = lim
    sim.reset()
    sim.turn_rate = turn_cmd
    for i in range(int(seconds / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
            return None, info
    return sim, info


print("cmd 0.4 rad/s, 20 s")
for kp, ki, kd, lim in ((1.0, 0, 0.02, 1.5), (2.0, 0, 0.05, 1.5), (4.0, 0, 0.1, 1.5),
                        (2.0, 0.2, 0.05, 1.5), (8.0, 0, 0.2, 1.5), (2.0, 0, 0.05, 0.8)):
    sim, info = run(kp, ki, kd, lim, 0.4)
    tag = f"kp={kp:4.1f} ki={ki:3.1f} kd={kd:4.2f} lim={lim:3.1f}"
    if sim is None:
        print(f"  {tag}  DIVERGED t={info['t']:.1f}")
        continue
    m = sim.measure()
    print(f"  {tag}  wy={m['omega'][2]:+7.3f} rad/s  yaw={np.degrees(m['yaw']):+7.1f} deg"
          f"  pitch={np.degrees(m['pitch']):+5.2f}  roll={np.degrees(m['roll']):+5.2f}"
          f"  x={sim.d.qpos[0]:+6.2f}")
