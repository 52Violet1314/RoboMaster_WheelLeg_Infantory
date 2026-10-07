"""Steering via the LQR yaw setpoint only (no separate PID)."""
import numpy as np
import sim_lqr as S


def run(rate, seconds=40.0, ramp=1.0):
    sim = S.WheelLegLQR(speed_ramp=2.0)
    sim.yaw_pid_enable = False
    sim.reset()
    for i in range(int(seconds / sim.dt)):
        # ramp the heading setpoint
        sim.target_yaw += rate * sim.dt if i > int(ramp / sim.dt) else 0.0
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
            return None, info
    return sim, info


print("pure-LQR heading setpoint tracking (no yaw PID)")
for rate in (0.05, 0.1, 0.2, 0.3, 0.5):
    sim, info = run(rate)
    if sim is None:
        print(f"  rate={rate:.2f} rad/s  DIVERGED t={info['t']:.1f}")
        continue
    m = sim.measure()
    hL = np.degrees(sim.d.qpos[sim.m.jnt_qposadr[sim.j_hip["left"]]])
    print(f"  rate={rate:.2f} rad/s  t={info['t']:5.1f}  wy={m['omega'][2]:+7.4f}"
          f"  yaw={np.degrees(m['yaw']):+7.1f} deg  target={np.degrees(sim.target_yaw):+7.1f}"
          f"  pitch={np.degrees(m['pitch']):+5.2f}  z={info['z']:.4f}  hipL={hL:+6.2f}")
