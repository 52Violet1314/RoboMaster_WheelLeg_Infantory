"""Find a robustly stable steering envelope, 30 s per run."""
import numpy as np
import sim_lqr as S


def run(turn_cmd, kp, lim, seconds=30.0, roll_kp=None):
    sim = S.WheelLegLQR(speed_ramp=2.0)
    sim.yaw_pid = S.LegPID(kp, 0.0, 0.02 * kp, out_limit=lim, int_limit=0.3)
    sim.turn_limit = lim
    sim.reset()
    sim.turn_rate = turn_cmd
    for i in range(int(seconds / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
            return None, info
    return sim, info


print("30 s steering runs (roll / z are the failure indicators)")
for cmd, kp, lim in ((0.1, 2.0, 0.3), (0.1, 4.0, 0.5), (0.2, 4.0, 0.5),
                     (0.2, 8.0, 0.8), (0.3, 4.0, 0.5), (0.3, 8.0, 0.8),
                     (0.4, 4.0, 0.5), (0.4, 8.0, 0.8)):
    sim, info = run(cmd, kp, lim)
    tag = f"cmd={cmd:.1f} kp={kp:3.1f} lim={lim:.1f}"
    if sim is None:
        print(f"  {tag}  DIVERGED t={info['t']:5.1f}")
        continue
    m = sim.measure()
    print(f"  {tag}  wy={m['omega'][2]:+6.3f} yaw={np.degrees(m['yaw']):+7.1f} deg"
          f"  roll={np.degrees(m['roll']):+6.2f} deg  z={sim.d.qpos[2]:.4f}"
          f"  x={sim.d.qpos[0]:+6.2f}")
