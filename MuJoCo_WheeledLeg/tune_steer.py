"""Tune the steering PID and the velocity weight."""
import numpy as np
import sim_lqr as S


def run(sim, seconds, speed=0.0, turn=0.0):
    sim.reset()
    sim.speed_cmd = speed
    sim.turn_rate = turn
    for i in range(int(seconds / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
            return None, info
    return sim, info


def report(sim, tag):
    m = sim.measure()
    vx = S.WHEEL_R * 0.5 * (m["wheel_vel"]["left"] + m["wheel_vel"]["right"])
    print(f"  {tag:26s} vx={vx:+7.3f} m/s  wy={m['omega'][2]:+7.3f} rad/s"
          f"  yaw={np.degrees(m['yaw']):+7.1f} deg"
          f"  pitch={np.degrees(m['pitch']):+5.2f}  roll={np.degrees(m['roll']):+5.2f}")


print("=== steering PID gains (cmd 0.6 rad/s, 20 s) ===")
for kp, ki, kd in ((12, 0, 0.5), (30, 0, 1.0), (30, 2, 1.0), (60, 0, 2.0), (20, 5, 1.5)):
    sim = S.WheelLegLQR(speed_ramp=2.0)
    sim.yaw_pid = S.LegPID(kp, ki, kd, out_limit=8.0, int_limit=2.0)
    s2, info = run(sim, 20.0, turn=0.6)
    if s2 is None:
        print(f"  kp={kp:3d} ki={ki:3.1f} kd={kd:3.1f}  DIVERGED at t={info['t']:.1f}")
    else:
        report(s2, f"kp={kp:3d} ki={ki:3.1f} kd={kd:3.1f}")

print()
print("=== velocity weight (cmd 0.6 m/s, 15 s) ===")
for qv in (1.0, 100.0, 400.0, 1000.0, 2000.0):
    sim = S.WheelLegLQR(speed_ramp=2.0, q_vel=qv)
    s2, info = run(sim, 15.0, speed=0.6)
    if s2 is None:
        print(f"  Q[ds]={qv:7.0f}  DIVERGED at t={info['t']:.1f}")
    else:
        report(s2, f"Q[ds]={qv:7.0f}")
