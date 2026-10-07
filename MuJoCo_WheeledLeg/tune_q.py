"""Tune the Q weights for velocity / yaw-rate tracking, and test a rate-mode
turn controller (yaw position error forced to zero)."""
import numpy as np
import sim_lqr as S
import lqr_k


def make(q_v, q_w, q_yaw=400.0):
    Q = np.diag([20.0, q_v, q_yaw, q_w, 35000.0, 10.0, 35000.0, 10.0, 40000.0, 1.0])
    sim = S.WheelLegLQR(speed_ramp=2.0)
    sim.K, sim.A, sim.B, _ = lqr_k.compute_K(sim.leg_length, S.BODY, Q=Q)
    return sim


def run(sim, seconds, speed=0.0, turn=0.0, rate_mode=False):
    sim.reset()
    sim.speed_cmd = speed
    for i in range(int(seconds / sim.dt)):
        if rate_mode:
            sim.turn_rate = 0.0
            sim._turn_ref = turn
        else:
            sim.turn_rate = turn
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
            return None, info
    return sim, info


print("=== A. velocity tracking vs Q[ds] ===")
for qv in (1.0, 50.0, 500.0, 5000.0):
    sim = make(qv, 1.0)
    sim.theta_mode = "abs"
    for spd in (0.4, 0.8):
        s2, info = run(sim, 12.0, speed=spd)
        if s2 is None:
            print(f"  Q[ds]={qv:8.1f}  cmd {spd:.1f} -> DIVERGED")
            continue
        m = s2.measure()
        vx = S.WHEEL_R * 0.5 * (m["wheel_vel"]["left"] + m["wheel_vel"]["right"])
        print(f"  Q[ds]={qv:8.1f}  cmd {spd:.1f} -> vx {vx:+.4f} m/s"
              f"   pitch {np.degrees(m['pitch']):+6.2f} deg")

print()
print("=== B. yaw-rate tracking vs Q[dyaw] ===")
for qw in (1.0, 100.0, 1000.0, 10000.0):
    sim = make(1.0, qw)
    for tr in (0.3, 0.6):
        s2, info = run(sim, 15.0, turn=tr)
        if s2 is None:
            print(f"  Q[dyaw]={qw:9.1f}  cmd {tr:.1f} -> DIVERGED at t={info['t']:.1f}")
            continue
        m = s2.measure()
        print(f"  Q[dyaw]={qw:9.1f}  cmd {tr:.1f} -> wy {m['omega'][2]:+.4f} rad/s"
              f"   pitch {np.degrees(m['pitch']):+6.2f} deg"
              f"   roll {np.degrees(m['roll']):+6.2f} deg")
