"""Behaviour tests: position hold vs Q[0], speed command, disturbance rejection."""
import numpy as np
import sim_lqr as S
import lqr_k


def run(sim, seconds, callback=None, log=None):
    rows = []
    for i in range(int(seconds / sim.dt)):
        info = sim.step()
        if callback:
            callback(i, info)
        if log and i % int(log / sim.dt) == 0:
            rows.append(info)
    return rows


print("=== 1. position hold vs Q[0] weight (40 s) ===")
for q0 in (20.0, 500.0, 5000.0):
    Q = np.diag([q0, 1.0, 400.0, 1.0, 35000.0, 10.0, 35000.0, 10.0, 40000.0, 1.0])
    sim = S.WheelLegLQR()
    sim.K, sim.A, sim.B, _ = lqr_k.compute_K(sim.leg_length, S.BODY, Q=Q)
    sim.reset()
    run(sim, 40.0)
    print(f"  Q[0]={q0:8.0f}  x_final={sim.d.qpos[0]:+8.3f}  "
          f"vx={sim.d.qvel[0]:+8.5f}  pitch={sim.measure()['pitch']:+.5f}")

print()
print("=== 2. speed command tracking (target 0.5 m/s) ===")
sim = S.WheelLegLQR(target_speed=0.5)
sim.reset()
for i in range(int(12.0 / sim.dt)):
    info = sim.step()
    if i % int(1.0 / sim.dt) == 0:
        print(f"  t={info['t']:5.1f}  x={info['x']:+8.3f}  vx={info['vx']:+8.4f}  "
              f"pitch={info['pitch']:+.5f}  L0={info['L0']:.4f}")

print()
print("=== 3. disturbance rejection (20 N lateral push for 0.3 s at t=5 s) ===")
sim = S.WheelLegLQR()
sim.reset()
peak = 0.0
for i in range(int(15.0 / sim.dt)):
    t = i * sim.dt
    if 5.0 <= t < 5.3:
        # push the body forward with 40 N
        sim.d.xfrc_applied[sim.b_base, 0] = 40.0
    else:
        sim.d.xfrc_applied[sim.b_base, 0] = 0.0
    info = sim.step()
    peak = max(peak, abs(info["pitch"]))
    if i % int(1.0 / sim.dt) == 0:
        print(f"  t={info['t']:5.1f}  x={info['x']:+8.3f}  vx={info['vx']:+8.4f}  "
              f"pitch={info['pitch']:+.5f}  roll={info['roll']:+.5f}")
print(f"  peak |pitch| = {peak:.5f} rad ({np.degrees(peak):.2f} deg)")

print()
print("=== 4. leg-length step (0.134 -> 0.180 m at t=3 s) ===")
sim = S.WheelLegLQR()
sim.reset()
for i in range(int(10.0 / sim.dt)):
    if i == int(3.0 / sim.dt):
        sim.leg_length = 0.180
    info = sim.step()
    if i % int(0.5 / sim.dt) == 0:
        print(f"  t={info['t']:5.1f}  L0={info['L0']:.4f}  z={info['z']:.4f}  "
              f"pitch={info['pitch']:+.5f}  F0={info['legF']:7.2f}  TbL={info['TbL']:+.3f}")
