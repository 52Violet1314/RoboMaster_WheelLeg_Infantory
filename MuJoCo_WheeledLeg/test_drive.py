"""Headless test of the new turn-rate / e-stop channels."""
import numpy as np
import sim_lqr as S

print("=== turn rate (GUI-safe 0.05 rad/s for 20 s) ===")
sim = S.WheelLegLQR()
sim.reset()
sim.turn_rate = 0.05
for i in range(int(20 / sim.dt)):
    info = sim.step()
    if i % int(1.0 / sim.dt) == 0:
        m = sim.measure()
        print(f"  t={info['t']:4.1f}  yaw={np.degrees(m['yaw']):+7.2f} deg"
              f"  target_yaw={np.degrees(sim.target_yaw):+7.2f} deg"
              f"  wy={m['omega'][2]:+.3f} rad/s  pitch={np.degrees(m['pitch']):+.3f} deg")

print()
print("=== combined: 0.6 m/s forward + 0.05 rad/s turn, 20 s ===")
sim = S.WheelLegLQR(target_speed=0.6)
sim.reset()
sim.turn_rate = 0.05
for i in range(int(20 / sim.dt)):
    info = sim.step()
    if i % int(2.0 / sim.dt) == 0:
        m = sim.measure()
        vx = S.WHEEL_R * 0.5 * (m["wheel_vel"]["left"] + m["wheel_vel"]["right"])
        print(f"  t={info['t']:4.1f}  x={sim.d.qpos[0]:+6.3f}  y={sim.d.qpos[1]:+6.3f}"
              f"  yaw={np.degrees(m['yaw']):+7.2f} deg  vx={vx:+.3f} m/s"
              f"  pitch={np.degrees(m['pitch']):+.3f} deg  roll={np.degrees(m['roll']):+.3f} deg")

print()
print("=== emergency stop then release ===")
sim = S.WheelLegLQR()
sim.reset()
for _ in range(int(2 / sim.dt)):
    sim.step()
z_before = sim.d.qpos[2]
sim.estop = True
for _ in range(int(1.5 / sim.dt)):
    sim.step()
print(f"  z before e-stop {z_before:.4f} -> after 1.5 s {sim.d.qpos[2]:.4f}"
      f"  (ctrl all zero: {np.allclose(sim.d.ctrl, 0.0)})")
# A balancing robot cannot recover from the ground by torque alone.  The GUI's
# second X press performs this upright reset before releasing the e-stop.
sim.reset()
sim.estop = False
for _ in range(int(6 / sim.dt)):
    sim.step()
print(f"  after release + 6 s: z={sim.d.qpos[2]:.4f}"
      f"  pitch={np.degrees(sim.measure()['pitch']):+.3f} deg")
