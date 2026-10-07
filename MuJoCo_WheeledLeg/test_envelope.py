"""Find the safe speed / turn envelope of the analytic-K controller."""
import numpy as np
import sim_lqr as S

print("forward speed envelope (12 s, ramp 2 s)")
for spd in (0.2, 0.4, 0.6, 0.8, 1.0):
    sim = S.WheelLegLQR(target_speed=spd, speed_ramp=2.0)
    sim.reset()
    ok = True
    for i in range(int(12 / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
            ok = False
            break
    if ok:
        m = sim.measure()
        vx = S.WHEEL_R * 0.5 * (m["wheel_vel"]["left"] + m["wheel_vel"]["right"])
        print(f"  cmd {spd:.1f} -> vx {vx:+.3f} m/s   pitch {np.degrees(m['pitch']):+6.2f} deg"
              f"   roll {np.degrees(m['roll']):+6.2f} deg")
    else:
        print(f"  cmd {spd:.1f} -> DIVERGED at t={sim.d.time:.2f} s")

print()
print("turn rate envelope (standing, 10 s)")
for tr in (0.2, 0.4, 0.6, 0.8):
    sim = S.WheelLegLQR(speed_ramp=2.0)
    sim.reset()
    sim.turn_rate = tr
    ok = True
    for i in range(int(10 / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
            ok = False
            break
    if ok:
        m = sim.measure()
        print(f"  cmd {tr:.1f} rad/s -> wy {m['omega'][2]:+.3f} rad/s"
              f"   pitch {np.degrees(m['pitch']):+6.2f} deg"
              f"   roll {np.degrees(m['roll']):+6.2f} deg")
    else:
        print(f"  cmd {tr:.1f} rad/s -> DIVERGED at t={sim.d.time:.2f} s")

print()
print("combined (speed 0.4, turn 0.3, 12 s)")
sim = S.WheelLegLQR(target_speed=0.4, speed_ramp=2.0)
sim.reset()
sim.turn_rate = 0.3
ok = True
for i in range(int(12 / sim.dt)):
    info = sim.step()
    if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
        ok = False
        break
m = sim.measure()
vx = S.WHEEL_R * 0.5 * (m["wheel_vel"]["left"] + m["wheel_vel"]["right"])
print(f"  {'OK' if ok else 'DIVERGED'}  x={sim.d.qpos[0]:+.3f} y={sim.d.qpos[1]:+.3f}"
      f"  yaw={np.degrees(m['yaw']):+7.1f} deg  vx={vx:+.3f} m/s"
      f"  pitch={np.degrees(m['pitch']):+.2f} deg  roll={np.degrees(m['roll']):+.2f} deg")
