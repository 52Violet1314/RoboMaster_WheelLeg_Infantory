"""Measure the yaw response to a differential wheel torque."""
import numpy as np
import mujoco
import sim_lqr as S

sim = S.WheelLegLQR()
m, d = sim.m, sim.d

for dl, dr in ((+2.0, -2.0), (-2.0, +2.0), (+2.0, +2.0)):
    sim.reset()
    # hold the leg force so the robot stays up, then apply wheel torques
    for i in range(int(3.0 / sim.dt)):
        meas = sim.measure()
        x, _ = sim.state_vector(meas)
        ctrl, _ = sim.control(meas, x, 0.0)
        ctrl[sim.a_wheel["left"]] = dl
        ctrl[sim.a_wheel["right"]] = dr
        d.ctrl[:] = ctrl
        mujoco.mj_step(m, d)
    meas = sim.measure()
    print(f"  wheel L={dl:+.1f} R={dr:+.1f}  ->  yaw={np.degrees(meas['yaw']):+8.3f} deg"
          f"   wy={meas['omega'][2]:+8.4f} rad/s"
          f"   pitch={np.degrees(meas['pitch']):+6.2f} deg")

print()
print("step response of wy to a L/R = +2/-2 differential (first 0.4 s):")
sim.reset()
for i in range(int(0.4 / sim.dt)):
    meas = sim.measure()
    x, _ = sim.state_vector(meas)
    ctrl, _ = sim.control(meas, x, 0.0)
    ctrl[sim.a_wheel["left"]] = +2.0
    ctrl[sim.a_wheel["right"]] = -2.0
    d.ctrl[:] = ctrl
    mujoco.mj_step(m, d)
    if i % 50 == 0:
        meas = sim.measure()
        print(f"    t={d.time:5.3f}  wy={meas['omega'][2]:+9.5f}  yaw={np.degrees(meas['yaw']):+8.3f} deg")
