"""Direct sign probes: wheel torque direction, hip torque direction."""
import numpy as np
import mujoco
import sim_lqr as S

sim = S.WheelLegLQR()
m, d = sim.m, sim.d

print("=== wheel torque sign ===")
for tq in (+2.0, -2.0):
    sim.reset()
    for i in range(200):                      # 0.2 s with the leg force only
        meas = sim.measure()
        ctrl = np.zeros(m.nu)
        ctrl[sim.a_leg["left"]] = sim.F0_ff
        ctrl[sim.a_leg["right"]] = sim.F0_ff
        d.ctrl[:] = ctrl
        mujoco.mj_step(m, d)
    # now apply a pure wheel torque for 0.2 s
    for i in range(200):
        ctrl = np.zeros(m.nu)
        ctrl[sim.a_leg["left"]] = sim.F0_ff
        ctrl[sim.a_leg["right"]] = sim.F0_ff
        ctrl[sim.a_wheel["left"]] = tq
        ctrl[sim.a_wheel["right"]] = tq
        d.ctrl[:] = ctrl
        mujoco.mj_step(m, d)
    print(f"  wheel ctrl={tq:+.1f}  -> base dx={d.qpos[0]:+.5f}  vx={d.qvel[0]:+.5f}  pitch={np.arcsin(-S.quat_to_mat(d.qpos[3:7])[2,0]):+.5f}")

print("=== hip torque sign ===")
for tq in (+5.0, -5.0):
    sim.reset()
    for i in range(200):
        ctrl = np.zeros(m.nu)
        ctrl[sim.a_leg["left"]] = sim.F0_ff
        ctrl[sim.a_leg["right"]] = sim.F0_ff
        d.ctrl[:] = ctrl
        mujoco.mj_step(m, d)
    for i in range(200):
        ctrl = np.zeros(m.nu)
        ctrl[sim.a_leg["left"]] = sim.F0_ff
        ctrl[sim.a_leg["right"]] = sim.F0_ff
        ctrl[sim.a_hip["left"]] = tq
        ctrl[sim.a_hip["right"]] = tq
        d.ctrl[:] = ctrl
        mujoco.mj_step(m, d)
    meas = sim.measure()
    print(f"  hip ctrl={tq:+.1f}  -> theta_L={meas['leg']['left'][1]:+.5f}  hip_q={d.qpos[m.jnt_qposadr[sim.j_hip['left']]]:+.5f}  pitch={meas['pitch']:+.5f}")
