"""Probe the roll control loop: apply a roll perturbation, watch the recovery,
and check the sign of the roll compensation term."""
import numpy as np
import mujoco
import sim_lqr as S

sim = S.WheelLegLQR()
m, d = sim.m, sim.d


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def roll_quat(a):
    return np.array([np.cos(a/2), np.sin(a/2), 0.0, 0.0])


print("A. roll impulse with the roll compensation DISABLED")
for kp_roll in (0.0,):
    sim.reset()
    for i in range(int(0.5 / sim.dt)):
        sim.step()
    # kick the roll by 5 deg
    d.qpos[3:7] = quat_mul(roll_quat(np.deg2rad(5.0)), d.qpos[3:7])
    mujoco.mj_forward(m, d)
    print(f"  roll_kp={kp_roll}")
    for i in range(int(3.0 / sim.dt)):
        info = sim.step()
        if i % int(0.5 / sim.dt) == 0:
            mm = sim.measure()
            print(f"    t={info['t']:4.1f} roll={np.degrees(mm['roll']):+8.3f} deg"
                  f"  wx={mm['omega'][0]:+8.4f}  z={info['z']:.4f}")

print()
print("B. which F0 differential reduces a POSITIVE roll?")
sim.reset()
for i in range(int(1.0 / sim.dt)):
    sim.step()
# hold the legs at a fixed force and apply a differential, no roll feedback
for dl, dr in ((-20.0, +20.0), (+20.0, -20.0)):
    sim.reset()
    for i in range(int(1.0 / sim.dt)):
        sim.step()
    d.qpos[3:7] = quat_mul(roll_quat(np.deg2rad(5.0)), d.qpos[3:7])
    mujoco.mj_forward(m, d)
    for i in range(int(1.0 / sim.dt)):
        meas = sim.measure()
        x, _ = sim.state_vector(meas)
        ctrl, _ = sim.control(meas, x, 0.0)
        ctrl[sim.a_leg["left"]] = sim.F0_ff + dl
        ctrl[sim.a_leg["right"]] = sim.F0_ff + dr
        d.ctrl[:] = ctrl
        mujoco.mj_step(m, d)
    mm = sim.measure()
    print(f"  F0L{sim.F0_ff+dl:6.1f} F0R{sim.F0_ff+dr:6.1f}"
          f"  ->  roll={np.degrees(mm['roll']):+8.3f} deg  wx={mm['omega'][0]:+8.4f}")
