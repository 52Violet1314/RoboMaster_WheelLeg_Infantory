"""Instantaneous acceleration response -> numerical B matrix of the model."""
import numpy as np
import mujoco
import sim_lqr as S

sim = S.WheelLegLQR()
m, d = sim.m, sim.d
sim.reset()

dof = {n: m.jnt_dofadr[j] for n, j in
       [("hipL", sim.j_hip["left"]), ("hipR", sim.j_hip["right"]),
        ("whL", sim.j_wheel["left"]), ("whR", sim.j_wheel["right"]),
        ("slL", sim.j_slide["left"]), ("slR", sim.j_slide["right"])]}
acts = {n: a for n, a in
        [("hipL", sim.a_hip["left"]), ("hipR", sim.a_hip["right"]),
         ("whL", sim.a_wheel["left"]), ("whR", sim.a_wheel["right"]),
         ("slL", sim.a_leg["left"]), ("slR", sim.a_leg["right"])]}

names = ["hipL", "hipR", "whL", "whR", "slL", "slR"]


def accel_with(ctrl_vec):
    d.ctrl[:] = 0.0
    for n, v in ctrl_vec.items():
        d.ctrl[acts[n]] = v
    mujoco.mj_forward(m, d)
    return d.qacc.copy()


base = accel_with({"slL": sim.F0_ff, "slR": sim.F0_ff})
print("unit-input acceleration response (qacc), nominal pose "
      f"(leg force {sim.F0_ff:.1f} N applied)")
print(f"{'input':>6}" + "".join(f"{n:>11}" for n in names))
for n in names:
    a = accel_with({"slL": sim.F0_ff, "slR": sim.F0_ff, n: 1.0})
    da = a - base
    print(f"{n:>6}" + "".join(f"{da[dof[k]]:+11.4f}" for k in names))

print()
print("free-joint (base) acceleration response")
print(f"{'input':>6}{'ax':>12}{'az':>12}{'wy(pitch)':>12}{'wx(roll)':>12}")
for n in names:
    a = accel_with({"slL": sim.F0_ff, "slR": sim.F0_ff, n: 1.0})
    da = a - base
    print(f"{n:>6}{da[0]:+12.4f}{da[2]:+12.4f}{da[4]:+12.4f}{da[3]:+12.4f}")
