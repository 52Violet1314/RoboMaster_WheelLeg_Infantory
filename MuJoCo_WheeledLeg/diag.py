import numpy as np
import mujoco
import sim_lqr as S

sim = S.WheelLegLQR()
sim.reset()
d, m = sim.d, sim.m
print(f"{'t':>6}{'base_x':>10}{'base_vx':>10}{'wheel_vx':>10}{'slip':>9}"
      f"{'z':>9}{'L0':>9}{'theta':>9}{'F0':>9}{'N_L':>9}")
for i in range(int(1.0 / sim.dt)):
    info = sim.step()
    if i % int(0.02 / sim.dt) == 0:
        # ground normal force on the left wheel
        nrm = 0.0
        wl = m.geom_bodyid
        left_wheel_geoms = [g for g in range(m.ngeom)
                            if m.geom_bodyid[g] == sim.b_wheel["left"]]
        for ci in range(d.ncon):
            c = d.contact[ci]
            if c.geom1 in left_wheel_geoms or c.geom2 in left_wheel_geoms:
                nrm += abs(d.efc_force[d.contact_efc_address[ci]]) if False else 0.0
        print(f"{info['t']:6.3f}{info['x']:10.5f}{d.qvel[0]:10.5f}"
              f"{info['vx']:10.5f}{d.qvel[0]-info['vx']:9.5f}"
              f"{info['z']:9.5f}{info['L0']:9.5f}{info['theta']:9.5f}"
              f"{info['legF']:9.2f}{nrm:9.2f}")
