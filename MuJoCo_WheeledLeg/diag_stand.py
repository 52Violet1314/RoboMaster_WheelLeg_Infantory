"""Watch the standing case: does the leg splay grow from t=0?"""
import numpy as np
import sim_lqr as S

sim = S.WheelLegLQR()
sim.reset()
print(f"{'t':>6}{'z':>8}{'pitch':>8}{'roll':>8}{'hipL':>8}{'hipR':>8}"
      f"{'wx':>9}{'wy':>9}{'x':>9}{'F0L':>8}{'F0R':>8}{'TbL':>8}{'TbR':>8}")
for i in range(int(40 / sim.dt)):
    info = sim.step()
    if i % int(2.0 / sim.dt) == 0:
        m = sim.measure()
        hL = np.degrees(sim.d.qpos[sim.m.jnt_qposadr[sim.j_hip["left"]]])
        hR = np.degrees(sim.d.qpos[sim.m.jnt_qposadr[sim.j_hip["right"]]])
        print(f"{info['t']:6.1f}{info['z']:8.4f}{np.degrees(m['pitch']):8.3f}"
              f"{np.degrees(m['roll']):8.3f}{hL:8.2f}{hR:8.2f}"
              f"{m['omega'][0]:9.5f}{m['omega'][2]:9.5f}{sim.d.qpos[0]:9.4f}"
              f"{sim.d.ctrl[sim.a_leg['left']]:8.1f}{sim.d.ctrl[sim.a_leg['right']]:8.1f}"
              f"{info['TbL']:8.3f}{info['TbR']:8.3f}")
