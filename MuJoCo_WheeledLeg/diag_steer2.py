"""Detailed log of a steering run to pinpoint the failure."""
import numpy as np
import sim_lqr as S

sim = S.WheelLegLQR(speed_ramp=2.0)
sim.yaw_pid = S.LegPID(4.0, 0.0, 0.08, out_limit=0.5, int_limit=0.3)
sim.turn_limit = 0.5
sim.reset()
sim.turn_rate = 0.2

print(f"{'t':>5}{'yaw':>8}{'pitch':>8}{'roll':>8}{'z':>8}"
      f"{'thL':>8}{'thR':>8}{'L0L':>7}{'L0R':>7}{'hipL':>8}{'hipR':>8}"
      f"{'slL':>8}{'slR':>8}{'F0L':>8}{'F0R':>8}")
for i in range(int(14 / sim.dt)):
    info = sim.step()
    if i % int(0.4 / sim.dt) == 0:
        m = sim.measure()
        hipL = sim.d.qpos[sim.m.jnt_qposadr[sim.j_hip["left"]]]
        hipR = sim.d.qpos[sim.m.jnt_qposadr[sim.j_hip["right"]]]
        slL = sim.d.qpos[sim.m.jnt_qposadr[sim.j_slide["left"]]]
        slR = sim.d.qpos[sim.m.jnt_qposadr[sim.j_slide["right"]]]
        print(f"{info['t']:5.1f}{np.degrees(m['yaw']):8.2f}"
              f"{np.degrees(m['pitch']):8.2f}{np.degrees(m['roll']):8.2f}"
              f"{info['z']:8.4f}{np.degrees(m['leg']['left'][1]):8.2f}"
              f"{np.degrees(m['leg']['right'][1]):8.2f}"
              f"{m['leg']['left'][0]*1000:7.1f}{m['leg']['right'][0]*1000:7.1f}"
              f"{np.degrees(hipL):8.2f}{np.degrees(hipR):8.2f}"
              f"{slL*1000:8.2f}{slR*1000:8.2f}"
              f"{sim.d.ctrl[sim.a_leg['left']]:8.1f}{sim.d.ctrl[sim.a_leg['right']]:8.1f}")
    if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
        print("DIVERGED at", round(info["t"], 2))
        break
