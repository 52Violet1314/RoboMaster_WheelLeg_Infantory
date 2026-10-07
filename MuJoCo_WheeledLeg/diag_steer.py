"""Log the divergence of a steering run to see what actually fails."""
import numpy as np
import sim_lqr as S

sim = S.WheelLegLQR(speed_ramp=2.0)
sim.yaw_pid = S.LegPID(2.0, 0.0, 0.05, out_limit=1.5, int_limit=0.5)
sim.turn_limit = 1.5
sim.reset()
sim.turn_rate = 0.4

print(f"{'t':>6}{'yaw':>9}{'wy':>9}{'pitch':>9}{'roll':>9}{'wx':>9}{'L0L':>8}{'L0R':>8}"
      f"{'TwL':>8}{'TwR':>8}{'turn':>8}{'z':>8}")
for i in range(int(14 / sim.dt)):
    info = sim.step()
    if i % int(0.5 / sim.dt) == 0:
        m = sim.measure()
        print(f"{info['t']:6.1f}{np.degrees(m['yaw']):9.2f}{m['omega'][2]:9.4f}"
              f"{np.degrees(m['pitch']):9.2f}{np.degrees(m['roll']):9.2f}"
              f"{m['omega'][0]:9.3f}{m['leg']['left'][0]*1000:8.1f}"
              f"{m['leg']['right'][0]*1000:8.1f}"
              f"{info['TwL']:8.3f}{info['TwR']:8.3f}{info['turn']:8.3f}{info['z']:8.3f}")
    if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
        print("DIVERGED at", info["t"])
        break
