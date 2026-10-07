"""Isolate what degraded the standing equilibrium."""
import numpy as np
import sim_lqr as S

def stand(seconds=40.0, anti=True, hip_range=None, model=None):
    sim = S.WheelLegLQR(model_path=model) if model else S.WheelLegLQR()
    sim.anti_crash_enable = anti
    if hip_range is not None:
        for side in ("left", "right"):
            jid = sim.j_hip[side]
            sim.m.jnt_range[jid] = [-hip_range, hip_range]
            sim.m.jnt_limited[jid] = 1
    sim.reset()
    for i in range(int(seconds / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 1.0:
            return f"DIVERGED t={info['t']:.1f}"
    m = sim.measure()
    hipL = np.degrees(sim.d.qpos[sim.m.jnt_qposadr[sim.j_hip['left']]])
    hipR = np.degrees(sim.d.qpos[sim.m.jnt_qposadr[sim.j_hip['right']]])
    return (f"z={info['z']:.4f} pitch={np.degrees(m['pitch']):+6.2f} "
            f"roll={np.degrees(m['roll']):+6.2f} hipL={hipL:+7.2f} hipR={hipR:+7.2f} "
            f"L0={m['leg']['left'][0]*1000:6.1f}")

print("XML hip range = +/-35 deg (current file)")
print("  anti_crash=True :", stand(anti=True))
print("  anti_crash=False:", stand(anti=False))
print()
print("hip range forced wide (+/-170 deg) at runtime")
print("  anti_crash=True :", stand(anti=True, hip_range=2.97))
print("  anti_crash=False:", stand(anti=False, hip_range=2.97))
