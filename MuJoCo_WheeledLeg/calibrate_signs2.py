"""Targeted check: fix pitch=+1, Tw=+1 (physically determined) and test the
remaining position/velocity/leg-angle/hip-torque signs."""
import numpy as np
import sim_lqr as S


def trial(signs, seconds=8.0):
    sim = S.WheelLegLQR(signs=signs)
    sim.reset()
    info = None
    for _ in range(int(seconds / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 5:
            return -1e9, "diverged"
    sc = -(20 * abs(info["z"] - 0.190) + 10 * abs(info["pitch"])
           + 10 * abs(info["roll"]) + 10 * abs(info["vx"])
           + 3 * abs(info["x"]))
    return sc, (f"z={info['z']:+.3f} pitch={info['pitch']:+.4f} "
                f"roll={info['roll']:+.4f} vx={info['vx']:+.3f} "
                f"x={info['x']:+.3f}")


base = dict(s=+1, ds=+1, yaw=+1, dyaw=+1, th=-1, dth=-1,
            pitch=+1, dpitch=+1, Tw=+1, Tb=+1)

cands = []
for sv in (+1, -1):
    for yv in (+1, -1):
        for tv in (+1, -1):
            for bv in (+1, -1):
                sg = dict(base)
                sg["s"] = sg["ds"] = sv
                sg["yaw"] = sg["dyaw"] = yv
                sg["th"] = sg["dth"] = tv
                sg["Tb"] = bv
                cands.append(sg)

res = []
for sg in cands:
    sc, msg = trial(sg)
    res.append((sc, sg, msg))
res.sort(key=lambda r: -r[0])
for sc, sg, msg in res[:8]:
    tag = " ".join(f"{k}={sg[k]:+d}" for k in ("s", "yaw", "th", "Tb"))
    print(f"  score={sc:10.3f}  pitch=+1 Tw=+1  {tag}   {msg}")
