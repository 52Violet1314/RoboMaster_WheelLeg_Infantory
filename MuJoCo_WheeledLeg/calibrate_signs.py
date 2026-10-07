"""Full 2^6 sign-convention scan on the improved model, 20 s horizon."""
import itertools
import numpy as np

import sim_lqr as S

PAIRS = ["s", "yaw", "th", "pitch"]
INPUTS = ["Tw", "Tb"]
RATE_OF = {"s": "ds", "yaw": "dyaw", "th": "dth", "pitch": "dpitch"}


def score(signs, seconds=20.0):
    sim = S.WheelLegLQR(signs=signs)
    sim.reset()
    info = None
    for _ in range(int(seconds / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 3:
            return -1e9, "diverged"
    sc = -(30 * abs(info["z"] - 0.184)
           + 20 * abs(info["pitch"]) + 20 * abs(info["roll"])
           + 30 * abs(info["vx"]) + 3 * abs(info["x"]))
    return sc, (f"x={info['x']:+8.3f} vx={info['vx']:+8.4f} z={info['z']:.4f} "
                f"pitch={info['pitch']:+.5f} roll={info['roll']:+.5f}")


results = []
for combo in itertools.product([+1, -1], repeat=6):
    signs = dict(S.DEFAULT_SIGNS)
    for name, v in zip(PAIRS, combo[:4]):
        signs[name] = v
        signs[RATE_OF[name]] = v
    for name, v in zip(INPUTS, combo[4:]):
        signs[name] = v
    sc, msg = score(signs)
    results.append((sc, signs, msg))

results.sort(key=lambda r: -r[0])
print("ranked sign combinations (20 s horizon):")
for sc, sg, msg in results[:8]:
    tag = " ".join(f"{k}={sg[k]:+d}" for k in PAIRS + INPUTS)
    print(f"  score={sc:10.3f}  {tag}   {msg}")
print("  ...")
for sc, sg, msg in results[-3:]:
    tag = " ".join(f"{k}={sg[k]:+d}" for k in PAIRS + INPUTS)
    print(f"  score={sc:10.3f}  {tag}   {msg}")
