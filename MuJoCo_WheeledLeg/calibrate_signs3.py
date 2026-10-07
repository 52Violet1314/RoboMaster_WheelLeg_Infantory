"""Scan sign conventions for theta_mode in {abs, hip} and score on
standing + speed tracking."""
import itertools
import numpy as np

import sim_lqr as S

PAIRS = ["s", "yaw", "th", "pitch"]
INPUTS = ["Tw", "Tb"]
RATE_OF = {"s": "ds", "yaw": "dyaw", "th": "dth", "pitch": "dpitch"}


def evaluate(signs, theta_mode, speed=0.0, seconds=12.0):
    sim = S.WheelLegLQR(signs=signs, theta_mode=theta_mode, target_speed=speed)
    sim.reset()
    info = None
    for _ in range(int(seconds / sim.dt)):
        info = sim.step()
        if not np.isfinite(info["z"]) or abs(info["z"]) > 3:
            return -1e9, "diverged"
    sc = -(20 * abs(info["pitch"]) + 20 * abs(info["roll"])
           + 40 * abs(info["vx"] - speed) + 3 * abs(info["x"]))
    return sc, (f"x={info['x']:+7.2f} vx={info['vx']:+7.3f} pitch={info['pitch']:+.4f}")


for mode in ("abs", "hip"):
    print(f"########## theta_mode = {mode} ##########")
    res = []
    for combo in itertools.product([+1, -1], repeat=6):
        signs = dict(S.DEFAULT_SIGNS)
        for name, v in zip(PAIRS, combo[:4]):
            signs[name] = v
            signs[RATE_OF[name]] = v
        for name, v in zip(INPUTS, combo[4:]):
            signs[name] = v
        s0, m0 = evaluate(signs, mode, speed=0.0)
        res.append((s0, signs, m0))
    res.sort(key=lambda r: -r[0])
    best = res[0][1]
    tag = " ".join(f"{k}={best[k]:+d}" for k in PAIRS + INPUTS)
    print(f"  best standing: score={res[0][0]:9.3f}  {tag}   {res[0][2]}")
    # speed tracking on the top-3 standing conventions
    for sc, sg, msg in res[:3]:
        s1, m1 = evaluate(sg, mode, speed=0.5)
        tag = " ".join(f"{k}={sg[k]:+d}" for k in PAIRS + INPUTS)
        print(f"    speed0.5: score={s1:9.3f}  {tag}   {m1}")
    print()
