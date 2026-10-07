"""Smoke test: can the passive viewer be created and stepped on this machine?"""
import time
import mujoco
import mujoco.viewer

import sim_lqr as S

sim = S.WheelLegLQR()
sim.reset()

print("launching passive viewer...")
try:
    with mujoco.viewer.launch_passive(sim.m, sim.d) as v:
        print("  viewer created:", v.is_running())
        t0 = time.perf_counter()
        n = 0
        while v.is_running() and time.perf_counter() - t0 < 3.0:
            for _ in range(20):
                sim.step()
                n += 1
            v.sync()
        print(f"  stepped {n} times in {time.perf_counter()-t0:.2f} s")
        print(f"  final z={sim.d.qpos[2]:.4f}  pitch={sim.measure()['pitch']:+.5f}")
    print("viewer closed cleanly -> GUI AVAILABLE")
except Exception as e:
    print("viewer failed:", type(e).__name__, e)
