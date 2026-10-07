"""Render a few frames offscreen so the scene can be checked without a window."""
from pathlib import Path
import numpy as np
import mujoco

import sim_lqr as S

OUT = Path(__file__).resolve().with_name("frames")
OUT.mkdir(parents=True, exist_ok=True)

sim = S.WheelLegLQR()
sim.reset()

W, H = 1280, 720
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [0.0, 0.0, 0.19]
cam.distance = 1.05
cam.azimuth = 135.0
cam.elevation = -14.0

opt = mujoco.MjvOption()
opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False

shots = {0.0: "01_start", 1.0: "02_settle", 3.0: "03_stand"}
renderer = mujoco.Renderer(sim.m, H, W)
taken = []
for i in range(int(6.0 / sim.dt)):
    info = sim.step()
    t = info["t"]
    for k, name in list(shots.items()):
        if abs(t - k) < sim.dt / 2:
            cam.lookat[0] = sim.d.qpos[0]
            cam.lookat[1] = sim.d.qpos[1]
            renderer.update_scene(sim.d, cam, opt)
            img = renderer.render()
            p = OUT / f"{name}.png"
            import imageio.v2 as imageio
            imageio.imwrite(p, img)
            taken.append((p, round(t, 2)))
            del shots[k]
print("rendered:")
for p, t in taken:
    print(f"  t={t:4.1f}s  {p}")
print("final state: z=%.4f pitch=%+.4f vx=%+.5f" % (info["z"], info["pitch"], info["vx"]))
