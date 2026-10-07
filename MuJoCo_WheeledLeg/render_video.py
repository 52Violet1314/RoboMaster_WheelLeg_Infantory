"""
Record an MP4 of the VMC + LQR simulation.

The clip shows, in order:
    0-3 s    stand / settle
    3-6 s    60 N push disturbance and recovery
    6-10 s   leg-length command step 134 -> 180 mm
    10-14 s  leg back down to 134 mm
    14-24 s  speed command 0.5 m/s
    24-34 s  heading setpoint turn (0.2 rad/s)
    34-38 s  stop

Run:
    python render_video.py                 # demo.mp4, 1920x1080 @ 50 fps
    python render_video.py --out run.mp4 --width 1280 --height 720
"""
from __future__ import annotations

import argparse
import os

import imageio.v2 as imageio
import mujoco
import numpy as np

import sim_lqr as S

OUTDIR = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(OUTDIR, "demo.mp4"))
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--azimuth", type=float, default=135.0)
    ap.add_argument("--elevation", type=float, default=-14.0)
    ap.add_argument("--distance", type=float, default=1.05)
    ap.add_argument("--track", action="store_true",
                    help="keep the camera on the robot")
    args = ap.parse_args()

    sim = S.WheelLegLQR()
    sim.reset()

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, 0.19]
    cam.distance = args.distance
    cam.azimuth = args.azimuth
    cam.elevation = args.elevation

    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

    renderer = mujoco.Renderer(sim.m, args.height, args.width)

    sim_dt = sim.dt
    stride = max(1, int(round(1.0 / (args.fps * sim_dt))))
    total = 38.0
    frames = []

    for i in range(int(total / sim_dt)):
        t = i * sim_dt

        # scheduled events
        if 3.0 <= t < 3.4:
            sim.d.xfrc_applied[sim.b_base, 0] = 60.0
        else:
            sim.d.xfrc_applied[sim.b_base, 0] = 0.0
        if i == int(6.0 / sim_dt):
            sim.leg_length = 0.180
        if i == int(10.0 / sim_dt):
            sim.leg_length = 0.134
        if i == int(14.0 / sim_dt):
            sim.speed_cmd = 0.5
        if i == int(24.0 / sim_dt):
            sim.turn_rate = 0.2
        if i == int(34.0 / sim.dt):
            sim.speed_cmd = 0.0
            sim.turn_rate = 0.0

        info = sim.step()

        if args.track:
            cam.lookat[:] = [sim.d.qpos[0], sim.d.qpos[1], 0.19]
        if i % stride == 0:
            renderer.update_scene(sim.d, cam, opt)
            frames.append(renderer.render())

    imageio.mimsave(args.out, frames, fps=args.fps, quality=8,
                    macro_block_size=None)
    print(f"wrote {args.out}  ({len(frames)} frames, "
          f"{len(frames)/args.fps:.1f} s @ {args.fps} fps, "
          f"{args.width}x{args.height})")
    print(f"final: z={sim.d.qpos[2]:.4f}  pitch={sim.measure()['pitch']:+.5f}"
          f"  x={sim.d.qpos[0]:+.3f}")


if __name__ == "__main__":
    main()
