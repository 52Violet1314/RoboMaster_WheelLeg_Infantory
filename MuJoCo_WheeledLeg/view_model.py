"""
Open either model in the MuJoCo GUI.

    python view_model.py                 # control model  (wheelbipe_lqr.xml)
    python view_model.py --full          # full-fidelity (wheelbipeV14_2.xml, 37 links)
    python view_model.py --urdf          # the exported URDF

`--full` / `--urdf` open at their authored rest pose for inspection.  Pass
`--simulate` if you intentionally want the uncontrolled mechanism to fall under
gravity.  The default control model runs VMC + LQR (same as view_gui.py).
"""
from __future__ import annotations

import argparse
import os
import time

import mujoco
import mujoco.viewer
import sim_lqr as S

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = {
    "control": os.path.join(HERE, "wheelbipe_lqr.xml"),
    "full": os.path.join(HERE, "wheelbipeV14_2.xml"),
    "urdf": os.path.join(HERE, "wheelbipeV14_2.urdf"),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full", action="store_true",
                    help="show the full-fidelity 37-link model")
    ap.add_argument("--urdf", action="store_true", help="show the exported URDF")
    ap.add_argument("--simulate", action="store_true",
                    help="run gravity for full/URDF models (off by default)")
    ap.add_argument("--settle", type=float, default=0.0,
                    help="physics seconds to settle before display with --simulate")
    ap.add_argument("--realtime", type=float, default=1.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="close after this many wall-clock seconds (0 = until Esc)")
    args = ap.parse_args()
    if args.realtime < 0 or args.settle < 0 or args.duration < 0:
        ap.error("--realtime, --settle and --duration must be >= 0")

    which = "urdf" if args.urdf else ("full" if args.full else "control")
    path = MODELS[which]

    if which == "control":
        sim = S.WheelLegLQR()
        sim.reset()
        m, d = sim.m, sim.d
        step = sim.step
        print("  control model: VMC + LQR running (see view_gui.py for key control)")
    else:
        m = mujoco.MjModel.from_xml_path(path)
        d = mujoco.MjData(m)
        mujoco.mj_resetData(m, d)
        mujoco.mj_forward(m, d)

        def step():
            mujoco.mj_step(m, d)

        print(f"  {which} model: {m.nbody} bodies, {m.njnt} joints, "
              f"{m.neq} equality constraints, {m.nu} actuators")
        if m.neq:
            print("  (the 6 equality constraints are the closed-chain loop joints)")
        if not args.simulate:
            print("  static rest-pose inspection; add --simulate to run gravity")

    should_step = which == "control" or args.simulate
    if should_step and args.settle > 0:
        for _ in range(int(args.settle / m.opt.timestep)):
            step()

    with mujoco.viewer.launch_passive(m, d) as v:
        with v.lock():
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
            v.cam.lookat[:] = [0.0, 0.0, 0.20]
            v.cam.distance = 1.0
            v.cam.azimuth = 135.0
            v.cam.elevation = -12.0
        v.sync()

        last_wall = time.perf_counter()
        accumulator = 0.0
        deadline = (last_wall + args.duration if args.duration > 0 else None)
        while v.is_running():
            if deadline is not None and time.perf_counter() >= deadline:
                break
            frame_started = time.perf_counter()
            if should_step and args.realtime > 0:
                now = time.perf_counter()
                elapsed = min(max(now - last_wall, 0.0), 0.05)
                last_wall = now
                accumulator = min(accumulator + elapsed * args.realtime,
                                  0.05 * args.realtime)
                count = int(accumulator / m.opt.timestep)
                accumulator -= count * m.opt.timestep
                for _ in range(count):
                    step()
            elif should_step:
                for _ in range(100):
                    step()
            v.sync()
            remaining = 1.0 / 60.0 - (time.perf_counter() - frame_started)
            if remaining > 0:
                time.sleep(remaining)

    print("  window closed")


if __name__ == "__main__":
    main()
