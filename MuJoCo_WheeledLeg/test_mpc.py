"""Explicit-policy correctness, host-C parity and MuJoCo integration checks.

Run: python -B -m unittest test_mpc -v
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import mujoco
from scipy.optimize import minimize

from explicit_mpc import ExplicitMPC
from generate_mpc import design
from sim_mpc import WheelLegMPC

ROOT = Path(__file__).resolve().parent


class PolicyTests(unittest.TestCase):
    def test_explicit_laws_match_independent_bounded_qp(self):
        rng = np.random.default_rng(20261103)
        policy = ExplicitMPC()
        worst = 0.0
        for length in (0.09, 0.1339, 0.15, 0.27, 0.35, 0.45):
            policy.select_length(length)
            gain, h, _, _ = design(length)
            for _ in range(20):
                x = rng.normal(size=10) * [1, 1, .2, .5, .1, 1, .1, 1, .1, 1]
                limit = np.array([7., 7., 2., 2.] if rng.random() < .5
                                 else [7., 7., 10., 10.])
                offset = rng.uniform(-1, 1, 4)
                desired = -gain @ x + offset
                result = minimize(lambda u: .5 * (u - desired) @ h @ (u - desired),
                                  np.clip(desired, -limit, limit),
                                  jac=lambda u: h @ (u - desired),
                                  bounds=list(zip(-limit, limit)), method="SLSQP",
                                  options={"ftol": 1e-11, "maxiter": 200})
                self.assertTrue(result.success, result.message)
                actual = policy.evaluate(x, limit, offset)
                worst = max(worst, float(max(abs(actual - result.x))))
                np.testing.assert_allclose(actual, result.x, atol=1e-3, rtol=0)
        print(f"\n120 independent QPs: maximum torque error {worst:.6g} Nm", flush=True)

    def test_runtime_rejects_invalid_state_period_and_length(self):
        with self.assertRaises(ValueError):
            ExplicitMPC(dt=.002)
        policy = ExplicitMPC()
        for length in (.08, .46, np.nan):
            with self.assertRaises(ValueError):
                policy.select_length(length)
        for state in (np.full(10, np.nan), np.zeros(9)):
            with self.assertRaises(ValueError):
                policy.evaluate(state)

    def test_c_runtime_matches_python_over_entire_model_bank(self):
        compiler = shutil.which("gcc")
        if not compiler:
            self.skipTest("Host gcc unavailable")

        class Context(ctypes.Structure):
            _fields_ = [("model", ctypes.c_uint16), ("region", ctypes.c_uint8),
                        ("checks", ctypes.c_uint8)]

        vector = ctypes.c_float * 4
        state = ctypes.c_float * 10
        with tempfile.TemporaryDirectory(prefix="wheelleg_mpc_") as temp:
            library = Path(temp) / ("mpc.dll" if os.name == "nt" else "mpc.so")
            flags = [] if os.name == "nt" else ["-fPIC"]
            compiler_env = os.environ.copy()
            compiler_env["PATH"] = str(Path(compiler).parent) + os.pathsep + compiler_env.get("PATH", "")
            compiled = subprocess.run([compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                                       "-shared", *flags, str(ROOT / "embedded_mpc/explicit_mpc.c"),
                                       "-o", str(library), "-lm"], capture_output=True, text=True,
                                      env=compiler_env)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            lib = ctypes.CDLL(str(library))
            lib.wheelleg_mpc_init.argtypes = [ctypes.POINTER(Context)]
            lib.wheelleg_mpc_select_length.argtypes = [ctypes.POINTER(Context), ctypes.c_float]
            lib.wheelleg_mpc_evaluate.argtypes = [ctypes.POINTER(Context), state, vector,
                                                 vector, vector]
            context, policy = Context(), ExplicitMPC()
            lib.wheelleg_mpc_init(ctypes.byref(context))
            rng = np.random.default_rng(731)
            worst = 0.0
            max_checks = 0
            for length in policy.lengths:
                self.assertEqual(lib.wheelleg_mpc_select_length(ctypes.byref(context), length), 1)
                policy.select_length(length)
                self.assertEqual(context.model, policy.index)
                for _ in range(40):
                    x = (rng.normal(size=10) * [.3, 1, .5, 1, .2, 2, .2, 2, .2, 2]).astype("f4")
                    limit = np.array([7, 7, 2 if rng.random() < .5 else 10, 10], dtype="f4")
                    offset = rng.uniform(-2, 2, 4).astype("f4")
                    actual = vector()
                    ok = lib.wheelleg_mpc_evaluate(ctypes.byref(context), state(*x),
                                                   vector(*limit), vector(*offset), actual)
                    self.assertEqual(ok, 1)
                    expected = policy.evaluate(x, limit, offset)
                    worst = max(worst, float(max(abs(actual - expected))))
                    max_checks = max(max_checks, context.checks)
                    np.testing.assert_allclose(actual, expected, atol=1e-3, rtol=0)
            actual = vector(1, 1, 1, 1)
            self.assertEqual(lib.wheelleg_mpc_evaluate(ctypes.byref(context),
                             state(*([float("nan")] * 10)), vector(7, 7, 10, 10),
                             vector(), actual), 0)
            self.assertEqual(list(actual), [0., 0., 0., 0.])
            # Unload before TemporaryDirectory cleanup on Windows.
            if os.name == "nt":
                import _ctypes
                _ctypes.FreeLibrary(lib._handle)
            print(f"\n1560 C/Python cases: max error {worst:.6g} Nm; "
                  f"max region checks {max_checks}", flush=True)


class SimulationTests(unittest.TestCase):
    def advance(self, sim, seconds, callback=None):
        peak = 0.0
        for i in range(round(seconds / sim.dt)):
            if callback:
                callback(i)
            info = sim.step()
            self.assertTrue(np.all(np.isfinite(sim.d.qpos)))
            self.assertLessEqual(max(abs(info["hubTauL"]), abs(info["hubTauR"])), 7.001)
            self.assertLessEqual(max(abs(info["jointTauL"]), abs(info["jointTauR"])),
                                 2.001 if sim.rear_pose_active else 10.001)
            peak = max(peak, abs(info["pitch"]))
        self.assertLess(peak, np.radians(15))
        self.assertGreater(sim.d.qpos[2], sim.leg_length + .025)
        return peak

    def test_balance_push_and_leg_presets_without_online_design(self):
        with patch("lqr_k.compute_K", side_effect=AssertionError("Online Riccati solve")):
            sim = WheelLegMPC()
            self.advance(sim, 3)
            def push(i):
                sim.d.xfrc_applied[sim.b_base, 0] = 40. if i < 200 else 0.
            peak = self.advance(sim, 4, push)
            self.assertLess(abs(sim.measure()["pitch"]), np.radians(3))
            for length in (.15, .27, .35, .1339):
                sim.leg_length = length
                self.advance(sim, 2)
                self.assertLess(abs(sim.measure()["leg"]["left"][0] - length), .008)
            sim.rear_pose_active = True
            self.advance(sim, 1)
            print(f"\nMPC push + leg presets: peak pitch {np.degrees(peak):.3f} deg", flush=True)

    def test_drive_brake_and_turn(self):
        sim = WheelLegMPC()
        sim.speed_ramp = 5.
        self.advance(sim, 1)
        start = float(sim.d.qpos[0])
        sim.speed_cmd = .35
        self.advance(sim, 2.5)
        release = float(sim.d.qpos[0])
        self.assertGreater(release - start, .3)
        sim.speed_cmd = 0.
        self.advance(sim, 3)
        self.assertLess(abs(sim.d.qvel[0]), .12)
        sim.turn_rate = .5
        self.advance(sim, 4)
        rate = sim.measure()["omega"][2]
        self.assertGreater(rate, .35)
        sim.turn_rate = 0.
        self.advance(sim, 3)
        error = np.arctan2(np.sin(sim.measure()["yaw"] - sim.target_yaw),
                           np.cos(sim.measure()["yaw"] - sim.target_yaw))
        self.assertLess(abs(error), np.radians(1))
        print(f"\nMPC drive/brake/turn: yaw rate {np.degrees(rate):.2f} deg/s; "
              f"final heading error {np.degrees(error):.3f} deg", flush=True)

    def test_sustained_stand_and_jump_handoff(self):
        sim = WheelLegMPC()
        peak = self.advance(sim, 30)
        self.assertLess(abs(sim.d.qvel[0]), .03)
        self.assertTrue(sim.request_jump(sim.leg_length))
        phases = set()
        for _ in range(10000):
            info = sim.step()
            phases.add(sim.jump_phase)
            if "flight" in phases and sim.jump_phase == "idle":
                break
        self.assertIn("flight", phases)
        self.assertEqual(sim.jump_phase, "idle")
        self.advance(sim, 2)
        self.assertLess(abs(sim.measure()["pitch"]), np.radians(3))
        self.assertGreater(info["jumpHeight"], .1)
        print(f"\n30 s MPC stand: peak pitch {np.degrees(peak):.3f} deg; "
              f"jump height {info['jumpHeight']:.3f} m, balance resumed", flush=True)

    def test_self_right_handoff_from_120_degrees(self):
        sim = WheelLegMPC(leg_length=.15)
        sim.d.qpos[:3] = [6., 1.8, .4]
        angle = np.radians(120.)
        sim.d.qpos[3:7] = [np.cos(angle / 2), 0., np.sin(angle / 2), 0.]
        for joint in sim.j_hip.values():
            sim.d.qpos[sim.m.jnt_qposadr[joint]] = -1.2
        mujoco.mj_forward(sim.m, sim.d)
        sim.estop = True
        for _ in range(1000):
            sim.step()
        fallen = sim.d.qpos.copy()
        sim.request_stand()
        np.testing.assert_array_equal(sim.d.qpos, fallen)
        phases = set()
        for _ in range(46000):
            sim.step()
            phases.add(sim.rescue_phase)
            self.assertTrue(np.all(sim.d.xfrc_applied == 0.))
            if sim.rescue_phase in ("idle", "fault"):
                break
        self.assertTrue({"lift", "sweep", "stand", "balance", "idle"} <= phases)
        for _ in range(5000):
            sim.step()
        meas = sim.measure()
        self.assertLess(abs(meas["pitch_full"]), .25)
        self.assertLess(abs(meas["roll"]), .25)
        for side in sim.j_hip:
            self.assertLess(abs(meas["leg"][side][1]), .25)
            self.assertLess(abs(meas["leg"][side][0] - .15), .015)
        print("\n120 deg self-right: MPC handoff and leg realignment passed", flush=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
