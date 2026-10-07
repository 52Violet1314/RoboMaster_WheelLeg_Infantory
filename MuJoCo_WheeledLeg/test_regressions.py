"""Focused regressions for model conversion and keyboard control."""
from __future__ import annotations

import os
import unittest

import mujoco
import numpy as np

import sim_lqr as S
import view_gui
import linkage_kinematics as LK


HERE = os.path.dirname(os.path.abspath(__file__))


class ModelRegressionTests(unittest.TestCase):
    def test_chassis_has_environment_only_collision_box(self):
        sim = S.WheelLegLQR()
        geom = sim.m.geom("base_collision").id
        self.assertEqual(sim.m.geom_type[geom], mujoco.mjtGeom.mjGEOM_BOX)
        self.assertTrue(np.allclose(sim.m.geom_size[geom], [0.25, 0.27, 0.085]))
        self.assertEqual(sim.m.geom_contype[geom], 2)
        self.assertEqual(sim.m.geom_conaffinity[geom], 0)
        self.assertLessEqual(sim.m.geom_friction[geom, 0], 0.001)
        for name in ("base_roller_fl", "base_roller_fr",
                     "base_roller_rl", "base_roller_rr"):
            roller = sim.m.geom(name).id
            self.assertEqual(sim.m.geom_contype[roller], 2)
            self.assertEqual(sim.m.geom_conaffinity[roller], 0)
            self.assertLessEqual(sim.m.geom_friction[roller, 0], 0.001)
        self.assertEqual(sim.m.geom_conaffinity[sim.m.geom("floor").id], 3)
        self.assertEqual(sim.m.geom_conaffinity[sim.m.geom("step").id], 3)

    def test_forward_step_dimensions_and_position(self):
        sim = S.WheelLegLQR()
        geom = sim.m.geom("step").id
        self.assertEqual(sim.m.geom_type[geom], mujoco.mjtGeom.mjGEOM_BOX)
        self.assertTrue(np.allclose(sim.m.geom_size[geom], [1.0, 0.5, 0.1]))
        self.assertTrue(np.allclose(sim.m.geom_pos[geom], [1.8, 0.0, 0.1]))
        self.assertNotEqual(sim.m.geom_contype[geom], 0)

    def test_full_model_closed_chains_start_closed(self):
        model = mujoco.MjModel.from_xml_path(
            os.path.join(HERE, "wheelbipeV14_2.xml"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        errors = []
        for index in range(model.neq):
            site1 = int(model.eq_obj1id[index])
            site2 = int(model.eq_obj2id[index])
            errors.append(np.linalg.norm(data.site_xpos[site1] - data.site_xpos[site2]))
        self.assertLess(max(errors), 1e-4)  # 0.1 mm

    def test_control_model_contains_aligned_source_visuals(self):
        sim = S.WheelLegLQR()
        sim.reset()
        names = {sim.m.mesh(index).name for index in range(sim.m.nmesh)}
        for required in (
                "left_front1_link_mesh", "right_front4_link_mesh",
                "left_front1_guide_link_mesh", "right_front1_guide_link_mesh"):
            self.assertIn(required, names)

        low = np.full(3, np.inf)
        high = np.full(3, -np.inf)
        for geom_id in range(sim.m.ngeom):
            if sim.m.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh_id = int(sim.m.geom_dataid[geom_id])
            start = int(sim.m.mesh_vertadr[mesh_id])
            count = int(sim.m.mesh_vertnum[mesh_id])
            points = np.asarray(sim.m.mesh_vert[start:start + count])
            rotation = np.asarray(sim.d.geom_xmat[geom_id]).reshape(3, 3)
            points = (rotation @ points.T).T + sim.d.geom_xpos[geom_id]
            low = np.minimum(low, points.min(axis=0))
            high = np.maximum(high, points.max(axis=0))
        self.assertTrue(np.all(high - low < 1.0), (low, high))
        self.assertTrue(np.all(low > [-0.5, -0.5, -0.1]), low)
        self.assertTrue(np.all(high < [0.5, 0.5, 0.8]), high)

    def test_long_leg_reset_starts_near_command(self):
        sim = S.WheelLegLQR(leg_length=0.180)
        sim.reset()
        measured = sim.measure()["leg"]["left"][0]
        self.assertLess(abs(measured - 0.180), 0.004)
        self.assertAlmostEqual(sim.d.qpos[2], 0.240, places=6)

    def test_model_supports_450_mm_virtual_leg_command(self):
        sim = S.WheelLegLQR(leg_length=0.450)
        measured = sim.measure()["leg"]["left"][0]
        self.assertGreater(measured, 0.440)
        for side in ("left", "right"):
            joint = sim.j_slide[side]
            actuator = sim.a_leg[side]
            self.assertGreaterEqual(sim.m.jnt_range[joint, 1], 0.326)
            self.assertEqual(sim.m.jnt_stiffness[joint], 0.0)
            self.assertEqual(sim.m.actuator_ctrlrange[actuator, 1], 300.0)

    def test_visual_linkage_ik_closes_across_travel(self):
        previous = np.zeros(8)
        for slide in (-0.05, 0.0, 0.10, 0.225):
            angles, error, success = LK.solve(LK.target_from_slide(slide), previous)
            self.assertTrue(success)
            self.assertLess(error, 1e-5)
            previous = angles

    def test_visual_link_geometries_follow_slide(self):
        raw = mujoco.MjModel.from_xml_path(os.path.join(HERE, "wheelbipe_lqr.xml"))
        sim = S.WheelLegLQR()
        for side in ("left", "right"):
            for geom in sim.linkage_visual.geom_ids[side].values():
                self.assertTrue(np.allclose(sim.m.geom_pos[geom], raw.geom_pos[geom],
                                            atol=1e-10))
                # q and -q encode the same rotation; compare matrices.
                self.assertTrue(np.allclose(
                    LK._quat_matrix(sim.m.geom_quat[geom]),
                    LK._quat_matrix(raw.geom_quat[geom]), atol=1e-10))
        geom = sim.linkage_visual.geom_ids["left"]["front2"]
        rest = sim.m.geom_pos[geom].copy()
        joint = sim.j_slide["left"]
        sim.d.qpos[sim.m.jnt_qposadr[joint]] = 0.20
        sim.linkage_visual.update(sim.d, sim.j_slide)
        self.assertGreater(np.linalg.norm(sim.m.geom_pos[geom] - rest), 0.02)

    def test_visual_mesh_transforms_match_full_model_fk(self):
        """Catch lost STL refpos/refquat offsets, not just closed joint points."""
        angles, error, success = LK.solve(LK.target_from_slide(0.10))
        self.assertTrue(success)
        self.assertLess(error, 1e-8)
        full = mujoco.MjModel.from_xml_path(
            os.path.join(HERE, "wheelbipeV14_2.xml"))
        full_data = mujoco.MjData(full)
        joint_links = ("front1", "front2", "front3", "front4",
                       "rear1", "rear2", "spring1", "spring2")
        joint_values = (*angles[:7], -angles[7])
        for side in ("left", "right"):
            for link, value in zip(joint_links, joint_values):
                joint = full.joint(f"{side}_{link}_joint").id
                full_data.qpos[full.jnt_qposadr[joint]] = value
        mujoco.mj_forward(full, full_data)

        sim = S.WheelLegLQR()
        for side in ("left", "right"):
            sim.d.qpos[sim.m.jnt_qposadr[sim.j_hip[side]]] = 0.0
            sim.d.qpos[sim.m.jnt_qposadr[sim.j_slide[side]]] = 0.10
        sim.linkage_visual.update(sim.d, sim.j_slide)
        mujoco.mj_forward(sim.m, sim.d)

        def mesh_geom(model, mesh_name):
            for geom in range(model.ngeom):
                mesh = int(model.geom_dataid[geom])
                if mesh >= 0 and model.mesh(mesh).name == mesh_name:
                    return geom
            self.fail(f"missing mesh geom {mesh_name}")

        full_base = full_data.xpos[full.body("base_link").id]
        sim_base = sim.d.xpos[sim.b_base]
        visual_links = (*joint_links[:6], "rear2_guide", *joint_links[6:])
        for side in ("left", "right"):
            for link in visual_links:
                mesh_name = f"{side}_{link}_link_mesh"
                full_geom = mesh_geom(full, mesh_name)
                sim_geom = mesh_geom(sim.m, mesh_name)
                self.assertTrue(np.allclose(
                    full_data.geom_xpos[full_geom] - full_base,
                    sim.d.geom_xpos[sim_geom] - sim_base, atol=1e-7), mesh_name)
                self.assertTrue(np.allclose(
                    full_data.geom_xmat[full_geom], sim.d.geom_xmat[sim_geom],
                    atol=1e-10), mesh_name)


class ControllerRegressionTests(unittest.TestCase):
    def test_space_command_can_decelerate_to_zero(self):
        sim = S.WheelLegLQR()
        sim.speed_cmd = 0.5
        for _ in range(600):
            sim.step()
        self.assertAlmostEqual(sim.target_speed, 0.5, places=6)
        sim.speed_cmd = 0.0
        for _ in range(600):
            sim.step()
        self.assertAlmostEqual(sim.target_speed, 0.0, places=6)

    def test_velocity_command_holds_release_position_instead_of_returning_origin(self):
        sim = S.WheelLegLQR()
        sim.speed_ramp = 0.0
        step = sim.m.geom("step").id
        sim.m.geom_contype[step] = 0
        sim.m.geom_conaffinity[step] = 0
        sim.speed_cmd = 0.35
        for _ in range(2500):
            sim.step()
        x_release = float(sim.d.qpos[0])
        self.assertGreater(x_release, 0.20)
        sim.speed_cmd = 0.0
        for _ in range(3000):
            sim.step()
        self.assertGreater(float(sim.d.qpos[0]), x_release - 0.08)
        self.assertLess(abs(float(sim.d.qvel[0])), 0.12)

    def test_long_legs_have_trimmed_idle_balance_and_integrated_position(self):
        for target in (0.250, 0.350):
            sim = S.WheelLegLQR()
            sim.speed_ramp = 5.0
            step = sim.m.geom("step").id
            sim.m.geom_contype[step] = 0
            sim.m.geom_conaffinity[step] = 0
            for _ in range(750):
                sim.step()
            for settle_step in range(8000):
                view_gui.apply_leg_command(sim, target)
                info = sim.step()
                if (settle_step >= 1500
                        and abs(float(sim.d.qvel[0])) < 0.03
                        and abs(float(sim.d.qvel[2])) < 0.03
                        and abs(info["L0"] - target) < 0.003):
                    break
            self.assertLess(abs(float(sim.d.qvel[0])), 0.05)
            for i in range(2500):
                sim.speed_cmd = min(0.4, (i + 1) * sim.dt)
                sim.step()
            x_release = float(sim.d.qpos[0])
            sim.speed_cmd = 0.0
            for _ in range(4000):
                sim.step()
            self.assertGreater(float(sim.d.qpos[0]), x_release - 0.08)
            self.assertLess(abs(float(sim.d.qvel[0])), 0.10)

    def test_yaw_error_wraps_across_pi(self):
        error = S.wrap_angle((-np.pi + 0.01) - (np.pi - 0.01))
        self.assertAlmostEqual(error, 0.02, places=9)

    def test_step_reports_vmc_and_motor_effort(self):
        info = S.WheelLegLQR().step()
        expected = {
            "vmcFL", "vmcFR", "vmcTbL", "vmcTbR",
            "legActFL", "legActFR", "jointTauL", "jointTauR",
            "hubTauL", "hubTauR", "stopBrake", "gasFL", "gasFR",
            "jumpHeight", "rearTpL", "rearTpR",
        }
        self.assertTrue(expected.issubset(info))
        self.assertTrue(all(np.isfinite(info[name]) for name in expected))
        self.assertIn("jumpPhase", info)
        self.assertIn("rearPose", info)

    def test_rear_mode_leaves_leg_target_unchanged_and_drives_wheels_at_2_nm(self):
        sim = S.WheelLegLQR(leg_length=0.350)
        original_leg = sim.leg_length
        self.assertTrue(sim.set_rear_pose(True))
        self.assertTrue(sim.rear_pose_active)
        self.assertEqual(sim.rear_pose_phase, "wheel_drive")
        self.assertAlmostEqual(sim.leg_length, original_leg, places=9)
        # A NaN gain matrix makes any accidental K @ x use immediately visible.
        sim.K[:] = np.nan
        for _ in range(10):
            info = sim.step()
        self.assertEqual((info["TwL"], info["TwR"]),
                         (S.REAR_POSE_WHEEL_TORQUE, S.REAR_POSE_WHEEL_TORQUE))
        self.assertEqual((info["rearTpL"], info["rearTpR"]), (0.0, 0.0))
        self.assertTrue(np.allclose(sim.d.ctrl[[sim.a_wheel["left"],
                                                 sim.a_wheel["right"]]],
                                    S.REAR_POSE_WHEEL_TORQUE))
        self.assertTrue(np.allclose(sim.d.ctrl[[sim.a_hip["left"],
                                                 sim.a_hip["right"]]], 0.0))
        self.assertTrue(sim.set_rear_pose(False))
        self.assertFalse(sim.rear_pose_active)
        self.assertAlmostEqual(sim.leg_length, original_leg)

    def test_vmc_and_motor_effort_stay_within_hardware_limits(self):
        sim = S.WheelLegLQR()
        sim.speed_cmd = 1.0
        sim.leg_length = S.LEG_MAX
        peak = dict(Fn=0.0, Tp=0.0, wheel=0.0, leg=0.0)
        for _ in range(2000):
            info = sim.step()
            peak["Fn"] = max(peak["Fn"], abs(info["vmcFL"]), abs(info["vmcFR"]))
            peak["Tp"] = max(peak["Tp"], abs(info["vmcTbL"]), abs(info["vmcTbR"]))
            peak["wheel"] = max(peak["wheel"], abs(info["hubTauL"]), abs(info["hubTauR"]))
            peak["leg"] = max(peak["leg"], abs(info["legActFL"]), abs(info["legActFR"]))
        self.assertLessEqual(peak["Fn"], S.VMC_FN_LIMIT + 1e-9)
        self.assertLessEqual(peak["Tp"], S.VMC_TP_LIMIT + 1e-9)
        self.assertLessEqual(peak["wheel"], S.WHEEL_TORQUE_LIMIT + 1e-9)
        self.assertLessEqual(peak["leg"], S.VMC_FN_LIMIT + 1e-9)

    def test_jump_respects_200_n_per_leg_thrust_limit(self):
        sim = S.WheelLegLQR()
        step = sim.m.geom("step").id
        sim.m.geom_contype[step] = 0
        sim.m.geom_conaffinity[step] = 0
        for _ in range(1000):
            sim.step()
        start_time = float(sim.d.time)
        self.assertTrue(sim.request_jump(sim.leg_length))
        saw_flight = False
        peak_fn = 0.0
        peak_leg_act = 0.0
        for _ in range(10000):
            info = sim.step()
            saw_flight |= sim.jump_phase == "flight"
            peak_fn = max(peak_fn, abs(info["vmcFL"]), abs(info["vmcFR"]))
            peak_leg_act = max(peak_leg_act,
                               abs(info["legActFL"]), abs(info["legActFR"]))
            self.assertGreaterEqual(float(sim.d.time), start_time)
            start_time = float(sim.d.time)
            if saw_flight and sim.jump_phase == "idle":
                break
        self.assertTrue(saw_flight)
        self.assertEqual(sim.jump_phase, "idle")
        self.assertGreaterEqual(info["jumpHeight"], 0.12)
        self.assertLessEqual(info["jumpHeight"], 0.15)
        self.assertGreaterEqual(peak_fn, 190.0)
        self.assertLessEqual(peak_fn, S.JUMP_FN_LIMIT + 1e-9)
        self.assertLessEqual(peak_leg_act, S.JUMP_FN_LIMIT + 1e-9)
        self.assertLess(abs(info["pitch"]), np.radians(3.0))
        self.assertLess(abs(float(sim.d.qvel[2])), 0.08)

    def test_tall_jump_retracts_immediately_then_restores_length(self):
        sim = S.WheelLegLQR(leg_length=0.300)
        step = sim.m.geom("step").id
        sim.m.geom_contype[step] = 0
        sim.m.geom_conaffinity[step] = 0
        for _ in range(1500):
            sim.step()
        self.assertTrue(sim.request_jump(0.300))
        saw_landing = False
        for _ in range(7000):
            info = sim.step()
            if sim.jump_phase == "landing" and not saw_landing:
                saw_landing = True
                self.assertAlmostEqual(
                    sim.leg_length, S.JUMP_RETRACT_LENGTH, places=9)
            if saw_landing and sim.jump_phase == "idle":
                break
        self.assertTrue(saw_landing)
        self.assertEqual(sim.jump_phase, "idle")
        for _ in range(1000):
            info = sim.step()
        self.assertGreaterEqual(info["jumpHeight"], 0.12)
        self.assertAlmostEqual(info["L0"], 0.300, delta=0.010)
        self.assertLess(abs(float(sim.d.qvel[0])), 0.08)
        self.assertLess(abs(info["pitch"]), 0.08)

    def test_jump_keeps_drive_speed_and_retracts_to_135_in_flight(self):
        sim = S.WheelLegLQR()
        step = sim.m.geom("step").id
        sim.m.geom_contype[step] = 0
        sim.m.geom_conaffinity[step] = 0
        sim.speed_ramp = 5.0
        for _ in range(1500):
            sim.speed_cmd = 0.5
            sim.step()
        self.assertGreater(float(sim.d.qvel[0]), 0.20)
        self.assertTrue(sim.request_jump())
        self.assertEqual(sim.speed_cmd, 0.5)
        saw_flight = False
        max_leg = 0.0
        min_flight_leg = float("inf")
        for _ in range(6000):
            info = sim.step()
            max_leg = max(max_leg, info["L0"])
            if sim.jump_phase == "flight":
                saw_flight = True
                min_flight_leg = min(min_flight_leg, info["L0"])
            if saw_flight and sim.jump_phase == "idle":
                break
        self.assertEqual(sim.jump_phase, "idle")
        self.assertGreaterEqual(max_leg, 0.345)
        self.assertAlmostEqual(min_flight_leg, 0.135, delta=0.005)
        self.assertGreaterEqual(info["jumpHeight"], 0.12)
        self.assertGreater(float(sim.d.qvel[0]), 0.15)

    def test_leg_pid_tracks_above_old_190_mm_limit(self):
        sim = S.WheelLegLQR()
        for _ in range(500):
            sim.step()
        for target_mm in range(140, 251, 5):
            sim.leg_length = target_mm / 1000.0
            for _ in range(50):
                info = sim.step()
        self.assertGreater(info["L0"], 0.230)
        self.assertLess(abs(info["pitch"]), np.radians(5.0))

    def test_zero_command_brakes_instead_of_continuing_to_accelerate(self):
        sim = S.WheelLegLQR()
        sim.speed_ramp = 5.0
        # Keep this controller regression independent of reaching the step.
        step = sim.m.geom("step").id
        sim.m.geom_contype[step] = 0
        sim.m.geom_conaffinity[step] = 0
        for i in range(1000):
            sim.speed_cmd = min(1.0, (i + 1) * sim.dt)
            sim.step()
        release_speed = float(sim.d.qvel[0])
        sim.speed_cmd = 0.0
        speeds = []
        for _ in range(5000):
            info = sim.step()
            speeds.append(float(sim.d.qvel[0]))
        self.assertAlmostEqual(sim.target_speed, 0.0, places=9)
        self.assertLess(max(speeds), release_speed + 0.08)
        self.assertLess(abs(speeds[-1]), 0.10)
        self.assertLess(abs(info["pitch"]), np.radians(3.0))

    def test_zero_command_starts_without_forward_runaway(self):
        sim = S.WheelLegLQR()
        positions = []
        for _ in range(3000):
            sim.step()
            positions.append(float(sim.d.qpos[0]))
        self.assertLess(max(abs(x) for x in positions), 0.05)
        self.assertLess(abs(float(sim.d.qvel[0])), 0.03)


class KeyboardRegressionTests(unittest.TestCase):
    def make_state(self):
        return {
            "speed": 0.0,
            "turn": 0.0,
            "leg": S.LEG_NOMINAL,
            "cam": "free",
            "hud": True,
            "contacts": True,
        }

    def test_drive_keys_and_space(self):
        state = self.make_state()
        for _ in range(300):
            view_gui.integrate_drive(state, {"8", "4"}, 0.01)
        self.assertAlmostEqual(state["speed"], 2.5)
        self.assertAlmostEqual(state["turn"], 0.5)
        view_gui.integrate_drive(state, set(), 0.01)
        self.assertEqual((state["speed"], state["turn"]), (0.0, 0.0))
        for _ in range(25):
            view_gui.integrate_drive(state, {"5", "6"}, 0.01)
        self.assertAlmostEqual(state["speed"], -1.25)
        self.assertAlmostEqual(state["turn"], -0.5)
        self.assertEqual(view_gui.apply_key(state, ord(" ")), "jump")
        self.assertAlmostEqual(state["speed"], -1.25)
        self.assertAlmostEqual(state["turn"], -0.5)
        for key, expected in (("1", 0.150), ("2", 0.200), ("3", 0.300)):
            self.assertEqual(view_gui.apply_key(state, ord(key)), "drive")
            self.assertAlmostEqual(state["leg"], expected)
        self.assertEqual(view_gui.apply_key(state, ord("r")), "reset")
        self.assertEqual(view_gui.apply_key(state, ord("z")), "reset")

    def test_rear_pose_key_requires_zero_without_eight(self):
        self.assertTrue(view_gui.rear_pose_requested({"0"}))
        self.assertTrue(view_gui.rear_pose_requested({"0", "5"}))
        self.assertFalse(view_gui.rear_pose_requested({"0", "8"}))
        self.assertFalse(view_gui.rear_pose_requested({"8"}))

    def test_numeric_keypad_codes(self):
        state = self.make_state()
        self.assertEqual([view_gui._key_char(keycode)
                          for keycode in (328, 324, 325, 326)],
                         ["8", "4", "5", "6"])
        for keycode, expected in ((321, 0.150), (322, 0.200), (323, 0.300)):
            self.assertEqual(view_gui.apply_key(state, keycode), "drive")
            self.assertAlmostEqual(state["leg"], expected)

    def test_leg_presets_apply_immediately_without_destabilizing_vmc(self):
        sim = S.WheelLegLQR()
        step = sim.m.geom("step").id
        sim.m.geom_contype[step] = 0
        sim.m.geom_conaffinity[step] = 0
        for target in view_gui.LEG_PRESETS.values():
            view_gui.apply_leg_command(sim, target)
            self.assertAlmostEqual(sim.leg_length, target, places=9)
            for _ in range(2000):
                view_gui.apply_leg_command(sim, target)
                info = sim.step()
            self.assertAlmostEqual(sim.leg_length, target, places=6)
            self.assertAlmostEqual(info["L0"], target, delta=0.003)
            self.assertLess(abs(info["pitch"]), np.radians(5.0))

    def test_callback_queue_is_bounded_and_nonblocking(self):
        events = view_gui.KeyEvents(capacity=2)
        for key in "84568456":
            events.callback(ord(key))
        self.assertLessEqual(len(list(events.drain())), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
