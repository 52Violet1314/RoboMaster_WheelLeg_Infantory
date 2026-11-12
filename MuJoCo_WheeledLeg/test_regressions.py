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
        # The step sits on the field deck (0.42 m up), so it is a 200 mm step
        # from the driving surface rather than from the ground plane.
        self.assertTrue(np.allclose(sim.m.geom_pos[geom],
                                    [1.8, 0.0, S.FIELD_HEIGHT + 0.1]))
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
            if sim.m.geom(geom_id).name == "arena":
                continue          # the 12.9 x 8.9 m field deck, not the robot
            mesh_id = int(sim.m.geom_dataid[geom_id])
            start = int(sim.m.mesh_vertadr[mesh_id])
            count = int(sim.m.mesh_vertnum[mesh_id])
            points = np.asarray(sim.m.mesh_vert[start:start + count])
            rotation = np.asarray(sim.d.geom_xmat[geom_id]).reshape(3, 3)
            points = (rotation @ points.T).T + sim.d.geom_xpos[geom_id]
            low = np.minimum(low, points.min(axis=0))
            high = np.maximum(high, points.max(axis=0))
        # The robot stands on the field deck, so compare its visuals relative to
        # the spawn and to the driving surface (wheel contact height) rather
        # than to the world origin.
        low -= [sim.d.qpos[0], sim.d.qpos[1], S.FIELD_HEIGHT]
        high -= [sim.d.qpos[0], sim.d.qpos[1], S.FIELD_HEIGHT]
        self.assertTrue(np.all(high - low < 1.0), (low, high))
        self.assertTrue(np.all(low > [-0.5, -0.5, -0.1]), low)
        self.assertTrue(np.all(high < [0.5, 0.5, 0.8]), high)

    def test_competition_field_mesh_and_collision(self):
        sim = S.WheelLegLQR()
        mesh_geom = sim.m.geom("arena").id
        self.assertEqual(sim.m.geom_type[mesh_geom], mujoco.mjtGeom.mjGEOM_MESH)
        # The CAD mesh is drawn only: MuJoCo would fill the tray-shaped field
        # with its convex hull, so tools/arena_collision.py bakes one box per
        # level into arena_collision.xml, pulled in with <include>.
        self.assertEqual(sim.m.geom_contype[mesh_geom], 0)
        self.assertEqual(sim.m.geom_conaffinity[mesh_geom], 0)
        deck = sim.m.geom("arena_deck").id
        self.assertEqual(sim.m.geom_type[deck], mujoco.mjtGeom.mjGEOM_BOX)
        self.assertAlmostEqual(float(sim.m.geom_size[deck][2]) * 2,
                               S.FIELD_HEIGHT, places=6)
        self.assertIn(deck, sim.g_ground)
        collision = [geom for geom in range(sim.m.ngeom)
                     if sim.m.geom(geom).name.startswith("arena")
                     and sim.m.geom_contype[geom] != 0]
        self.assertGreater(len(collision), 10)          # deck + walls + platforms
        # The CAD deck spans the full 12.9 x 8.9 m and is 0.42 m high.  MuJoCo
        # re-centres mesh vertices, so measure them in the geom's world frame.
        mesh_id = int(sim.m.geom_dataid[mesh_geom])
        start = int(sim.m.mesh_vertadr[mesh_id])
        count = int(sim.m.mesh_vertnum[mesh_id])
        points = np.asarray(sim.m.mesh_vert[start:start + count])
        rotation = np.asarray(sim.d.geom_xmat[mesh_geom]).reshape(3, 3)
        world = (rotation @ points.T).T + sim.d.geom_xpos[mesh_geom]
        self.assertTrue(np.allclose(world.min(axis=0), [0.0, 0.0, 0.0], atol=2e-5))
        self.assertTrue(np.allclose(world.max(axis=0),
                                    [12.9, 8.9, 0.42], atol=2e-5))

    def test_reset_spawns_on_the_field_deck(self):
        sim = S.WheelLegLQR()
        self.assertAlmostEqual(sim.d.qpos[0], S.ARENA_START_X, places=6)
        self.assertAlmostEqual(sim.d.qpos[1], S.ARENA_START_Y, places=6)
        self.assertAlmostEqual(sim.d.qpos[2],
                               sim.leg_length + S.WHEEL_R + S.FIELD_HEIGHT,
                               places=6)
        # Inside the perimeter wall (inner faces at 0.45 / 12.45 / 8.45) and
        # clear of the two 220 mm platforms.
        self.assertGreater(sim.d.qpos[0], 0.9)
        self.assertLess(sim.d.qpos[0], 12.0)
        self.assertGreater(sim.d.qpos[1], 0.9)
        self.assertLess(sim.d.qpos[1], 8.0)

    def test_leg_joints_carry_the_requested_friction(self):
        sim = S.WheelLegLQR()
        for side in ("left", "right"):
            dof = sim.m.jnt_dofadr[sim.j_hip[side]]
            self.assertAlmostEqual(float(sim.m.dof_frictionloss[dof]), 0.5,
                                   places=6)

    def test_long_leg_reset_starts_near_command(self):
        sim = S.WheelLegLQR(leg_length=0.180)
        sim.reset()
        measured = sim.measure()["leg"]["left"][0]
        self.assertLess(abs(measured - 0.180), 0.004)
        self.assertAlmostEqual(sim.d.qpos[2], 0.240 + S.FIELD_HEIGHT, places=6)

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

    def test_yaw_error_is_a_lqr_state_that_moves_the_wheels(self):
        """x[2] = yaw - target_yaw must drive a differential wheel torque."""
        sim = S.WheelLegLQR()
        x, _ = sim.state_vector(sim.measure())
        self.assertAlmostEqual(x[2], 0.0, places=9)
        sim.target_yaw = 0.30
        x, _ = sim.state_vector(sim.measure())
        self.assertAlmostEqual(x[2], -0.30, places=6)
        # K[:,2] is the yaw column: a positive error has to turn the robot back.
        self.assertGreater(abs(sim.K[0, 2]), 1.0)
        self.assertLess(sim.K[0, 2] * sim.K[1, 2], 0.0)   # differential pair
        ctrl, _ = sim.control(sim.measure(), x)
        self.assertLess(ctrl[sim.a_wheel["left"]] * ctrl[sim.a_wheel["right"]], 0.0)
        # ... and it has to be strong enough to break the tyres loose: the
        # 400-weight gains (7.6 Nm/rad) left permanent heading errors after a
        # 40 Nm yaw pulse, which the 4000-weight gains reject completely.
        self.assertGreaterEqual(-(sim.K[0, 2]) + sim.K[1, 2], 20.0)

    def test_heading_hold_rejects_a_yaw_disturbance(self):
        # 20 N*m is twice the 10 N*m hip ceiling, so the LQR has to recover with
        # its hip authority saturated.  Bigger kicks (>= 30 N*m for 0.3 s)
        # overwhelm that hardware limit and are not expected to recover.
        sim = S.WheelLegLQR()
        sim.reset()
        for _ in range(int(2.0 / sim.dt)):
            sim.step()
        y0 = sim.measure()["yaw"]
        sim.target_yaw = y0
        for i in range(int(14.0 / sim.dt)):
            t = sim.d.time
            sim.d.xfrc_applied[sim.b_base, 5] = 20.0 if 3.0 <= t < 3.3 else 0.0
            sim.step()
        error = abs(S.wrap_angle(sim.measure()["yaw"] - sim.target_yaw))
        self.assertLess(np.degrees(error), 1.0)
        self.assertGreater(abs(float(sim.d.qpos[2])), 0.19)   # still standing

    def test_turn_command_tracks_the_requested_yaw_rate(self):
        """4/6 must reach the commanded rate, not jam with the legs splayed."""
        sim = S.WheelLegLQR()
        sim.turn_rate = 0.5
        rates = []
        for i in range(int(6.0 / sim.dt)):
            sim.step()
            meas = sim.measure()
            if i > int(3.0 / sim.dt):
                rates.append(meas["omega"][2])
                self.assertLess(
                    abs(meas["leg"]["left"][1] - meas["leg"]["right"][1]),
                    np.radians(0.5))
        self.assertGreater(np.degrees(np.mean(rates)), 20.0)
        sim.turn_rate = 0.0
        for _ in range(int(6.0 / sim.dt)):
            sim.step()
        meas = sim.measure()
        self.assertLess(abs(S.wrap_angle(meas["yaw"] - sim.target_yaw)),
                        np.radians(1.0))
        self.assertLess(abs(meas["leg"]["left"][1] - meas["leg"]["right"][1]),
                        np.radians(0.2))

    def test_leg_angle_measurement_is_yaw_invariant(self):
        """A body yaw must not leak into the fore-aft leg angle.

        The wheel sits 48 mm outboard of its hip, so the raw world-frame
        hip->wheel vector picks up a lateral component that a body yaw turns
        into a fore-aft one: at 10 deg of yaw the untouched leg reads -3.8 deg
        on the left and +3.8 deg on the right.  The LQR then chases a phantom
        splay while steering.
        """
        sim = S.WheelLegLQR()
        for degrees in (0.0, 10.0, 30.0, 90.0, 170.0, -45.0):
            half = np.radians(degrees) / 2.0
            sim.d.qpos[3:7] = [np.cos(half), 0.0, 0.0, np.sin(half)]
            mujoco.mj_forward(sim.m, sim.d)
            meas = sim.measure()
            self.assertAlmostEqual(meas["leg"]["left"][1],
                                   meas["leg"]["right"][1], places=9)
            self.assertLess(abs(meas["leg"]["left"][1]), np.radians(0.05),
                            f"leg angle moved at yaw {degrees} deg")

    def test_field_walls_and_platforms_block_the_robot(self):
        """The CAD field is solid: walls contain it, the 220 mm platforms stop it."""
        def drive(yaw_deg, secs=5.0):
            sim = S.WheelLegLQR()
            half = np.radians(yaw_deg) / 2.0
            sim.d.qpos[3:7] = [np.cos(half), 0.0, 0.0, np.sin(half)]
            sim.target_yaw = np.radians(yaw_deg)
            mujoco.mj_forward(sim.m, sim.d)
            sim.speed_cmd = 0.8
            for _ in range(int(secs / sim.dt)):
                sim.step()
            return sim

        sim = drive(90.0)                       # +y into the perimeter wall
        self.assertLess(float(sim.d.qpos[1]), 8.45 - 0.20)
        self.assertLess(abs(float(sim.d.qvel[1])), 0.2)
        sim = drive(0.0)                        # +x into the 220 mm platform
        self.assertLess(float(sim.d.qpos[0]), 2.94 - 0.20)
        self.assertLess(abs(float(sim.d.qvel[0])), 0.2)
        sim = drive(180.0)                      # -x into the perimeter wall
        self.assertGreater(float(sim.d.qpos[0]), 0.45 + 0.20)
        self.assertLess(abs(float(sim.d.qvel[0])), 0.2)

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

    def test_hold_zero_clamps_hip_torque_to_2_nm(self):
        """Key 0 keeps the normal controller but makes the legs compliant."""
        sim = S.WheelLegLQR(leg_length=0.270)
        original_leg = sim.leg_length
        self.assertTrue(sim.set_rear_pose(True))
        self.assertTrue(sim.rear_pose_active)
        self.assertAlmostEqual(sim.leg_length, original_leg, places=9)
        # A hard pitch disturbance forces the LQR to use its hip authority;
        # the controller must then never exceed the 2 N*m clamp.
        sim.d.xfrc_applied[sim.b_base, 0] = 120.0
        peak_tp = 0.0
        peak_wheel = 0.0
        for i in range(600):
            info = sim.step()
            peak_tp = max(peak_tp, abs(info["vmcTbL"]), abs(info["vmcTbR"]),
                          abs(sim.d.ctrl[sim.a_hip["left"]]),
                          abs(sim.d.ctrl[sim.a_hip["right"]]))
            peak_wheel = max(peak_wheel, abs(sim.d.ctrl[sim.a_wheel["left"]]))
            if i == 300:
                sim.d.xfrc_applied[sim.b_base, 0] = 0.0
        self.assertLessEqual(peak_tp, S.REAR_POSE_TP_LIMIT + 1e-9)
        self.assertLessEqual(peak_wheel, S.WHEEL_TORQUE_LIMIT + 1e-9)
        # The normal limit is higher and the LQR still runs in this mode.
        self.assertTrue(sim.set_rear_pose(False))
        self.assertFalse(sim.rear_pose_active)
        self.assertGreater(S.VMC_TP_LIMIT, S.REAR_POSE_TP_LIMIT)
        self.assertAlmostEqual(sim.leg_length, original_leg)

    def test_hip_torque_limit_is_10_nm_normally(self):
        sim = S.WheelLegLQR(leg_length=0.270)
        sim.d.xfrc_applied[sim.b_base, 0] = 150.0
        peak_tp = 0.0
        for i in range(600):
            info = sim.step()
            peak_tp = max(peak_tp, abs(info["vmcTbL"]), abs(info["vmcTbR"]),
                          abs(sim.d.ctrl[sim.a_hip["left"]]),
                          abs(sim.d.ctrl[sim.a_hip["right"]]))
            if i == 300:
                sim.d.xfrc_applied[sim.b_base, 0] = 0.0
        self.assertLessEqual(peak_tp, S.VMC_TP_LIMIT + 1e-9)
        self.assertAlmostEqual(S.VMC_TP_LIMIT, 10.0)

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
        # The 0.5 N*m hip friction eats part of the landing momentum (0.32 ->
        # 0.12 m/s), but the robot must still roll forward after touchdown.
        self.assertGreater(float(sim.d.qvel[0]), 0.05)

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
        x0 = float(sim.d.qpos[0])
        positions = []
        for _ in range(3000):
            sim.step()
            positions.append(float(sim.d.qpos[0]) - x0)
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
        self.assertAlmostEqual(state["speed"], view_gui.SPEED_MAX)
        self.assertAlmostEqual(state["turn"], view_gui.TURN_MAX)
        view_gui.integrate_drive(state, set(), 0.01)
        self.assertEqual((state["speed"], state["turn"]), (0.0, 0.0))
        for _ in range(25):
            view_gui.integrate_drive(state, {"5", "6"}, 0.01)
        self.assertAlmostEqual(state["speed"], -1.25)
        self.assertAlmostEqual(state["turn"], -view_gui.TURN_MAX)
        self.assertEqual(view_gui.apply_key(state, ord(" ")), "jump")
        self.assertAlmostEqual(state["speed"], -1.25)
        self.assertAlmostEqual(state["turn"], -view_gui.TURN_MAX)
        for key, expected in (("1", 0.150), ("2", 0.270), ("3", 0.350)):
            self.assertEqual(view_gui.apply_key(state, ord(key)), "drive")
            self.assertAlmostEqual(state["leg"], expected)
        self.assertEqual(view_gui.apply_key(state, ord("r")), "reset")
        self.assertEqual(view_gui.apply_key(state, ord("z")), "reset")

    def test_turn_keys_ramp_five_times_faster(self):
        """4/6 spool up at 25 rad/s^2 to a 2.5 rad/s ceiling (5x the original)."""
        self.assertAlmostEqual(view_gui.TURN_ACCEL, 25.0)
        self.assertAlmostEqual(view_gui.TURN_MAX, 2.5)
        # 20 ms of holding 6 already asks for 0.5 rad/s, the old ceiling.
        state = self.make_state()
        view_gui.integrate_drive(state, {"6"}, 0.02)
        self.assertAlmostEqual(state["turn"], -0.5)
        # Two maximum-length GUI frames (dt is capped at 50 ms) saturate it.
        state = self.make_state()
        view_gui.integrate_drive(state, {"4"}, 0.05)
        self.assertAlmostEqual(state["turn"], 1.25)
        view_gui.integrate_drive(state, {"4"}, 0.05)
        self.assertAlmostEqual(state["turn"], view_gui.TURN_MAX)

    def test_rear_pose_key_is_hold_zero(self):
        self.assertTrue(view_gui.rear_pose_requested({"0"}))
        self.assertTrue(view_gui.rear_pose_requested({"0", "5"}))
        self.assertTrue(view_gui.rear_pose_requested({"0", "8"}))
        self.assertFalse(view_gui.rear_pose_requested({"8"}))

    def test_numeric_keypad_codes(self):
        state = self.make_state()
        self.assertEqual([view_gui._key_char(keycode)
                          for keycode in (328, 324, 325, 326)],
                         ["8", "4", "5", "6"])
        for keycode, expected in ((321, 0.150), (322, 0.270), (323, 0.350)):
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
