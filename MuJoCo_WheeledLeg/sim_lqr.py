"""
MuJoCo simulation of the wheel-legged robot with the traditional
VMC + offline explicit MPC / matrix-LQR controller (no reinforcement learning).

Model
-----
`wheelbipe_lqr.xml` is generated from the open-source `wheelbipeV14_2` USD
(see gen_model.py): real link masses / centres of mass / inertia tensors,
real joint locations, with the six-bar closed-chain leg collapsed into one
rigid leg per side plus a prismatic leg-length axis.

Controller
----------
The loop reproduces the on-target implementation
(`RoboMaster_InfantoryV2.1.4/APP/APP_Task/src/CalculateTask.cpp`):

  1. VMC leg kinematics:  from the hip/leg geometry compute the virtual leg
     length L0, the leg angle theta and their rates, plus the normal force.
  2. Leg-length PID:      F0 = ff + PID(L_ref - L0)  -> virtual leg force.
     Roll differential compensation is added on top of F0.
  3. Balance controller: offline explicit MPC, or matrix LQR u = -K x, with
        x = [s, ds, yaw, dyaw, thL, dthL, thR, dthR, pitch, dpitch]
        u = [TwL, TwR, TbL, TbR]
     MPC uses `mpc_tables.npz`, generated offline by `generate_mpc.py`.
     LQR K comes from `lqr_k.py`, the Python port of
     `Simmulation/get_K_jiao_LQR.m`.
  4. Virtual-model torque mapping: F0 along the leg axis, Tb about the hip.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import mujoco

import lqr_k
import linkage_kinematics

MODEL_PATH = Path(__file__).resolve().with_name("wheelbipe_lqr.xml")

G = 9.81
WHEEL_R = 0.06
LEG_NOMINAL = 0.1339
# RMUL 2027 STEP geometry uses z=0 for the playing surface.
FIELD_HEIGHT = 0.0
# Spawn supplied with the audited field package, clear of fixed obstacles.
ARENA_START_X = 1.25
ARENA_START_Y = 6.75
# Command envelope of the reduced virtual-leg model.  The source six-bar
# closes visually through ~359 mm, so the 350 mm top GUI preset is displayed
# exactly; longer experimental commands (up to 450 mm) clamp the mesh only.
# The slide range is asymmetric because
# q=0 corresponds to the 133.9 mm nominal pose.
LEG_MIN = 0.090
LEG_MAX = 0.450
TRACK_HALF = 0.222
# hip angle that makes the virtual leg exactly vertical (computed from the USD
# geometry: the rest pose has the leg tilted atan2(0.00661, 0.12483) rad)
HIP_VERTICAL = 0.052942
# Static trim of the reduced virtual leg.  The complete USD-derived mass
# distribution has a different balance angle at each extension.  A fixed
# 0.040 rad trim works near the nominal leg but makes 250--350 mm legs creep
# backwards, so interpolate the measured zero-speed trims instead.
THETA_TRIM = 0.040
THETA_TRIM_LENGTHS = np.array([0.090, 0.1339, 0.150, 0.200, 0.250, 0.300, 0.350, 0.450])
THETA_TRIM_VALUES = np.array([0.040, 0.040, 0.037, 0.027, 0.0205, 0.017, 0.0145, 0.012])

# Jump / gas-spring model.  The hardware statement is interpreted as one
# 15 N m gas spring per leg acting through the rear linkage angle.  The reduced
# slide receives the equivalent force tau * d(theta_rear2)/dL.
GAS_SPRING_TORQUE = 15.0
JUMP_CROUCH_LENGTH = 0.150
JUMP_EXTENSION_LENGTH = 0.350
JUMP_RETRACT_LENGTH = 0.135
# User-selected 200 N per-leg thrust.  In the current 20 kg model this gives
# a repeatable 0.134 m flight apex; 30 cm would require about 279 N per leg.
JUMP_TARGET_HEIGHT = 0.134
JUMP_TOTAL_FORCE_PER_LEG = 200.0

# Hardware / virtual-model safety envelope.  These limits are applied both
# to the virtual commands and again to the physical actuator commands below.
VMC_TP_LIMIT = 10.0             # hip virtual torque [N m]
REAR_POSE_TP_LIMIT = 2.0        # hip virtual torque while key 0 is held [N m]
VMC_FN_LIMIT = 200.0            # leg axial force [N]
JUMP_FN_LIMIT = 200.0           # leg axial force during thrust [N]
WHEEL_TORQUE_LIMIT = 7.0        # hub motor torque [N m]
# Max heading lead while 4/6 drives the LQR yaw setpoint.  The differential
# wheel torque that steers the robot is proportional to this lead, so it caps
# the steering authority; see the note on q_yaw below.
YAW_TARGET_LEAD_LIMIT = 0.16    # [rad]

# Holding key 0 keeps the normal controller but clamps the hip (virtual leg
# swing) torque to REAR_POSE_TP_LIMIT, i.e. it makes the legs compliant.
# (It used to switch to an open-loop +2 N m hub drive with unpowered hips.)

# Analytic-LQR parameters calibrated from the generated USD-derived model.
BODY = dict(R_w=WHEEL_R, R_l=TRACK_HALF, l_c=0.0024, m_w=0.512, m_l=1.056,
            m_b=16.862, I_z=0.189, I_b=0.3129)


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def theta_trim(leg_length):
    """Interpolated static virtual-leg balance angle for a commanded length."""
    return float(np.interp(leg_length, THETA_TRIM_LENGTHS, THETA_TRIM_VALUES))


def wrap_angle(angle):
    """Return an angle in [-pi, pi] without a discontinuous yaw error."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


class LegPID:
    """Same structure as the firmware Float_PID (P + I, D on measurement)."""

    def __init__(self, kp, ki, kd, out_limit, int_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit, self.int_limit = out_limit, int_limit
        self.reset()

    def reset(self):
        self.err_int = 0.0
        self.last_meas = None

    def __call__(self, ref, meas, dt):
        err = ref - meas
        self.err_int = float(np.clip(self.err_int + err * dt,
                                     -self.int_limit, self.int_limit))
        dmeas = (meas - self.last_meas) / dt if self.last_meas is not None else 0.0
        self.last_meas = meas
        out = self.kp * err + self.ki * self.err_int - self.kd * dmeas
        return float(np.clip(out, -self.out_limit, self.out_limit))


# Sign convention calibration: maps the analytic state / input vector onto the
# MuJoCo measurement.  Obtained by `calibrate_signs.py`, which scans all 2^6
# combinations and keeps the one that balances the robot.
#   th = -1  : the analytic leg angle is positive when the body leans forward
#              (our measured angle is positive when the wheel is ahead).
#   pitch=+1 : nose-down positive (matches `asin(-R[2,0])`).
#   Tw/Tb=+1 : the analytic wheel / hip torques map directly onto the MuJoCo
#              motor commands.
DEFAULT_SIGNS = dict(s=+1, ds=+1, yaw=+1, dyaw=+1, th=-1, dth=-1,
                     pitch=+1, dpitch=+1, Tw=+1, Tb=+1)


class WheelLegLQR:
    def __init__(self, leg_length=LEG_NOMINAL, target_speed=0.0, target_yaw=0.0,
                 signs=None, model_path=MODEL_PATH, dt=None,
                 leg_kp=3000.0, leg_ki=10.0, leg_kd=80.0, pos_ki=0.6,
                 speed_ramp=1.0, theta_mode="abs", ds_mode="joint",
                 q_vel=1.0, q_yaw=4000.0, q_yawrate=1.0, controller="lqr"):
        if controller not in ("lqr", "mpc"):
            raise ValueError("controller must be 'lqr' or 'mpc'")
        self.controller = controller
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        # Articulated link collision meshes change their local transforms.
        # Do not use the static per-body BVH built at model load for them.
        self.m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_MIDPHASE
        self.d = mujoco.MjData(self.m)
        self.dt = dt or self.m.opt.timestep
        self.signs = dict(DEFAULT_SIGNS)
        if signs:
            self.signs.update(signs)

        self._leg_length = float(np.clip(leg_length, LEG_MIN, LEG_MAX))
        # Q follows Simmulation/get_K_jiao_LQR.m, with the yaw weight raised
        # from 400 to 4000: the yaw error is a real LQR state (x[2]) but the
        # 400 weight leaves only ~7.6 N*m/rad of differential wheel torque, far
        # below what the tyres need to break loose sideways (measured: a 40 N*m
        # yaw pulse or a 30 N*m sustained push left a permanent 98-116 deg
        # heading error, and a 30 deg heading step settled 7.3 deg short).
        # 4000 makes the heading hold actually converge (<=1.8 deg after a
        # 30 deg step, disturbances fully rejected) at the same wheel effort as
        # before, because YAW_TARGET_LEAD_LIMIT bounds the turning lead.
        self.Q = np.diag([20.0, q_vel, q_yaw, q_yawrate,
                          35000.0, 10.0, 35000.0, 10.0, 40000.0, 1.0])
        self.mpc = None
        if controller == "mpc":
            from explicit_mpc import ExplicitMPC
            self.mpc = ExplicitMPC(self.dt)
            if not np.array_equal(self.Q, self.mpc.Q):
                raise ValueError("MPC weights differ from offline design; regenerate tables")
            self.mpc.select_length(self._leg_length)
            self.K = self.mpc.gain
            self.A = self.B = self.params = None
        else:
            self.K, self.A, self.B, self.params = lqr_k.compute_K(
                self._leg_length, BODY, Q=self.Q)
        self._gain_leg_length = self._leg_length
        self.target_speed = target_speed
        self.target_yaw = target_yaw
        self._yaw0 = target_yaw
        # position integral (the firmware's Pos_INT): removes the steady-state
        # position offset left by the constant model mismatch.
        self.pos_ki = pos_ki

        # per-leg vertical load
        self.mass = sum(self.m.body_mass)
        self.F0_ff = 0.5 * self.mass * G
        self.pid_l = LegPID(leg_kp, leg_ki, leg_kd, out_limit=200.0, int_limit=20.0)
        self.pid_r = LegPID(leg_kp, leg_ki, leg_kd, out_limit=200.0, int_limit=20.0)
        # yaw rate loop.  The reference firmware does NOT use the LQR yaw
        # columns for steering: `Chassis::SynthesizeMotion()` runs a separate
        # yaw-speed PID and adds its output differentially to the two wheel
        # torques on top of the LQR.  Reproduced here.
        self.yaw_pid = LegPID(kp=12.0, ki=0.0, kd=0.5,
                              out_limit=8.0, int_limit=2.0)
        # steering: the firmware adds a separate yaw-speed PID on top of the
        # LQR, but on the converted model that loop makes the legs splay, so the
        # default is the pure-LQR heading path (see README section 6).
        self.yaw_pid_enable = False
        self.turn_limit = 1.5         # max differential wheel torque [Nm]
        # leg-angle differential loop.  The firmware's `anti_crash_` PID keeps
        # the two legs symmetric:
        #     err = phi0_L - phi0_R
        #     T_leg_L = Tp_L + out ,  T_leg_R = Tp_R - out
        # Without it the hip angles drift apart under sustained steering and the
        # robot slowly squats.
        self.anti_crash = LegPID(kp=3.0, ki=0.3, kd=0.1,
                                 out_limit=6.0, int_limit=2.0)
        self.anti_crash_enable = True
        # Held-0 is direct torque mode: no LQR, no hip torque, fixed hubs.
        self.rear_pose_active = False
        self.last_rear_tp = {"left": 0.0, "right": 0.0}
        self.wheel_ff = True          # cancel the wheel reaction on the hips
        # The balancing LQR is intentionally gentle around zero speed and can
        # otherwise coast while its position and velocity terms cancel.  This
        # filtered PI term supplies bounded zero-command braking.
        self.stop_brake_kp = 3.0
        self.stop_brake_ki = 4.0
        self.stop_brake_leak = 0.25
        self.stop_brake_limit = 4.0
        self.stop_brake_tau = 0.10
        self.jump_phase = "idle"
        self.jump_time = 0.0
        self.jump_return_leg = self._leg_length
        self.jump_takeoff_z = None
        self.jump_apex_z = 0.0
        self.jump_height = 0.0
        self._jump_had_contact = False
        self.last_gas_force = {"left": 0.0, "right": 0.0}
        self.last_vmc_total = {"left": 0.0, "right": 0.0}
        # Runtime telemetry consumed by the GUI data view.  These are copies
        # of the values used by the controller, not a second calculation.
        self.last_lqr_state = np.zeros(10)
        self.last_virtual_input = np.zeros(4)
        self.last_vmc_pid_force = {"left": 0.0, "right": 0.0}
        self.last_vmc_roll_comp = 0.0
        self.speed_ramp = speed_ramp
        self.speed_cmd = target_speed
        self.turn_rate = 0.0          # yaw rate command [rad/s]
        self.estop = False            # emergency stop: all torques zeroed
        self.rescue_phase = "idle"
        self.theta_mode = theta_mode
        self.ds_mode = ds_mode

        self._ids()
        self.linkage_visual = linkage_kinematics.LinkageVisuals(self.m)
        # Contact meshes must track the linkage on every physics step.
        self._linkage_visual_stride = 1
        self._linkage_visual_countdown = 0
        self.reset()
        # exact static leg force (gravity + contact) so the leg does not jump
        # at t = 0; computed with the inverse dynamics of the nominal pose.
        self.F0_ff = self._static_leg_force()

    @property
    def leg_length(self):
        return self._leg_length

    @leg_length.setter
    def leg_length(self, value):
        """Set leg length and select an offline MPC model or recompute LQR."""
        target = float(np.clip(value, LEG_MIN, LEG_MAX))
        self._leg_length = target
        if getattr(self, "mpc", None) is not None:
            self._gain_leg_length = self.mpc.select_length(target)
            self.K = self.mpc.gain
            return
        # Key presses change the target by 5 mm.  Recompute at that granularity
        # rather than on every 1 kHz control step.
        gain_length = getattr(self, "_gain_leg_length", target)
        at_endpoint = (target <= LEG_MIN + 1e-9 or target >= LEG_MAX - 1e-9)
        if (hasattr(self, "Q") and
                (abs(target - gain_length) >= 0.004 or
                 (at_endpoint and abs(target - gain_length) > 1e-9))):
            self.K, self.A, self.B, self.params = lqr_k.compute_K(
                target, BODY, Q=self.Q)
            self._gain_leg_length = target

    # ------------------------------------------------------------------ ids
    def _ids(self):
        m = self.m
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        aid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        self.j_hip = {"left": jid("left_hip_joint"), "right": jid("right_hip_joint")}
        self.j_wheel = {"left": jid("left_wheel_joint"), "right": jid("right_wheel_joint")}
        self.j_slide = {"left": jid("left_legslide_joint"), "right": jid("right_legslide_joint")}
        self.a_wheel = {"left": aid("left_wheel_motor"), "right": aid("right_wheel_motor")}
        self.a_hip = {"left": aid("left_hip_motor"), "right": aid("right_hip_motor")}
        self.a_leg = {"left": aid("left_leg_motor"), "right": aid("right_leg_motor")}
        self.b_hip = {"left": bid("left_leg"), "right": bid("right_leg")}
        self.b_wheel = {"left": bid("left_wheel"), "right": bid("right_wheel")}
        self.b_base = bid("base_link")
        self.g_wheel = {"left": gid("left_wheel_geom"),
                        "right": gid("right_wheel_geom")}
        # Everything static the wheels can rest on: the ground plane, the step
        # and every generated arena collision box (deck, walls, platforms).
        self.g_ground = {geom for geom in range(m.ngeom)
                         if m.geom(geom).name in ("floor", "step")
                         or m.geom(geom).name.startswith("arena")}

    def _static_leg_force(self):
        """
        Slide force that holds the robot in the nominal pose.

        The slide axis only carries the wheel assembly, so the force needed is
        the ground normal load minus the wheel's own weight.  The body + wheel
        weight is shared by the two legs; the leg links themselves hang from the
        hip joints.
        """
        m_wheel = self.m.body_mass[self.b_wheel["left"]]
        # Each wheel-ground contact carries half of the complete robot.  The
        # slide actuator does not need to carry its wheel's own weight, but it
        # does carry the base and all leg-link mass above that wheel.
        return (0.5 * self.mass - m_wheel) * G

    def reset(self, z=None, leg_vertical=True):
        d, m = self.d, self.m
        mujoco.mj_resetData(m, d)
        # The RMUL 2027 playing surface spans X=[0, 12], Y=[0, 8] m at z=0.
        d.qpos[0] = ARENA_START_X
        d.qpos[1] = ARENA_START_Y
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[2] = (z if z is not None
                     else (self.leg_length + WHEEL_R + FIELD_HEIGHT))
        if leg_vertical:
            # the USD rest pose has the virtual leg tilted ~3 deg forward; the
            # LQR linearisation uses theta = 0 (leg vertical), so pre-rotate the
            # hip joints to remove the start-up transient.
            for side in ("left", "right"):
                d.qpos[m.jnt_qposadr[self.j_hip[side]]] = HIP_VERTICAL
                # Start close to the requested leg length instead of dropping a
                # long-leg start from the nominal 134 mm pose.
                slide = self.leg_length - LEG_NOMINAL
                j_slide = self.j_slide[side]
                lo, hi = m.jnt_range[j_slide]
                d.qpos[m.jnt_qposadr[j_slide]] = float(np.clip(slide, lo, hi))
        d.ctrl[:] = 0.0
        self.linkage_visual.update(d, self.j_slide)
        self._linkage_visual_countdown = self._linkage_visual_stride
        mujoco.mj_forward(m, d)
        self.pid_l.reset()
        self.pid_r.reset()
        self.yaw_pid.reset()
        self.anti_crash.reset()
        self.rear_pose_active = False
        self.last_rear_tp = {"left": 0.0, "right": 0.0}
        self.s = 0.0
        self.position_tracking = False
        self.position_hold = False
        self.s_int = 0.0
        self.stop_brake_int = 0.0
        self.stop_brake_torque = 0.0
        self.jump_phase = "idle"
        self.rescue_phase = "idle"
        self.jump_time = 0.0
        self.jump_takeoff_z = None
        self.jump_apex_z = float(d.qpos[2])
        self.jump_height = 0.0
        self._jump_had_contact = False
        self._prev = {}
        self.target_speed = 0.0        # ramped up inside step()
        self.target_yaw = self._yaw0

    def set_rear_pose(self, active):
        """Toggle the held-0 compliant-leg mode (hip torque clamped to 2 N m)."""
        active = (bool(active) and not self.jump_active and not self.estop
                  and self.rescue_phase == "idle")
        if active == self.rear_pose_active:
            return False
        self.rear_pose_active = active
        # The clamp only ever lowers the hip torque, so no state reset is
        # needed; keep the reported value honest.
        self.last_rear_tp = {"left": 0.0, "right": 0.0}
        return True

    def request_stand(self):
        """Start physical self-righting; hand off to 150 mm balance last."""
        self.estop = False
        self.rear_pose_active = False
        self.leg_length = 0.150
        self.speed_cmd = self.target_speed = 0.0
        self.turn_rate = 0.0
        self.target_yaw = self.measure()["yaw"]
        self.jump_phase = "idle"
        self.jump_time = 0.0
        self.pid_l.reset()
        self.pid_r.reset()
        self.yaw_pid.reset()
        self.anti_crash.reset()
        self.position_tracking = False
        self.position_hold = False
        self.s = self.s_int = 0.0
        self.stop_brake_int = self.stop_brake_torque = 0.0
        self._prev = {}
        self.rescue_phase = "body" if abs(self.measure()["pitch_full"]) > np.pi/2 else "lift"
        self.rescue_time = 0.0
        self.rescue_total = 0.0
        self.rescue_sign = 1.0 if self.measure()["pitch_full"] >= 0 else -1.0
        self.rescue_q = {side: float(self.d.qpos[self.m.jnt_qposadr[j]])
                         for side, j in self.j_hip.items()}
        self.rescue_targets = dict(self.rescue_q)
        self.rescue_front = {side: False for side in self.j_hip}
        self.rescue_ctrl = np.zeros(self.m.nu)
        self.rescue_length = max(0.150, float(np.mean([self.measure()["leg"][s][0]
                                           for s in self.j_hip])))
        if (abs(self.measure()["pitch_full"]) < 0.35 and
                max(abs(self.measure()["leg"][s][1]) for s in self.j_hip) < 0.4):
            self.rescue_phase = "balance"
        self._rescue_enter(self.rescue_phase)

    def _rescue_enter(self, phase):
        self.rescue_phase = phase
        self.rescue_time = 0.0
        for side, j in self.j_hip.items():
            q = float(self.d.qpos[self.m.jnt_qposadr[j]])
            local = wrap_angle(q - HIP_VERTICAL)
            if phase == "lift":
                # Front is negative hip rotation on BOTH reduced-model joints.
                # Use each side's own angle, and travel over the top (decreasing
                # q), rather than dragging a forward leg through the floor.
                self.rescue_front[side] = local < -0.15
                self.rescue_targets[side] = q - np.pi - local if self.rescue_front[side] else q
            elif phase == "sweep":
                self.rescue_targets[side] = (q - ((local - np.pi/2) % (2*np.pi))
                    if self.rescue_front[side] else q + wrap_angle(np.pi/2-local))

    def _rescue_step(self):
        self.speed_cmd = self.target_speed = self.turn_rate = 0.0
        self.rescue_time += self.dt
        self.rescue_total += self.dt
        meas = self.measure()
        p = meas["pitch_full"]
        phase = self.rescue_phase
        if self.rescue_total > 45.0 or not np.isfinite(self.d.qpos).all():
            self.estop = True
            self.rescue_phase = "fault"
            return self.step()
        if abs(p) > np.pi/2 + 0.08 and phase not in ("body", "body_settle"):
            self.rescue_sign = 1.0 if p >= 0 else -1.0
            self._rescue_enter("body")
            phase = "body"
        if phase == "body" and abs(p) < 1.2:
            self._rescue_enter("body_settle")
            phase = "body_settle"
        if phase == "body_settle" and abs(p) < 0.35:
            self._rescue_enter("lift")
            phase = "lift"
        if phase in ("lift", "sweep"):
            reached = all(abs(self.d.qpos[self.m.jnt_qposadr[self.j_hip[s]]] -
                              self.rescue_targets[s]) < 0.12 for s in self.j_hip)
            if reached and self.rescue_time > 0.2:
                self._rescue_enter("sweep" if phase == "lift" else "stand")
                phase = self.rescue_phase
        if phase == "stand" and abs(p) < 0.20 and abs(meas["roll"]) < 0.20 and max(
                abs(meas["leg"][s][1]) for s in self.j_hip) < 0.20 and max(
                abs(self.d.qvel[self.m.jnt_dofadr[j]]) for j in self.j_hip.values()) < 0.6:
            self._rescue_enter("balance")
            phase = "balance"
        if phase == "balance":
            # Handoff only near the LQR's upright operating point.
            self.leg_length = 0.150
            if (self.rescue_time > 0.8 and abs(p) < 0.25 and
                    all(abs(meas["leg"][s][0] - 0.150) < 0.02 for s in self.j_hip)):
                self.rescue_phase = "idle"
            return None
        desired_length = 0.30 if phase in ("body", "body_settle", "lift") else 0.150
        self.rescue_length += float(np.clip(desired_length-self.rescue_length,
                                          -0.06*self.dt, 0.06*self.dt))
        ctrl = np.zeros(self.m.nu)
        for side, j in self.j_hip.items():
            q = float(self.d.qpos[self.m.jnt_qposadr[j]])
            v = float(self.d.qvel[self.m.jnt_dofadr[j]])
            if phase == "body":
                target = self.rescue_q[side] + self.rescue_sign*0.6*self.dt
            elif phase in ("body_settle", "stand"):
                rear = (0.30 if phase == "body_settle" else 0.08)*self.rescue_sign
                target = q + wrap_angle(HIP_VERTICAL-p+rear-q)
            else:
                target = self.rescue_targets[side]
            self.rescue_q[side] += float(np.clip(target-self.rescue_q[side],
                                                -0.6*self.dt, 0.6*self.dt))
            torque = np.clip(30*(self.rescue_q[side]-q)-4*v, -20, 20)
            if abs(v) > 1.5 and torque*v > 0:
                torque = -np.sign(v)*min(20.0, 6*(abs(v)-1.5))
            ctrl[self.a_hip[side]] = torque
            js = self.j_slide[side]
            slide = self.d.qpos[self.m.jnt_qposadr[js]]
            vel = self.d.qvel[self.m.jnt_dofadr[js]]
            ctrl[self.a_leg[side]] = np.clip(1800*(self.rescue_length-LEG_NOMINAL-slide)
                                                    -60*vel+50, -100, 200)
        self.rescue_ctrl += np.clip(ctrl-self.rescue_ctrl, -400*self.dt, 400*self.dt)
        self.d.ctrl[:] = self.rescue_ctrl
        self.d.qfrc_applied[:] = 0.0
        self.last_gas_force = {"left": 0.0, "right": 0.0}
        self._update_linkage_visual()
        mujoco.mj_step(self.m, self.d)
        return dict(t=self.d.time, x=float(self.d.qpos[0]), z=float(self.d.qpos[2]),
                    pitch=p, roll=meas["roll"], vx=0.0, L0=meas["leg"]["left"][0],
                    theta=meas["leg"]["left"][1], TwL=0.0, TwR=0.0,
                    TbL=ctrl[self.a_hip["left"]], TbR=ctrl[self.a_hip["right"]],
                    legF=ctrl[self.a_leg["left"]], vmcFL=ctrl[self.a_leg["left"]],
                    vmcFR=ctrl[self.a_leg["right"]], vmcTbL=ctrl[self.a_hip["left"]],
                    vmcTbR=ctrl[self.a_hip["right"]], legActFL=ctrl[self.a_leg["left"]],
                    legActFR=ctrl[self.a_leg["right"]], jointTauL=ctrl[self.a_hip["left"]],
                    jointTauR=ctrl[self.a_hip["right"]], hubTauL=0.0, hubTauR=0.0,
                    gasFL=0.0, gasFR=0.0, stopBrake=0.0, rearPose=False,
                    rearTpL=0.0, rearTpR=0.0, jumpPhase="idle", jumpHeight=0.0,
                    rescuePhase=self.rescue_phase)

    @property
    def jump_active(self):
        return self.jump_phase != "idle"

    def request_jump(self, return_leg=None):
        """Start one crouch/thrust/flight/landing sequence."""
        if self.jump_active or self.estop or self.rescue_phase != "idle":
            return False
        self.set_rear_pose(False)
        self.jump_phase = "crouch"
        self.jump_time = 0.0
        self.jump_return_leg = float(np.clip(
            self.leg_length if return_leg is None else return_leg,
            LEG_MIN, LEG_MAX))
        self.jump_takeoff_z = None
        self.jump_apex_z = float(self.d.qpos[2])
        self.jump_height = 0.0
        self._jump_had_contact = False
        self.position_tracking = False
        self.position_hold = False
        self.s = 0.0
        self.s_int = 0.0
        return True

    def _gas_force_for_slide(self, slide):
        """Equivalent extension force of one 15 N m linkage gas spring."""
        eps = 0.0005
        q_lo = self.linkage_visual.angles(slide - eps)
        q_hi = self.linkage_visual.angles(slide + eps)
        dtheta_dlength = (q_hi[5] - q_lo[5]) / (2.0 * eps)
        return float(np.clip(abs(GAS_SPRING_TORQUE * dtheta_dlength), 0.0, 120.0))

    def _gas_force(self, side):
        joint = self.j_slide[side]
        slide = float(self.d.qpos[self.m.jnt_qposadr[joint]])
        return self._gas_force_for_slide(slide)

    def _wheels_grounded(self):
        wheel_geoms = set(self.g_wheel.values())
        for contact in self.d.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            if pair & wheel_geoms and pair & self.g_ground:
                return True
        return False

    def _update_jump(self, meas):
        if not self.jump_active:
            return
        self.jump_time += self.dt
        self.jump_apex_z = max(self.jump_apex_z, float(self.d.qpos[2]))
        grounded = self._wheels_grounded()

        if self.jump_phase == "crouch":
            # Leg-length commands are intentionally not slew limited.  The
            # VMC force limits and the mechanism dynamics determine the real
            # motion; delaying the setpoint would make retraction too slow.
            self.leg_length = JUMP_CROUCH_LENGTH
            crouched = max(meas["leg"][s][0] for s in ("left", "right")) <= 0.158
            settled = (abs(float(self.d.qvel[2])) < 0.15
                       and abs(meas["pitch"]) < 0.10)
            if (crouched and settled and self.jump_time >= 0.15):
                self.jump_phase = "thrust"
                self.jump_time = 0.0
                self._jump_had_contact = grounded
                self.leg_length = JUMP_EXTENSION_LENGTH
                self.pid_l.reset()
                self.pid_r.reset()
        elif self.jump_phase == "thrust":
            self.leg_length = JUMP_EXTENSION_LENGTH
            self._jump_had_contact |= grounded
            if (self._jump_had_contact and not grounded
                    and self.jump_time >= 0.02 and self.d.qvel[2] > 0.2):
                self.jump_phase = "flight"
                self.jump_time = 0.0
                self.jump_takeoff_z = float(self.d.qpos[2])
                self.jump_apex_z = self.jump_takeoff_z
                self.leg_length = JUMP_RETRACT_LENGTH
                self.pid_l.reset()
                self.pid_r.reset()
            elif self.jump_time >= 0.35:
                self.jump_phase = "flight"
                self.jump_time = 0.0
                self.jump_takeoff_z = float(self.d.qpos[2])
                self.jump_apex_z = self.jump_takeoff_z
                self.leg_length = JUMP_RETRACT_LENGTH
                self.pid_l.reset()
                self.pid_r.reset()
        elif self.jump_phase == "flight":
            # Extension has completed its work at take-off.  Tuck both legs
            # immediately so the compact pose is reached before touchdown.
            self.leg_length = JUMP_RETRACT_LENGTH
            if self.jump_takeoff_z is not None:
                self.jump_height = max(
                    self.jump_height, self.jump_apex_z - self.jump_takeoff_z)
            if self.jump_time >= 0.10 and grounded and self.d.qvel[2] <= 0.0:
                self.jump_phase = "landing"
                self.jump_time = 0.0
                # Retract all the way to the compact landing pose immediately.
                # Restoring a tall pre-jump pose is postponed until touchdown
                # motion has been absorbed.
                self.leg_length = JUMP_RETRACT_LENGTH
                self.pid_l.reset()
                self.pid_r.reset()
                self.position_tracking = False
                self.position_hold = False
                self.s = 0.0
                self.s_int = 0.0
                self._prev = {}
        elif self.jump_phase == "landing":
            self.leg_length = JUMP_RETRACT_LENGTH
            legs_retracted = max(
                abs(meas["leg"][side][0] - JUMP_RETRACT_LENGTH)
                for side in ("left", "right")) < 0.010
            settled = (abs(float(self.d.qvel[2])) < 0.10
                       and abs(meas["pitch"]) < 0.10)
            if self.jump_time >= 0.25 and legs_retracted and settled:
                self.jump_phase = "recover"
                self.jump_time = 0.0
                self.leg_length = self.jump_return_leg
                self.pid_l.reset()
                self.pid_r.reset()
        elif self.jump_phase == "recover":
            self.leg_length = self.jump_return_leg
            legs_restored = max(
                abs(meas["leg"][side][0] - self.jump_return_leg)
                for side in ("left", "right")) < 0.010
            settled = (abs(float(self.d.qvel[2])) < 0.08
                       and abs(meas["pitch"]) < 0.08)
            if self.jump_time >= 0.50 and legs_restored and settled:
                self.jump_phase = "idle"
                self.jump_time = 0.0

    def _apply_gas_springs(self):
        self.d.qfrc_applied[:] = 0.0
        for side in ("left", "right"):
            force = self._gas_force(side)
            self.last_gas_force[side] = force
            dof = self.m.jnt_dofadr[self.j_slide[side]]
            self.d.qfrc_applied[dof] = force

    # ------------------------------------------------------- measurements
    def measure(self):
        d = self.d
        R = quat_to_mat(d.qpos[3:7])
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        pitch = float(np.arcsin(np.clip(-R[2, 0], -1, 1)))
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        # MuJoCo stores a free joint's angular velocity in the BODY frame
        # (verified in tools/probe_frame.py), so rotate it into the world frame.
        omega_body = d.qvel[3:6].copy()
        omega = R @ omega_body
        # The pitch/roll loops need rates about the robot's own heading axes.
        # Rotating the world rate back by -yaw gives `Ry(pitch) Rx(roll) w_body`,
        # whose Y component is exactly the rate of `pitch = asin(-R[2,0])`.
        # World omega[1] is only `q*cos(yaw) - p*sin(yaw)`: it even inverts past
        # 90 deg of heading, which turned the LQR pitch-rate column into
        # positive feedback during a long turn.  At yaw = 0 both agree.
        c, s = np.cos(yaw), np.sin(yaw)
        yaw_free = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        omega_heading = yaw_free @ omega

        leg = {}
        # Measure the leg angle in the robot's own sagittal plane.  Each wheel
        # sits 48 mm *outboard* of its hip, so the raw world-frame hip->wheel
        # vector carries a lateral component that a body yaw turns into a
        # fore-aft one: at 10 deg of yaw the untouched leg already reads
        # -3.8 deg on the left and +3.8 deg on the right (measured).  Feeding
        # that to the LQR makes its leg-angle columns chase a phantom splay that
        # grows with the heading, so while 4/6 turns the legs jam ~4.7 deg apart
        # and the yaw error freezes ~9 deg short of the target.  Undo the yaw.
        for side in ("left", "right"):
            hip = d.xpos[self.b_hip[side]]
            wheel = d.xpos[self.b_wheel[side]]
            v = yaw_free @ (wheel - hip)
            L0 = float(np.linalg.norm(v))
            theta = float(np.arctan2(v[0], -v[2]))   # >0 : wheel ahead of hip
            q_hip = float(d.qpos[self.m.jnt_qposadr[self.j_hip[side]]])
            leg[side] = (L0, theta, q_hip)
        w = {s: float(d.qvel[self.m.jnt_dofadr[self.j_wheel[s]]])
             for s in ("left", "right")}
        return dict(R=R, yaw=yaw, pitch=pitch, pitch_full=float(np.arctan2(-R[2, 0], R[2, 2])), roll=roll, omega=omega,
                    omega_heading=omega_heading, leg=leg, wheel_vel=w)

    def state_vector(self, meas):
        sg = self.signs
        d = self.d
        L = meas["leg"]["left"]
        Rl = meas["leg"]["right"]
        # theta source: absolute leg angle from vertical, or the hip joint angle
        # (leg angle relative to the body)
        thL = L[2] if self.theta_mode == "hip" else L[1]
        thR = Rl[2] if self.theta_mode == "hip" else Rl[1]
        # body speed.  The firmware builds it from the wheel joint rates plus
        # the leg's own rotation rate (the wheel encoder measures rotation
        # relative to the leg, not to the world).
        vx = WHEEL_R * 0.5 * (meas["wheel_vel"]["left"] + meas["wheel_vel"]["right"])
        dt = self.dt
        prev = self._prev
        dthL = wrap_angle(thL - prev["thL"]) / dt if "thL" in prev else 0.0
        dthR = wrap_angle(thR - prev["thR"]) / dt if "thR" in prev else 0.0
        prev["thL"], prev["thR"] = thL, thR
        if self.ds_mode == "absolute":
            vx += WHEEL_R * 0.5 * (dthL + dthR)

        # This is actual position minus the position obtained by integrating
        # the speed command.  It stays disabled while the robot merely settles
        # after reset; the first drive command defines a local origin, then the
        # same reference is preserved through motion and key-release hold.
        if self.position_tracking:
            self.s += (vx - self.target_speed) * dt
        else:
            self.s = 0.0
        x = np.array([
            sg["s"] * self.s,
            sg["ds"] * (vx - self.target_speed),
            sg["yaw"] * wrap_angle(meas["yaw"] - self.target_yaw),
            sg["dyaw"] * meas["omega"][2],
            sg["th"] * (thL - theta_trim(self.leg_length)),
            sg["dth"] * dthL,
            sg["th"] * (thR - theta_trim(self.leg_length)),
            sg["dth"] * dthR,
            sg["pitch"] * meas["pitch"],
            sg["dpitch"] * meas["omega_heading"][1],
        ])
        return x, vx

    # ------------------------------------------------------------ control
    def control(self, meas, x, turn=0.0):
        sg = self.signs
        rear_pose = self.rear_pose_active
        # Holding 0 only lowers the hip torque ceiling; everything else (LQR,
        # steering, VMC leg loop, braking) keeps running exactly as normal.
        tp_limit = REAR_POSE_TP_LIMIT if rear_pose else VMC_TP_LIMIT
        # Position integral removes the steady-state offset between actual and
        # speed-command-integrated position.  x[0] is actual minus target, so
        # its integral has the same control sign as LQR's term.
        self.s_int = float(np.clip(self.s_int + x[0] * self.dt, -1.0, 1.0))
        offset = np.array([self.pos_ki * self.s_int - turn,
                           self.pos_ki * self.s_int + turn, 0.0, 0.0])
        limits = np.array([WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT,
                           tp_limit, tp_limit])
        if self.mpc is not None:
            u = self.mpc.evaluate(x, limits, offset)
        else:
            # Keep the original LQR arithmetic order for reproducible contacts.
            u = -self.K @ x
            u[0] += self.pos_ki * self.s_int
            u[1] += self.pos_ki * self.s_int
            u[0] -= turn
            u[1] += turn
        # Keep the final virtual-input limits before VMC and auxiliary loops.
        u = np.clip(u, -limits, limits)
        self.last_lqr_state = x.copy()
        self.last_virtual_input = u.copy()
        ctrl = np.zeros(self.m.nu)
        for i, side in enumerate(("left", "right")):
            ctrl[self.a_wheel[side]] = sg["Tw"] * u[i]
        for i, side in enumerate(("left", "right")):
            ctrl[self.a_hip[side]] = sg["Tb"] * u[2 + i]

        # Select braking from the operator command, not target_speed, so key
        # release acts immediately while the short inner ramp removes the LQR
        # setpoint discontinuity.  Project world velocity onto body-forward so
        # this remains correct after steering.
        if (abs(self.speed_cmd) <= 1e-9
                and (not self.jump_active
                     or self.jump_phase in ("landing", "recover"))):
            forward_v = float(np.dot(meas["R"][:, 0], self.d.qvel[:3]))
            # Do not let integral accumulated while braking one direction push
            # the chassis through zero in the opposite direction.
            if forward_v * self.stop_brake_int < 0.0:
                self.stop_brake_int = 0.0
            self.stop_brake_int = float(np.clip(
                self.stop_brake_int
                + (forward_v - self.stop_brake_leak * self.stop_brake_int)
                * self.dt,
                -0.75, 0.75))
            wanted = float(np.clip(
                self.stop_brake_kp * forward_v
                + self.stop_brake_ki * self.stop_brake_int,
                -self.stop_brake_limit, self.stop_brake_limit))
            alpha = min(1.0, self.dt / self.stop_brake_tau)
            self.stop_brake_torque += alpha * (wanted - self.stop_brake_torque)
            for side in ("left", "right"):
                ctrl[self.a_wheel[side]] += self.stop_brake_torque
        else:
            self.stop_brake_int = 0.0
            self.stop_brake_torque = 0.0

        # wheel-reaction feed-forward.  The wheel motor torque reacts on the leg
        # through the hip axis; a *differential* wheel torque therefore splays
        # the two legs unless the hips cancel it.  Add the wheel torque to the
        # hip of the same side.
        if self.wheel_ff:
            for side in ("left", "right"):
                ctrl[self.a_hip[side]] += ctrl[self.a_wheel[side]]

        # leg-angle differential (firmware `anti_crash_`)
        if self.anti_crash_enable:
            dtheta = meas["leg"]["left"][1] - meas["leg"]["right"][1]
            ac = float(np.clip(self.anti_crash(0.0, dtheta, self.dt), -6.0, 6.0))
            # theta increases when hip q decreases. For thetaL > thetaR,
            # increase left q and decrease right q to close the difference.
            ctrl[self.a_hip["left"]] -= sg["Tb"] * ac
            ctrl[self.a_hip["right"]] += sg["Tb"] * ac

        # Report the hip torque this frame actually asks for (the hold-0 mode
        # shows the clamped value here).
        for side in ("left", "right"):
            self.last_rear_tp[side] = float(np.clip(
                ctrl[self.a_hip[side]], -tp_limit, tp_limit))

        # ---- VMC leg-length loop -----------------------------------------
        roll_comp = float(np.clip(60.0 * meas["roll"]
                                  + 3.0 * meas["omega_heading"][0],
                                  -30.0, 30.0))
        self.last_vmc_roll_comp = roll_comp
        for side, sign in (("left", -1.0), ("right", +1.0)):
            L0 = meas["leg"][side][0]
            pid = self.pid_l if side == "left" else self.pid_r
            pid_force = pid(self.leg_length, L0, self.dt)
            self.last_vmc_pid_force[side] = pid_force
            total_force = self.F0_ff + pid_force + sign * roll_comp
            if self.jump_phase == "thrust":
                total_force = JUMP_TOTAL_FORCE_PER_LEG + sign * roll_comp
            elif self.jump_phase == "flight":
                # No ground load exists in flight, so omit gravity feedforward.
                # PID pulls toward 135 mm; at target zero total force makes the
                # motor cancel the passive gas spring instead of re-extending.
                total_force = pid_force
            fn_limit = (JUMP_FN_LIMIT if self.jump_phase == "thrust"
                        else VMC_FN_LIMIT)
            total_force = float(np.clip(total_force, -fn_limit, fn_limit))
            self.last_vmc_total[side] = float(total_force)
            # The passive gas spring supplies part of the requested total leg
            # force; report/control only the remaining motor contribution.
            motor_force = total_force - self.last_gas_force[side]
            ctrl[self.a_leg[side]] = float(np.clip(motor_force,
                                                    -VMC_FN_LIMIT, fn_limit))
        if self.jump_phase == "flight":
            for side in ("left", "right"):
                ctrl[self.a_wheel[side]] = 0.0
                ctrl[self.a_hip[side]] = 0.0
        elif self.jump_phase in ("landing", "recover"):
            # A wheel impact can instantaneously spin both wheel joints and
            # excite the derivative states.  Feeding those raw states straight
            # into the full LQR used to saturate the hip/wheel motors and make
            # the simulation explode.  Re-engage balance progressively while
            # keeping enough authority to arrest pitch.
            blend = (float(np.clip((self.jump_time - 0.12) / 0.80, 0.0, 1.0))
                     if self.jump_phase == "landing" else 1.0)
            for side in ("left", "right"):
                wheel = self.a_wheel[side]
                hip = self.a_hip[side]
                ctrl[wheel] = float(np.clip(blend * ctrl[wheel], -6.0, 6.0))
                ctrl[hip] = float(np.clip(blend * ctrl[hip], -12.0, 12.0))
        # Feed-forward, anti-crash and braking are summed above; make the
        # hardware limits authoritative after every contribution.
        for side in ("left", "right"):
            ctrl[self.a_wheel[side]] = float(np.clip(
                ctrl[self.a_wheel[side]], -WHEEL_TORQUE_LIMIT,
                WHEEL_TORQUE_LIMIT))
            ctrl[self.a_hip[side]] = float(np.clip(
                ctrl[self.a_hip[side]], -tp_limit, tp_limit))
        return ctrl, u

    def step(self):
        # Disabled motors: no actuator, simulated gas-spring, or external force.
        # Keep integrating gravity and contacts so the robot can fall naturally.
        if self.estop:
            self.d.ctrl[:] = 0.0
            self.d.qfrc_applied[:] = 0.0
            self.d.xfrc_applied[:] = 0.0
            self.last_gas_force = {"left": 0.0, "right": 0.0}
            self._update_linkage_visual()
            mujoco.mj_step(self.m, self.d)
            meas = self.measure()
            return dict(t=self.d.time, x=float(self.d.qpos[0]),
                        z=float(self.d.qpos[2]), pitch=meas["pitch"],
                        roll=meas["roll"], vx=0.0,
                        L0=meas["leg"]["left"][0], theta=meas["leg"]["left"][1],
                        TwL=0.0, TwR=0.0, TbL=0.0, TbR=0.0, legF=0.0,
                        vmcFL=0.0, vmcFR=0.0, vmcTbL=0.0, vmcTbR=0.0,
                        legActFL=0.0, legActFR=0.0,
                        jointTauL=0.0, jointTauR=0.0,
                        hubTauL=0.0, hubTauR=0.0, stopBrake=0.0,
                        gasFL=self.last_gas_force["left"],
                        gasFR=self.last_gas_force["right"],
                        rearPose=self.rear_pose_active,
                        rearTpL=0.0, rearTpR=0.0,
                        jumpPhase=self.jump_phase, jumpHeight=self.jump_height)

        if self.rescue_phase not in ("idle", "fault"):
            recovery = self._rescue_step()
            if recovery is not None:
                return recovery

        dt = self.dt
        # ramp the speed command (the analytic model has no friction, so a step
        # in target speed would saturate the wheels)
        if self.speed_ramp > 0:
            err = self.speed_cmd - self.target_speed
            # `speed_ramp` is the acceleration/deceleration limit in m/s^2.
            # Basing this on abs(speed_cmd), as the old implementation did,
            # made a zero-speed command unable to decelerate at all.
            step_max = self.speed_ramp * dt
            self.target_speed += float(np.clip(err, -step_max, step_max))
        else:
            self.target_speed = self.speed_cmd

        if abs(self.speed_cmd) > 1e-9 and not self.jump_active:
            if not self.position_tracking or self.position_hold:
                # Start a fresh local speed-command trajectory from the
                # current pose rather than from a previous parking point.
                self.s = 0.0
                self.s_int = 0.0
            self.position_tracking = True
            self.position_hold = False
        elif self.position_tracking and not self.position_hold:
            # Keep the trajectory reference reached at release; position
            # error now accumulates against this local parking point.
            self.position_hold = True

        meas = self.measure()
        self._update_jump(meas)
        self._apply_gas_springs()

        # yaw loop.  The reference firmware keeps the LQR yaw columns and
        # *adds* a separate yaw-speed PID differentially on top of them.
        #   * no turn command -> the LQR holds the heading (position mode);
        #     this is essential, dropping the yaw position error makes the
        #     heading drift and the legs splay.
        #   * turn command    -> rate mode: the heading setpoint is dragged
        #     along so the LQR does not fight the steering PID.
        steering = abs(self.turn_rate) > 1e-6
        if steering:
            # Bound the heading lead.  The LQR yaw error *is* the steering
            # authority (differential torque ~= K_yaw * lead), so an
            # unlimited target makes the wheel differential saturate and
            # pump the leg swing states.  With the stiff 4000 yaw weight the
            # 0.16 rad cap gives the same ~3.8 N*m differential as the old
            # 400 weight did at 0.5 rad, i.e. 4/6 keeps its turning
            # authority while the heading hold gets 10x stiffer.
            requested_yaw = wrap_angle(self.target_yaw + self.turn_rate * dt)
            lead = wrap_angle(requested_yaw - meas["yaw"])
            if abs(lead) > YAW_TARGET_LEAD_LIMIT:
                requested_yaw = wrap_angle(
                    meas["yaw"] + np.sign(lead) * YAW_TARGET_LEAD_LIMIT)
            self.target_yaw = requested_yaw
        turn = 0.0
        if self.yaw_pid_enable:
            turn = float(np.clip(self.yaw_pid(self.turn_rate, meas["omega"][2], dt),
                                 -self.turn_limit, self.turn_limit))

        x, vx = self.state_vector(meas)
        if steering and self.yaw_pid_enable:
            x[2] = 0.0
        ctrl, u = self.control(meas, x, turn)
        self.d.ctrl[:] = ctrl
        # Update collision-free source meshes from the virtual slide state.
        # Dynamics still use the reduced rigid-leg model.
        self._update_linkage_visual()
        mujoco.mj_step(self.m, self.d)
        return dict(t=self.d.time, x=float(self.d.qpos[0]), z=float(self.d.qpos[2]),
                    pitch=meas["pitch"], roll=meas["roll"], vx=vx,
                    L0=meas["leg"]["left"][0], theta=meas["leg"]["left"][1],
                    TwL=u[0], TwR=u[1], TbL=u[2], TbR=u[3],
                    legF=ctrl[self.a_leg["left"]], turn=turn,
                    # Total VMC force and passive gas-spring contribution.
                    vmcFL=self.last_vmc_total["left"],
                    vmcFR=self.last_vmc_total["right"],
                    gasFL=self.last_gas_force["left"],
                    gasFR=self.last_gas_force["right"],
                    rearPose=self.rear_pose_active,
                    rearTpL=self.last_rear_tp["left"],
                    rearTpR=self.last_rear_tp["right"],
                    vmcTbL=u[2], vmcTbR=u[3],
                    # MuJoCo actuator outputs after ctrlrange/gear processing.
                    legActFL=float(self.d.actuator_force[self.a_leg["left"]]),
                    legActFR=float(self.d.actuator_force[self.a_leg["right"]]),
                    jointTauL=float(self.d.actuator_force[self.a_hip["left"]]),
                    jointTauR=float(self.d.actuator_force[self.a_hip["right"]]),
                    hubTauL=float(self.d.actuator_force[self.a_wheel["left"]]),
                    hubTauR=float(self.d.actuator_force[self.a_wheel["right"]]),
                    stopBrake=self.stop_brake_torque,
                    jumpPhase=self.jump_phase, jumpHeight=self.jump_height)

    def _update_linkage_visual(self):
        self._linkage_visual_countdown -= 1
        if self._linkage_visual_countdown <= 0:
            self.linkage_visual.update(self.d, self.j_slide)
            self._linkage_visual_countdown = self._linkage_visual_stride

    # -------------------------------------------------------------- run
    def run(self, seconds=5.0, log_every=0.1, viewer=False, callback=None):
        steps = int(seconds / self.dt)
        every = max(1, int(log_every / self.dt))
        rows = []
        handle = None
        if viewer:
            import mujoco.viewer
            handle = mujoco.viewer.launch_passive(self.m, self.d)
        for i in range(steps):
            info = self.step()
            if callback is not None:
                callback(i, info)
            if i % every == 0:
                rows.append(info)
            if handle is not None:
                if not handle.is_running():
                    break
                handle.sync()
        if handle is not None:
            handle.close()
        return rows


def main(default_controller="mpc"):
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--speed", type=float, default=0.0)
    ap.add_argument("--leg", type=float, default=LEG_NOMINAL)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--controller", choices=("mpc", "lqr"), default=default_controller)
    ap.add_argument("--signs", type=str, default="",
                    help="override, e.g. th=-1,Tb=1")
    args = ap.parse_args()

    signs = dict(DEFAULT_SIGNS)
    for kv in filter(None, args.signs.split(",")):
        k, v = kv.split("=")
        signs[k] = int(v)

    sim = WheelLegLQR(leg_length=args.leg, target_speed=args.speed, signs=signs,
                      controller=args.controller)
    print(f"{args.controller.upper()}: |K|max={np.abs(sim.K).max():.1f}   "
          f"mass={sim.mass:.2f} kg   F0_ff={sim.F0_ff:.1f} N")
    print(f"{'t':>6}{'x':>9}{'z':>9}{'pitch':>9}{'roll':>9}{'L0':>8}"
          f"{'theta':>9}{'TwL':>9}{'TbL':>9}{'F0':>9}")
    rows = sim.run(args.seconds, viewer=args.view)
    for r in rows:
        print(f"{r['t']:6.2f}{r['x']:9.3f}{r['z']:9.3f}{r['pitch']:9.3f}"
              f"{r['roll']:9.3f}{r['L0']:8.4f}{r['theta']:9.4f}"
              f"{r['TwL']:9.2f}{r['TbL']:9.2f}{r['legF']:9.1f}")
    print("final z =", rows[-1]["z"] if rows else None)


if __name__ == "__main__":
    main()
