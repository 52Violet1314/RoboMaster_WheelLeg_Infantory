"""
Keyboard-driven MuJoCo GUI for the wheel-legged VMC + LQR simulation.

    python view_gui.py                 # start standing still
    python view_gui.py --speed 0.5     # start with a speed command
    python view_gui.py --leg 0.18      # start with a longer leg

------------------------------------------------------------------ controls
  hold 8 / 5     forward / backward trapezoid    (5.0 m/s^2, limit +/- 2.5 m/s)
  hold 4 / 6     turn left / right trapezoid     (5.0 rad/s^2, limit +/- 0.5 rad/s)
  1 / 2 / 3      leg length presets              (150 / 200 / 300 mm)
  hold 0          rear-horizontal leg pose at 135 mm (disabled while holding 8)
  Space          jump (keeps the current drive command)
  X              emergency stop (all torques 0)  -- press again to release
  Z / R          reset/reload the robot at the selected leg preset
  C              cycle camera (free / side / front / top)
  F              toggle camera follow
  V              toggle contact-force display
  T              toggle the on-screen HUD
  H              print the state to the console
  Esc            quit
------------------------------------------------------------------ mouse
  left-drag orbit | right-drag pan | scroll zoom | double-click pick body
"""
from __future__ import annotations

import argparse
import ctypes
import queue
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

import sim_lqr as S

LEG_PRESETS = {"1": 0.150, "2": 0.200, "3": 0.300}
SPEED_ACCEL = 5.0
TURN_ACCEL = 5.0
SPEED_MAX = 2.5
TURN_MAX = 0.5
DRIVE_KEYS = frozenset("8546")
POSE_KEYS = frozenset("0")

# camera presets: (azimuth, elevation, distance, lookat_z)
CAMERAS = {
    "free":  (135.0, -14.0, 1.05, 0.19),
    "side":  (90.0, -8.0, 1.15, 0.19),
    "front": (180.0, -8.0, 1.15, 0.19),
    "top":   (135.0, -70.0, 1.30, 0.19),
}
CAM_ORDER = list(CAMERAS)


class KeyEvents:
    """Small, bounded bridge from MuJoCo's UI thread to the sim thread."""

    _DEBOUNCED = frozenset(" xzcvthfr123")

    def __init__(self, capacity=64, debounce_seconds=0.20):
        self._events = queue.Queue(maxsize=capacity)
        self._last = {}
        self._drive_seen = {}
        self._lock = threading.Lock()
        self._debounce_seconds = debounce_seconds

    def callback(self, keycode):
        """MuJoCo callback: enqueue only; never touch sim/viewer state here."""
        key = _key_char(keycode)
        if key in DRIVE_KEYS | POSE_KEYS:
            with self._lock:
                self._drive_seen[key] = time.monotonic()
        # The 0 pose is sampled from actual press/release state below; do not
        # enqueue OS key-repeat events and crowd out one-shot commands.
        if key in POSE_KEYS:
            return
        if key in self._DEBOUNCED:
            now = time.monotonic()
            if now - self._last.get(key, -float("inf")) < self._debounce_seconds:
                return
            self._last[key] = now
        try:
            self._events.put_nowait(keycode)
        except queue.Full:
            # A key-repeat storm must not make the UI thread wait for physics.
            pass

    def drain(self):
        while True:
            try:
                yield self._events.get_nowait()
            except queue.Empty:
                return

    def fallback_drive_keys(self, timeout=0.12):
        """Best-effort key-repeat fallback for platforms without key polling."""
        now = time.monotonic()
        with self._lock:
            return {key for key, seen in self._drive_seen.items()
                    if now - seen <= timeout}


class DriveKeyPoller:
    """Read real press/release state for the main digits and Windows NumPad."""

    _VK = {
        "0": (ord("0"), 0x60),
        "8": (ord("8"), 0x68), "5": (ord("5"), 0x65),
        "4": (ord("4"), 0x64), "6": (ord("6"), 0x66),
    }

    def __init__(self, events):
        self.events = events
        self._get_key_state = None
        if sys.platform == "win32":
            self._get_key_state = ctypes.WinDLL("user32").GetAsyncKeyState
            self._get_key_state.argtypes = [ctypes.c_int]
            self._get_key_state.restype = ctypes.c_short

    def pressed(self):
        if self._get_key_state is None:
            return self.events.fallback_drive_keys()
        return {key for key, virtual_keys in self._VK.items()
                if any(self._get_key_state(vk) & 0x8000 for vk in virtual_keys)}


def integrate_drive(state, pressed, dt):
    """Integrate held keys into ramped commands; release snaps to zero."""
    dt = float(np.clip(dt, 0.0, 0.05))
    old_speed, old_turn = state["speed"], state["turn"]
    speed_direction = int("8" in pressed) - int("5" in pressed)
    turn_direction = int("4" in pressed) - int("6" in pressed)

    if speed_direction == 0:
        state["speed"] = 0.0
    else:
        if state["speed"] * speed_direction < 0.0:
            state["speed"] = 0.0
        state["speed"] = float(np.clip(
            state["speed"] + speed_direction * SPEED_ACCEL * dt,
            -SPEED_MAX, SPEED_MAX))

    if turn_direction == 0:
        state["turn"] = 0.0
    else:
        if state["turn"] * turn_direction < 0.0:
            state["turn"] = 0.0
        state["turn"] = float(np.clip(
            state["turn"] + turn_direction * TURN_ACCEL * dt,
            -TURN_MAX, TURN_MAX))
    return state["speed"] != old_speed or state["turn"] != old_turn


def rear_pose_requested(pressed):
    """Hold 0 for the rear pose, except when forward key 8 takes priority."""
    return "0" in pressed and "8" not in pressed


def apply_leg_command(sim, target):
    """Apply a leg setpoint immediately; VMC limits the physical actuator."""
    sim.leg_length = float(np.clip(target, S.LEG_MIN, S.LEG_MAX))


class SimulationClock:
    """Bounded real-time accumulator that remains responsive after a stall."""

    def __init__(self, timestep, realtime, max_catch_up=0.05):
        self.timestep = timestep
        self.realtime = realtime
        self.max_catch_up = max_catch_up
        self.reset()

    def reset(self):
        self._last_wall = time.perf_counter()
        self._accumulator = 0.0

    def steps_due(self):
        if self.realtime <= 0:
            return 100
        now = time.perf_counter()
        elapsed = min(max(now - self._last_wall, 0.0), self.max_catch_up)
        self._last_wall = now
        self._accumulator = min(
            self._accumulator + elapsed * self.realtime,
            self.max_catch_up * self.realtime,
        )
        count = int(self._accumulator / self.timestep)
        self._accumulator -= count * self.timestep
        return count


def _key_char(keycode):
    """Normalize GLFW/MuJoCo ASCII and numeric-keypad key codes."""
    if not isinstance(keycode, (int, np.integer)):
        return ""
    keycode = int(keycode)
    # GLFW_KEY_KP_0 .. GLFW_KEY_KP_9 are consecutive (320 .. 329).
    if 320 <= keycode <= 329:
        return str(keycode - 320)
    if 0 <= keycode < 128:
        return chr(keycode).lower()
    return ""


def apply_key(state, keycode):
    """Update command state on the simulation thread and return an action."""
    key = _key_char(keycode)
    # 8/5/4/6 are handled from real-time press/release state in the main loop.
    if key in LEG_PRESETS:
        state["leg"] = LEG_PRESETS[key]
        return "drive"
    if key == " ":
        return "jump"
    if key == "x":
        return "estop"
    if key in ("z", "r"):
        return "reset"
    if key == "c":
        index = CAM_ORDER.index(state["cam"])
        state["cam"] = CAM_ORDER[(index + 1) % len(CAM_ORDER)]
        return "camera"
    if key == "f":
        state["follow"] = not state["follow"]
        return "follow"
    if key == "v":
        state["contacts"] = not state["contacts"]
        return "contacts"
    if key == "t":
        state["hud"] = not state["hud"]
        return "hud"
    if key == "h":
        return "print"
    return None


class HUD:
    """On-screen state readout, drawn with the viewer's `set_texts` overlay."""

    def __init__(self, model):
        self.model = model
        self.lines: list[str] = []
        self.enabled = True

    def set(self, lines):
        self.lines = lines

    def draw(self, viewer):
        if not self.enabled or not self.lines:
            viewer.clear_texts()
            return
        fs = mujoco.mjtFontScale.mjFONTSCALE_150
        gp = mujoco.mjtGridPos.mjGRID_BOTTOMLEFT
        # each line becomes its own overlay box stacked from the bottom up
        texts = [(fs, gp, "", "\n".join(self.lines))]
        viewer.set_texts(texts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.0)
    ap.add_argument("--leg", type=float, default=S.LEG_NOMINAL)
    ap.add_argument("--realtime", type=float, default=1.0,
                    help="1.0 = real time, 0 = maximum responsive speed")
    ap.add_argument("--no-hud", action="store_true")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="close after this many wall-clock seconds (0 = until Esc)")
    args = ap.parse_args()
    if args.realtime < 0 or args.duration < 0:
        ap.error("--realtime and --duration must be >= 0")

    sim = S.WheelLegLQR(leg_length=args.leg, target_speed=args.speed)
    # pure-LQR heading tracking (no separate steering PID): more robust on the
    # converted model, see README section 6
    sim.yaw_pid_enable = False
    # The key integrator owns the visible 1 m/s^2 trapezoid.  This faster inner
    # ramp only softens the discontinuity when release makes cmd zero; from the
    # 1 m/s limit it reaches zero in at most 0.2 s.
    sim.speed_ramp = 5.0
    sim.reset()

    print("=" * 70)
    print("  wheelbipeV14_2   VMC + LQR   keyboard control")
    print("=" * 70)
    print(f"  mass {sim.mass:.2f} kg    leg {sim.leg_length*1000:.1f} mm"
          f"    wheel r {S.WHEEL_R*1000:.0f} mm    |K|max {np.abs(sim.K).max():.1f}")
    print()
    print("  hold 8/5  speed ramp (5.0 m/s^2, limit +-2.5 m/s, release = 0)")
    print("  hold 4/6  turn ramp  (5.0 rad/s^2, limit +-0.5 rad/s, release = 0)")
    print("  1/2/3 leg 150/200/300 mm   hold 0 rear-horizontal @135 mm")
    print("  Space jump (keeps speed)   X  e-stop")
    print("  Z/R reset    C  camera    F  follow    V  contacts    T  HUD    H  print")
    print()

    state = {
        "speed": float(np.clip(args.speed, -SPEED_MAX, SPEED_MAX)),
        "turn": 0.0,
        "leg": sim.leg_length,
        "cam": "free",
        "follow": True,
        "hud": not args.no_hud,
        "contacts": False,
    }
    hud = HUD(sim.m)
    key_events = KeyEvents()
    drive_keys = DriveKeyPoller(key_events)

    def apply_commands():
        was_turning = abs(sim.turn_rate) > 1e-6
        sim.speed_cmd = state["speed"]
        sim.turn_rate = state["turn"]
        if was_turning and abs(sim.turn_rate) <= 1e-6:
            # Do not keep chasing an old integrated heading after the opposite
            # steering key brings the requested rate back to zero.
            sim.target_yaw = sim.measure()["yaw"]

    apply_commands()

    with mujoco.viewer.launch_passive(
            sim.m, sim.d, key_callback=key_events.callback) as v:
        with v.lock():
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False

        def set_camera():
            az, el, dist, lz = CAMERAS[state["cam"]]
            with v.lock():
                v.cam.azimuth = az
                v.cam.elevation = el
                v.cam.distance = dist
                v.cam.lookat[:] = [sim.d.qpos[0], 0.0, lz]

        set_camera()
        v.sync()

        clock = SimulationClock(sim.dt, args.realtime)
        deadline = (time.perf_counter() + args.duration
                    if args.duration > 0 else None)
        n_step = 0
        drive_last_wall = time.perf_counter()
        last_info = {
            "vmcFL": 0.0, "vmcFR": 0.0, "vmcTbL": 0.0, "vmcTbR": 0.0,
            "legActFL": 0.0, "legActFR": 0.0,
            "jointTauL": 0.0, "jointTauR": 0.0,
            "hubTauL": 0.0, "hubTauR": 0.0,
            "stopBrake": 0.0,
            "gasFL": 0.0, "gasFR": 0.0,
            "rearPose": False, "rearTpL": 0.0, "rearTpR": 0.0,
            "jumpPhase": "idle", "jumpHeight": 0.0,
        }

        while v.is_running():
            if deadline is not None and time.perf_counter() >= deadline:
                break
            frame_started = time.perf_counter()
            drive_dt = frame_started - drive_last_wall
            drive_last_wall = frame_started
            pressed = drive_keys.pressed()
            drive_dirty = integrate_drive(state, pressed, drive_dt)
            sim.set_rear_pose(rear_pose_requested(pressed) and not sim.jump_active)
            for keycode in key_events.drain():
                action = apply_key(state, keycode)
                if action == "drive":
                    drive_dirty = True
                elif action == "jump":
                    apply_commands()
                    if sim.request_jump(state["leg"]):
                        print("  -> jump: crouch 150 mm, extend 350 mm, retract 135 mm")
                elif action == "estop":
                    if sim.estop:
                        # A torque-free balancing robot is normally on the
                        # ground by now; re-enabling its controller in that pose
                        # can generate an unstable solver impulse.  Release via
                        # a clean upright reset and a stopped command.
                        state["speed"] = 0.0
                        state["turn"] = 0.0
                        apply_commands()
                        sim.leg_length = state["leg"]
                        sim.set_rear_pose(False)
                        sim.reset()
                        sim.estop = False
                        clock.reset()
                        print("  e-stop released -> robot reset upright")
                    else:
                        sim.estop = True
                        print("  e-stop ENGAGED")
                elif action == "reset":
                    apply_commands()
                    sim.leg_length = state["leg"]
                    sim.set_rear_pose(False)
                    sim.reset()
                    sim.estop = False
                    clock.reset()
                    print("  -> reset")
                elif action == "camera":
                    set_camera()
                    print(f"  camera: {state['cam']}")
                elif action == "follow":
                    print(f"  camera follow {'on' if state['follow'] else 'off'}")
                elif action == "contacts":
                    with v.lock():
                        v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = state["contacts"]
                    print(f"  contact forces {'on' if state['contacts'] else 'off'}")
                elif action == "hud":
                    hud.enabled = state["hud"]
                elif action == "print":
                    meas = sim.measure()
                    link_l = sim.linkage_visual.angle_degrees("left")
                    link_r = sim.linkage_visual.angle_degrees("right")
                    vx = S.WHEEL_R * 0.5 * (meas["wheel_vel"]["left"]
                                            + meas["wheel_vel"]["right"])
                    print(f"  t={sim.d.time:6.2f}  x={sim.d.qpos[0]:+7.3f}"
                          f"  yaw={np.degrees(meas['yaw']):+7.1f} deg"
                          f"  pitch={np.degrees(meas['pitch']):+6.2f} deg"
                          f"  roll={np.degrees(meas['roll']):+6.2f} deg"
                          f"  L0={meas['leg']['left'][0]*1000:.1f} mm"
                          f"  vx={vx:+.3f} m/s")
                    print(f"    VMC Fcmd L/R {last_info['vmcFL']:+7.1f} /"
                          f" {last_info['vmcFR']:+7.1f} N"
                          f"    Tb L/R {last_info['vmcTbL']:+6.2f} /"
                          f" {last_info['vmcTbR']:+6.2f} Nm")
                    print(f"    leg Fact L/R {last_info['legActFL']:+7.1f} /"
                          f" {last_info['legActFR']:+7.1f} N"
                          f"    joint motor L/R {last_info['jointTauL']:+6.2f} /"
                          f" {last_info['jointTauR']:+6.2f} Nm"
                          f"    hub motor L/R {last_info['hubTauL']:+6.2f} /"
                          f" {last_info['hubTauR']:+6.2f} Nm")
                    print(f"    zero-cmd brake {last_info['stopBrake']:+6.2f} Nm")
                    print(f"    gas spring L/R {last_info['gasFL']:+6.1f} /"
                          f" {last_info['gasFR']:+6.1f} N"
                          f"    jump {last_info['jumpPhase']}"
                          f"  height {last_info['jumpHeight']*100:.1f} cm")
                    print(f"    rear pose {'on' if last_info['rearPose'] else 'off'}"
                          f"  PID Tp L/R {last_info['rearTpL']:+6.2f} /"
                          f" {last_info['rearTpR']:+6.2f} Nm")
                    print("    linkage deg  "
                          f"L[f1 {link_l['front1']:+5.1f}, f2 {link_l['front2']:+5.1f}, "
                          f"r1 {link_l['rear1']:+5.1f}, r2 {link_l['rear2']:+5.1f}]  "
                          f"R[f1 {link_r['front1']:+5.1f}, f2 {link_r['front2']:+5.1f}, "
                          f"r1 {link_r['rear1']:+5.1f}, r2 {link_r['rear2']:+5.1f}]")

            if drive_dirty:
                apply_commands()

            for _ in range(clock.steps_due()):
                if not v.is_running():
                    break
                if not sim.jump_active and not sim.rear_pose_active:
                    apply_leg_command(sim, state["leg"])
                last_info = sim.step()
                n_step += 1

            if state["follow"]:
                with v.lock():
                    # Smoothly follow translation without changing the user's
                    # azimuth, elevation or zoom.  F disables this for free pan.
                    v.cam.lookat[0] += 0.18 * (sim.d.qpos[0] - v.cam.lookat[0])
                    v.cam.lookat[1] += 0.18 * (sim.d.qpos[1] - v.cam.lookat[1])

            if hud.enabled:
                meas = sim.measure()
                link_l = sim.linkage_visual.angle_degrees("left")
                link_r = sim.linkage_visual.angle_degrees("right")
                vx = S.WHEEL_R * 0.5 * (meas["wheel_vel"]["left"]
                                        + meas["wheel_vel"]["right"])
                hud.set([
                    f"speed cmd  {state['speed']:+.2f} m/s      "
                    f"actual {vx:+.3f} m/s",
                    f"turn  cmd  {state['turn']:+.2f} rad/s    "
                    f"yaw {np.degrees(meas['yaw']):+6.1f} deg",
                    f"leg   cmd  {state['leg']*1000:5.1f} mm      "
                    f"actual {meas['leg']['left'][0]*1000:5.1f} mm",
                    f"VMC Fcmd L/R {last_info['vmcFL']:+6.1f} / "
                    f"{last_info['vmcFR']:+6.1f} N   "
                    f"Fact {last_info['legActFL']:+6.1f} / "
                    f"{last_info['legActFR']:+6.1f} N",
                    f"VMC Tb   L/R {last_info['vmcTbL']:+6.2f} / "
                    f"{last_info['vmcTbR']:+6.2f} Nm",
                    f"joint motor L/R {last_info['jointTauL']:+6.2f} / "
                    f"{last_info['jointTauR']:+6.2f} Nm   "
                    f"hub motor {last_info['hubTauL']:+6.2f} / "
                    f"{last_info['hubTauR']:+6.2f} Nm",
                    f"zero-cmd brake {last_info['stopBrake']:+6.2f} Nm",
                    f"gas spring L/R {last_info['gasFL']:+6.1f} / "
                    f"{last_info['gasFR']:+6.1f} N   jump {last_info['jumpPhase']} "
                    f"{last_info['jumpHeight']*100:4.1f} cm",
                    f"rear pose {'ON' if last_info['rearPose'] else 'off'}  "
                    f"PID Tp L/R {last_info['rearTpL']:+5.1f} / "
                    f"{last_info['rearTpR']:+5.1f} Nm",
                    f"link q deg  F1 L/R {link_l['front1']:+5.1f} / "
                    f"{link_r['front1']:+5.1f}   R1 L/R "
                    f"{link_l['rear1']:+5.1f} / {link_r['rear1']:+5.1f}",
                    f"pitch {np.degrees(meas['pitch']):+6.2f} deg   "
                    f"roll {np.degrees(meas['roll']):+6.2f} deg   "
                    f"z {sim.d.qpos[2]:.3f} m",
                    ("E-STOP ENGAGED" if sim.estop else
                     f"cam {state['cam']}   follow {'on' if state['follow'] else 'off'}"),
                ])
            hud.draw(v)
            v.sync()
            # Limit redraw work and give the UI thread regular time slices.
            remaining = 1.0 / 60.0 - (time.perf_counter() - frame_started)
            if remaining > 0:
                time.sleep(remaining)

    print(f"  window closed ({n_step} physics steps)")


if __name__ == "__main__":
    main()
