"""Planar inverse kinematics for the wheelbipe six-bar leg visuals.

The balance model intentionally uses a rigid virtual leg plus a prismatic
length axis.  This module maps that virtual wheel position back to the source
USD linkage angles, so the original meshes can articulate without putting the
closed-chain constraint forces back into the controller dynamics.
"""
from __future__ import annotations

import numpy as np
import mujoco
from scipy.optimize import least_squares


# All x/z dimensions come directly from the left leg joints in the source USD.
# The right leg is a y-mirror and has identical planar kinematics.
F12 = np.array([0.112650, 0.013046])
F23 = np.array([-0.108790, -0.079941])
F34 = np.array([-0.165050, 0.113860])
F3_LOOP = np.array([-0.117260, 0.066890])
R1_LOOP = np.array([-0.113400, 0.0])
F4_LOOP = np.array([-0.096600, 0.0])

R12 = np.array([-0.210000, 0.0])
R2_LOOP = np.array([-0.047780, 0.046960])
R2_WHEEL = np.array([0.216610, -0.124830])

R1_SPRING = np.array([0.0023551, -0.0449380])
SPRING12 = np.array([-0.0329500, 0.0044022])
SPRING2_LOOP = np.array([-0.133000, 0.0])
R2_SPRING_LOOP = np.array([0.047580, -0.022920])
SPRING2_REST_ANGLE = 0.132820  # source spring2 body quaternion about +Y

REST_WHEEL_XZ = np.array([0.00661001, -0.124830])
SLIDE_AXIS_XZ = REST_WHEEL_XZ / np.linalg.norm(REST_WHEEL_XZ)

# [front1, front2, front3, front4, rear1, rear2, spring1, spring_slide]
# The source export's passive-joint stops only cover roughly 120..325 mm.
# The normal virtual-leg envelope is 90..350 mm, so the visual-only solver
# allows the small additional travel.  The reduced model also permits
# experimental 450 mm virtual-leg commands; the source six-bar has no closed
# solution there, so LinkageVisuals deliberately clamps the mesh at this
# 350 mm endpoint while dynamics continue to the commanded length.
LOWER = np.array([-2.5, -1.3, -0.1, -2.0, -2.5, -0.30, -0.5, -0.015])
UPPER = np.array([+2.5, +0.3, +1.0, +0.3, +2.5, +1.20, +0.1, +0.065])

SLIDE_MIN = -0.050
SLIDE_MAX = +0.225
TABLE_STEP = 0.0025
_TABLE = None


def rotate(vector, angle):
    """Rotate an [x, z] vector about MuJoCo's +Y axis."""
    x, z = vector
    c, s = np.cos(angle), np.sin(angle)
    return np.array([c * x + s * z, -s * x + c * z])


def forward(angles):
    """Return planar link origins/orientations and the four closure points."""
    f1, f2, f3, f4, r1, r2, s1, slide = angles
    tf1 = f1
    pf1 = np.zeros(2)
    pf2 = pf1 + rotate(F12, tf1)
    tf2 = tf1 + f2
    pf3 = pf2 + rotate(F23, tf2)
    tf3 = tf2 + f3
    pf4 = pf3 + rotate(F34, tf3)
    tf4 = tf3 + f4

    pr1 = np.zeros(2)
    tr1 = r1
    pr2 = pr1 + rotate(R12, tr1)
    tr2 = tr1 + r2
    wheel = pr2 + rotate(R2_WHEEL, tr2)

    ps1 = pr1 + rotate(R1_SPRING, tr1)
    ts1 = tr1 + s1
    ps2 = (ps1 + rotate(SPRING12, ts1)
           - rotate(np.array([slide, 0.0]), ts1 + SPRING2_REST_ANGLE))
    ts2 = ts1 + SPRING2_REST_ANGLE

    return {
        "front1": (pf1, tf1), "front2": (pf2, tf2),
        "front3": (pf3, tf3), "front4": (pf4, tf4),
        "rear1": (pr1, tr1), "rear2": (pr2, tr2),
        "spring1": (ps1, ts1), "spring2": (ps2, ts2),
        "wheel": wheel,
        "loop_f3": pf3 + rotate(F3_LOOP, tf3),
        "loop_r1": pr1 + rotate(R1_LOOP, tr1),
        "loop_f4": pf4 + rotate(F4_LOOP, tf4),
        "loop_r2": pr2 + rotate(R2_LOOP, tr2),
        "loop_s2": ps2 + rotate(SPRING2_LOOP, ts2),
        "loop_r2_spring": pr2 + rotate(R2_SPRING_LOOP, tr2),
    }


def residual(angles, wheel_target_xz):
    pose = forward(angles)
    return np.concatenate([
        pose["loop_f3"] - pose["loop_r1"],
        pose["loop_f4"] - pose["loop_r2"],
        pose["loop_s2"] - pose["loop_r2_spring"],
        pose["wheel"] - wheel_target_xz,
    ])


def solve(wheel_target_xz, initial=None):
    """Solve one configuration, preserving the branch selected by ``initial``."""
    guess = np.zeros(8) if initial is None else np.asarray(initial, dtype=float)
    result = least_squares(
        residual, guess, args=(np.asarray(wheel_target_xz, dtype=float),),
        bounds=(LOWER, UPPER), xtol=1e-12, ftol=1e-12, gtol=1e-12,
        max_nfev=300,
    )
    return result.x, float(np.max(np.abs(result.fun))), result.success


def target_from_slide(slide):
    return REST_WHEEL_XZ + float(slide) * SLIDE_AXIS_XZ


def _build_table():
    """Build a branch-continuous 1 mm lookup table once per Python process."""
    global _TABLE
    if _TABLE is not None:
        return _TABLE
    slides = np.linspace(SLIDE_MIN, SLIDE_MAX,
                         round((SLIDE_MAX - SLIDE_MIN) / TABLE_STEP) + 1)
    angles = np.zeros((len(slides), 8))
    zero = int(np.argmin(np.abs(slides)))
    # Preserve the authored USD rest transforms exactly.  Rounded joint-anchor
    # constants disagree by at most 10 micrometres; asking least_squares to
    # remove that export noise would rotate otherwise-correct rest meshes.
    angles[zero] = 0.0
    for indexes in (range(zero + 1, len(slides)), range(zero - 1, -1, -1)):
        previous = zero if indexes.start in (zero + 1, zero - 1) else None
        for index in indexes:
            if previous is None:
                previous = index - 1 if index > zero else index + 1
            q, error, ok = solve(target_from_slide(slides[index]), angles[previous])
            if not ok or error > 1e-5:
                raise RuntimeError(
                    f"linkage IK failed at slide {slides[index]:+.3f}: {error:.3e}")
            angles[index] = q
            previous = index
    _TABLE = slides, angles
    return _TABLE


def _quat_y(angle):
    return np.array([np.cos(0.5 * angle), 0.0, np.sin(0.5 * angle), 0.0])


def _rotation_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _quat_matrix(quaternion):
    w, x, y, z = quaternion
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def _matrix_quat(matrix):
    """Convert a rotation matrix to a normalized [w, x, y, z] quaternion."""
    matrix = np.asarray(matrix)
    trace = np.trace(matrix)
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2
        q = np.array([0.25 * scale,
                      (matrix[2, 1] - matrix[1, 2]) / scale,
                      (matrix[0, 2] - matrix[2, 0]) / scale,
                      (matrix[1, 0] - matrix[0, 1]) / scale])
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            q = np.array([(matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                          (matrix[0, 1] + matrix[1, 0]) / scale,
                          (matrix[0, 2] + matrix[2, 0]) / scale])
        elif index == 1:
            scale = np.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            q = np.array([(matrix[0, 2] - matrix[2, 0]) / scale,
                          (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale])
        else:
            scale = np.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            q = np.array([(matrix[1, 0] - matrix[0, 1]) / scale,
                          (matrix[0, 2] + matrix[2, 0]) / scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale])
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


class LinkageVisuals:
    """Animate source-link meshes while leaving reduced dynamics untouched."""

    LINKS = ("front1", "front2", "front3", "front4", "rear1", "rear2",
             "rear2_guide", "spring1", "spring2")

    def __init__(self, model):
        self.model = model
        self.slides, self.angle_table = _build_table()
        self.geom_ids = {side: {} for side in ("left", "right")}
        for geom_id in range(model.ngeom):
            mesh_id = int(model.geom_dataid[geom_id])
            if mesh_id < 0:
                continue
            mesh_name = model.mesh(mesh_id).name or ""
            for side in ("left", "right"):
                for link in self.LINKS:
                    if mesh_name == f"{side}_{link}_link_mesh":
                        self.geom_ids[side][link] = geom_id
        self.enabled = all(len(self.geom_ids[side]) == len(self.LINKS)
                           for side in ("left", "right"))
        self.states = {"left": np.zeros(8), "right": np.zeros(8)}
        self.geom_offsets = {side: {} for side in ("left", "right")}
        if self.enabled:
            rest = forward(np.zeros(8))
            pr2, tr2 = rest["rear2"]
            rest["rear2_guide"] = (
                pr2 + rotate(np.array([-0.037468, -0.014006]), tr2), tr2)
            for side in ("left", "right"):
                for link in self.LINKS:
                    geom_id = self.geom_ids[side][link]
                    position_xz, angle = rest[link]
                    link_position = np.array([position_xz[0], 0.0, position_xz[1]])
                    link_rotation = _rotation_y(angle)
                    geom_position = model.geom_pos[geom_id].copy()
                    geom_rotation = _quat_matrix(model.geom_quat[geom_id])
                    self.geom_offsets[side][link] = (
                        link_rotation.T @ (geom_position - link_position),
                        link_rotation.T @ geom_rotation,
                    )

    def angles(self, slide):
        slide = float(np.clip(slide, self.slides[0], self.slides[-1]))
        upper = int(np.searchsorted(self.slides, slide, side="right"))
        upper = min(max(upper, 1), len(self.slides) - 1)
        lower = upper - 1
        span = self.slides[upper] - self.slides[lower]
        fraction = (slide - self.slides[lower]) / span
        return ((1.0 - fraction) * self.angle_table[lower]
                + fraction * self.angle_table[upper])

    def update_side(self, side, slide):
        if not self.enabled:
            return
        q = self.angles(slide)
        self.states[side] = q
        poses = forward(q)
        # rear2_guide is a passive child fixed to rear2 at its rest angle.
        pr2, tr2 = poses["rear2"]
        poses["rear2_guide"] = (
            pr2 + rotate(np.array([-0.037468, -0.014006]), tr2), tr2)
        for link in self.LINKS:
            geom_id = self.geom_ids[side][link]
            position, angle = poses[link]
            link_position = np.array([position[0], 0.0, position[1]])
            link_rotation = _rotation_y(angle)
            offset_position, offset_rotation = self.geom_offsets[side][link]
            self.model.geom_pos[geom_id] = (
                link_position + link_rotation @ offset_position)
            self.model.geom_quat[geom_id] = _matrix_quat(
                link_rotation @ offset_rotation)

    def update(self, data, slide_joints):
        if not self.enabled:
            return
        for side in ("left", "right"):
            joint = slide_joints[side]
            slide = data.qpos[self.model.jnt_qposadr[joint]]
            self.update_side(side, slide)

    def angle_degrees(self, side):
        q = np.degrees(self.states[side])
        return {name: float(q[index]) for index, name in enumerate(
            ("front1", "front2", "front3", "front4",
             "rear1", "rear2", "spring1"))}


if __name__ == "__main__":
    q = np.zeros(8)
    for slide in np.linspace(0.0, 0.225, 10):
        q, error, ok = solve(target_from_slide(slide), q)
        length = np.linalg.norm(target_from_slide(slide))
        print(f"slide={slide:+.3f}  planar_length={length:.3f}  "
              f"closure={error:.3e}  ok={ok}  q={np.round(q, 3)}")
