"""
Build the MuJoCo model used for the traditional VMC + LQR simulation.

The RL repository only ships an Isaac Sim USD for `wheelbipeV14_2`.  That USD
contains a six-bar closed-chain leg (6 spherical loop joints) which is awkward
to balance and not what the on-target controller assumes.  The LQR derivation
in `Simmulation/get_K_jiao_LQR.m` models the robot as

        body  +  2 rigid pendulum legs (hip torque)  +  2 driving wheels

so this generator keeps every *physical* quantity from the USD (link masses,
centres of mass, inertia tensors, joint locations) but collapses the parallel
linkage into one rigid leg body per side, adds a prismatic leg-length axis for
the VMC force loop, and welds the gimbal into the body.

Outputs:  wheelbipe_lqr.xml  +  meshes_lqr/*.stl
"""
import os
from pathlib import Path

import numpy as np

from urdf_builder import (R_to_quat, body_world_pose, load_dump, mesh_in_body_frame,
                          mesh_rgba, quat_R, write_binary_stl)

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
MESH_DIR = "meshes_lqr"
(OUT / MESH_DIR).mkdir(parents=True, exist_ok=True)

dump = load_dump()
bodies = dump["bodies"]
# --------------------------------------------------------------------- helpers
def link_offset(name):
    """Position of a link origin in the world frame at the default pose."""
    return np.array(bodies[name]["t"])


def lump(names, frame_name):
    """Combine links into one rigid body expressed in `frame_name`'s frame."""
    origin = link_offset(frame_name)
    M = 0.0
    coms, Is = [], []
    for n in names:
        m = bodies[n]["mass"]
        c = np.array(bodies[n]["com"] or [0, 0, 0])
        p = link_offset(n) - origin + c
        I = np.diag(bodies[n]["diag_I"] or [1e-6] * 3)
        M += m
        coms.append((m, p))
        Is.append((m, p, I))
    com = sum(m * p for m, p in coms) / M
    Itot = np.zeros((3, 3))
    for m, p, I in Is:
        d = p - com
        Itot += I + m * (d @ d * np.eye(3) - np.outer(d, d))
    # diagonalise (the lumped body is not exactly axis-aligned)
    w, V = np.linalg.eigh(Itot)
    return dict(mass=M, com=com, inertia=Itot, diag=w, axes=V)


def quat_from_matrix(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def fmt(v):
    return " ".join(f"{float(x):.6g}" for x in v)


def visual_pose_in_frame(link_name, frame_name):
    """Authored link pose expressed in a simplified body's reference frame."""
    link_t, link_q = body_world_pose(link_name, dump)
    frame_t, frame_q = body_world_pose(frame_name, dump)
    frame_R = quat_R(frame_q)
    relative_pos = frame_R.T @ (link_t - frame_t)
    relative_R = frame_R.T @ quat_R(link_q)
    return relative_pos, R_to_quat(relative_R)


def geom_rgba(link_name, alpha=1.0):
    rgba = mesh_rgba(link_name, dump).copy()
    rgba[3] = alpha
    return fmt(rgba)


# ------------------------------------------------------------------ link sets
GUIDE = ("bottom1_guide", "bottom2_guide", "bottom3_guide", "bottom4_guide",
         "front1_guide", "front_guide", "rear_guide")

base_links = ["base_link", "gimbal_yaw_link", "gimbal_pitch_link"]
base_links += [f"left_{g}_link" for g in GUIDE]
base_links += [f"right_{g}_link" for g in GUIDE]

leg_links = {}
for side in ("left", "right"):
    leg_links[side] = [f"{side}_{n}_link" for n in
                       ("rear1", "rear2", "rear2_guide", "spring1", "spring2",
                        "front1", "front2", "front3", "front4")]

wheel_links = {s: f"{s}_wheel_link" for s in ("left", "right")}

base_lump = lump(base_links, "base_link")
leg_lump = {s: lump(leg_links[s], f"{s}_rear1_link") for s in ("left", "right")}
wheel_lump = {s: lump([wheel_links[s]], wheel_links[s]) for s in ("left", "right")}

# geometry
hip = {s: link_offset(f"{s}_rear1_link") for s in ("left", "right")}
wheel_origin = {s: link_offset(wheel_links[s]) for s in ("left", "right")}
LEG_VEC = {s: wheel_origin[s] - hip[s] for s in ("left", "right")}
LEG_LEN = float(np.linalg.norm(LEG_VEC["left"]))
WHEEL_R = 0.06

# ---------------------------------------------------------------------------
# The VMC + LQR derivation models each leg as a uniform pendulum with its CoM
# at half the virtual leg length.  The raw USD linkage is very back-heavy (its
# lumped CoM sits ~95 mm *behind* the hip), which is inconsistent with that
# abstraction and inverts the position/velocity feedback.  Re-place the lumped
# leg CoM on the hip->wheel line, keeping the true mass and a rod-like inertia.
SYMMETRIC_LEG = True
if SYMMETRIC_LEG:
    for s in ("left", "right"):
        lg = leg_lump[s]
        m = lg["mass"]
        d = LEG_VEC[s] / LEG_LEN
        lg["com"] = d * (LEG_LEN / 2.0)
        i_perp = m * (LEG_LEN ** 2 + 0.048 ** 2) / 12.0
        i_axis = m * 0.048 ** 2 / 6.0
        # principal frame: first axis along the leg direction
        e1 = d
        tmp = np.array([0.0, 1.0, 0.0])
        if abs(e1 @ tmp) > 0.9:
            tmp = np.array([1.0, 0.0, 0.0])
        e2 = np.cross(e1, tmp); e2 /= np.linalg.norm(e2)
        e3 = np.cross(e1, e2)
        lg["diag"] = np.array([i_axis, i_perp, i_perp])
        lg["axes"] = np.column_stack([e1, e2, e3])

# ------------------------------------------------------------------- mesh I/O
def export(name):
    pts = mesh_in_body_frame(name, dump)
    if pts is None or len(pts) < 3:
        return False
    write_binary_stl(OUT / MESH_DIR / f"{name}.stl",
                      [list(p) for p in pts])
    return True


mesh_names = []
for n in base_links + sum(leg_links.values(), []) + list(wheel_links.values()):
    if export(n):
        mesh_names.append(n)

# ------------------------------------------------------------------ write MJCF
BASE_Z = float(WHEEL_R - LEG_VEC["left"][2])
lines = []
lines.append('<mujoco model="wheelbipe_lqr">')
lines.append(f'  <compiler angle="radian" meshdir="{MESH_DIR}" autolimits="true"/>')
lines.append('  <option timestep="0.001" solver="Newton" iterations="80"'
             ' cone="elliptic" jacobian="dense" integrator="implicitfast"/>')
lines.append('  <default>')
lines.append('    <geom friction="1.2 0.002 0.0001" solref="0.01 1"'
             ' solimp="0.95 0.99 0.0005"/>')
lines.append('  </default>')
lines.append('  <worldbody>')
lines.append('    <geom name="floor" type="plane" size="8 8 0.1"'
             ' friction="1.5 0.002 0.0001" condim="3" conaffinity="3"'
             ' material="floor_mat"/>')
lines.append('    <geom name="step" type="box" size="1 0.5 0.1" pos="1.8 0 0.1"'
             ' friction="1.5 0.002 0.0001" condim="3" conaffinity="3"'
             ' material="step_mat"/>')
lines.append('    <light pos="0 0 4" dir="0 0 -1" castshadow="false"/>')

# ---- body
lines.append(f'    <body name="base_link" pos="0 0 {BASE_Z:.6g}">')
lines.append('      <freejoint name="root"/>')
lines.append(f'      <inertial pos="{fmt(base_lump["com"])}"'
             f' mass="{base_lump["mass"]:.6g}"'
             f' diaginertia="{fmt(base_lump["diag"])}"'
             f' quat="{fmt(quat_from_matrix(base_lump["axes"]))}"/>')
# A convex primitive is much more stable than using the detailed STL as a
# dynamic collision mesh.  Category 2 collides with the environment's
# conaffinity=3, while conaffinity=0 prevents false contacts with the robot's
# own wheel/leg geoms.  The box follows the main chassis shell but excludes
# low guide brackets that would otherwise give it unrealistically little
# ground clearance.
lines.append('      <geom name="base_collision" type="box"'
             ' size="0.25 0.27 0.085" pos="0.0075 0 -0.04"'
             ' contype="2" conaffinity="0" friction="1.0 0.002 0.0001"'
             ' condim="3" rgba="0.2 0.8 0.25 0.10" group="3"/>')
for n in base_links:
    pos, quat = visual_pose_in_frame(n, "base_link")
    lines.append(f'      <geom type="mesh" mesh="{n}_mesh" contype="0"'
                 f' conaffinity="0" pos="{fmt(pos)}" quat="{fmt(quat)}"'
                 f' rgba="{geom_rgba(n)}"/>')

for side, sy in (("left", 1), ("right", -1)):
    h = hip[side]
    lines.append(f'      <body name="{side}_leg" pos="{fmt(h)}">')
    # hip limits: the USD declares the hip joints as unlimited, but a real
    # wheel-legged robot cannot splay its legs arbitrarily.  +/- 35 deg keeps
    # the leg inside the geometry the LQR was linearised about.
    lines.append(f'        <joint name="{side}_hip_joint" type="hinge"'
                 ' axis="0 1 0" range="-0.61 0.61" armature="0.002" damping="0.05"/>')
    lg = leg_lump[side]
    lines.append(f'        <inertial pos="{fmt(lg["com"])}" mass="{lg["mass"]:.6g}"'
                 f' diaginertia="{fmt(lg["diag"])}"'
                 f' quat="{fmt(quat_from_matrix(lg["axes"]))}"/>')
    for n in leg_links[side]:
        if n not in mesh_names:
            continue
        pos, quat = visual_pose_in_frame(n, f"{side}_rear1_link")
        lines.append(f'        <geom type="mesh" mesh="{n}_mesh"'
                     f' pos="{fmt(pos)}" quat="{fmt(quat)}"'
                     f' contype="0" conaffinity="0" rgba="{geom_rgba(n)}"/>')
    # leg length axis.  The wheel sits ~48 mm outboard of the hip, so the raw
    # hip->wheel direction is tilted ~21 deg sideways.  Sliding along it would
    # give every leg force a lateral component, i.e. a constant disturbance that
    # slowly rolls / splays the robot.  Project the axis onto the fore/aft plane
    # instead, keeping the wheel at its true position.
    d = LEG_VEC[side] / LEG_LEN
    slide_axis = np.array([d[0], 0.0, d[2]])
    slide_axis = slide_axis / np.linalg.norm(slide_axis)
    lines.append(f'        <body name="{side}_leg_ext" pos="{fmt(LEG_VEC[side])}">')
    # q=0 is the 133.9 mm nominal pose.  The asymmetric travel covers the
    # mechanism's 90..350 mm command envelope after accounting for the fixed
    # 48 mm lateral hip-to-wheel offset.  Do not add a passive spring here:
    # the VMC leg PID already supplies stiffness and damping, and a second
    # 3000 N/m spring would halve the commanded extension.
    lines.append(f'          <joint name="{side}_legslide_joint" type="slide"'
                 f' axis="{fmt(slide_axis)}" range="{-0.05:.6g} {0.225:.6g}"'
                 ' damping="40"/>')
    lines.append(f'          <inertial pos="0 0 0" mass="0.001"'
                 ' diaginertia="1e-6 1e-6 1e-6"/>')
    wl = wheel_lump[side]
    lines.append(f'          <body name="{side}_wheel" pos="0 0 0">')
    lines.append(f'            <joint name="{side}_wheel_joint" type="hinge"'
                 ' axis="0 1 0" armature="0.0002" damping="0.0"/>')
    lines.append(f'            <inertial pos="{fmt(wl["com"])}" mass="{wl["mass"]:.6g}"'
                 f' diaginertia="{fmt(wl["diag"])}"/>')
    lines.append(f'            <geom name="{side}_wheel_geom" type="cylinder"'
                 f' size="{WHEEL_R} 0.022" euler="1.5707963268 0 0"'
                 f' pos="{fmt(wl["com"])}" rgba="0.08 0.08 0.08 0"'
                 ' friction="1.5 0.002 0.0001"/>')
    if wheel_links[side] in mesh_names:
        lines.append(f'            <geom type="mesh" mesh="{wheel_links[side]}_mesh"'
                     f' contype="0" conaffinity="0"'
                     f' rgba="{geom_rgba(wheel_links[side])}"/>')
    lines.append('          </body>')
    lines.append('        </body>')
    lines.append('      </body>')

lines.append('    </body>')
lines.append('  </worldbody>')

lines.append('  <visual>')
lines.append('    <global offwidth="1920" offheight="1080"/>')
lines.append('    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.4" specular="0.1 0.1 0.1"/>')
lines.append('    <rgba haze="0.12 0.16 0.23 1"/>')
lines.append('    <map znear="0.05" zfar="40" fogstart="4" fogend="14"/>')
lines.append('    <quality shadowsize="4096"/>')
lines.append('  </visual>')
lines.append('  <statistic center="0 0 0.19" extent="0.55"/>')

lines.append('  <asset>')
lines.append('    <texture name="sky" type="skybox" builtin="gradient"'
             ' rgb1="0.08 0.11 0.17" rgb2="0.28 0.34 0.44" width="512" height="3072"/>')
lines.append('    <texture name="floor_grid" type="2d" builtin="checker"'
             ' rgb1="0.22 0.25 0.30" rgb2="0.30 0.34 0.40" width="512" height="512"/>')
lines.append('    <material name="floor_mat" texture="floor_grid" texrepeat="8 8"'
             ' texuniform="true" reflectance="0.08"/>')
lines.append('    <material name="step_mat" rgba="0.40 0.43 0.49 1"'
             ' specular="0.12" shininess="0.18" reflectance="0.04"/>')
for n in mesh_names:
    lines.append(f'    <mesh name="{n}_mesh" file="{n}.stl"/>')
lines.append('  </asset>')

lines.append('  <actuator>')
for side in ("left", "right"):
    lines.append(f'    <motor name="{side}_wheel_motor"'
                 f' joint="{side}_wheel_joint" gear="1" ctrlrange="-30 30"/>')
    lines.append(f'    <motor name="{side}_hip_motor"'
                 f' joint="{side}_hip_joint" gear="1" ctrlrange="-60 60"/>')
    lines.append(f'    <motor name="{side}_leg_motor"'
                 f' joint="{side}_legslide_joint" gear="1" ctrlrange="-200 300"/>')
lines.append('  </actuator>')

lines.append('  <sensor>')
lines.append('    <framequat name="body_quat" objtype="body" objname="base_link"/>')
lines.append('    <framepos name="body_pos" objtype="body" objname="base_link"/>')
lines.append('    <framelinvel name="body_linvel" objtype="body" objname="base_link"/>')
lines.append('    <frameangvel name="body_angvel" objtype="body" objname="base_link"/>')
lines.append('  </sensor>')
lines.append('</mujoco>')

path = OUT / "wheelbipe_lqr.xml"
with open(path, "w") as f:
    f.write("\n".join(lines))
print("written:", path)
print(f"leg length = {LEG_LEN:.4f} m   base z = {BASE_Z:.4f} m")
print(f"base lump: mass={base_lump['mass']:.3f} kg  com={np.round(base_lump['com'],4)}")
for s in ("left", "right"):
    print(f"{s} leg lump: mass={leg_lump[s]['mass']:.3f} kg "
          f"com={np.round(leg_lump[s]['com'],4)}")
    print(f"{s} wheel: mass={wheel_lump[s]['mass']:.3f} kg")

import mujoco
m = mujoco.MjModel.from_xml_path(str(path))
print("nq", m.nq, "nv", m.nv, "nbody", m.nbody, "nu", m.nu)
