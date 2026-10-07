"""
Generate MJCF (and STL collision meshes) from the wheelbipeV14_2 USD dump.

Key differences vs. naive conversion:
  * revolute limits are degrees in USD -> rad
  * collision enabled ONLY on wheels + floor (other links are visuals;
    this avoids self-collision on a closed-chain mechanism and matches the
    Isaac Lab cfg enabled_self_collisions=False)
  * spherical loop joints -> <equality><connect> with sites
  * rear2/prismatic spring joints get a suspension spring (Isaac implicit
    spring joint approximated by joint stiffness+springref)
"""
import os
import numpy as np

from urdf_builder import load_dump, mesh_rgba, quat_R

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
os.makedirs(OUT, exist_ok=True)
os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)

dump = load_dump()

WHEEL_R = 0.06          # measured from mesh
BASE_Z = 0.185

# rear2 spring: rear2 joint is the one carrying the wheel
# suspension spring is approximated on the *rear2* joint (leg length axis)
SPRING_DAMP = 3.0
SPRING_K = 200.0        # Nm/rad, approximate implicit spring for leg compliance
SPRING_REF = 0.0        # neutral

actuated = {
    "left_wheel_joint":  ("L_wheel", 6.0, 0.2, 0.0),
    "right_wheel_joint": ("R_wheel", 6.0, 0.2, 0.0),
    "left_rear1_joint":  ("L_rear1", 40.0, 60.0, 2.0),
    "right_rear1_joint": ("R_rear1", 40.0, 60.0, 2.0),
    "left_front1_joint": ("L_front1", 40.0, 60.0, 2.0),
    "right_front1_joint":("R_front1", 40.0, 60.0, 2.0),
    "gimbal_yaw_joint":  ("gimbal_yaw", 2.0, 0.0, 0.5),
    "gimbal_pitch_joint":("gimbal_pitch", 10.0, 20.0, 0.5),
}


def fmt(v):
    return " ".join(f"{float(x):.6g}" for x in v)


def rgba(name, alpha=1.0):
    color = mesh_rgba(name, dump).copy()
    color[3] = alpha
    return fmt(color)


def quat_conjugate(q):
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


tree_joints = [j for j in dump["joints"] if j["type"] != "SphericalJoint"]
loop_joints = [j for j in dump["joints"] if j["type"] == "SphericalJoint"]
parent_of = {j["b1"]: j for j in tree_joints}
children = {}
for j in tree_joints:
    children.setdefault(j["b0"], []).append(j)

bodies = dump["bodies"]

lines = []
lines.append('<mujoco model="wheelbipeV14_2">')
lines.append('  <compiler angle="radian" meshdir="meshes" autolimits="true"/>')
lines.append('  <option timestep="0.002" solver="Newton" iterations="60" cone="elliptic" jacobian="dense"/>')
lines.append('  <default>')
lines.append('    <joint armature="0.0005" damping="0.05"/>')
lines.append('    <geom friction="1.2 0.002 0.0001" solref="0.02 1" solimp="0.9 0.95 0.001"/>')
lines.append('    <motor ctrlrange="-100 100" ctrllimited="true"/>')
lines.append('  </default>')
lines.append('  <worldbody>')
lines.append('    <geom name="floor" type="plane" size="6 6 0.1" pos="0 0 0"'
             ' friction="1.5 0.002 0.0001" material="floor_mat"/>')
lines.append('    <light pos="0 0 4" dir="0 0 -1"/>')

wheel_links = {n for n in bodies if n.endswith("wheel_link")}
guide_like = {"guide_link", "spring1_link", "spring2_link"}


def emit(bname, joint, depth):
    ind = "    " + "  " * depth
    b = bodies[bname]
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    if joint:
        # USD body transforms describe the actual rest pose.  Keeping only the
        # joint translation loses the spring2 link's 7.6 degree orientation and
        # starts the closed-chain equality constraints 17.7 mm apart.
        parent = bodies[joint["b0"]]
        parent_t = np.asarray(parent["t"], dtype=float)
        parent_q = np.asarray(parent["q"], dtype=float)
        child_t = np.asarray(b["t"], dtype=float)
        child_q = np.asarray(b["q"], dtype=float)
        pos = quat_R(parent_q).T @ (child_t - parent_t)
        quat = quat_multiply(quat_conjugate(parent_q), child_q)
        quat /= np.linalg.norm(quat)
    else:
        pos = [0, 0, BASE_Z]
    mass = b["mass"]
    com = b["com"] if b["com"] else [0, 0, 0]
    diag = b["diag_I"] if b["diag_I"] else [1e-4, 1e-4, 1e-4]
    inertial_q = b.get("inertial_q") or [1, 0, 0, 0]
    out = []
    if bname == "base_link":
        out.append(f'{ind}<body name="{bname}" pos="0 0 {BASE_Z}">')
        out.append(f'{ind}  <freejoint name="root"/>')
    else:
        quat_attr = "" if np.allclose(quat, [1, 0, 0, 0], atol=1e-8) else f' quat="{fmt(quat)}"'
        out.append(f'{ind}<body name="{bname}" pos="{fmt(pos)}"{quat_attr}>')
    out.append(f'{ind}  <inertial pos="{fmt(com)}" mass="{mass:.6g}"'
               f' diaginertia="{fmt(diag)}" quat="{fmt(inertial_q)}"/>')
    if bname in wheel_links:
        out.append(f'{ind}  <geom type="cylinder" size="{WHEEL_R} 0.022"'
                   f' euler="1.5707963268 0 0" pos="{fmt(com)}"'
                   ' rgba="0.08 0.08 0.08 0"/>')
        out.append(f'{ind}  <geom type="mesh" mesh="{bname}_mesh" contype="0"'
                   f' conaffinity="0" rgba="{rgba(bname)}"/>')
    else:
        out.append(f'{ind}  <geom type="mesh" mesh="{bname}_mesh" contype="0"'
                   f' conaffinity="0" rgba="{rgba(bname)}"/>')
    if joint is not None:
        jname = joint["name"]
        if joint["type"] == "PrismaticJoint":
            axis = np.array([1, 0, 0] if joint["axis"] == "X" else ([0, 1, 0] if joint["axis"] == "Y" else [0, 0, 1]))
            lo = joint["lower"] if joint["lower"] is not None else -0.02
            hi = joint["upper"] if joint["upper"] is not None else 0.07
            out.append(f'{ind}  <joint name="{jname}" type="slide" axis="{fmt(axis)}" range="{lo:.6g} {hi:.6g}" stiffness="1800" damping="80" springref="0.025"/>')
        else:
            axis = np.array([0, 1, 0] if joint["axis"] == "Y" else ([1, 0, 0] if joint["axis"] == "X" else [0, 0, 1]))
            lo = joint["lower"]
            hi = joint["upper"]
            rng = ""
            if lo is not None and hi is not None and np.isfinite(lo) and np.isfinite(hi):
                rng = f' range="{np.deg2rad(lo):.6g} {np.deg2rad(hi):.6g}"'
            arm = 0.0015 if jname in ("left_rear1_joint", "right_rear1_joint", "left_front1_joint", "right_front1_joint") else 0.0004
            out.append(f'{ind}  <joint name="{jname}" type="hinge" axis="{fmt(axis)}"{rng} armature="{arm}"/>')
    for j in children.get(bname, []):
        out.extend(emit(j["b1"], j, depth + 1))
    for lj in loop_joints:
        if lj["b0"] == bname:
            out.append(f'{ind}  <site name="{lj["name"]}_s0" pos="{fmt(lj["pos0"])}"/>')
        if lj["b1"] == bname:
            out.append(f'{ind}  <site name="{lj["name"]}_s1" pos="{fmt(lj["pos1"])}"/>')
    out.append(f'{ind}</body>')
    return out


lines.extend(emit("base_link", None, 0))
lines.append('  </worldbody>')

lines.append('  <equality>')
for lj in loop_joints:
    lines.append(f'    <connect name="{lj["name"]}" site1="{lj["name"]}_s0" site2="{lj["name"]}_s1" solimp="0.99 0.995 0.0001" solref="0.004 1"/>')
lines.append('  </equality>')

lines.append('  <visual>')
lines.append('    <global offwidth="1920" offheight="1080"/>')
lines.append('    <headlight diffuse="0.75 0.75 0.75" ambient="0.35 0.38 0.42" specular="0.15 0.15 0.15"/>')
lines.append('    <rgba haze="0.12 0.16 0.23 1"/>')
lines.append('    <map znear="0.05" zfar="40" fogstart="4" fogend="14"/>')
lines.append('  </visual>')

lines.append('  <asset>')
lines.append('    <texture name="sky" type="skybox" builtin="gradient"'
             ' rgb1="0.08 0.11 0.17" rgb2="0.28 0.34 0.44" width="512" height="3072"/>')
lines.append('    <texture name="floor_grid" type="2d" builtin="checker"'
             ' rgb1="0.22 0.25 0.30" rgb2="0.30 0.34 0.40" width="512" height="512"/>')
lines.append('    <material name="floor_mat" texture="floor_grid" texrepeat="8 8"'
             ' texuniform="true" reflectance="0.08"/>')
for name in bodies:
    lines.append(f'    <mesh name="{name}_mesh" file="{name}.stl"/>')
lines.append('  </asset>')

lines.append('  <actuator>')
for jn, (act, lim, kp, kd) in actuated.items():
    if jn.endswith("wheel_joint"):
        lines.append(f'    <motor name="{act}" joint="{jn}" gear="1" ctrlrange="{-lim} {lim}"/>')
    elif jn in ("left_rear1_joint", "right_rear1_joint", "left_front1_joint", "right_front1_joint"):
        lines.append(f'    <motor name="{act}" joint="{jn}" gear="1" ctrlrange="{-lim} {lim}"/>')
    else:
        lines.append(f'    <motor name="{act}" joint="{jn}" gear="1" ctrlrange="{-lim} {lim}"/>')
lines.append('  </actuator>')

lines.append('</mujoco>')

path = os.path.join(OUT, "wheelbipeV14_2.xml")
with open(path, "w") as f:
    f.write("\n".join(lines))
print("written:", path)

import mujoco
m = mujoco.MjModel.from_xml_path(path)
print("nq", m.nq, "nv", m.nv, "nbody", m.nbody, "neq", m.neq, "nu", m.nu)
