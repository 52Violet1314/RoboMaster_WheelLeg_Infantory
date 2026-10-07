"""Generate URDF from USD dump."""
import os
from pathlib import Path

from urdf_builder import (load_dump, quat_R, R_to_quat, write_binary_stl,
                         body_world_pose, mesh_in_body_frame, mesh_rgba)

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
(OUT / "meshes").mkdir(parents=True, exist_ok=True)

dump = load_dump()

# 1. export STL meshes (body-local) -> triangle soup of visual mesh
for link_name in dump["bodies"]:
    pts = mesh_in_body_frame(link_name, dump)
    if pts is None or len(pts) < 3:
        continue
    # points are triangle soup (STL import) - each 3 consecutive = triangle
    out = OUT / "meshes" / f"{link_name}.stl"
    write_binary_stl(out, [list(p) for p in pts])

# 2. build URDF
lines = []
lines.append('<?xml version="1.0"?>')
lines.append('<robot name="wheelbipeV14_2">')

def fmt_vec(v):
    return " ".join(f"{float(x):.6g}" for x in v)


def matrix_to_rpy(R):
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if abs(np.cos(pitch)) > 1e-8:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw])

for name, b in dump["bodies"].items():
    lines.append(f'  <link name="{name}">')
    # inertia
    com = b["com"] if b["com"] is not None else [0, 0, 0]
    diag = b["diag_I"] if b["diag_I"] is not None else [1e-4, 1e-4, 1e-4]
    lines.append('    <inertial>')
    lines.append(f'      <origin xyz="{fmt_vec(com)}" rpy="0 0 0"/>')
    lines.append(f'      <mass value="{b["mass"]:.6g}"/>')
    lines.append(f'      <inertia ixx="{diag[0]:.6g}" ixy="0" ixz="0" iyy="{diag[1]:.6g}" iyz="0" izz="{diag[2]:.6g}"/>')
    lines.append('    </inertial>')
    mesh_file = f"meshes/{name}.stl"
    if (OUT / mesh_file).exists():
        rgba = fmt_vec(mesh_rgba(name, dump))
        lines.append(f'    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{mesh_file}"/></geometry><material name="{name}_material"><color rgba="{rgba}"/></material></visual>')
        lines.append(f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{mesh_file}"/></geometry></collision>')
    else:
        lines.append(f'    <visual><origin xyz="0 0 0"/><geometry><box size="0.05 0.05 0.05"/></geometry></visual>')
    lines.append('  </link>')

# joints: parent/child with origin in parent frame = pos0 (b1 at origin)
# URDF joint origin is the joint location in the PARENT link frame.
# USD: pos0 in b0 frame. Since b0..b1 frames align such that b1 origin is at its own frame,
# and body world poses are identity-rotated, joint origin in parent frame = the world position
# of the joint = pos0 (in b0 frame) + t_b0? Wait - need care.
# b0's world pose is (t_b0, q_b0). pos0 is expressed in b0's local frame.
# joint_world = t_b0 + R_b0 @ pos0.
# URDF joint origin is expressed in the PARENT link's local frame: it equals
# the joint's position in parent local frame = R_b0^T (joint_world - t_b0) = pos0.
# So joint origin xyz in parent = pos0, rotation = ? URDF joint rotation convention:
# joint axis is expressed in the joint frame. USD: axis expressed in... the rot0/rot1
# quaternions convert between joint frame and body frames. Since rot0/rot1 were identity,
# the joint axis is expressed directly in b0's (and b1's) frame.
# URDF: joint axis is defined in joint frame; the joint frame is obtained by rotating
# the parent frame by the joint's origin rpy. If we set origin rpy=0 and axis in parent
# frame directly, that is consistent since URDF allows specifying axis in the frame
# after applying the origin's rotation. Simpler: set origin rpy = 0 and axis directly.
import numpy as np
for j in dump["joints"]:
    if j["type"] == "SphericalJoint":
        # closed-loop constraint; cannot express in URDF
        lines.append(f'  <!-- loop closure joint {j["name"]} (ball) from {j["b0"]} to {j["b1"]} not representable in URDF -->')
        continue
    b0 = j["b0"]; b1 = j["b1"]
    if b0 is None or b1 is None:
        continue
    b0t, b0q = body_world_pose(b0, dump)
    b1t, b1q = body_world_pose(b1, dump)
    parent_R = quat_R(b0q)
    child_R = quat_R(b1q)
    origin = parent_R.T @ (b1t - b0t)
    origin_rpy = matrix_to_rpy(parent_R.T @ child_R)
    axis = np.array([1,0,0] if j["axis"] == "X" else ([0,1,0] if j["axis"] == "Y" else [0,0,1]))
    jtype = "revolute" if j["type"] == "RevoluteJoint" else "prismatic"
    lim = ""
    if j["type"] == "RevoluteJoint":
        lo = j["lower"] if j["lower"] is not None else -6.2832
        hi = j["upper"] if j["upper"] is not None else 6.2832
        # USD revolute limits are in degrees
        lo = np.deg2rad(lo); hi = np.deg2rad(hi)
        lim = f' lower="{lo:.4g}" upper="{hi:.4g}" effort="40" velocity="17"'
    else:
        lo = j["lower"] if j["lower"] is not None else 0
        hi = j["upper"] if j["upper"] is not None else 0
        lim = f' lower="{lo:.4g}" upper="{hi:.4g}" effort="100" velocity="1"'
    lines.append(f'  <joint name="{j["name"]}" type="{jtype}">')
    lines.append(f'    <parent link="{b0}"/>')
    lines.append(f'    <child link="{b1}"/>')
    lines.append(f'    <origin xyz="{fmt_vec(origin)}" rpy="{fmt_vec(origin_rpy)}"/>')
    lines.append(f'    <axis xyz="{fmt_vec(axis)}"/>')
    lines.append(f'    <limit{lim}/>')
    lines.append('  </joint>')

lines.append('</robot>')
urdf = "\n".join(lines)
urdf_path = OUT / "wheelbipeV14_2.urdf"
with open(urdf_path, "w") as f:
    f.write(urdf)
print("URDF written:", urdf_path)
print("links:", len(dump["bodies"]), "tree joints:", len(dump["joints"]) - 6)
