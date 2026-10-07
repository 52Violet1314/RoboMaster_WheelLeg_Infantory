"""Shared utilities: USD dump JSON -> meshes / transforms."""
import json
import os
from pathlib import Path
import struct
import tempfile
import numpy as np

DUMP = Path(os.environ.get(
    "WHEELBIPE_DUMP",
    Path(tempfile.gettempdir()) / "wheelbipe_mujoco" / "usd_dump.json",
)).resolve()


def load_dump():
    if not DUMP.is_file():
        raise FileNotFoundError(
            f"USD conversion cache not found: {DUMP}. Run usd_dump_full.py first."
        )
    with DUMP.open(encoding="utf-8") as f:
        return json.load(f)


def quat_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


def R_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr+1.0)*2
        w = 0.25*s; x = (R[2,1]-R[1,2])/s; y = (R[0,2]-R[2,0])/s; z = (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2
        w = (R[2,1]-R[1,2])/s; x = 0.25*s; y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2
        w = (R[0,2]-R[2,0])/s; x = (R[0,1]+R[1,0])/s; y = 0.25*s; z = (R[1,2]+R[2,1])/s
    else:
        s = np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2
        w = (R[1,0]-R[0,1])/s; x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s; z = 0.25*s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def write_binary_stl(path, points, faces=None):
    """Write binary STL. points must be flat list of triangle vertex positions."""
    n = len(points) // 3
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", n))
        for i in range(0, len(points), 3):
            p0 = np.array(points[i]); p1 = np.array(points[i+1]); p2 = np.array(points[i+2])
            n_ = np.cross(p1-p0, p2-p0)
            ln = np.linalg.norm(n_)
            if ln > 1e-12:
                n_ = n_/ln
            f.write(struct.pack("<3f", *n_))
            f.write(struct.pack("<3f", *p0))
            f.write(struct.pack("<3f", *p1))
            f.write(struct.pack("<3f", *p2))
            f.write(struct.pack("<H", 0))


def mesh_world_points(link_name, dump):
    """Return visual triangle vertices in the authored USD world frame."""
    points = mesh_in_body_frame(link_name, dump)
    if points is None:
        return None
    translation, quaternion = body_world_pose(link_name, dump)
    return (quat_R(quaternion) @ points.T).T + translation


def _legacy_mesh_world_points(link_name, dump):
    """Read caches produced by the original converter for migration only."""
    vm = dump["vis_meshes"].get(link_name)
    if vm is None:
        return None
    pts = np.array(vm["points"])
    t = np.array(vm["t"]); q = np.array(vm["q"]); s = np.array(vm["s"])
    pts = pts * s
    R = quat_R(q)
    pts = (R @ pts.T).T + t
    return pts


def body_world_pose(name, dump):
    b = dump["bodies"][name]
    return np.array(b["t"]), np.array(b["q"])


def mesh_in_body_frame(link_name, dump):
    """Return visual triangle vertices expressed in the rigid body's frame."""
    vm = dump["vis_meshes"].get(link_name)
    if vm is None:
        return None
    if "points_body" in vm:
        return np.asarray(vm["points_body"], dtype=float)
    # Backward-compatible fallback for a legacy cache.  New dumps should never
    # enter this branch because the old world/body interpretation was wrong.
    pts = _legacy_mesh_world_points(link_name, dump)
    t, q = body_world_pose(link_name, dump)
    R = quat_R(q)
    return (R.T @ (pts - t).T).T


def mesh_rgba(link_name, dump):
    vm = dump["vis_meshes"].get(link_name) or {}
    return np.asarray(vm.get("rgba", [0.72, 0.75, 0.80, 1.0]), dtype=float)
