"""Export the audited RMUL 2027 STEP tessellation for MuJoCo.

The source package already transforms CAD millimetres into simulation metres:
X=[0,12], Y=[0,8] on the playing surface and Z=0 at floor level.
Use its face colours and UV convention; do not reorient these coordinates.
"""
from __future__ import annotations

import gzip
import json
import shutil
from collections import defaultdict
from pathlib import Path

import numpy as np

from convert_arena_stl import write_binary_stl

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "arena_source"
OUT = ROOT / "meshes_lqr"

MATERIALS = {
    "paper": (1, 1, 1),
    "white": (1, 1, 1),
    "red": (1, 0, 0),
    "blue": (0, 0.00784314, 1),
    "green": (0, 1, 0),
    "dark": (0.2, 0.2, 0.2),
    "gray": (0.64705883, 0.64705883, 0.64705883),
    "light": (0.87840001, 0.94900001, 1),
    "control": (0.63768799, 0.79598002, 0.9),
}


def material_for(tri: np.ndarray, rgb: np.ndarray) -> str:
    n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
    n /= max(np.linalg.norm(n), 1e-15)
    glyph = rgb.max() - rgb.min() > 0.8 and rgb.max() > 0.99
    if (n[2] > 0.99 and not glyph and
            np.all(tri[:, 0] >= -1e-6) and np.all(tri[:, 0] <= 12.000001) and
            np.all(tri[:, 1] >= -1e-6) and np.all(tri[:, 1] <= 8.000001) and
            np.all(tri[:, 2] >= -0.0001) and np.all(tri[:, 2] <= 0.404)):
        return "paper"
    return min((k for k in MATERIALS if k != "paper"),
               key=lambda k: np.linalg.norm(rgb - MATERIALS[k]))


def main() -> None:
    data = json.loads(gzip.decompress((SOURCE / "step_geometry.json.gz").read_bytes()))
    groups: dict[str, list[np.ndarray]] = defaultdict(list)
    all_triangles = []
    for face in data["faces"]:
        rgb = np.asarray(face["srgb"])
        for tri in face["triangles_m"]:
            tri = np.asarray(tri, dtype=np.float64)
            if np.linalg.norm(np.cross(tri[1] - tri[0], tri[2] - tri[0])) < 1e-12:
                continue
            groups[material_for(tri, rgb)].append(tri)
            all_triangles.append(tri)
    OUT.mkdir(exist_ok=True)
    for name, triangles in groups.items():
        with (OUT / f"arena_{name}.obj").open("w", encoding="ascii") as out:
            for tri in triangles:
                for x, y, z in tri:
                    out.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
            if name == "paper":
                for tri in triangles:
                    for x, y, _ in tri:
                        out.write(f"vt {x/12:.9f} {y/8:.9f}\n")
            for i in range(len(triangles)):
                j = 3*i + 1
                out.write(f"f {j}/{j} {j+1}/{j+1} {j+2}/{j+2}\n" if name == "paper"
                          else f"f {j} {j+1} {j+2}\n")
        print(name, len(triangles))
    write_binary_stl(OUT / "arena.stl", np.asarray(all_triangles),
                     b"RMUL 2027 audited STEP; metres Z up")
    shutil.copyfile(SOURCE / "official_top.png", OUT / "arena_official_top.png")
    print("bounds", np.min(all_triangles, axis=(0, 1)),
          np.max(all_triangles, axis=(0, 1)))


if __name__ == "__main__":
    main()
