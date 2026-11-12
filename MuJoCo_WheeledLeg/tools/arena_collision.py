"""Turn the field mesh into a set of collision boxes.

    python tools/arena_collision.py [mesh] [out.xml] [resolution]

MuJoCo fills a non-convex mesh with its convex hull, so the arena cannot collide
as a mesh: a tray-shaped field would become one solid block.  Instead this
rasterises triangle interiors and edges (including vertical thin faces), then
covers positive heights with boxes. This is a conservative solid approximation.

Run it again whenever `meshes_lqr/arena.stl` changes.
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_MESH = HERE.parent / "meshes_lqr" / "arena.stl"
DEFAULT_OUT = HERE.parent / "arena_collision.xml"
QUANTUM = 0.01           # round box tops upward, never below the CAD surface


def read_binary_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    count = struct.unpack("<I", raw[80:84])[0]
    body = np.frombuffer(raw, dtype=np.uint8, count=count * 50,
                         offset=84).reshape(count, 50)
    return np.frombuffer(body[:, 12:48].tobytes(),
                         dtype="<f4").reshape(count, 3, 3).astype(np.float64)


def height_map(triangles: np.ndarray, resolution: float):
    """Max height of the mesh over an axis-aligned grid (top-down silhouette)."""
    lo = triangles.reshape(-1, 3).min(axis=0)
    hi = triangles.reshape(-1, 3).max(axis=0)
    nx = int(np.ceil((hi[0] - lo[0]) / resolution))
    ny = int(np.ceil((hi[1] - lo[1]) / resolution))
    height = np.zeros((nx, ny))
    for t in triangles:
        xs, ys, zs = t[:, 0], t[:, 1], t[:, 2]
        det = (xs[1] - xs[0]) * (ys[2] - ys[0]) - (xs[2] - xs[0]) * (ys[1] - ys[0])
        if abs(det) >= 1e-12:
            i0 = max(0, int((xs.min() - lo[0]) / resolution))
            i1 = min(nx - 1, int((xs.max() - lo[0]) / resolution) + 1)
            j0 = max(0, int((ys.min() - lo[1]) / resolution))
            j1 = min(ny - 1, int((ys.max() - lo[1]) / resolution) + 1)
            if i1 >= i0 and j1 >= j0:
                X = lo[0] + ((np.arange(i0, i1 + 1) + 0.5) * resolution)[:, None]
                Y = lo[1] + ((np.arange(j0, j1 + 1) + 0.5) * resolution)[None, :]
                w1 = ((X - xs[0]) * (ys[2] - ys[0]) - (Y - ys[0]) * (xs[2] - xs[0])) / det
                w2 = ((xs[1] - xs[0]) * (Y - ys[0]) - (ys[1] - ys[0]) * (X - xs[0])) / det
                inside = (w1 >= -0.01) & (w2 >= -0.01) & (w1 + w2 <= 1.01)
                z = zs[0] + w1 * (zs[1] - zs[0]) + w2 * (zs[2] - zs[0])
                block = height[i0:i1 + 1, j0:j1 + 1]
                np.maximum(block, np.where(inside, z, 0.0), out=block)
        # Centre sampling misses sub-cell caps and every vertical face.
        for k in range(3):
            p, q = t[k], t[(k + 1) % 3]
            steps = max(1, int(np.ceil(np.linalg.norm(q[:2] - p[:2]) /
                                            (resolution * 0.4))))
            u = np.linspace(0.0, 1.0, steps + 1)
            samples = p[None, :] + u[:, None] * (q - p)[None, :]
            ii = np.clip(np.floor((samples[:, 0] - lo[0]) / resolution).astype(int), 0, nx - 1)
            jj = np.clip(np.floor((samples[:, 1] - lo[1]) / resolution).astype(int), 0, ny - 1)
            np.maximum.at(height, (ii, jj), samples[:, 2])
    return height, lo


def greedy_rectangles(mask: np.ndarray):
    """Cover the True cells with disjoint maximal rectangles (i, j) ranges."""
    pending = mask.copy()
    nx, ny = pending.shape
    for i in range(nx):
        j = 0
        while j < ny:
            if not pending[i, j]:
                j += 1
                continue
            j2 = j
            while j2 + 1 < ny and pending[i, j2 + 1]:
                j2 += 1
            i2 = i
            while i2 + 1 < nx and pending[i2 + 1, j:j2 + 1].all():
                i2 += 1
            pending[i:i2 + 1, j:j2 + 1] = False
            yield i, i2, j, j2
            j = j2 + 1


def build(mesh: Path, out: Path, resolution: float) -> None:
    triangles = read_binary_stl(mesh)
    height, lo = height_map(triangles, resolution)
    deck = 0.0  # RMUL 2027 playing surface is the z=0 collision plane.
    clipped = np.clip(height, 0.0, 0.5)
    quantised = np.ceil((clipped - 1e-8) / QUANTUM).astype(int)
    levels = [round(float(v) * QUANTUM, 3)
              for v in np.unique(quantised)]
    levels = [v for v in levels if v > deck + QUANTUM / 2]

    # MuJoCo's <include> wants well-formed XML, so the geoms live in one static
    # body; it has no joints and therefore does not move.
    lines = ['<body name="arena_collision">']
    # The 2027 CAD floor is exactly z=0, so the model's plane is its collision
    # surface. An empty zero-thickness deck box would make MJCF invalid.
    if deck > 0:
        lines.extend([
            f'  <geom name="arena_deck" type="box"'
            f' pos="{lo[0] + (height.shape[0]*resolution)/2:.4f}'
            f' {lo[1] + (height.shape[1]*resolution)/2:.4f} {deck/2:.4f}"'
            f' size="{(height.shape[0]*resolution)/2:.4f}'
            f' {(height.shape[1]*resolution)/2:.4f} {deck/2:.4f}"'
            ' friction="1.5 0.002 0.0001" condim="3" conaffinity="3"'
            ' rgba="0 0 0 0" group="3"/>'
        ])
    count = 0
    for level in levels:
        mask = quantised == int(round(level / QUANTUM))
        for i, i2, j, j2 in greedy_rectangles(mask):
            x0 = lo[0] + i * resolution
            x1 = lo[0] + (i2 + 1) * resolution
            y0 = lo[1] + j * resolution
            y1 = lo[1] + (j2 + 1) * resolution
            lines.append(
                f'  <geom name="arena_h{int(round(level*1000)):03d}_{count:02d}"'
                f' type="box" pos="{(x0+x1)/2:.4f} {(y0+y1)/2:.4f} {level/2:.4f}"'
                f' size="{(x1-x0)/2:.4f} {(y1-y0)/2:.4f} {level/2:.4f}"'
                ' friction="1.2 0.002 0.0001" condim="3" conaffinity="3"'
                ' rgba="0 0 0 0" group="3"/>')
            count += 1
    lines.append('</body>')
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"{mesh.name} -> {out}")
    print(f"  deck level {deck:.3f} m, {count} obstacle boxes at levels {levels}")


def main() -> None:
    mesh = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_MESH
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_OUT
    resolution = float(sys.argv[3]) if len(sys.argv) > 3 else 0.025
    build(mesh, out, resolution)


if __name__ == "__main__":
    main()
