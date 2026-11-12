"""Convert the CAD competition-field STL into the control model's mesh.

    python tools/convert_arena_stl.py <source.STL> [out.STL]

The CAD export (``对抗赛场地.STL``) is millimetres in a Y-up frame.  MuJoCo
wants metres with Z up, and the control model expects the field to start at the
world origin, so this bakes the whole transform into a new STL:

    mm -> m,  (x, y, z)_cad -> (x, z, ymax - y)_mujoco,  min corner at (0, 0, 0)

Result for the RoboMaster field: 12.9 x 8.9 m footprint, 0.42 m walls.
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE.parent / "meshes_lqr" / "arena.stl"


def read_binary_stl(path: Path) -> np.ndarray:
    """Return the (n, 3, 3) triangle array of a binary STL."""
    raw = path.read_bytes()
    count = struct.unpack("<I", raw[80:84])[0]
    if len(raw) != 84 + count * 50:
        raise ValueError(f"{path.name}: not a binary STL ({len(raw)} bytes, "
                         f"{count} triangles)")
    body = np.frombuffer(raw, dtype=np.uint8, count=count * 50,
                         offset=84).reshape(count, 50)
    return np.frombuffer(body[:, 12:48].tobytes(),
                         dtype="<f4").reshape(count, 3, 3).astype(np.float64)


def write_binary_stl(path: Path, triangles: np.ndarray, header: bytes) -> None:
    n = len(triangles)
    normals = np.cross(triangles[:, 1] - triangles[:, 0],
                       triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals),
                        where=lengths > 0)
    records = np.zeros((n, 50), dtype=np.uint8)
    records[:, 0:12] = np.frombuffer(normals.astype("<f4").tobytes(),
                                     dtype=np.uint8).reshape(n, 12)
    records[:, 12:48] = np.frombuffer(triangles.astype("<f4").tobytes(),
                                      dtype=np.uint8).reshape(n, 36)
    with path.open("wb") as fh:
        fh.write(header[:80].ljust(80, b" "))
        fh.write(struct.pack("<I", n))
        fh.write(records.tobytes())


def convert(source: Path, out: Path) -> np.ndarray:
    triangles = read_binary_stl(source)
    lo = triangles.reshape(-1, 3).min(axis=0)
    hi = triangles.reshape(-1, 3).max(axis=0)
    size = hi - lo
    if size[1] > size[0] or size[1] > size[2]:
        raise ValueError(f"unexpected CAD orientation, size = {size}")
    # Y-up millimetres -> Z-up metres, minimum corner at the origin.  A proper
    # rotation about X (+90 deg) keeps the CAD's up direction: (x, y, z) ->
    # (x, -z, y).  Getting this sign wrong flips the field upside down, i.e. the
    # walls hang below the floor.
    x = triangles[:, :, 0]
    y = triangles[:, :, 1]
    z = triangles[:, :, 2]
    moved = np.stack([x - lo[0], hi[2] - z, y - lo[1]], axis=2) * 1e-3
    write_binary_stl(out, moved, b"RoboMaster competition field (m, Z-up)")
    return moved


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    source = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_OUT
    moved = convert(source, out)
    lo = moved.reshape(-1, 3).min(axis=0)
    hi = moved.reshape(-1, 3).max(axis=0)
    print(f"{source.name} -> {out}")
    print(f"  triangles {len(moved)}")
    print(f"  bounds min {np.round(lo, 4)}  max {np.round(hi, 4)}")


if __name__ == "__main__":
    main()
