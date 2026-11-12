"""Check that exported collision boxes cover the CAD triangle surfaces."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

from arena_collision import read_binary_stl

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    tri = read_binary_stl(ROOT / "meshes_lqr" / "arena.stl")
    geoms = ET.parse(ROOT / "arena_collision.xml").getroot().findall("geom")
    boxes = []
    for geom in geoms:
        pos = np.fromstring(geom.attrib["pos"], sep=" ")
        size = np.fromstring(geom.attrib["size"], sep=" ")
        boxes.append((pos - size, pos + size))

    # Vertices, centroids and edge midpoints expose missing caps and walls.
    points = np.concatenate((tri.reshape(-1, 3), tri.mean(axis=1),
                             (tri[:, 0] + tri[:, 1]) / 2,
                             (tri[:, 1] + tri[:, 2]) / 2,
                             (tri[:, 2] + tri[:, 0]) / 2))
    points = points[points[:, 2] > 0.012]  # lower surfaces use the floor plane
    covered = np.zeros(len(points), dtype=bool)
    for lo, hi in boxes:
        covered |= (((points[:, :2] >= lo[:2] - 1e-5) &
                     (points[:, :2] <= hi[:2] + 1e-5)).all(axis=1) &
                    (points[:, 2] <= hi[2] + 1e-3))
    missing = points[~covered]
    assert not len(missing), f"{len(missing)} uncovered CAD points, e.g. {missing[:5]}"
    print(f"{len(points)} CAD surface samples covered by {len(boxes)} collision boxes")

    # Independent contact probes catch disabled collision flags or bad XML
    # placement even when box coordinates cover the source triangles.
    collision_xml = (ROOT / "arena_collision.xml").read_text(encoding="utf-8")
    for name, x, y, z, vx, threshold in (
        ("west wall", 0.8, 4.0, 0.25, -2.0, 0.1),
        ("highland", 3.0, 5.0, 0.8, 0.0, 0.23),
    ):
        xml = ('<mujoco><option timestep="0.001"/><worldbody>'
               '<geom type="plane" size="8 8 0.1"/>' + collision_xml +
               f'<body pos="{x} {y} {z}"><freejoint/>'
               '<geom type="sphere" size="0.06" density="1000"/>'
               '</body></worldbody></mujoco>')
        model = mujoco.MjModel.from_xml_string(xml)
        state = mujoco.MjData(model)
        state.qvel[0] = vx
        for _ in range(1000):
            mujoco.mj_step(model, state)
        measured = state.qpos[0] if vx else state.qpos[2]
        assert measured > threshold, f"{name} penetration: {measured:.3f}"
        print(f"{name}: final coordinate {measured:.3f} m")


if __name__ == "__main__":
    main()
