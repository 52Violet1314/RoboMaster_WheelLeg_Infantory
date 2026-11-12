"""Physical recovery regression: no qpos reset or external body assistance."""
import argparse
import numpy as np
import mujoco
import sim_lqr as S


def run(angle, asymmetric=False):
    sim = S.WheelLegLQR(leg_length=0.150)
    sim.d.qpos[:3] = [6.0, 1.8, 0.4]
    a = np.radians(angle)
    sim.d.qpos[3:7] = [np.cos(a/2), 0, np.sin(a/2), 0]
    for side, joint in sim.j_hip.items():
        sim.d.qpos[sim.m.jnt_qposadr[joint]] = 0.4 if asymmetric and side == "right" else -1.2
    mujoco.mj_forward(sim.m, sim.d)
    sim.estop = True
    touched_links = set()
    for _ in range(1000):
        sim.step()
        for contact in sim.d.contact:
            for gid in (contact.geom1, contact.geom2):
                mid = sim.m.geom_dataid[gid]
                if mid >= 0 and sim.m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH:
                    touched_links.add(sim.m.mesh(mid).name)
    assert np.all(sim.d.ctrl == 0) and np.all(sim.d.qfrc_applied == 0)
    before = sim.d.qpos.copy()
    sim.request_stand()
    assert np.array_equal(sim.d.qpos, before), "stand command teleported robot"
    phases = []
    for i in range(46000):
        sim.step()
        assert np.all(sim.d.xfrc_applied == 0), "external assistance"
        if not phases or phases[-1] != sim.rescue_phase:
            phases.append(sim.rescue_phase)
            print(angle, i/1000, sim.rescue_phase, flush=True)
        if sim.rescue_phase in ("idle", "fault"):
            break
    assert sim.rescue_phase == "idle", phases
    for _ in range(5000):
        sim.step()
    meas = sim.measure()
    assert abs(meas["pitch_full"]) < 0.25, meas["pitch_full"]
    assert abs(meas["roll"]) < 0.25, meas["roll"]
    assert all(abs(meas["leg"][s][1]) < 0.25 for s in sim.j_hip), meas["leg"]
    assert all(abs(meas["leg"][s][0] - 0.150) < 0.015 for s in sim.j_hip), meas["leg"]
    assert all(p in phases for p in ("lift", "sweep", "stand", "balance")), phases
    assert any("front" in n or "rear" in n or "bottom" in n for n in touched_links), touched_links
    print("PASS", angle, "legs", [round(meas["leg"][s][0], 4) for s in sim.j_hip],
          "link contacts", sorted(touched_links), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle", type=float, default=120)
    parser.add_argument("--asymmetric", action="store_true")
    args = parser.parse_args()
    run(args.angle, args.asymmetric)
