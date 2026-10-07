"""
Numerically linearise the MuJoCo wheel-legged model and compute the LQR gain.

`lqr_k.py` reproduces the analytic derivation of
`Simmulation/get_K_jiao_LQR.m`.  That derivation models each leg as a uniform
pendulum; the USD leg linkage is very back-heavy, so the analytic gain does not
regulate position / velocity well on the converted model.  This module does the
same job the MATLAB script does -- compute A, B from the *model*, then solve the
Riccati equation -- but takes A, B from the MuJoCo model itself.

State (same layout as the analytic one, so the two gains are interchangeable):
    [s, ds, yaw, dyaw, theta_L, dtheta_L, theta_R, dtheta_R, pitch, dpitch]
Input:
    [TwL, TwR, TbL, TbR]   wheel / hip torques
"""
from __future__ import annotations

import numpy as np
import mujoco

import sim_lqr as S

# physical perturbation directions (one per state component)
#   index -> (description, apply(qpos, qvel, delta))
STATE_DESC = ["s (base x)", "ds (body speed)", "yaw", "dyaw",
              "theta_L", "dtheta_L", "theta_R", "dtheta_R", "pitch", "dpitch"]

DELTA = 1e-5


def _rot_y(delta):
    c, s = np.cos(delta), np.sin(delta)
    return np.array([c, 0.0, s, 0.0])


def _rot_z(delta):
    c, s = np.cos(delta), np.sin(delta)
    return np.array([c, 0.0, 0.0, s])


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


class NumericLQR:
    def __init__(self, sim: "S.WheelLegLQR", Q=None, R=None):
        self.sim = sim
        self.m, self.d = sim.m, sim.d
        self.Q = Q if Q is not None else S.__dict__.get("Q_DEFAULT", None)
        self.R = R
        self.K = None
        self.A = None
        self.B = None
        self.J = None

    # ------------------------------------------------------------ helpers
    def _pure_state(self):
        """state vector without touching the controller's integrators."""
        sim = self.sim
        save_s, save_prev = sim.s, dict(sim._prev)
        x, _ = sim.state_vector(sim.measure())
        sim.s, sim._prev = save_s, save_prev
        return x

    def _settle(self, seconds=8.0, use_numeric=False):
        sim = self.sim
        sim.reset()
        for _ in range(int(seconds / sim.dt)):
            sim.step()

    def _apply(self, idx, delta):
        d, m = self.d, self.m
        qpos, qvel = d.qpos, d.qvel
        if idx == 0:
            qpos[0] += delta
        elif idx == 1:
            for side in ("left", "right"):
                qvel[m.jnt_dofadr[self.sim.j_wheel[side]]] += delta / S.WHEEL_R
        elif idx == 2:
            qpos[3:7] = _quat_mul(_rot_z(delta), qpos[3:7])
        elif idx == 3:
            qvel[5] += delta
        elif idx in (4, 6):
            jid = self.sim.j_hip["left" if idx == 4 else "right"]
            qpos[m.jnt_qposadr[jid]] += delta
        elif idx in (5, 7):
            jid = self.sim.j_hip["left" if idx == 5 else "right"]
            qvel[m.jnt_dofadr[jid]] += delta
        elif idx == 8:
            qpos[3:7] = _quat_mul(_rot_y(delta), qpos[3:7])
        elif idx == 9:
            qvel[4] += delta

    # --------------------------------------------------------- linearise
    def linearise(self):
        sim, m, d = self.sim, self.m, self.d
        self._settle()
        # hold the equilibrium control
        meas = sim.measure()
        x0, _ = sim.state_vector(meas)
        ctrl0, _ = sim.control(meas, x0)
        qpos0, qvel0 = d.qpos.copy(), d.qvel.copy()
        dt = sim.dt

        def restore():
            d.qpos[:] = qpos0
            d.qvel[:] = qvel0
            d.ctrl[:] = ctrl0
            mujoco.mj_forward(m, d)

        def xdot(idx=None, delta=0.0, uidx=None, udelta=0.0):
            restore()
            if idx is not None:
                self._apply(idx, delta)
                mujoco.mj_forward(m, d)
            xa = self._pure_state()
            ctrl = ctrl0.copy()
            if uidx is not None:
                ctrl[uidx] += udelta
            d.ctrl[:] = ctrl
            mujoco.mj_step(m, d)
            xb = self._pure_state()
            return (xb - xa) / dt

        f0 = xdot()
        # J[i] = d(state_i) / d(perturbation_i)
        J = np.zeros(10)
        for i in range(10):
            restore()
            self._apply(i, DELTA)
            mujoco.mj_forward(m, d)
            J[i] = (self._pure_state()[i] - x0[i]) / DELTA

        A_p = np.zeros((10, 10))
        for i in range(10):
            A_p[:, i] = (xdot(idx=i, delta=DELTA) - f0) / DELTA
        B_p = np.zeros((10, m.nu))
        for k in range(m.nu):
            B_p[:, k] = (xdot(uidx=k, udelta=DELTA) - f0) / DELTA

        # keep only the four control inputs of the analytic model
        # [TwL, TwR, TbL, TbR]
        sim_ = self.sim
        keep = [sim_.a_wheel["left"], sim_.a_wheel["right"],
                sim_.a_hip["left"], sim_.a_hip["right"]]
        B_p = B_p[:, keep]

        Js = np.where(np.abs(J) < 1e-9, 1.0, J)
        A = A_p * (Js[:, None] / Js[None, :])
        B = B_p * Js[:, None]
        self.J, self.A, self.B = J, A, B
        return A, B

    def compute_K(self, Q=None, R=None):
        A, B = self.linearise()
        Q = Q if Q is not None else np.diag(
            [20.0, 1.0, 400.0, 1.0, 35000.0, 10.0, 35000.0, 10.0, 40000.0, 1.0])
        R = R if R is not None else np.diag([10.0, 10.0, 1.0, 1.0])
        from scipy import linalg
        P = linalg.solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.solve(R, B.T @ P)
        return self.K


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True, linewidth=220)
    sim = S.WheelLegLQR()
    nl = NumericLQR(sim)
    K = nl.compute_K()
    print("numeric J (d state / d perturbation):")
    for i, (dsc, j) in enumerate(zip(STATE_DESC, nl.J)):
        print(f"  {dsc:16s} {j:+.6f}")
    print("\nnumeric A =\n", nl.A)
    print("\nnumeric B =\n", nl.B)
    print("\nnumeric K =\n", K)
    print("\neig(A):", np.round(np.linalg.eigvals(nl.A), 4))
    print("eig(A-BK):", np.round(np.linalg.eigvals(nl.A - nl.B @ K), 4))
