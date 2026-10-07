"""
Matrix LQR gain computation for the wheel-legged robot.

Python port of  Simmulation/get_K_jiao_LQR.m  (Shangjiao-style 10-state model).

State vector (10):
    s, ds                 natural-frame longitudinal position / velocity
    phi, dphi             yaw angle / rate
    theta_ll, dtheta_ll   left  leg swing-rod angle from vertical (rad) / rate
    theta_lr, dtheta_lr   right leg swing-rod angle from vertical (rad) / rate
    theta_b, dtheta_b     body pitch from horizontal (rad) / rate

Input vector (4):
    T_wl, T_wr            left / right driving wheel torque      [N*m]
    T_bl, T_br            left / right hip (leg swing) torque     [N*m]

The 5 rigid-body equations (3.11)-(3.15) of the referenced derivation are
linear in the angular accelerations, so writing them as

    M(q) * qdd  +  N * x  +  P * u  =  0      =>   qdd = -M^-1 (N x + P u)

gives the linearisation about the upright equilibrium *exactly*:

    d(qdd)/dx = -M^-1 N ,   d(qdd)/du = -M^-1 P

The gain then comes from the continuous-time algebraic Riccati equation

    K = R^-1 B^T P ,  A^T P + P A - P B R^-1 B^T P + Q = 0

which is what MATLAB's `icare` / `lqr` solves.
"""
from __future__ import annotations

import numpy as np
from scipy import linalg

G = 9.81

# index helpers
WL, WR, LL, LR, BODY = 0, 1, 2, 3, 4    # acceleration components
X_LL, X_LR, X_B = 0, 1, 2                  # state columns
U_WL, U_WR, U_BL, U_BR = 0, 1, 2, 3        # input columns


def build_MNP(p: dict):
    """Assemble the 5x5 mass matrix M, 5x3 state matrix N, 5x4 input matrix P."""
    R_w, R_l = p["R_w"], p["R_l"]
    l_l, l_r = p["l_l"], p["l_r"]
    l_wl, l_wr = p["l_wl"], p["l_wr"]
    l_bl, l_br = p["l_bl"], p["l_br"]
    l_c = p["l_c"]
    m_w, m_l, m_b = p["m_w"], p["m_l"], p["m_b"]
    I_w, I_ll, I_lr, I_b, I_z = (p["I_w"], p["I_ll"], p["I_lr"],
                                 p["I_b"], p["I_z"])
    g = p["g"]

    M = np.zeros((5, 5))
    N = np.zeros((5, 3))
    P = np.zeros((5, 4))

    # eqn 1 -- left leg swing
    M[0, WL] = I_w * l_l / R_w + m_w * R_w * l_l + m_l * R_w * l_bl
    M[0, LL] = m_l * l_wl * l_bl - I_ll
    N[0, X_LL] = (m_l * l_wl + m_b * l_l / 2) * g
    P[0, U_BL] = 1.0
    P[0, U_WL] = -(1 + l_l / R_w)

    # eqn 2 -- right leg swing
    M[1, WR] = I_w * l_r / R_w + m_w * R_w * l_r + m_l * R_w * l_br
    M[1, LR] = m_l * l_wr * l_br - I_lr
    N[1, X_LR] = (m_l * l_wr + m_b * l_r / 2) * g
    P[1, U_BR] = 1.0
    P[1, U_WR] = -(1 + l_r / R_w)

    # eqn 3 -- horizontal translation
    k3 = m_w * R_w ** 2 + I_w + m_l * R_w ** 2 + m_b * R_w ** 2 / 2
    M[2, WL] = -k3
    M[2, WR] = -k3
    M[2, LL] = -(m_l * R_w * l_wl + m_b * R_w * l_l / 2)
    M[2, LR] = -(m_l * R_w * l_wr + m_b * R_w * l_r / 2)
    P[2, U_WL] = 1.0
    P[2, U_WR] = 1.0

    # eqn 4 -- body pitch
    k4 = m_w * R_w * l_c + I_w * l_c / R_w + m_l * R_w * l_c
    M[3, WL] = k4
    M[3, WR] = k4
    M[3, LL] = m_l * l_wl * l_c
    M[3, LR] = m_l * l_wr * l_c
    M[3, BODY] = -I_b
    N[3, X_B] = m_b * g * l_c
    P[3, U_WL] = -l_c / R_w
    P[3, U_WR] = -l_c / R_w
    P[3, U_BL] = -1.0
    P[3, U_BR] = -1.0

    # eqn 5 -- yaw
    k5 = (I_z * R_w) / (2 * R_l) + I_w * R_l / R_w
    M[4, WL] = k5
    M[4, WR] = -k5
    M[4, LL] = I_z * l_l / (2 * R_l)
    M[4, LR] = -I_z * l_r / (2 * R_l)
    P[4, U_WL] = -R_l / R_w
    P[4, U_WR] = R_l / R_w

    return M, N, P


def compute_AB(p: dict):
    """Numeric A (10x10) / B (10x4) linearisation."""
    M, N, P = build_MNP(p)
    Minv = np.linalg.inv(M)
    dqdd_dx = -Minv @ N          # 5x3
    dqdd_du = -Minv @ P          # 5x4

    R_w, R_l = p["R_w"], p["R_l"]
    l_l, l_r = p["l_l"], p["l_r"]

    A = np.zeros((10, 10))
    Bm = np.zeros((10, 4))
    for k, col in enumerate((4, 6, 8)):          # theta_ll / theta_lr / theta_b
        A[1, col] = R_w * (dqdd_dx[WL, k] + dqdd_dx[WR, k]) / 2
        A[3, col] = (R_w * (-dqdd_dx[WL, k] + dqdd_dx[WR, k])) / (2 * R_l) \
            - (l_l * dqdd_dx[LL, k]) / (2 * R_l) \
            + (l_r * dqdd_dx[LR, k]) / (2 * R_l)
        for row in (5, 7, 9):
            A[row, col] = dqdd_dx[row // 2, k]
    for h in range(4):
        Bm[1, h] = R_w * (dqdd_du[WL, h] + dqdd_du[WR, h]) / 2
        Bm[3, h] = (R_w * (-dqdd_du[WL, h] + dqdd_du[WR, h])) / (2 * R_l) \
            - (l_l * dqdd_du[LL, h]) / (2 * R_l) \
            + (l_r * dqdd_du[LR, h]) / (2 * R_l)
        for row in (5, 7, 9):
            Bm[row, h] = dqdd_du[row // 2, h]

    # even (0-based) rows are pure integrators (x_dot = v); odd rows keep only
    # the acceleration couplings filled above
    for r in range(10):
        if r % 2 == 0:
            A[r, :] = 0.0
            A[r, r + 1] = 1.0
            Bm[r, :] = 0.0
        else:
            for c in (0, 1, 2, 3, 5, 7, 9):
                A[r, c] = 0.0
    return A, Bm


def lqr_gain(A, B, Q, R):
    """Continuous-time LQR gain  K = R^-1 B^T P."""
    P = linalg.solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


# ---------------------------------------------------------------- model data
def leg_params(leg_length: float, body: dict | None = None) -> dict:
    """Physical parameters for a given fixed leg length (metres)."""
    b = body or {}
    p = dict(
        R_w=b.get("R_w", 0.06),      # driving wheel radius           [m]
        R_l=b.get("R_l", 0.222),     # half track width               [m]
        l_c=b.get("l_c", 0.004),     # body CoM above leg joint axis  [m]
        m_w=b.get("m_w", 0.512),     # wheel mass                     [kg]
        m_l=b.get("m_l", 1.132),     # leg mass                       [kg]
        m_b=b.get("m_b", 19.986),    # body mass                      [kg]
        I_z=b.get("I_z", 0.226),     # yaw inertia                    [kg m^2]
        I_b=b.get("I_b", 0.3742),    # body pitch inertia             [kg m^2]
        g=G,
    )
    L = float(leg_length)
    p["l_l"] = p["l_r"] = L
    p["l_wl"] = p["l_wr"] = L / 2
    p["l_bl"] = p["l_br"] = L / 2
    p["I_w"] = 0.5 * p["m_w"] * p["R_w"] ** 2
    p["I_ll"] = p["I_lr"] = p["m_l"] * (L ** 2 + 0.048 ** 2) / 12.0
    return p


Q_DEFAULT = np.diag([20.0, 1.0, 400.0, 1.0, 35000.0, 10.0,
                     35000.0, 10.0, 40000.0, 1.0])
R_DEFAULT = np.diag([10.0, 10.0, 1.0, 1.0])


def compute_K(leg_length: float, body=None, Q=None, R=None):
    """geometry -> A/B -> K (4x10)."""
    p = leg_params(leg_length, body)
    A, B = compute_AB(p)
    K = lqr_gain(A, B, Q if Q is not None else Q_DEFAULT,
                 R if R is not None else R_DEFAULT)
    return K, A, B, p


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True, linewidth=220)
    for L in (0.150, 0.252, 0.350):
        K, A_, B_, p = compute_K(L)
        print(f"=== leg length {L*1000:.0f} mm ===")
        print("open-loop eigenvalues:", np.round(np.linalg.eigvals(A_), 4))
        print("K =\n", K)
        print("closed-loop eigenvalues:",
              np.round(np.linalg.eigvals(A_ - B_ @ K), 4))
        print()
