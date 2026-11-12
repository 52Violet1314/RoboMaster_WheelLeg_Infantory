"""Factored explicit MPC policy: table lookup and arithmetic only at runtime.

The offline QP is min .5*(u-v)'H*(u-v), |u_i| <= limit_i,
where v = -gain*x + offset. Each of the 3**4 active sets defines an affine
law and a polyhedral KKT region. All inverses are computed by generate_mpc.py.
The 81 laws share just 16 free-set correction matrices per leg length.
"""
from __future__ import annotations

from itertools import product
from pathlib import Path

import numpy as np

TABLE_PATH = Path(__file__).with_name("mpc_tables.npz")
# Test the unconstrained region first, then faces, edges and vertices.
REGIONS = np.array(sorted(product((0, -1, 1), repeat=4),
                          key=lambda s: sum(v != 0 for v in s)), dtype=np.int8)
FREE_MASKS = np.array([sum(1 << i for i in range(4) if s[i] == 0)
                       for s in REGIONS])


class ExplicitMPC:
    def __init__(self, dt=0.001, table_path=TABLE_PATH):
        with np.load(table_path, allow_pickle=False) as table:
            if int(table["version"]) != 1:
                raise ValueError("Unsupported MPC table version; regenerate tables")
            if not np.isclose(dt, float(table["dt"]), rtol=0, atol=1e-10):
                raise ValueError("MPC sample period differs from offline tables")
            self.lengths = table["lengths"].copy()
            self.gains = table["gain"].copy()
            self.hessians = table["hessian"].copy()
            self.corrections = table["correction"].copy()
            self.horizon = int(table["horizon"])
            self.Q = table["Q"].copy()
        self.last_region = 0
        self.last_checks = 0
        self.index = 0

    def select_length(self, length):
        """Nearest offline model; no interpolation of explicit control laws."""
        if not np.isfinite(length) or not self.lengths[0] <= length <= self.lengths[-1]:
            raise ValueError("Leg length is outside the offline MPC model bank")
        self.index = int(np.argmin(abs(self.lengths - length)))
        return float(self.lengths[self.index])

    @property
    def gain(self):
        return self.gains[self.index]

    def evaluate(self, x, limits=(7.0, 7.0, 10.0, 10.0), offset=None):
        x, limits = np.asarray(x), np.asarray(limits)
        offset = np.zeros(4) if offset is None else np.asarray(offset)
        if (x.shape != (10,) or limits.shape != (4,) or offset.shape != (4,)
                or not np.all(np.isfinite(x)) or not np.all(np.isfinite(limits))
                or not np.all(np.isfinite(offset)) or np.any(limits <= 0)):
            raise ValueError("MPC expects finite x[10], offset[4] and positive limits[4]")
        desired = -self.gain @ x + offset
        h = self.hessians[self.index]
        correction = self.corrections[self.index]

        def candidate(region):
            status = REGIONS[region]
            active = status != 0
            delta = np.where(active, status * limits - desired, 0.0)
            u = desired + correction[FREE_MASKS[region]] @ delta
            u[active] = status[active] * limits[active]
            gradient = h @ (u - desired)
            # Upper-bound gradient <= 0; lower-bound gradient >= 0.
            tol = 2e-5 * (1.0 + np.max(np.abs(h) @ np.abs(u - desired)))
            valid = (np.all(np.abs(u) <= limits + 2e-5)
                     and np.all(status * gradient <= tol))
            return valid, u

        self.last_checks = 1
        valid, u = candidate(self.last_region)
        if valid:
            return np.clip(u, -limits, limits)
        # Batched region membership tests; there is no online QP iteration.
        delta = np.where(REGIONS != 0, REGIONS * limits - desired, 0.0)
        u = desired + np.einsum("rij,rj->ri", correction[FREE_MASKS], delta)
        u = np.where(REGIONS != 0, REGIONS * limits, u)
        gradient = (u - desired) @ h.T
        tol = 2e-5 * (1.0 + np.max(np.abs(u - desired) @ np.abs(h).T, axis=1))
        valid = (np.all(np.abs(u) <= limits + 2e-5, axis=1)
                 & np.all(REGIONS * gradient <= tol[:, None], axis=1))
        matches = np.flatnonzero(valid)
        if not matches.size:
            raise RuntimeError("No explicit MPC region matched (table/numerical fault)")
        self.last_region = int(matches[0])
        self.last_checks += self.last_region + 1
        return np.clip(u[self.last_region], -limits, limits)
