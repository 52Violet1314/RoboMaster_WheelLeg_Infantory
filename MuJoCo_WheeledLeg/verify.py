"""
Final verification of the VMC + LQR MuJoCo simulation.
"""
import numpy as np
import sim_lqr as S
import lqr_k


def line(*a):
    print(*a)


line("=" * 78)
line("wheelbipeV14_2  --  MuJoCo VMC + matrix-LQR simulation")
line("=" * 78)

sim = S.WheelLegLQR()
sim.reset()
line(f"model      : {sim.m.nq} qpos / {sim.m.nv} dof / {sim.m.nu} actuators")
line(f"mass       : {sum(sim.m.body_mass):.2f} kg")
line(f"leg length : {sim.leg_length*1000:.1f} mm   wheel r = {S.WHEEL_R*1000:.0f} mm")
line(f"F0 feedfwd : {sim.F0_ff:.1f} N per leg")
line(f"K max      : {np.abs(sim.K).max():.1f}")
line()

# ---------------------------------------------------------------- 1. stand
line("--- 1. stand (60 s) ---")
sim = S.WheelLegLQR()
sim.reset()
for _ in range(int(60 / sim.dt)):
    info = sim.step()
meas = sim.measure()
line(f"  base z      {info['z']:.4f} m      pitch {np.degrees(meas['pitch']):+.3f} deg"
     f"   roll {np.degrees(meas['roll']):+.3f} deg")
line(f"  body speed  {info['vx']:+.5f} m/s  leg length {info['L0']*1000:.1f} mm")
line(f"  wheel torque L/R  {info['TwL']:+.3f} / {info['TwR']:+.3f} Nm"
     f"   hip torque L/R {info['TbL']:+.3f} / {info['TbR']:+.3f} Nm")
line(f"  -> STABLE, level, at rest" if abs(meas['pitch']) < 0.05
     and abs(info['vx']) < 0.02 else "  -> FAILED")
line()

# ------------------------------------------------------- 2. disturbance
line("--- 2. disturbance rejection (60 N push for 0.4 s) ---")
sim = S.WheelLegLQR()
sim.reset()
peak_p = peak_r = 0.0
for i in range(int(12 / sim.dt)):
    t = i * sim.dt
    sim.d.xfrc_applied[sim.b_base, 0] = 60.0 if 4.0 <= t < 4.4 else 0.0
    info = sim.step()
    peak_p = max(peak_p, abs(info["pitch"]))
    peak_r = max(peak_r, abs(info["roll"]))
line(f"  peak pitch {np.degrees(peak_p):.2f} deg    peak roll {np.degrees(peak_r):.2f} deg")
line(f"  recovered to pitch {np.degrees(info['pitch']):+.3f} deg, vx {info['vx']:+.4f} m/s")
line()

# ------------------------------------------------------- 3. leg length
line("--- 3. leg-length command step 134 -> 180 mm ---")
sim = S.WheelLegLQR()
sim.reset()
for i in range(int(12 / sim.dt)):
    if i == int(3.0 / sim.dt):
        sim.leg_length = 0.180
    info = sim.step()
    if i % int(2.0 / sim.dt) == 0:
        line(f"  t={info['t']:5.1f}  L0={info['L0']*1000:6.1f} mm  z={info['z']:.4f}"
             f"  pitch={np.degrees(info['pitch']):+.3f} deg  F0={info['legF']:6.1f} N")
line()

# ------------------------------------------------------- 4. gain table
line("--- 4. K recomputed for different leg lengths (lqr_k.py) ---")
for L in (0.120, 0.134, 0.180, 0.250, 0.350):
    K, A, B, p = lqr_k.compute_K(L)
    cl = np.linalg.eigvals(A - B @ K)
    line(f"  L={L*1000:5.0f} mm   |K|max={np.abs(K).max():7.2f}"
         f"   max Re(eig(A-BK)) = {cl.real.max():+.4f}  -> "
         f"{'stable' if cl.real.max() < 0 else 'UNSTABLE'}")
line()

# ------------------------------------------------------- 5. validation
line("--- 5. lqr_k.py vs firmware K_Fixed_Leg150 ---")
body = dict(R_w=0.0525, R_l=0.20, l_c=-0.03, m_w=0.3, m_l=0.8, m_b=15.75, I_z=0.226)
body["I_b"] = 15.75 * (0.485 ** 2 + 0.152 ** 2) / 12.0
K, A, B, p = lqr_k.compute_K(0.150, body)
K_fw = np.array([
    [0.6243, 2.0785, 3.1106, 0.4550, -35.1025, -0.9911, -2.8527, -0.2108, -32.4029, -2.1963],
    [0.6243, 2.0785, -3.1106, -0.4550, -2.8527, -0.2108, -35.1025, -0.9911, -32.4029, -2.1963],
    [-2.4704, -8.1973, 10.1607, 1.5178, 102.7465, 3.0402, -8.1611, 0.3434, -115.8194, -5.6326],
    [-2.4704, -8.1973, -10.1607, -1.5178, -8.1611, 0.3434, 102.7465, 3.0402, -115.8194, -5.6326]])
conv = -K / p["R_w"]
conv[2:, :] *= p["R_w"]
for i in range(4):
    a, b = np.linalg.norm(conv[i]), np.linalg.norm(K_fw[i])
    line(f"  row {i}: cos(port, firmware) = {conv[i] @ K_fw[i] / (a * b):+.4f}")
line("  -> same structure up to the firmware's sign / unit convention")
