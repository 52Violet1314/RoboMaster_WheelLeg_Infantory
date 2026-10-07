"""Validate the Python LQR port against the firmware's K_Fixed_Leg150 table."""
import numpy as np
import lqr_k as L

np.set_printoptions(precision=4, suppress=True, linewidth=220)

# parameters taken from  Simmulation/get_K_jiao_LQR.m  "机器人机体与轮部参数"
body = dict(R_w=0.0525, R_l=0.20, l_c=-0.03, m_w=0.3, m_l=0.8, m_b=15.75,
            I_z=0.226)
body["I_b"] = 15.75 * (0.485 ** 2 + 0.152 ** 2) / 12.0

leg = 0.150
K, A, B, p = L.compute_K(leg, body)
print("params:", {k: round(v, 6) for k, v in p.items()})
print()
print("K from port (wheel TORQUE, u = -K x):")
print(K)

# firmware table (wheel FORCE per state, u = +K x)
K_fw = np.array([
    [0.6243, 2.0785, 3.1106, 0.4550, -35.1025, -0.9911, -2.8527, -0.2108, -32.4029, -2.1963],
    [0.6243, 2.0785, -3.1106, -0.4550, -2.8527, -0.2108, -35.1025, -0.9911, -32.4029, -2.1963],
    [-2.4704, -8.1973, 10.1607, 1.5178, 102.7465, 3.0402, -8.1611, 0.3434, -115.8194, -5.6326],
    [-2.4704, -8.1973, -10.1607, -1.5178, -8.1611, 0.3434, 102.7465, 3.0402, -115.8194, -5.6326]])
print()
print("firmware K_Fixed_Leg150:")
print(K_fw)

# convert the port's torque gain to a force gain with the firmware's sign
conv = -K / p["R_w"]
conv[2:, :] *= p["R_w"]     # rows 3/4 are hip torques, leave as torque
print()
print("port converted (-K/R_w for wheel rows):")
print(conv)

print()
print("relative difference (row-normalised):")
for i in range(4):
    a = np.linalg.norm(conv[i]); b = np.linalg.norm(K_fw[i])
    print(f"  row {i}: |port|={a:8.3f}  |fw|={b:8.3f}  cos={conv[i] @ K_fw[i] / (a*b):+.4f}")
