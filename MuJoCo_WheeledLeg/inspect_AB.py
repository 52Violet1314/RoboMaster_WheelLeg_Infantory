import numpy as np
import lqr_k as L

np.set_printoptions(precision=4, suppress=True, linewidth=250)

# parameters measured from the generated MuJoCo model
body = dict(R_w=0.06, R_l=0.222, l_c=0.0026, m_w=0.512, m_l=1.056,
            m_b=20.140, I_z=0.226, I_b=0.3742)
LEG = 0.1339

K, A, B, p = L.compute_K(LEG, body)
print("leg length", LEG)
print()
print("A =")
print(A)
print()
print("B =")
print(B)
print()
print("K (u = -K x) =")
print(K)
print()
print("eig(A):", np.round(np.linalg.eigvals(A), 4))
print("eig(A-BK):", np.round(np.linalg.eigvals(A - B @ K), 4))
print()
print("sign pattern of A (nonzero):")
names_x = ["s", "ds", "yaw", "dyaw", "thL", "dthL", "thR", "dthR", "pitch", "dpitch"]
for r in range(10):
    row = [f"{names_x[c]}={A[r, c]:+.3f}" for c in range(10) if abs(A[r, c]) > 1e-9]
    print(f"  d{names_x[r]:6s}/dt: " + ", ".join(row))
print()
names_u = ["TwL", "TwR", "TbL", "TbR"]
for r in range(10):
    row = [f"{names_u[c]}={B[r, c]:+.3f}" for c in range(4) if abs(B[r, c]) > 1e-9]
    if row:
        print(f"  d{names_x[r]:6s}/dt: " + ", ".join(row))
