"""Determine the frame of a freejoint's qvel[3:6]."""
import numpy as np
import mujoco

m = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <body name="b">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
    </body>
  </worldbody>
</mujoco>""")
d = mujoco.MjData(m)

# rotate the body 90 deg about x, then give it angular velocity about world z
c = np.cos(np.pi / 4); s = np.sin(np.pi / 4)
d.qpos[3:7] = [c, s, 0, 0]        # 90 deg about x
d.qvel[3:6] = [0, 0, 1.0]         # some angular velocity
mujoco.mj_forward(m, d)
print("body quat      :", np.round(d.qpos[3:7], 4))
print("qvel[3:6]      :", d.qvel[3:6])
print("cvel (body ang):", np.round(d.cvel[0, 0:3], 4), " <- body-frame angular velocity")
print("cvel[3:6]      :", np.round(d.cvel[0, 3:6], 4), " <- linear part")

# integrate a step and see which world axis the body actually rotates about
q0 = d.qpos[3:7].copy()
mujoco.mj_step(m, d)
q1 = d.qpos[3:7].copy()


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


R0, R1 = quat_to_mat(q0), quat_to_mat(q1)
dR = R1 @ R0.T
# rotation vector of dR
ang = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
axis = np.array([dR[2, 1]-dR[1, 2], dR[0, 2]-dR[2, 0], dR[1, 0]-dR[0, 1]])
if np.linalg.norm(axis) > 1e-12:
    axis /= np.linalg.norm(axis)
print(f"\nover one step the body rotated {ang:.6f} rad about world axis {np.round(axis, 4)}")
print("-> qvel[3:6] is interpreted in the", "WORLD" if abs(axis[2]) > 0.9 else "BODY", "frame")
