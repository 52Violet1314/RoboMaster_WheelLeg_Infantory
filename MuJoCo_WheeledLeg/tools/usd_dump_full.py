"""Extract the wheelbipeV14_2 USD into a portable JSON conversion cache.

The visual mesh vertices in ``wheelbipeV14_2_base.usd`` are already expressed
in each link's local frame.  The old converter treated them as world-space
vertices and subtracted the body pose a second time, which displaced almost
every rendered part.  This extractor composes the mesh-node and visual-wrapper
transforms exactly once and stores body-local triangle vertices.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics


DEFAULT_SOURCE = Path(
    r"D:\wheeled-legged_RL\source\agent_world\agent_world\assets"
    r"\usd_files\wheelbipeV14_2_1"
)
SOURCE = Path(os.environ.get("WHEELBIPE_USD_DIR", DEFAULT_SOURCE)).resolve()
MAIN = SOURCE / "wheelbipeV14_2.usd"
BASE_LAYER = SOURCE / "configuration" / "wheelbipeV14_2_base.usd"
DUMP_PATH = Path(os.environ.get(
    "WHEELBIPE_DUMP",
    Path(tempfile.gettempdir()) / "wheelbipe_mujoco" / "usd_dump.json",
)).resolve()


def quat_list(quaternion):
    imaginary = quaternion.GetImaginary()
    return [float(quaternion.GetReal()), *(float(x) for x in imaginary)]


def quat_to_R(quaternion):
    w, x, y, z = quaternion
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def mat_to_tq(matrix):
    translation = np.array(matrix.ExtractTranslation(), dtype=float)
    quaternion = matrix.ExtractRotation().GetQuaternion()
    return translation, np.array(quat_list(quaternion), dtype=float)


def spec_xform(spec):
    """Read the common translate/orient/scale transform authored on a prim."""
    def value(name, default):
        attribute = spec.attributes.get(name)
        return attribute.default if attribute is not None else default

    translation = np.array(value("xformOp:translate", [0, 0, 0]), dtype=float)
    orient_attr = spec.attributes.get("xformOp:orient")
    orientation = (quat_list(orient_attr.default)
                   if orient_attr is not None else [1, 0, 0, 0])
    scale = np.array(value("xformOp:scale", [1, 1, 1]), dtype=float)
    return translation, np.array(orientation, dtype=float), scale


def apply_spec_xform(points, spec):
    translation, quaternion, scale = spec_xform(spec)
    return (quat_to_R(quaternion) @ (points * scale).T).T + translation


def material_rgba(layer, visual_spec):
    relation = visual_spec.relationships.get("material:binding")
    if relation is None:
        return [0.72, 0.75, 0.80, 1.0]
    targets = relation.targetPathList.GetAddedOrExplicitItems()
    if not targets:
        return [0.72, 0.75, 0.80, 1.0]
    material = layer.GetPrimAtPath(targets[0])
    if material is None:
        return [0.72, 0.75, 0.80, 1.0]
    shader = next((child for child in material.nameChildren
                   if child.typeName == "Shader"), None)
    if shader is None:
        return [0.72, 0.75, 0.80, 1.0]
    color_attr = shader.attributes.get("inputs:diffuse_color_constant")
    color = list(color_attr.default) if color_attr is not None else [0.72, 0.75, 0.80]
    return [*(float(x) for x in color), 1.0]


if not MAIN.is_file() or not BASE_LAYER.is_file():
    raise FileNotFoundError(
        f"wheelbipeV14_2 USD not found under {SOURCE}. "
        "Set WHEELBIPE_USD_DIR to the wheelbipeV14_2_1 directory."
    )

stage = Usd.Stage.Open(str(MAIN))
root = stage.GetDefaultPrim()

# 1. Rigid bodies: physical parameters and authored rest poses.
body_info = {}
for prim in root.GetChildren():
    if prim.GetName() in ("joints", "loop_joints", "Looks"):
        continue
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation, quaternion = mat_to_tq(transform)
    mass_api = UsdPhysics.MassAPI(prim)
    mass = mass_api.GetMassAttr().Get()
    diag_inertia = (list(mass_api.GetDiagonalInertiaAttr().Get())
                    if mass_api.GetDiagonalInertiaAttr().HasAuthoredValue() else None)
    center_of_mass = (list(mass_api.GetCenterOfMassAttr().Get())
                      if mass_api.GetCenterOfMassAttr().HasAuthoredValue() else None)
    principal_axes = (quat_list(mass_api.GetPrincipalAxesAttr().Get())
                      if mass_api.GetPrincipalAxesAttr().HasAuthoredValue()
                      else [1, 0, 0, 0])
    body_info[prim.GetName()] = {
        "t": [round(float(x), 8) for x in translation],
        "q": [round(float(x), 8) for x in quaternion],
        "mass": float(mass) if mass is not None else None,
        "diag_I": diag_inertia,
        "com": center_of_mass,
        "inertial_q": principal_axes,
    }

# 2. Joints and loop-closure anchors.
joint_info = []
supported = ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsSphericalJoint")
for prim in Usd.PrimRange(root):
    type_name = prim.GetTypeName()
    if type_name not in supported:
        continue
    joint = UsdPhysics.Joint(prim)
    body0 = joint.GetBody0Rel().GetTargets()
    body1 = joint.GetBody1Rel().GetTargets()
    info = {
        "name": prim.GetName(),
        "type": type_name.replace("Physics", ""),
        "b0": body0[0].GetPrimPath().name if body0 else None,
        "b1": body1[0].GetPrimPath().name if body1 else None,
        "pos0": list(joint.GetLocalPos0Attr().Get()),
        "pos1": list(joint.GetLocalPos1Attr().Get()),
        "rot0": quat_list(joint.GetLocalRot0Attr().Get()),
        "rot1": quat_list(joint.GetLocalRot1Attr().Get()),
    }
    if type_name == "PhysicsRevoluteJoint":
        typed = UsdPhysics.RevoluteJoint(prim)
        info["axis"] = str(typed.GetAxisAttr().Get())
        lower, upper = typed.GetLowerLimitAttr().Get(), typed.GetUpperLimitAttr().Get()
        info["lower"] = float(lower) if np.isfinite(lower) else None
        info["upper"] = float(upper) if np.isfinite(upper) else None
    elif type_name == "PhysicsPrismaticJoint":
        typed = UsdPhysics.PrismaticJoint(prim)
        info["axis"] = str(typed.GetAxisAttr().Get())
        info["lower"] = float(typed.GetLowerLimitAttr().Get())
        info["upper"] = float(typed.GetUpperLimitAttr().Get())
    joint_info.append(info)

# 3. Visual meshes. Compose the asset mesh with its visual wrapper, exactly once.
layer = Sdf.Layer.FindOrOpen(str(BASE_LAYER))
mesh_scope = layer.GetPrimAtPath("/meshes")
visual_scope = layer.GetPrimAtPath("/visuals")
vis_meshes = {}
for link_name, link_spec in mesh_scope.nameChildren.items():
    visual_root = visual_scope.nameChildren.get(link_name)
    visual_spec = next(iter(visual_root.nameChildren), None) if visual_root else None
    for node_spec in link_spec.nameChildren:
        mesh_spec = node_spec.nameChildren.get("mesh")
        points_attr = mesh_spec.attributes.get("points") if mesh_spec else None
        if points_attr is None:
            continue
        points = np.asarray(list(points_attr.default), dtype=float)
        points = apply_spec_xform(points, node_spec)
        if visual_spec is not None:
            points = apply_spec_xform(points, visual_spec)
        vis_meshes[link_name] = {
            "points_body": [[round(float(c), 7) for c in point] for point in points],
            "rgba": material_rgba(layer, visual_spec) if visual_spec else [0.72, 0.75, 0.80, 1.0],
        }
        break

output = {
    "source": str(MAIN),
    "bodies": body_info,
    "joints": joint_info,
    "vis_meshes": vis_meshes,
}
DUMP_PATH.parent.mkdir(parents=True, exist_ok=True)
with DUMP_PATH.open("w", encoding="utf-8") as stream:
    json.dump(output, stream)

print("source:", MAIN)
print("dump:", DUMP_PATH)
print("bodies:", len(body_info), "joints:", len(joint_info),
      "visual meshes:", len(vis_meshes))
