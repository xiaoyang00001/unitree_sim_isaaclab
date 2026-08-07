#!/usr/bin/env python3
"""Lint and compose-test the pure-visual peer LOD (requires USD ``pxr``)."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/peer_visual_lod.py"
ASSET_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot/g1_43dof_visual_lod.usda"
)


def _load_fk_module():
    spec = importlib.util.spec_from_file_location("_peer_visual_lod_validator", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load FK module: {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _forbidden_schema_prims(prims: list[Usd.Prim]) -> list[str]:
    forbidden = []
    schema_fragments = (
        "Physics",
        "Physx",
        "RigidBody",
        "Collision",
        "Articulation",
        "ContactReport",
        "IsaacRobot",
        "IsaacLink",
    )
    for prim in prims:
        applied = tuple(prim.GetAppliedSchemas())
        if any(fragment in schema for schema in applied for fragment in schema_fragments):
            forbidden.append(f"{prim.GetPath()} applied={applied}")
        if prim.IsA(UsdPhysics.Joint):
            forbidden.append(f"{prim.GetPath()} type={prim.GetTypeName()}")
    return forbidden


def main() -> int:
    fk = _load_fk_module()
    stage = Usd.Stage.Open(str(ASSET_PATH))
    if stage is None:
        raise RuntimeError(f"failed to open {ASSET_PATH}")
    if stage.GetDefaultPrim().GetPath() != "/PeerVisualLod":
        raise RuntimeError(f"unexpected defaultPrim: {stage.GetDefaultPrim().GetPath()}")

    prims = list(stage.Traverse())
    gprims = [prim for prim in prims if prim.IsA(UsdGeom.Gprim)]
    meshes = [prim for prim in prims if prim.IsA(UsdGeom.Mesh)]
    materials = [prim for prim in prims if prim.IsA(UsdShade.Material)]
    forbidden = _forbidden_schema_prims(prims)
    if forbidden:
        raise RuntimeError("physics schemas found:\n" + "\n".join(forbidden))
    if len(gprims) != 47 or meshes or len(materials) != 3:
        raise RuntimeError(
            f"unexpected LOD inventory: gprims={len(gprims)} meshes={len(meshes)} "
            f"materials={len(materials)}"
        )
    for link_name, path in fk.link_paths("/PeerVisualLod").items():
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"missing FK link {link_name}: {path}")

    # Exercise the exact reference composition and authored-override path used
    # by UsdFileCfg/scene-state at runtime, without starting Isaac Sim.
    composed = Usd.Stage.CreateInMemory()
    target = composed.DefinePrim("/World/envs/env_0/PeerRobot", "Xform")
    if not target.GetReferences().AddReference(str(ASSET_PATH)):
        raise RuntimeError("failed to compose LOD reference")
    mirror = fk.UsdVisualLodMirror(str(target.GetPath()), stage=composed)
    joints = [0.0] * len(fk.JOINT_NAMES)
    elbow_index = fk.JOINT_NAMES.index("left_elbow_joint")
    joints[elbow_index] = 0.7
    mirror.apply_state(
        (
            1.0,
            2.0,
            3.0,
            math.cos(0.2),
            0.0,
            0.0,
            math.sin(0.2),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
        joints,
        fk.JOINT_NAMES,
    )
    if tuple(target.GetAttribute("xformOp:translate").Get()) != (1.0, 2.0, 3.0):
        raise RuntimeError("root pose override did not author")
    elbow_path = fk.link_paths(str(target.GetPath()))["left_elbow_link"]
    elbow_orient = composed.GetPrimAtPath(elbow_path).GetAttribute("xformOp:orient").Get()
    if elbow_orient is None or abs(float(elbow_orient.GetImaginary()[1])) < 0.1:
        raise RuntimeError("joint orient override did not author")

    composed_prims = list(Usd.PrimRange(target))
    composed_forbidden = _forbidden_schema_prims(composed_prims)
    if composed_forbidden:
        raise RuntimeError("composed physics schemas found:\n" + "\n".join(composed_forbidden))

    print(
        "peer visual_lod USD OK: "
        f"source_prims={len(prims)} gprims={len(gprims)} meshes=0 materials={len(materials)} "
        f"composed_prims={len(composed_prims)} physics_schemas=0 FK=43"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
