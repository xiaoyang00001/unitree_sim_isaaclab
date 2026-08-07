"""Robot spawners that author ContactReport only on ankle-roll bodies."""

from __future__ import annotations

from typing import Any

import isaaclab.sim as sim_utils
from isaaclab.sim.utils import clone
from pxr import PhysxSchema, Usd, UsdPhysics


_ANKLE_SUFFIX = "_ankle_roll_link"


def _apply_ankle_contact_reports(root_prim: Usd.Prim) -> tuple[str, ...]:
    """Apply ContactReportAPI to exactly the two ankle-roll rigid bodies."""

    matched: list[str] = []
    for prim in Usd.PrimRange(root_prim):
        if not prim.GetName().endswith(_ANKLE_SUFFIX):
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(f"脚踝 ContactReport 目标不是刚体: {prim.GetPath()}")
        api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        api.CreateThresholdAttr().Set(0.0)
        matched.append(str(prim.GetPath()))

    if len(matched) != 2:
        raise RuntimeError(
            "脚踝 ContactReport 必须精确匹配 2 个 *_ankle_roll_link，"
            f"实际 {len(matched)} 个: {matched}"
        )
    print(f"[conveyor_contact] ankle-only ContactReport: {matched}")
    return tuple(matched)


@clone
def spawn_urdf_with_ankle_contact_reports(
    prim_path: str,
    cfg: sim_utils.UrdfFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs: Any,
) -> Usd.Prim:
    """Spawn a URDF articulation and add reporters before simulation starts."""

    prim = sim_utils.spawn_from_urdf(
        prim_path,
        cfg,
        translation=translation,
        orientation=orientation,
        **kwargs,
    )
    _apply_ankle_contact_reports(prim)
    return prim


@clone
def spawn_usd_with_ankle_contact_reports(
    prim_path: str,
    cfg: sim_utils.UsdFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs: Any,
) -> Usd.Prim:
    """Spawn a USD articulation and add reporters before simulation starts."""

    prim = sim_utils.spawn_from_usd(
        prim_path,
        cfg,
        translation=translation,
        orientation=orientation,
        **kwargs,
    )
    _apply_ankle_contact_reports(prim)
    return prim


def configure_robot_contact_reports(spawn_cfg: Any, mode: str) -> None:
    """Mutate a freshly-created robot spawner for ``ankles/all/off`` mode."""

    if mode == "all":
        spawn_cfg.activate_contact_sensors = True
        return

    spawn_cfg.activate_contact_sensors = False
    if mode == "off":
        return
    if mode != "ankles":
        raise ValueError(f"未知 ContactReport 模式: {mode}")

    if isinstance(spawn_cfg, sim_utils.UrdfFileCfg):
        spawn_cfg.func = spawn_urdf_with_ankle_contact_reports
    elif isinstance(spawn_cfg, sim_utils.UsdFileCfg):
        spawn_cfg.func = spawn_usd_with_ankle_contact_reports
    else:
        raise TypeError(
            "ankles ContactReport 只支持 UrdfFileCfg/UsdFileCfg，"
            f"实际 {type(spawn_cfg).__name__}"
        )
