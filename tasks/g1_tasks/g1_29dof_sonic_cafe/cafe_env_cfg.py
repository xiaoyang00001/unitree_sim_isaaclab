# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Lightwheel KitchenRoom cafe scene on the conveyor SONIC topology.

Only the background, cup, work poses, anchors, and camera are cafe-specific.
Robot construction, host/viewer/peer roles, mirror assets, ZMQ scene-state
transport, and reset transport are reused from conveyor baseline 87e1f4e.
"""

from __future__ import annotations

import os
from typing import Any

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.sim.utils import clone
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from pxr import PhysxSchema, Usd, UsdPhysics

from tasks.common_event.event_manager import SimpleEvent
from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    G129Dex3SonicSceneCfg,
    G129SonicEnvCfg,
    ObservationsCfg as SonicObservationsCfg,
)
from tasks.g1_tasks.g1_29dof_sonic_conveyor.conveyor_env_cfg import (
    HOST_MODE,
    LOCAL_ROBOT_ID,
    MIRROR_OBJECTS,
    OBJECT_AUTHORITY,
    PEER_VISUAL_LOD,
    SCENE_SYNC_ENABLED,
    VIEWER_MODE,
    ConveyorActionsCfg,
    HostConveyorActionsCfg,
    HostObservationsCfg,
    ViewerObservationsCfg,
    _PEER_ROBOT_USD,
    _PEER_VISUAL_LOD_USD,
    _make_additional_local_robot_cfg as _make_conveyor_additional_local_robot_cfg,
    _make_additional_peer_scene_cfg as _make_conveyor_additional_peer_scene_cfg,
    _make_foot_contact_sensor,
    _make_local_robot_cfg as _make_conveyor_local_robot_cfg,
    _make_peer_scene_cfg as _make_conveyor_peer_scene_cfg,
    _scene_state_sync_cfg as _make_conveyor_scene_state_sync_cfg,
)
from tasks.g1_tasks.g1_29dof_sonic_conveyor.conveyor_events import reset_scene_mirror_safe


def resolve_kitchen_room_usd_path() -> str:
    """Return the external KitchenRoom path with a deployment override."""

    root_dir = os.environ.get(
        "LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR", "/home/nolo/Lightwheel_OpenSource"
    )
    return os.path.join(root_dir, "Locomotion", "KitchenRoom", "KitchenRoom.usd")


KITCHEN_ROOM_USD_PATH = resolve_kitchen_room_usd_path()
KITCHEN_MODE_ENV = "ISAACLAB_CAFE_KITCHEN_MODE"
KITCHEN_MODE_STATIC = "static_background"
KITCHEN_MODE_PROXY = "proxy_background"
KITCHEN_MODE_LEGACY = "legacy_dynamic"
_KITCHEN_MODES = (KITCHEN_MODE_STATIC, KITCHEN_MODE_PROXY, KITCHEN_MODE_LEGACY)


def resolve_kitchen_room_mode() -> str:
    """Resolve the KitchenRoom physics mode and reject ambiguous values."""

    raw_mode = os.environ.get(KITCHEN_MODE_ENV, KITCHEN_MODE_PROXY)
    mode = raw_mode.strip().lower() or KITCHEN_MODE_PROXY
    if mode not in _KITCHEN_MODES:
        choices = ", ".join(_KITCHEN_MODES)
        raise ValueError(f"{KITCHEN_MODE_ENV}={raw_mode!r} invalid; expected one of: {choices}")
    return mode


KITCHEN_ROOM_MODE = resolve_kitchen_room_mode()


def _kitchen_prims(root_prim: Usd.Prim, *, include_instance_proxies: bool) -> list[Usd.Prim]:
    predicate = Usd.TraverseInstanceProxies() if include_instance_proxies else Usd.PrimDefaultPredicate
    return list(Usd.PrimRange(root_prim, predicate))


def _collision_enabled_state(root_prim: Usd.Prim) -> dict[str, bool | None]:
    """Snapshot every composed collider path and its effective enabled value."""

    state: dict[str, bool | None] = {}
    for prim in _kitchen_prims(root_prim, include_instance_proxies=True):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            state[str(prim.GetPath())] = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
    return state


def _deinstance_kitchen(root_prim: Usd.Prim) -> int:
    """Make nested instances editable, one composed instance level at a time."""

    deinstanced = 0
    while True:
        instance_prims = [
            prim for prim in _kitchen_prims(root_prim, include_instance_proxies=False)
            if prim.IsInstance()
        ]
        if not instance_prims:
            return deinstanced

        instance_paths = tuple(str(prim.GetPath()) for prim in instance_prims)
        for prim in instance_prims:
            prim.SetInstanceable(False)
        deinstanced += len(instance_prims)

        remaining_paths = tuple(
            str(prim.GetPath())
            for prim in _kitchen_prims(root_prim, include_instance_proxies=False)
            if prim.IsInstance()
        )
        if remaining_paths == instance_paths:
            raise RuntimeError(
                "KitchenRoom static_background could not de-instance composed prims: "
                f"{remaining_paths[:8]}"
            )


def _remove_api(prims: list[Usd.Prim], schema_type: Any, label: str) -> None:
    for prim in prims:
        if not prim.RemoveAPI(schema_type):
            raise RuntimeError(
                f"KitchenRoom static_background failed to remove {label} from {prim.GetPath()}"
            )


def _staticize_kitchen(root_prim: Usd.Prim, *, strip_collisions: bool = False) -> None:
    """Remove Kitchen dynamics and optionally replace all composed colliders."""

    root_prim.Load(Usd.LoadWithDescendants)
    mode = KITCHEN_MODE_PROXY if strip_collisions else KITCHEN_MODE_STATIC
    payload_paths = tuple(
        str(prim.GetPath())
        for prim in _kitchen_prims(root_prim, include_instance_proxies=True)
        if prim.HasAuthoredPayloads()
    )
    collision_before = _collision_enabled_state(root_prim)
    stage = root_prim.GetStage()

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        deinstanced = _deinstance_kitchen(root_prim)
        prims = _kitchen_prims(root_prim, include_instance_proxies=False)
        joints = [prim for prim in prims if prim.IsA(UsdPhysics.Joint)]
        usd_rigid_bodies = [prim for prim in prims if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        physx_rigid_bodies = [prim for prim in prims if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI)]
        usd_articulations = [prim for prim in prims if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
        physx_articulations = [prim for prim in prims if prim.HasAPI(PhysxSchema.PhysxArticulationAPI)]
        physx_collisions = [prim for prim in prims if prim.HasAPI(PhysxSchema.PhysxCollisionAPI)]
        mesh_collisions = [prim for prim in prims if prim.HasAPI(UsdPhysics.MeshCollisionAPI)]
        usd_collisions = [prim for prim in prims if prim.HasAPI(UsdPhysics.CollisionAPI)]

        for prim in joints:
            UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
        _remove_api(usd_articulations, UsdPhysics.ArticulationRootAPI, "UsdPhysics.ArticulationRootAPI")
        _remove_api(physx_articulations, PhysxSchema.PhysxArticulationAPI, "PhysxArticulationAPI")
        _remove_api(usd_rigid_bodies, UsdPhysics.RigidBodyAPI, "UsdPhysics.RigidBodyAPI")
        _remove_api(physx_rigid_bodies, PhysxSchema.PhysxRigidBodyAPI, "PhysxRigidBodyAPI")
        if strip_collisions:
            _remove_api(physx_collisions, PhysxSchema.PhysxCollisionAPI, "PhysxCollisionAPI")
            _remove_api(mesh_collisions, UsdPhysics.MeshCollisionAPI, "UsdPhysics.MeshCollisionAPI")
            _remove_api(usd_collisions, UsdPhysics.CollisionAPI, "UsdPhysics.CollisionAPI")

    prims_after = _kitchen_prims(root_prim, include_instance_proxies=True)
    remaining_instances = [
        str(prim.GetPath()) for prim in prims_after if prim.IsInstance() or prim.IsInstanceProxy()
    ]
    remaining_joints = [
        str(prim.GetPath())
        for prim in prims_after
        if prim.IsA(UsdPhysics.Joint)
        and UsdPhysics.Joint(prim).GetJointEnabledAttr().Get() is not False
    ]
    remaining_apis = [
        str(prim.GetPath())
        for prim in prims_after
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
        or prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI)
        or prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        or prim.HasAPI(PhysxSchema.PhysxArticulationAPI)
    ]
    if remaining_instances or remaining_joints or remaining_apis:
        raise RuntimeError(
            f"KitchenRoom {mode} incomplete: "
            f"instances={remaining_instances[:8]}, enabled_joints={remaining_joints[:8]}, "
            f"physics_apis={remaining_apis[:8]}"
        )

    collision_after = _collision_enabled_state(root_prim)
    remaining_collision_apis = [
        str(prim.GetPath())
        for prim in prims_after
        if prim.HasAPI(PhysxSchema.PhysxCollisionAPI)
        or prim.HasAPI(UsdPhysics.MeshCollisionAPI)
        or prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    if strip_collisions and (collision_after or remaining_collision_apis):
        raise RuntimeError(
            "KitchenRoom proxy_background retained composed collision APIs: "
            f"colliders={sorted(collision_after)[:8]}, apis={remaining_collision_apis[:8]}"
        )
    if not strip_collisions and collision_before != collision_after:
        before_paths = set(collision_before)
        after_paths = set(collision_after)
        changed_paths = sorted(
            path
            for path in before_paths & after_paths
            if collision_before[path] != collision_after[path]
        )
        raise RuntimeError(
            "KitchenRoom static_background changed the collision contract: "
            f"missing={sorted(before_paths - after_paths)[:8]}, "
            f"added={sorted(after_paths - before_paths)[:8]}, changed={changed_paths[:8]}"
        )

    print(
        f"[sonic_cafe] KitchenRoom mode={mode} "
        f"payload_roots={len(payload_paths)} deinstanced={deinstanced} "
        f"usd_rigid_removed={len(usd_rigid_bodies)} "
        f"physx_rigid_removed={len(physx_rigid_bodies)} "
        f"usd_articulation_removed={len(usd_articulations)} "
        f"physx_articulation_removed={len(physx_articulations)} "
        f"joints_disabled={len(joints)} "
        f"collisions_preserved={len(collision_after)} "
        f"collisions_removed={len(collision_before) - len(collision_after)}"
    )


@clone
def _spawn_kitchen_room(
    prim_path: str,
    cfg: UsdFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs: Any,
) -> Usd.Prim:
    root_prim = sim_utils.spawn_from_usd(
        prim_path,
        cfg,
        translation=translation,
        orientation=orientation,
        **kwargs,
    )
    if KITCHEN_ROOM_MODE in (KITCHEN_MODE_STATIC, KITCHEN_MODE_PROXY):
        _staticize_kitchen(
            root_prim,
            strip_collisions=KITCHEN_ROOM_MODE == KITCHEN_MODE_PROXY,
        )
    else:
        print(f"[sonic_cafe] KitchenRoom mode={KITCHEN_MODE_LEGACY} staticization=off")
    return root_prim


def _make_kitchen_room_spawn_cfg() -> UsdFileCfg:
    cfg = UsdFileCfg(usd_path=KITCHEN_ROOM_USD_PATH)
    cfg.func = _spawn_kitchen_room
    return cfg


# Measured from the current Lightwheel KitchenRoom in world coordinates. These
# two simple static boxes replace 298 composed mesh colliders in proxy mode.
CAFE_FLOOR_PROXY_POS = (0.0, 0.052293800, -0.010499233)
CAFE_FLOOR_PROXY_SIZE = (5.275517464, 4.987211227, 0.020000000)
CAFE_ISLAND_PROXY_POS = (0.213442038, 0.225252678, 0.429156477)
CAFE_ISLAND_PROXY_SIZE = (1.149627462, 0.761342592, 0.858449757)


def _make_kitchen_proxy_cfg(
    name: str,
    pos: tuple[float, float, float],
    size: tuple[float, float, float],
) -> AssetBaseCfg | None:
    if KITCHEN_ROOM_MODE != KITCHEN_MODE_PROXY:
        return None
    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        init_state=AssetBaseCfg.InitialStateCfg(pos=pos),
        spawn=sim_utils.CuboidCfg(
            size=size,
            visible=False,
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
        ),
    )

# Cafe-only layout. The robot configs themselves still come from conveyor.
ROBOT_1_POS = (0.18, -0.42, 0.76)
ROBOT_1_ROT = (0.7071068, 0.0, 0.0, 0.7071068)
ROBOT_2_POS = (0.26, 0.80, 0.76)
ROBOT_2_ROT = (0.7071068, 0.0, 0.0, -0.7071068)
CUP_SPAWN_POS = (0.02, 0.04, 0.91)
HANDOVER_ZONE_POS = (0.22, 0.22, 1.00)
SERVE_ZONE_POS = (0.45, 0.46, 0.94)
VIEWER_ANCHOR_POS = HANDOVER_ZONE_POS
SYNC_OBJECT_NAMES = ("cup",)


def _robot_pose(robot_id: int):
    if robot_id == 1:
        return ROBOT_1_POS, ROBOT_1_ROT
    if robot_id == 2:
        return ROBOT_2_POS, ROBOT_2_ROT
    raise ValueError(f"unsupported cafe robot id: {robot_id}")


def _make_local_robot_cfg() -> ArticulationCfg:
    """Use the conveyor local robot unchanged, replacing only its cafe pose."""

    cfg = _make_conveyor_local_robot_cfg()
    if not VIEWER_MODE:
        cfg.init_state.pos, cfg.init_state.rot = _robot_pose(LOCAL_ROBOT_ID)
    return cfg


def _make_second_local_robot_cfg() -> ArticulationCfg:
    """Use the conveyor host robot_2 unchanged, replacing only its cafe pose."""

    cfg = _make_conveyor_additional_local_robot_cfg(2)
    cfg.init_state.pos, cfg.init_state.rot = _robot_pose(2)
    return cfg


def _set_cafe_robot_pose(
    cfg: ArticulationCfg | AssetBaseCfg,
    robot_id: int,
) -> ArticulationCfg | AssetBaseCfg:
    """Change only the pose of a robot produced by the conveyor factory."""

    cfg.init_state.pos, cfg.init_state.rot = _robot_pose(robot_id)
    return cfg


def _make_primary_peer_scene_cfg() -> ArticulationCfg | AssetBaseCfg:
    peer_robot_id = 1 if VIEWER_MODE else 3 - LOCAL_ROBOT_ID
    return _set_cafe_robot_pose(_make_conveyor_peer_scene_cfg(), peer_robot_id)


def _make_second_peer_scene_cfg() -> ArticulationCfg | AssetBaseCfg:
    return _set_cafe_robot_pose(_make_conveyor_additional_peer_scene_cfg(2), 2)


def _make_cup_cfg() -> RigidObjectCfg:
    """Create the dynamic authority cup or its kinematic synchronized mirror."""

    mirror = MIRROR_OBJECTS
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cup",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=CUP_SPAWN_POS,
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Objects/Mug/mug.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=mirror,
                disable_gravity=mirror,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.25),
        ),
    )


def _make_anchor(name: str, pos: tuple[float, float, float]) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        init_state=AssetBaseCfg.InitialStateCfg(pos=pos),
        spawn=sim_utils.SphereCfg(
            radius=0.01,
            visible=False,
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
        ),
    )


def _scene_state_sync_cfg():
    """Reuse conveyor robot routing and replace only its object inventory."""

    cfg = _make_conveyor_scene_state_sync_cfg()
    if SCENE_SYNC_ENABLED:
        cfg.publish_object_names = SYNC_OBJECT_NAMES if OBJECT_AUTHORITY else ()
        cfg.apply_object_names = () if OBJECT_AUTHORITY else SYNC_OBJECT_NAMES
    return cfg


@configclass
class G129DualSonicCafeSceneCfg(G129Dex3SonicSceneCfg):
    """KitchenRoom with the conveyor host/viewer/peer robot topology."""

    ground = None
    # KitchenRoom supplies the scene lighting. Do not stack the base DomeLight.
    light = None
    background = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Background",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=_make_kitchen_room_spawn_cfg(),
    )
    floor_proxy: AssetBaseCfg | None = _make_kitchen_proxy_cfg(
        "CafeFloorProxy", CAFE_FLOOR_PROXY_POS, CAFE_FLOOR_PROXY_SIZE
    )
    island_proxy: AssetBaseCfg | None = _make_kitchen_proxy_cfg(
        "CafeIslandProxy", CAFE_ISLAND_PROXY_POS, CAFE_ISLAND_PROXY_SIZE
    )

    cup: RigidObjectCfg = _make_cup_cfg()
    robot: ArticulationCfg = _make_local_robot_cfg()
    peer_robot: ArticulationCfg | AssetBaseCfg | None = (
        None if HOST_MODE else _make_primary_peer_scene_cfg()
    )
    peer_robot_2: ArticulationCfg | AssetBaseCfg | None = (
        _make_second_peer_scene_cfg() if VIEWER_MODE else None
    )
    robot_2: ArticulationCfg | None = (
        _make_second_local_robot_cfg() if HOST_MODE else None
    )
    foot_contact: ContactSensorCfg | None = _make_foot_contact_sensor("Robot")
    foot_contact_2: ContactSensorCfg | None = (
        _make_foot_contact_sensor("Robot2") if HOST_MODE else None
    )

    robot_spawn_a = _make_anchor("RobotSpawnA", ROBOT_1_POS)
    robot_spawn_b = _make_anchor("RobotSpawnB", ROBOT_2_POS)
    cup_spawn = _make_anchor("CupSpawn", CUP_SPAWN_POS)
    handover_zone = _make_anchor("HandoverZone", HANDOVER_ZONE_POS)
    serve_zone = _make_anchor("ServeZone", SERVE_ZONE_POS)
    viewer_anchor = _make_anchor("ViewerAnchor", VIEWER_ANCHOR_POS)


@configclass
class CafeActionsCfg(ConveyorActionsCfg):
    """Conveyor actions with only the synchronized object inventory replaced."""

    scene_state_sync = _scene_state_sync_cfg()


@configclass
class HostCafeActionsCfg(HostConveyorActionsCfg):
    """Exact conveyor dual-host actions with the cup as the only scene object."""

    scene_state_sync = _scene_state_sync_cfg()


@configclass
class G129DualSonicCafeEnvCfg(G129SonicEnvCfg):
    """Cafe assets on the exact conveyor multi-machine SONIC topology."""

    scene: G129DualSonicCafeSceneCfg = G129DualSonicCafeSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )
    actions: CafeActionsCfg | HostCafeActionsCfg = (
        HostCafeActionsCfg() if HOST_MODE else CafeActionsCfg()
    )
    observations: SonicObservationsCfg | ViewerObservationsCfg | HostObservationsCfg = (
        HostObservationsCfg()
        if HOST_MODE
        else ViewerObservationsCfg()
        if VIEWER_MODE
        else SonicObservationsCfg()
    )

    def __post_init__(self):
        super().__post_init__()
        if MIRROR_OBJECTS:
            mirror_reset = SimpleEvent(func=reset_scene_mirror_safe)
            self.event_manager.register("reset_object_self", mirror_reset)
            self.event_manager.register("reset_all_self", mirror_reset)

        self.viewer.eye = (2.6, -2.6, 1.9)
        self.viewer.lookat = VIEWER_ANCHOR_POS

        if VIEWER_MODE and not SCENE_SYNC_ENABLED:
            raise RuntimeError(
                "Cafe viewer mode (ISAACLAB_LOCAL_ROBOT_ID=0) requires ISAACLAB_SCENE_SYNC=1"
            )
        if not HOST_MODE and PEER_VISUAL_LOD and not _PEER_VISUAL_LOD_USD.exists():
            raise RuntimeError(f"missing SONIC visual peer asset: {_PEER_VISUAL_LOD_USD}")
        if not HOST_MODE and not PEER_VISUAL_LOD and not _PEER_ROBOT_USD.exists():
            raise RuntimeError(f"missing SONIC articulation peer asset: {_PEER_ROBOT_USD}")

        if HOST_MODE:
            topology = "host: robot + robot_2 authoritative physics"
        elif VIEWER_MODE:
            topology = "viewer: peer_robot + peer_robot_2 synchronized mirrors"
        else:
            topology = "peer: one authoritative robot + one synchronized mirror"
        peer_mode = "visual_lod" if PEER_VISUAL_LOD else "articulation"
        print(f"[sonic_cafe] topology={topology}; peer_mode={peer_mode}")
