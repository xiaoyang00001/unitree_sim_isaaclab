"""Multi-robot real-scene variants built on the Conveyor robot contract.

The original conveyor configuration couples three concerns: the SONIC robot
and scene-sync contract, the warehouse conveyor assets, and the conveyor
events.  This module reuses the first concern while making the latter two
explicit.  Warehouse, Apartment, Staircase, and Office therefore keep all robot fields
and DDS/sync wiring but do not accidentally spawn boxes or conveyor colliders
against an unrelated floor asset.

The existing ``Isaac-G1-29DoF-Sonic-Conveyor`` task remains the full warehouse
conveyor baseline.  The task IDs registered by this module are previewable
multi-robot real-scene variants; their default render policy is global
``robot_1`` only (controlled by ``ISAACLAB_CONVEYOR_VISIBLE_ROBOTS``).
"""

from __future__ import annotations

import os
from copy import deepcopy
from dataclasses import dataclass

from isaaclab.assets import AssetBaseCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    ActionsCfg as SonicActionsCfg,
    G129SonicEnvCfg,
)
from .conveyor_env_cfg import (
    HOST_MODE,
    LOCAL_ROBOT_ID,
    PEER_ROBOT_GLOBAL_NAME,
    SCENE_SYNC_ENABLED,
    VIEWER_MODE,
    ConveyorEventsCfg,
    G129SonicConveyorSceneCfg,
    HostConveyorActionsCfg,
    HostObservationsCfg,
    SonicObservationsCfg,
    _STANDBY_ROBOT_USD,
    _STANDBY_ROBOT_PRIM_NAMES,
    _scene_state_sync_cfg,
    _env_reset_sync_cfg,
    is_robot_visual_visible,
)


_LIGHTWHEEL_ROOT = os.environ.get(
    "LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR", "/home/nolo/Lightwheel_OpenSource"
)
REAL_WAREHOUSE_USD = os.environ.get(
    "ISAAC_REAL_WAREHOUSE_USD",
    f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",
)
REAL_COMPLETE_SCENE_USDS = {
    "warehouse": REAL_WAREHOUSE_USD,
    "apartment": os.path.join(
        _LIGHTWHEEL_ROOT, "Locomotion", "Apartment", "scene_04.usd"
    ),
    "staircase": os.path.join(
        _LIGHTWHEEL_ROOT, "Locomotion", "2-StoryStaircase", "2-StoryStaircase.usd"
    ),
    "office": f"{ISAAC_NUCLEUS_DIR}/Environments/Office/office.usd",
}


@dataclass(frozen=True)
class MultiRobotSceneSpec:
    """One complete background and safe initial poses for all robot roles."""

    key: str
    background_usd: str
    background_prim_name: str
    robot_positions: dict[str, tuple[float, float, float]]
    robot_rotations: dict[str, tuple[float, float, float, float]]
    camera_eye: tuple[float, float, float]
    camera_lookat: tuple[float, float, float]


# These are intentionally conservative open-floor poses.  The hidden robots
# remain real articulations in host mode, so keeping them away from walls and
# furniture matters even when the default camera only renders robot_1.
MULTI_SCENE_SPECS = {
    "warehouse": MultiRobotSceneSpec(
        key="warehouse",
        background_usd=REAL_COMPLETE_SCENE_USDS["warehouse"],
        background_prim_name="Warehouse",
        robot_positions={
            # The Simple Warehouse has a clear central aisle between the
            # pillar rows. Keep the physical pair in that aisle so the room
            # variant remains walkable when HOST_BOTH_ROBOTS is enabled.
            "robot_1": (0.0, 0.0, 0.76),
            "robot_2": (0.0, 2.0, 0.76),
            "standby_robot_1": (-3.0, 0.0, 0.76),
            "standby_robot_2": (3.0, 0.0, 0.76),
            "standby_robot_3": (0.0, 4.8, 0.76),
        },
        robot_rotations={
            "robot_1": (1.0, 0.0, 0.0, 0.0),
            "robot_2": (0.0, 0.0, 0.0, 1.0),
            "standby_robot_1": (1.0, 0.0, 0.0, 0.0),
            "standby_robot_2": (0.0, 0.0, 0.0, 1.0),
            "standby_robot_3": (1.0, 0.0, 0.0, 0.0),
        },
        camera_eye=(7.5, -8.5, 4.5),
        camera_lookat=(0.0, 0.0, 1.0),
    ),
    "apartment": MultiRobotSceneSpec(
        key="apartment",
        background_usd=REAL_COMPLETE_SCENE_USDS["apartment"],
        background_prim_name="Apartment",
        robot_positions={
            # ⚠️ 原站位 (-2.5,5.5) 所在"北房"没有碰撞地面,robot_1 会直接
            # 坠落穿地;老冒烟位 (0,0) 虽稳定但被一圈墙围住(相机拍不到)。
            # (15,10) 物理稳定(漂移 0.03m)且视野开阔。
            "robot_1": (15.0, 10.0, 0.76),
            "robot_2": (0.0, 5.5, 0.76),
            "standby_robot_1": (-5.0, 5.5, 0.76),
            "standby_robot_2": (-2.5, 7.0, 0.76),
            "standby_robot_3": (0.0, 7.0, 0.76),
        },
        robot_rotations={
            "robot_1": (1.0, 0.0, 0.0, 0.0),
            "robot_2": (0.0, 0.0, 0.0, 1.0),
            "standby_robot_1": (1.0, 0.0, 0.0, 0.0),
            "standby_robot_2": (0.70710678, 0.0, 0.0, 0.70710678),
            "standby_robot_3": (0.0, 0.0, 0.0, 1.0),
        },
        # robot_1 已挪到开阔稳定的 (15,10)(原北房无碰撞地面会坠落、(0,0)
        # 被墙围住拍不到)。相机在 robot_1 正前方 2.5m 低角度平视,画面中心
        # 高对比(robot 亮色可见,std≈45)。
        camera_eye=(17.5, 10.0, 1.6),
        camera_lookat=(15.0, 10.0, 1.0),
    ),
    "staircase": MultiRobotSceneSpec(
        key="staircase",
        background_usd=REAL_COMPLETE_SCENE_USDS["staircase"],
        background_prim_name="TwoStoryStaircase",
        robot_positions={
            # ⚠️ Loft 一层只有 y≈-3 一带是平地,复用 Warehouse 的 y=0 队形
            # 会让 robot_1 落在楼梯/斜面上滑倒(120 帧物理实测)。robot_1
            # 必须站 (2,-3)(x[-1,2] 范围实测稳定)。其余角色是镜像/纯显示,
            # 无物理,位置只影响视觉编队,保持旧值。
            "robot_1": (2.0, -3.0, 0.76),
            "robot_2": (0.0, 2.0, 0.76),
            "standby_robot_1": (-3.0, 0.0, 0.76),
            "standby_robot_2": (3.0, 0.0, 0.76),
            "standby_robot_3": (0.0, 4.8, 0.76),
        },
        robot_rotations={
            "robot_1": (1.0, 0.0, 0.0, 0.0),
            "robot_2": (0.0, 0.0, 0.0, 1.0),
            "standby_robot_1": (1.0, 0.0, 0.0, 0.0),
            "standby_robot_2": (0.0, 0.0, 0.0, 1.0),
            "standby_robot_3": (1.0, 0.0, 0.0, 0.0),
        },
        # robot_1 已从 (0,0) 挪到 (2,-3)(Loft 一层唯一平地)。相机在 robot_1
        # 正前方(+X)2m 处低角度平视,视线已射线验证无遮挡(东墙在 4.3m 外)。
        camera_eye=(3.5, -3.0, 1.5),
        camera_lookat=(2.0, -3.0, 1.0),
    ),
    "office": MultiRobotSceneSpec(
        key="office",
        background_usd=REAL_COMPLETE_SCENE_USDS["office"],
        background_prim_name="Office",
        robot_positions={
            # 前台/接待区在官方 Office USD 的 y≈0 一带；接待台位于 x≈-7
            # 和 x≈-2 两侧。robot_1 放在接待台南侧的开阔地面，避开台体
            # 与隔断，同时让前台和机器人一起进入低角度相机视野。
            "robot_1": (-4.5, -5.5, 0.76),
            "robot_2": (-5.6, 21.0, 0.76),
            "standby_robot_1": (-10.4, 21.0, 0.76),
            "standby_robot_2": (-8.0, 23.4, 0.76),
            "standby_robot_3": (-5.6, 23.4, 0.76),
        },
        robot_rotations={
            "robot_1": (1.0, 0.0, 0.0, 0.0),
            "robot_2": (0.0, 0.0, 0.0, 1.0),
            "standby_robot_1": (1.0, 0.0, 0.0, 0.0),
            "standby_robot_2": (0.70710678, 0.0, 0.0, 0.70710678),
            "standby_robot_3": (0.0, 0.0, 0.0, 1.0),
        },
        # 相机位于前台南侧约 3m、略高于机器人胸口，朝北平视，避免从
        # 建筑外墙方向取景；画面中心同时包含 robot_1 与前台接待台。
        camera_eye=(-4.5, -9.0, 2.1),
        camera_lookat=(-4.5, -5.5, 1.0),
    ),
}


def _set_pose(cfg, global_name: str, spec: MultiRobotSceneSpec):
    """Copy a freshly-created asset config and place it in ``spec``."""

    if cfg is None:
        return None
    cfg = deepcopy(cfg)
    cfg.init_state.pos = spec.robot_positions[global_name]
    cfg.init_state.rot = spec.robot_rotations[global_name]
    return cfg


def _make_scene_standby_cfg(index: int, spec: MultiRobotSceneSpec) -> AssetBaseCfg:
    """Create a standby visual even if the conveyor layout switch is ``0``."""

    global_name = f"standby_robot_{index + 1}"
    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{_STANDBY_ROBOT_PRIM_NAMES[index]}",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=spec.robot_positions[global_name],
            rot=spec.robot_rotations[global_name],
        ),
        spawn=UsdFileCfg(
            usd_path=str(_STANDBY_ROBOT_USD),
            variants={"Physics": "None", "Robot": "None", "Sensor": "None"},
            activate_contact_sensors=False,
            visible=is_robot_visual_visible(global_name),
        ),
    )


def _make_background_cfg(spec: MultiRobotSceneSpec) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="/World/envs/env_.*/Background",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=UsdFileCfg(usd_path=spec.background_usd),
    )


# Every conveyor-only field is explicitly removed from the complete-room
# variants.  The robot and peer fields are intentionally *not* in this list.
_NON_CONVEYOR_FIELDS = (
    "packing_table",
    "conveyor_collider",
    "conveyor_stop_collider",
    "conveyor_guide_positive_x",
    "conveyor_guide_negative_x",
    "conveyor_curve",
    "conveyor_xleg_1",
    "conveyor_xleg_2",
    "conveyor_xleg_3",
    "conveyor_xleg_4",
    "conveyor_xleg_5",
    "conveyor_corner_plate",
    "conveyor_branch_plate",
    "pushcart",
    "cart_box1",
    "cart_box2",
    "test_box",
    "pushcart_2",
    "cart2_tote1",
    "cart2_tote2",
    "cube_1",
    "cube_2",
    "cube_3",
    *(f"belt_box_{index}" for index in range(1, 18)),
)


@configclass
class MultiRealSceneEventsCfg(ConveyorEventsCfg):
    """No conveyor events when the background is a room/building USD."""

    lock_sorting_bins = None
    configure_surface_velocity = None
    drive_totes = None
    drive_belt_boxes = None
    recycle_surface_totes = None


def _empty_object_sync_cfg():
    """Keep robot scene-sync frames, but never mention conveyor objects."""

    cfg = _scene_state_sync_cfg()
    cfg.publish_object_names = ()
    cfg.apply_object_names = ()
    return cfg


@configclass
class MultiRealActionsCfg(SonicActionsCfg):
    scene_state_sync = _empty_object_sync_cfg()
    env_reset_sync = _env_reset_sync_cfg()


@configclass
class MultiRealHostActionsCfg(HostConveyorActionsCfg):
    scene_state_sync = _empty_object_sync_cfg()
    env_reset_sync = _env_reset_sync_cfg()


# ``@configclass`` removes field defaults from the class namespace, so keep
# one already-built instance as the source for the robot-role templates below.
# Each role is deep-copied before its pose is changed.
_CONVEYOR_SCENE_TEMPLATE = G129SonicConveyorSceneCfg(
    num_envs=1,
    env_spacing=0.0,
    replicate_physics=True,
)


def _make_scene_cfg_class(class_name: str, spec: MultiRobotSceneSpec):
    """Build a configclass while preserving every Conveyor robot field."""

    local_global_name = "robot_2" if LOCAL_ROBOT_ID == 2 else "robot_1"
    peer_global_name = PEER_ROBOT_GLOBAL_NAME

    # Reuse the already-built Conveyor templates.  Re-running the SONIC URDF
    # converter once per scene variant would be needlessly expensive and can
    # race on the shared generated USD during eager task registration.
    local_robot = _set_pose(_CONVEYOR_SCENE_TEMPLATE.robot, local_global_name, spec)
    peer_robot = _set_pose(_CONVEYOR_SCENE_TEMPLATE.peer_robot, peer_global_name, spec)
    peer_robot_2 = (
        _set_pose(_CONVEYOR_SCENE_TEMPLATE.peer_robot_2, "robot_2", spec)
        if VIEWER_MODE
        else None
    )
    robot_2 = (
        _set_pose(_CONVEYOR_SCENE_TEMPLATE.robot_2, "robot_2", spec)
        if HOST_MODE
        else None
    )

    attrs = {
        "__module__": __name__,
        "__doc__": f"{spec.key} complete scene with Conveyor-compatible multi-robot assets.",
        "ground": None,
        "light": None,
        # Complete room/building USDs carry their own lighting; the conveyor
        # baseline's extra sun would wash out the imported materials.
        "sun": None,
        "background": _make_background_cfg(spec),
        "robot": local_robot,
        "peer_robot": peer_robot,
        "peer_robot_2": peer_robot_2,
        "robot_2": robot_2,
        "standby_robot_1": _make_scene_standby_cfg(0, spec),
        "standby_robot_2": _make_scene_standby_cfg(1, spec),
        "standby_robot_3": _make_scene_standby_cfg(2, spec),
    }
    attrs.update({field_name: None for field_name in _NON_CONVEYOR_FIELDS})
    # Dynamic subclasses keep the four scene specs declarative without
    # duplicating the Conveyor robot/sync fields in four hand-written classes.
    return configclass(type(class_name, (G129SonicConveyorSceneCfg,), attrs))


G129SonicMultiWarehouseSceneCfg = _make_scene_cfg_class(
    "G129SonicMultiWarehouseSceneCfg", MULTI_SCENE_SPECS["warehouse"]
)
G129SonicMultiApartmentSceneCfg = _make_scene_cfg_class(
    "G129SonicMultiApartmentSceneCfg", MULTI_SCENE_SPECS["apartment"]
)
G129SonicMultiStaircaseSceneCfg = _make_scene_cfg_class(
    "G129SonicMultiStaircaseSceneCfg", MULTI_SCENE_SPECS["staircase"]
)
G129SonicMultiOfficeSceneCfg = _make_scene_cfg_class(
    "G129SonicMultiOfficeSceneCfg", MULTI_SCENE_SPECS["office"]
)


def _make_env_cfg_class(class_name: str, scene_cfg_cls, spec: MultiRobotSceneSpec):
    """Create the manager config without inheriting conveyor object events."""

    def _post_init(self):
        G129SonicEnvCfg.__post_init__(self)
        self.viewer.eye = spec.camera_eye
        self.viewer.lookat = spec.camera_lookat

        # Keep the Conveyor XR semantics when a complete-room variant is
        # selected.  ``teleop_devices`` owns copied XrCfg objects, so updating
        # only ``self.xr`` would leave OpenXR anchored to the hidden Robot
        # ghost (the same subtle bug the Conveyor config previously fixed).
        from isaaclab.devices.openxr import XrAnchorRotationMode

        if VIEWER_MODE:
            anchor_prim = (
                "PeerRobot2"
                if os.environ.get("ISAACLAB_XR_ANCHOR_ROBOT_ID", "1").strip() == "2"
                else "PeerRobot"
            )
            anchor_extra = {
                "anchor_position_smoothing_time": float(
                    os.environ.get("ISAACLAB_XR_ANCHOR_POS_SMOOTHING", "0.15")
                ),
                "recenter_yaw_on_start": True,
            }
            if os.environ.get("ISAACLAB_XR_ANCHOR_ROT_FOLLOW", "0") != "1":
                anchor_extra["anchor_rotation_mode"] = XrAnchorRotationMode.FIXED
        elif HOST_MODE and os.environ.get("ISAACLAB_XR_ANCHOR_ROBOT_ID", "1").strip() == "2":
            anchor_prim = "Robot2"
            anchor_extra = {}
        else:
            anchor_prim = "Robot"
            anchor_extra = {}

        anchor = f"/World/envs/env_0/{anchor_prim}"
        xr_targets = [self.xr]
        for device_cfg in getattr(self.teleop_devices, "devices", {}).values():
            device_xr = getattr(device_cfg, "xr_cfg", None)
            if device_xr is not None and device_xr is not self.xr:
                xr_targets.append(device_xr)
        for xr_cfg in xr_targets:
            xr_cfg.anchor_prim_path = f"{anchor}/torso_link/head_link"
            xr_cfg.anchor_rotation_prim_path = f"{anchor}/pelvis"
            xr_cfg.fixed_anchor_height = False
            for key, value in anchor_extra.items():
                setattr(xr_cfg, key, value)
        print(
            f"[multi_scene] XR anchor -> {anchor_prim} "
            f"({len(xr_targets)} xr_cfg copies)"
        )
        if VIEWER_MODE and not SCENE_SYNC_ENABLED:
            raise RuntimeError(
                "multi-scene viewer requires ISAACLAB_SCENE_SYNC=1 and pyzmq"
            )
        print(
            f"[multi_scene] {spec.key}: multi-robot background={spec.background_usd}; "
            "default visible global robot_1 (override ISAACLAB_CONVEYOR_VISIBLE_ROBOTS)"
        )

    attrs = {
        "__module__": __name__,
        "__doc__": f"Multi-robot SONIC {spec.key} real-scene task.",
        "scene": scene_cfg_cls(num_envs=1, env_spacing=0.0, replicate_physics=True),
        "actions": MultiRealHostActionsCfg() if HOST_MODE else MultiRealActionsCfg(),
        "observations": HostObservationsCfg() if HOST_MODE else SonicObservationsCfg(),
        "events": MultiRealSceneEventsCfg(),
        "__post_init__": _post_init,
    }
    return configclass(type(class_name, (G129SonicEnvCfg,), attrs))


G129SonicMultiWarehouseEnvCfg = _make_env_cfg_class(
    "G129SonicMultiWarehouseEnvCfg",
    G129SonicMultiWarehouseSceneCfg,
    MULTI_SCENE_SPECS["warehouse"],
)
G129SonicMultiApartmentEnvCfg = _make_env_cfg_class(
    "G129SonicMultiApartmentEnvCfg",
    G129SonicMultiApartmentSceneCfg,
    MULTI_SCENE_SPECS["apartment"],
)
G129SonicMultiStaircaseEnvCfg = _make_env_cfg_class(
    "G129SonicMultiStaircaseEnvCfg",
    G129SonicMultiStaircaseSceneCfg,
    MULTI_SCENE_SPECS["staircase"],
)
G129SonicMultiOfficeEnvCfg = _make_env_cfg_class(
    "G129SonicMultiOfficeEnvCfg",
    G129SonicMultiOfficeSceneCfg,
    MULTI_SCENE_SPECS["office"],
)
