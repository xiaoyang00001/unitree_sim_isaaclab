# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""SONIC G1 + warehouse 流水线场景 + ZMQ 双机同步。

场景与流水线驱动移植自 IsaacLab 分叉 feat/conveyor-loop-totes-wip 的
``pick_place/locomanipulation_g1_env_cfg.py``（tip 17e71c0a0），机器人与
DDS/XR 链路沿用本工程 ``g1_29dof_dex3_sonic`` 的 SONIC 底座。

双机形态（对等/混合权威）：
* 每台机器跑本文件的同一个任务，用 ``ISAACLAB_LOCAL_ROBOT_ID``（1/2）区分身份；
* 本机 G1 名为 ``robot``（prim ``Robot``，DDS/观测/XR 全链路与 SONIC 任务一致），
  对端 G1 是 ``peer_robot`` 镜像体（无碰撞、无重力，由 scene_state 帧驱动）；
* 场景物体的物理权威固定在 ID=1：ID=2 侧物体 spawn 成 kinematic 纯跟随；
* 流水线驱动事件只在物体权威端生效；
* 已知限制：只有 ID=1 侧机器人能与物体发生物理交互（镜像体无碰撞）。

配置读取是**单轨制**：进程环境变量永远优先；``configs/scene_sync.env``
（或 ``ISAACLAB_SCENE_SYNC_ENV_FILE`` 指定的文件）里的值在 import 时
``setdefault`` 进 ``os.environ`` 作为默认值。不存在源分支"部分变量只认
env 文件"的双轨陷阱。
"""

from __future__ import annotations

import os
import re
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass

from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    SONIC_CUBE_INITIAL_Z,
    G129SonicEnvCfg,
    G129SonicSceneCfg,
    _make_sonic_cube_cfg,
    make_sonic_robot_cfg,
)
from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    ActionsCfg as SonicActionsCfg,
)

from . import conveyor_events
from .zmq_scene_sync import ZmqEnvResetSyncActionCfg, ZmqSceneStateSyncActionCfg

_ASSETS_DIR = Path(__file__).resolve().parent / "scene_assets"

# ==================================================================
# 配置加载：configs/scene_sync.env → os.environ.setdefault（进程 env 永远优先）
# ==================================================================

_ENV_REF_RE = re.compile(r"\$(?:\{([A-Za-z_][A-Za-z0-9_]*)\}|([A-Za-z_][A-Za-z0-9_]*))")


def _expand_config_refs(values: dict[str, str]) -> dict[str, str]:
    expanded = dict(values)
    for _ in range(10):
        changed = False
        next_values = {}
        for key, value in expanded.items():
            next_value = _ENV_REF_RE.sub(
                lambda match: expanded.get(match.group(1) or match.group(2), ""),
                value,
            )
            next_values[key] = next_value
            changed |= next_value != value
        expanded = next_values
        if not changed:
            break
    return expanded


def _load_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            values[key] = value
    return _expand_config_refs(values)


def _project_root() -> Path:
    # sim_main.py 启动即设 PROJECT_ROOT；独立诊断脚本走 __file__ 回退
    # （本文件在 tasks/g1_tasks/<pkg>/ 下，parents[3] 即仓库根）。
    root = os.environ.get("PROJECT_ROOT", "").strip()
    if root:
        return Path(root)
    return Path(__file__).resolve().parents[3]


def _load_scene_sync_config() -> None:
    candidates = []
    explicit = os.environ.get("ISAACLAB_SCENE_SYNC_ENV_FILE", "").strip()
    if explicit:
        candidates.append(Path(explicit).expanduser())
    candidates.append(_project_root() / "configs" / "scene_sync.env")
    for path in candidates:
        values = _load_env_file(path)
        if values:
            print(f"[conveyor_env_cfg] scene-sync config loaded: {path}")
            for key, value in values.items():
                os.environ.setdefault(key, value)
            return
    print("[conveyor_env_cfg] no scene-sync config file; using built-in defaults")


_load_scene_sync_config()


def _env_str(name: str, default: str) -> str:
    value = os.environ.get(name)
    return default if value is None or not value.strip() else value.strip()


def _env_int(name: str, default: int) -> int:
    try:
        return int(_env_str(name, str(default)))
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    try:
        return float(_env_str(name, str(default)))
    except ValueError:
        return default


def _env_optional_float(name: str) -> float | None:
    value = _env_str(name, "")
    if not value or value.lower() in {"none", "null"}:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _env_str_tuple(name: str, default: tuple[str, ...]) -> tuple[str, ...]:
    value = os.environ.get(name)
    if value is None:
        return default
    return tuple(item.strip() for item in value.split(",") if item.strip())


# ==================================================================
# 双机身份与同步端点
# ==================================================================


def _local_robot_id() -> int:
    raw_value = _env_str("ISAACLAB_LOCAL_ROBOT_ID", "1")
    try:
        robot_id = int(raw_value)
    except ValueError:
        print(f"[conveyor_env_cfg] Invalid ISAACLAB_LOCAL_ROBOT_ID={raw_value!r}; using robot 1.")
        return 1
    if robot_id not in {1, 2}:
        print(f"[conveyor_env_cfg] Unsupported ISAACLAB_LOCAL_ROBOT_ID={raw_value!r}; using robot 1.")
        robot_id = 1
    return robot_id


LOCAL_ROBOT_ID = _local_robot_id()
PEER_ROBOT_ID = 3 - LOCAL_ROBOT_ID
LOCAL_ROBOT_GLOBAL_NAME = f"robot_{LOCAL_ROBOT_ID}"
PEER_ROBOT_GLOBAL_NAME = f"robot_{PEER_ROBOT_ID}"

# 场景物体的物理权威固定在 ID=1（流水线驱动也只在这端跑）。
OBJECT_AUTHORITY = LOCAL_ROBOT_ID == 1

# pyzmq 缺失时强制退回单机权威模式：否则 ID=2 的物体已 spawn 成 kinematic、
# 驱动已关，而同步 term 只打一条 error 就退化 no-op——整个场景悬空冻结却无硬失败。
try:
    import zmq as _zmq  # noqa: F401

    _PYZMQ_AVAILABLE = True
except ModuleNotFoundError:
    _PYZMQ_AVAILABLE = False

SCENE_SYNC_ENABLED = _env_bool("ISAACLAB_SCENE_SYNC", True) and _PYZMQ_AVAILABLE
if _env_bool("ISAACLAB_SCENE_SYNC", True) and not _PYZMQ_AVAILABLE:
    print("[conveyor_env_cfg] ⚠️ pyzmq 未安装，场景同步强制关闭，本机按单机权威模式跑")

# 本机物体是否是 kinematic 镜像（= 非权威端且同步开着）。
# 同步关掉时（单机自测）任何一端都当权威跑，物体保持动态、流水线照常驱动。
MIRROR_OBJECTS = SCENE_SYNC_ENABLED and not OBJECT_AUTHORITY

# 端口方案：base + (id-1)。ID=1 绑 15555、ID=2 绑 15556，双方互连对方端口。
# 端口不对称跨机无副作用，同机双进程自测则开箱即用。
_PORT_BASE = _env_int("ISAACLAB_SCENE_SYNC_PORT_BASE", 15555)
_MY_PORT = _PORT_BASE + LOCAL_ROBOT_ID - 1
_PEER_PORT = _PORT_BASE + PEER_ROBOT_ID - 1
SCENE_SYNC_PEER_IP = _env_str("ISAACLAB_SCENE_SYNC_PEER_IP", "127.0.0.1")
SCENE_SYNC_BIND_ENDPOINT = _env_str("ISAACLAB_SCENE_SYNC_BIND_ENDPOINT", f"tcp://0.0.0.0:{_MY_PORT}")
SCENE_SYNC_CONNECT_ENDPOINT = _env_str(
    "ISAACLAB_SCENE_SYNC_CONNECT_ENDPOINT", f"tcp://{SCENE_SYNC_PEER_IP}:{_PEER_PORT}"
)

# 进 scene_state 帧的场景物体（两端清单必须逐字一致）。
# 比源分支多了 pushcart（被撞动时镜像端也能看到）和底座打包桌三方块
# cube_1/2/3（不进清单的话两端各自模拟、碰一下就静默分叉），代价可忽略。
SYNC_OBJECT_NAMES = _env_str_tuple(
    "ISAACLAB_SCENE_SYNC_OBJECTS",
    (
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
    ),
)


# 方案 b（本分支默认）：同步收发挂 sim_main 主循环而非 ActionTerm——SONIC 锁步下
# deploy 停发 lowcmd 时 env.step 停摆，ActionTerm 挂载的同步会随之冻结；主循环
# 挂载不受影响。置 0 退回方案 a（ActionTerm 每物理步收发、发布按 decimation 节流）。
SCENE_SYNC_MAINLOOP = _env_bool("ISAACLAB_SCENE_SYNC_MAINLOOP", True)


def _scene_state_sync_cfg() -> ZmqSceneStateSyncActionCfg:
    """对等场景同步：各发布本机机器人；物体只有权威端发布、镜像端应用。"""

    if not SCENE_SYNC_ENABLED:
        return ZmqSceneStateSyncActionCfg(asset_name="robot")
    return ZmqSceneStateSyncActionCfg(
        asset_name="robot",
        bind_endpoint=SCENE_SYNC_BIND_ENDPOINT,
        connect_endpoint=SCENE_SYNC_CONNECT_ENDPOINT,
        topic=_env_str("ISAACLAB_SCENE_SYNC_TOPIC", "scene_state"),
        local_sender_name=LOCAL_ROBOT_GLOBAL_NAME,
        publish_robots={LOCAL_ROBOT_GLOBAL_NAME: "robot"},
        apply_robots={PEER_ROBOT_GLOBAL_NAME: "peer_robot"},
        publish_object_names=SYNC_OBJECT_NAMES if OBJECT_AUTHORITY else (),
        apply_object_names=() if OBJECT_AUTHORITY else SYNC_OBJECT_NAMES,
        external_pump=SCENE_SYNC_MAINLOOP,
        # 主循环模式 pump 频率本身就是 step_hz（50Hz），不再需要 4:1 节流。
        publish_decimation=_env_int("ISAACLAB_SCENE_SYNC_PUBLISH_DECIMATION", 1 if SCENE_SYNC_MAINLOOP else 4),
        send_hwm=_env_int("ISAACLAB_SCENE_SYNC_SEND_HWM", 3),
        receive_hwm=_env_int("ISAACLAB_SCENE_SYNC_RECEIVE_HWM", 3),
        stale_timeout_s=_env_float("ISAACLAB_SCENE_SYNC_STALE_TIMEOUT_S", 0.5),
        stale_log_interval_s=_env_float("ISAACLAB_SCENE_SYNC_STALE_LOG_INTERVAL_S", 2.0),
    )


def _env_reset_sync_cfg() -> ZmqEnvResetSyncActionCfg:
    """复位事件保持源版单向语义：ID=1 是复位权威，ID=2 跟随。共用 PUB socket。"""

    if not SCENE_SYNC_ENABLED:
        return ZmqEnvResetSyncActionCfg(asset_name="robot")
    return ZmqEnvResetSyncActionCfg(
        asset_name="robot",
        role="publisher" if OBJECT_AUTHORITY else "subscriber",
        endpoint=SCENE_SYNC_BIND_ENDPOINT if OBJECT_AUTHORITY else SCENE_SYNC_CONNECT_ENDPOINT,
        topic=_env_str("ISAACLAB_ENV_RESET_SYNC_TOPIC", "env_reset"),
        external_pump=SCENE_SYNC_MAINLOOP,
        repeat_frames=_env_int("ISAACLAB_ENV_RESET_SYNC_REPEAT_FRAMES", 10),
        send_hwm=_env_int("ISAACLAB_SCENE_SYNC_SEND_HWM", 3),
        receive_hwm=_env_int("ISAACLAB_SCENE_SYNC_RECEIVE_HWM", 3),
    )


# ==================================================================
# 场景布局开关 ISAACLAB_TOTES_ON_CONVEYOR（默认 1）——语义与源分支一致：
#
#   1 = 流水线布局：两塑料筐缩小一半（scale 0.005）放上流水线滚轮面，
#       由 drive_totes 事件沿 -Y 从第一段送到第二段工位停住；双机站第二段两侧
#       (x=-4.75 / -6.7, y=14.148)，pushcart_2 空车留在 y=19.39363。
#   0 = 原布局：两筐恢复原尺寸（scale 0.01）叠放回 pushcart_2 拖车顶面
#       (x=-5.62, y=18.75)；双机回到拖车两侧工位；筐被机器人搬上入料端后
#       自动流到出料段停住（作业闭环）。
#
# conveyor_collider 碰撞板常驻不随开关回退；背景 USD 里烘入的桌子/料箱平移
# 与镜像改动也不随开关回退（要回退得换 USD 文件）。
# ==================================================================
TOTES_ON_CONVEYOR = _env_bool("ISAACLAB_TOTES_ON_CONVEYOR", True)

CART_GROUP_X = _env_float("ISAACLAB_CART_GROUP_X", -5.62)
CART_GROUP_Y = _env_float("ISAACLAB_CART_GROUP_Y", 18.75)
ROBOT_SIDE_OFFSET = _env_float("ISAACLAB_ROBOT_SIDE_OFFSET", 0.80)

ROBOT_WORKSTATION_Y = _env_float("ISAACLAB_ROBOT_WORKSTATION_Y", 14.148 if TOTES_ON_CONVEYOR else CART_GROUP_Y)
ROBOT_1_X = _env_float("ISAACLAB_ROBOT_1_X", -4.75 if TOTES_ON_CONVEYOR else CART_GROUP_X + ROBOT_SIDE_OFFSET)
ROBOT_2_X = _env_float("ISAACLAB_ROBOT_2_X", -6.7 if TOTES_ON_CONVEYOR else CART_GROUP_X - ROBOT_SIDE_OFFSET)

# 第二台拖车。流水线布局下是留在原工作位的空车；原布局下载着两筐顶到流水线入料端。
PUSHCART_2_POS = [-5.4, 19.39363, 0.0] if TOTES_ON_CONVEYOR else [CART_GROUP_X, CART_GROUP_Y, 0.0]

# 两塑料筐的初始摆放（几何推导见源分支 docs/场景布局开关-流水线与原布局切换.md）。
CART2_TOTE1_POS = [-5.35, 16.4, 0.775] if TOTES_ON_CONVEYOR else [CART_GROUP_X, CART_GROUP_Y, 0.3794]
CART2_TOTE2_POS = [-5.89, 17.0, 0.775] if TOTES_ON_CONVEYOR else [CART_GROUP_X, CART_GROUP_Y, 0.6814]

# 双机站位（面对面）：robot_1 在 +X 侧朝 -X（yaw 180°），robot_2 在 -X 侧朝 +X（identity）。
# ⚠️ SONIC 底座任务刻意保持 identity 出生朝向（policy/world 约定）；ID=1 的 180° yaw
# 出生是否影响 deploy 行走需 Phase 1 实测，异常时先用 ISAACLAB_ROBOT_YAW_IDENTITY=1
# 兜底（两台都 identity 朝 +X，牺牲面对面布局）。
# Isaac Lab 6 的配置四元数是 xyzw。这里保留具名常量，避免把源任务的 wxyz
# 字面量再次直接移植进来。
_ISAAC6_QUAT_IDENTITY_XYZW = (0.0, 0.0, 0.0, 1.0)
_ISAAC6_QUAT_YAW_180_XYZW = (0.0, 0.0, 1.0, 0.0)
_ISAAC6_QUAT_YAW_POS_90_XYZW = (0.0, 0.0, 0.70710678, 0.70710678)
_ISAAC6_QUAT_ROLL_POS_45_XYZW = (0.38268343, 0.0, 0.0, 0.92387953)

_ROBOT_YAW_IDENTITY = _env_bool("ISAACLAB_ROBOT_YAW_IDENTITY", False)
_ROBOT_1_ROT = (
    _ISAAC6_QUAT_IDENTITY_XYZW
    if _ROBOT_YAW_IDENTITY
    else _ISAAC6_QUAT_YAW_180_XYZW
)
_ROBOT_2_ROT = _ISAAC6_QUAT_IDENTITY_XYZW

LOCAL_ROBOT_POS = (
    (ROBOT_1_X, ROBOT_WORKSTATION_Y, 0.76) if LOCAL_ROBOT_ID == 1 else (ROBOT_2_X, ROBOT_WORKSTATION_Y, 0.76)
)
LOCAL_ROBOT_ROT = _ROBOT_1_ROT if LOCAL_ROBOT_ID == 1 else _ROBOT_2_ROT
PEER_ROBOT_POS = (
    (ROBOT_1_X, ROBOT_WORKSTATION_Y, 0.76) if PEER_ROBOT_ID == 1 else (ROBOT_2_X, ROBOT_WORKSTATION_Y, 0.76)
)
PEER_ROBOT_ROT = _ROBOT_1_ROT if PEER_ROBOT_ID == 1 else _ROBOT_2_ROT

# ==================================================================
# 流水线驱动参数（语义与源分支一致；镜像端强制关驱动，本地驱动会和同步打架，
# 且 CPU pipeline 下给 kinematic 筐写速度会刷爆 PhysX 错误上限掐停仿真）。
# ==================================================================
CONVEYOR_TOTE_NAMES = ("cart2_tote1", "cart2_tote2")
CONVEYOR_SPEED = _env_float("ISAACLAB_CONVEYOR_SPEED", 0.3)
CONVEYOR_Y_RECYCLE = _env_float("ISAACLAB_CONVEYOR_Y_RECYCLE", 10.6)
CONVEYOR_Y_RESPAWN = _env_float("ISAACLAB_CONVEYOR_Y_RESPAWN", 18.0)
CONVEYOR_Y_STOP = _env_float("ISAACLAB_CONVEYOR_Y_STOP", ROBOT_WORKSTATION_Y if TOTES_ON_CONVEYOR else 11.5)
CONVEYOR_ENABLED = _env_bool("ISAACLAB_CONVEYOR_ENABLED", True) and not MIRROR_OBJECTS

# 背景里的分拣料箱：bin_02 是动态刚体，开局下沉且会被机器人撞飞，锁成 kinematic。
BACKGROUND_LOCK_PRIM_NAMES = ("blue_sorting_bin_02",)


def _log_scene_layout() -> None:
    """启动就把身份、布局与驱动的实际生效值打出来，省得靠现象猜配置。"""

    tag = "[conveyor_env_cfg]"
    print(
        f"{tag} 本机身份: {LOCAL_ROBOT_GLOBAL_NAME}"
        f"（物体权威={'是' if OBJECT_AUTHORITY else '否'}，物体镜像={'是' if MIRROR_OBJECTS else '否'}）"
    )
    if SCENE_SYNC_ENABLED:
        print(
            f"{tag} 场景同步: bind={SCENE_SYNC_BIND_ENDPOINT} connect={SCENE_SYNC_CONNECT_ENDPOINT}"
            f" 发布节流=每{_env_int('ISAACLAB_SCENE_SYNC_PUBLISH_DECIMATION', 4)}物理步一帧"
        )
    else:
        print(f"{tag} 场景同步: 关 [ISAACLAB_SCENE_SYNC=0]")
    if TOTES_ON_CONVEYOR:
        print(f"{tag} 场景布局: 流水线（两筐缩半在传送带上流动） [ISAACLAB_TOTES_ON_CONVEYOR=1]")
    else:
        print(f"{tag} 场景布局: 原布局（两筐原尺寸叠在拖车上） [ISAACLAB_TOTES_ON_CONVEYOR=0]")
    print(
        f"{tag}   拖车/筐 x={PUSHCART_2_POS[0]:.3f} y={PUSHCART_2_POS[1]:.3f}"
        f" | robot_1 x={ROBOT_1_X:.3f} robot_2 x={ROBOT_2_X:.3f} y={ROBOT_WORKSTATION_Y:.3f}"
        f" | 流水线中线 x=-5.620 入料端 y=18.222"
    )
    print(
        f"{tag}   朝向(xyzw): robot_1={_ROBOT_1_ROT}"
        f"（{'朝 +X' if _ROBOT_YAW_IDENTITY else '朝 -X'}）"
        f" | robot_2={_ROBOT_2_ROT}（朝 +X）"
        f" | warehouse={_ISAAC6_QUAT_YAW_POS_90_XYZW}（+90° Z）"
    )
    drive = (
        "关"
        if not CONVEYOR_ENABLED
        else ("开，一路循环不停" if CONVEYOR_Y_STOP <= 0 else f"开，流到 y={CONVEYOR_Y_STOP:.3f} 停住")
    )
    print(f"{tag}   流水线驱动: {drive}（{CONVEYOR_SPEED} m/s 沿 -Y，作用于带面 y[10.19,18.22]）")


# ==================================================================
# 场景物体 spawn 工厂（镜像端翻 kinematic + 无碰撞去重力，maxDepenVel 只能 None 不能 0）
# ==================================================================


def _is_mirror_object(name: str) -> bool:
    """该物体在本机是否为 kinematic 镜像体。

    按名对齐 SYNC_OBJECT_NAMES：被移出同步清单的物体必须保持动态刚体，
    否则它在镜像端既无人驱动也不受物理，悬空冻结。
    """

    return MIRROR_OBJECTS and name in SYNC_OBJECT_NAMES


def _make_conveyor_cube_cfg(
    object_name: str,
    prim_name: str,
    initial_pos: tuple[float, float, float],
    color: tuple[float, float, float],
) -> RigidObjectCfg:
    """底座打包桌方块的镜像感知版：进了同步清单的方块在镜像端翻 kinematic。"""

    cfg = _make_sonic_cube_cfg(prim_name=prim_name, initial_pos=initial_pos, color=color)
    if _is_mirror_object(object_name):
        cfg.spawn.rigid_props = sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True)
    return cfg


def _make_pushcart_spawn_cfg(object_name: str) -> UsdFileCfg:
    is_mirror = _is_mirror_object(object_name)
    return UsdFileCfg(
        usd_path=str(_ASSETS_DIR / "props" / "pushcart_physics.usda"),
        scale=(0.5, 0.5, 1.0),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            kinematic_enabled=is_mirror,
            disable_gravity=is_mirror,
            solver_position_iteration_count=8,
            max_depenetration_velocity=None if is_mirror else 5.0,
        ),
    )


def _make_graspable_cart_box_spawn_cfg(object_name: str) -> UsdFileCfg:
    is_mirror = _is_mirror_object(object_name)
    return UsdFileCfg(
        usd_path=str(_ASSETS_DIR / "props" / "cart_box_d05_physics.usda"),
        mass_props=sim_utils.MassPropertiesCfg(mass=1.5),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            kinematic_enabled=is_mirror,
            disable_gravity=is_mirror,
            linear_damping=0.1,
            angular_damping=0.1,
            max_depenetration_velocity=None if is_mirror else 0.5,
            enable_gyroscopic_forces=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=2,
            sleep_threshold=0.0,
            stabilization_threshold=0.0,
        ),
    )


def _grasp_object_rigid_props() -> sim_utils.RigidBodyPropertiesCfg:
    """塑料筐权威端刚体参数——源分支实测能抓的参数包，保持同名环境变量可调。

    镜像/动态的分流由调用方按物体名判定（_is_mirror_object），这里只管权威端。
    """

    return sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        linear_damping=_env_float("ISAACLAB_GRASP_OBJECT_LINEAR_DAMPING", 0.05),
        angular_damping=_env_float("ISAACLAB_GRASP_OBJECT_ANGULAR_DAMPING", 5.0),
        max_angular_velocity=_env_float("ISAACLAB_GRASP_OBJECT_MAX_ANGULAR_VELOCITY", 90.0),
        max_contact_impulse=_env_optional_float("ISAACLAB_GRASP_OBJECT_MAX_CONTACT_IMPULSE"),
        enable_gyroscopic_forces=_env_bool("ISAACLAB_GRASP_OBJECT_ENABLE_GYROSCOPIC_FORCES", False),
        solver_position_iteration_count=_env_int("ISAACLAB_GRASP_OBJECT_SOLVER_POSITION_ITERATIONS", 12),
        solver_velocity_iteration_count=_env_int("ISAACLAB_GRASP_OBJECT_SOLVER_VELOCITY_ITERATIONS", 4),
        max_depenetration_velocity=_env_float("ISAACLAB_GRASP_OBJECT_MAX_DEPENETRATION_VELOCITY", 2.0),
    )


def _make_cart2_tote_spawn_cfg(object_name: str) -> UsdFileCfg:
    """塑料筐（Tote_B04）。高摩擦材质（2.0/1.6，combine=min）绑定在 usda 内。

    原 0.01 → 0.6×0.4×0.3 m；流水线布局缩小一半到 0.005 → 0.3×0.2×0.15 m
    （原点仍在筐底面）。TOTES_ON_CONVEYOR=0 时恢复原尺寸。
    """

    return UsdFileCfg(
        usd_path=str(_ASSETS_DIR / "props" / "tote_b04_physics.usda"),
        scale=(0.005, 0.005, 0.005) if TOTES_ON_CONVEYOR else (0.01, 0.01, 0.01),
        mass_props=sim_utils.MassPropertiesCfg(mass=_env_float("ISAACLAB_GRASP_OBJECT_MASS", 0.45)),
        rigid_props=(
            sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True)
            if _is_mirror_object(object_name)
            else _grasp_object_rigid_props()
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=_env_float("ISAACLAB_GRASP_OBJECT_CONTACT_OFFSET", 0.006),
            rest_offset=_env_float("ISAACLAB_GRASP_OBJECT_REST_OFFSET", 0.0),
        ),
    )


# ==================================================================
# 机器人：本机 = SONIC 底座 43-DoF（DDS 驱动）；对端 = 无碰撞无重力镜像体
# ==================================================================


def _make_local_robot_cfg() -> ArticulationCfg:
    cfg = make_sonic_robot_cfg()
    cfg.init_state.pos = LOCAL_ROBOT_POS
    cfg.init_state.rot = LOCAL_ROBOT_ROT
    return cfg


_PEER_ROBOT_USD = _ASSETS_DIR / "peer_robot" / "g1_43dof_peer.usd"

# 镜像体的无重力/零阻尼自由体刚体参数：关节与 root 由 scene_state 帧直写，
# 去穿透无意义，但 PhysX 不接受 0，只能不设置（None）。
_PEER_RIGID_PROPS = dict(
    disable_gravity=True,
    retain_accelerations=False,
    linear_damping=0.0,
    angular_damping=0.0,
    max_linear_velocity=1000.0,
    max_angular_velocity=1000.0,
    max_depenetration_velocity=None,
)


def _make_peer_robot_cfg() -> ArticulationCfg:
    cfg = make_sonic_robot_cfg()
    cfg.prim_path = "{ENV_REGEX_NS}/PeerRobot"
    cfg.init_state.pos = PEER_ROBOT_POS
    cfg.init_state.rot = PEER_ROBOT_ROT
    if _PEER_ROBOT_USD.exists():
        # 首选：tools/build_peer_robot_usd.py 烘好的无碰撞产物。
        # 不能直接在 URDF 直转的产物上关碰撞：UrdfConverter 固定输出
        # instanceable 格式，碰撞体在 instance 原型里，spawn 的 collision_props
        # 改不到（实测镜像体开局被地面弹出、无重力下 ~0.58 m/s 恒速上飘）。
        cfg.spawn = UsdFileCfg(
            usd_path=str(_PEER_ROBOT_USD),
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(**_PEER_RIGID_PROPS),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                # 镜像体关节/root 每帧由 scene_state 硬写,PD 只在两帧间兜底,
                # 求解精度不影响语义。默认 1/1:8/4 时第二台 43-DoF articulation
                # 的 CPU 求解要多吃 ~2-3ms/步,是闭环 50Hz 预算的大头之一
                # (2026-07-28 实测,闭环 E 14→11.5ms)。
                solver_position_iteration_count=_env_int("ISAACLAB_PEER_SOLVER_POS_ITERS", 1),
                solver_velocity_iteration_count=_env_int("ISAACLAB_PEER_SOLVER_VEL_ITERS", 1),
            ),
        )
    else:
        # 回退：URDF 直转（带碰撞体，镜像体会上飘）。不能 raise——tasks 包是
        # eager import，抛异常会把其他任务一起带崩。
        print(
            "[conveyor_env_cfg] ⚠️ 缺少无碰撞镜像机器人产物 "
            f"{_PEER_ROBOT_USD}\n"
            "[conveyor_env_cfg] ⚠️ 先运行: python tools/build_peer_robot_usd.py"
            "（否则 peer_robot 会被地面弹出上飘）"
        )
        cfg.spawn.rigid_props = sim_utils.RigidBodyPropertiesCfg(**_PEER_RIGID_PROPS)
        cfg.spawn.collision_props = sim_utils.CollisionPropertiesCfg(collision_enabled=False)
    return cfg


# ==================================================================
# 场景
# ==================================================================


@configclass
class G129SonicConveyorSceneCfg(G129SonicSceneCfg):
    """SONIC 底座（打包桌+三方块+本机 G1）+ warehouse 流水线工作区 + 对端镜像 G1。"""

    # warehouse 背景 USD 自带地面，去掉底座的无限地平面避免 z-fighting。
    ground = None

    background = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Background",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-4.68, 14.39363, 0],
            # +90 degrees around Z in Isaac Lab 6 xyzw order.
            rot=_ISAAC6_QUAT_YAW_POS_90_XYZW,
        ),
        spawn=UsdFileCfg(
            usd_path=str(_ASSETS_DIR / "warehouse-simple6_v48.usd"),
        ),
    )

    # 流水线（背景 USD ConveyorBelt_A08 ×3 段）只有视觉、无物理。这里补一块
    # 不可见 kinematic 碰撞板托住物体：顶面对齐滚轮顶 z≈0.772（板厚 0.04 →
    # 中心 z=0.752），覆盖滚轮可用宽度 x∈[-6.07,-5.17] 与整条 y 跨度 [10.19,18.22]。
    # 世界坐标由背景放置变换(pos=[-4.68,14.39363,0],rot=90°Z)换算自 USD 内几何。
    conveyor_collider = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/ConveyorCollider",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-5.62, 14.205, 0.752],
            rot=_ISAAC6_QUAT_IDENTITY_XYZW,
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.90, 8.03, 0.04),
            visible=False,  # 只提供碰撞，视觉沿用背景 USD 的流水线模型
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.003, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.8,
                dynamic_friction=0.6,
                restitution=0.0,
            ),
        ),
    )

    # 纸箱推车组（外侧位 x=-6.8）：拖车 + 两纸箱 + 顶上的长条测试箱。
    pushcart = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Pushcart",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-6.8, 19.39363, 0.0], rot=[0.0, 0.0, 0.0, 1.0]),
        spawn=_make_pushcart_spawn_cfg("pushcart"),
    )
    cart_box1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/CartBox1",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-6.8, 19.39363, 0.45], rot=[0.0, 0.0, 0.0, 1.0]),
        spawn=_make_graspable_cart_box_spawn_cfg("cart_box1"),
    )
    cart_box2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/CartBox2",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-6.8, 19.39363, 0.60], rot=[0.0, 0.0, 0.0, 1.0]),
        spawn=_make_graspable_cart_box_spawn_cfg("cart_box2"),
    )
    test_box = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TestBox",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-6.8, 19.39363, 1.095],
            rot=[0.0, 0.0, 0.0, 1.0],
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.20, 0.05, 0.10),
            rigid_props=(
                sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True)
                if _is_mirror_object("test_box")
                else sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False,
                    max_depenetration_velocity=3.0,
                )
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.25),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.003, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.2,
                dynamic_friction=0.9,
                restitution=0.0,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.76, 0.56, 0.28), roughness=0.70),
        ),
    )

    # 第二台拖车与两个塑料筐（位置随 TOTES_ON_CONVEYOR 切换，见上方常量段）。
    pushcart_2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Pushcart2",
        init_state=RigidObjectCfg.InitialStateCfg(pos=PUSHCART_2_POS, rot=[0.0, 0.0, 0.0, 1.0]),
        spawn=_make_pushcart_spawn_cfg("pushcart_2"),
    )
    cart2_tote1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cart2Tote1",
        init_state=RigidObjectCfg.InitialStateCfg(pos=CART2_TOTE1_POS, rot=[0.0, 0.0, 0.0, 1.0]),
        spawn=_make_cart2_tote_spawn_cfg("cart2_tote1"),
    )
    cart2_tote2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cart2Tote2",
        init_state=RigidObjectCfg.InitialStateCfg(pos=CART2_TOTE2_POS, rot=[0.0, 0.0, 0.0, 1.0]),
        spawn=_make_cart2_tote_spawn_cfg("cart2_tote2"),
    )

    # 底座打包桌三方块换成镜像感知版（位置/颜色与底座一致）：不同步的话
    # 两端各自模拟，任一端机器人碰一下就静默分叉。
    cube_1 = _make_conveyor_cube_cfg("cube_1", "Cube1", (0.31243, -0.00553, SONIC_CUBE_INITIAL_Z), (0.82, 0.66, 0.36))
    cube_2 = _make_conveyor_cube_cfg("cube_2", "Cube2", (0.31397, 0.10565, SONIC_CUBE_INITIAL_Z), (0.88, 0.72, 0.40))
    cube_3 = _make_conveyor_cube_cfg("cube_3", "Cube3", (0.41625, 0.04810, SONIC_CUBE_INITIAL_Z), (0.76, 0.56, 0.28))

    # 本机 G1：SONIC 底座同款（DDS 驱动、名字仍是 robot/prim Robot），只挪到工位。
    robot: ArticulationCfg = _make_local_robot_cfg()

    # 对端 G1 镜像体：由 scene_state 帧驱动。
    peer_robot: ArticulationCfg = _make_peer_robot_cfg()

    # 方向光制造明暗面，避免 DomeLight 均匀照明导致的"塑料感"。
    sun = AssetBaseCfg(
        prim_path="/World/sunLight",
        init_state=AssetBaseCfg.InitialStateCfg(rot=_ISAAC6_QUAT_ROLL_POS_45_XYZW),
        spawn=sim_utils.DistantLightCfg(color=(1.0, 0.98, 0.95), intensity=3000.0, angle=0.53),
    )


# ==================================================================
# MDP
# ==================================================================


@configclass
class ConveyorActionsCfg(SonicActionsCfg):
    """SONIC DDS 三段动作 + 两个零维同步 term（不改动作张量宽度）。"""

    scene_state_sync = _scene_state_sync_cfg()
    env_reset_sync = _env_reset_sync_cfg()


@configclass
class ConveyorEventsCfg:
    """背景料箱锁 kinematic + 流水线送筐（权威端才驱动）。"""

    lock_sorting_bins = EventTerm(
        func=conveyor_events.lock_background_rigid_bodies,
        mode="startup",
        params={
            "prim_names": BACKGROUND_LOCK_PRIM_NAMES,
            "parent_path": "Background/ConveyorBelt",
            "kinematic": True,
        },
    )

    drive_totes = EventTerm(
        func=conveyor_events.drive_totes_on_conveyor,
        mode="interval",
        interval_range_s=(0.02, 0.02),
        params={
            "object_names": CONVEYOR_TOTE_NAMES,
            "velocity_y": -CONVEYOR_SPEED,
            "enabled": CONVEYOR_ENABLED,
            "y_stop": CONVEYOR_Y_STOP if CONVEYOR_Y_STOP > 0 else None,
            "y_recycle": CONVEYOR_Y_RECYCLE,
            "y_respawn": CONVEYOR_Y_RESPAWN,
        },
    )


# 性能 A/B 诊断开关（默认全关，不影响任务语义）。用途:拆分 conveyor 相对底座
# SONIC 任务多出的 env.step 成本。取值逗号分隔,如 ISAACLAB_CONVEYOR_PERF_AB=no_peer,no_props:
#   no_peer      摘掉对端镜像机器人(第二台 43-DoF articulation)
#   no_props     摘掉流水线道具(双拖车/纸箱/测试箱/两筐,连带关 drive_totes)
#   plain_ground warehouse 背景换回底座的无限地平面(连带关 lock_sorting_bins)
# ⚠️ no_peer/no_props 只能配 ISAACLAB_SCENE_SYNC=0 用(同步 term 会引用被摘的实体)。
_PERF_AB = {
    item.strip()
    for item in os.environ.get("ISAACLAB_CONVEYOR_PERF_AB", "").split(",")
    if item.strip()
}
if _PERF_AB and SCENE_SYNC_ENABLED:
    raise RuntimeError(
        "ISAACLAB_CONVEYOR_PERF_AB 是诊断开关,必须与 ISAACLAB_SCENE_SYNC=0 联用"
    )

_PROP_NAMES = ("pushcart", "cart_box1", "cart_box2", "test_box", "pushcart_2", "cart2_tote1", "cart2_tote2")


@configclass
class G129SonicConveyorEnvCfg(G129SonicEnvCfg):
    """SONIC DDS 控制 + warehouse 流水线场景 + ZMQ 双机同步。"""

    scene: G129SonicConveyorSceneCfg = G129SonicConveyorSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )
    actions: ConveyorActionsCfg = ConveyorActionsCfg()
    events: ConveyorEventsCfg = ConveyorEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        # GUI 开局相机对准流水线工位(默认相机看世界原点,工作区在 (-5,14) 附近,
        # 打开就是空镜头还得手动飞过去)。
        self.viewer.eye = (-2.4, 11.9, 2.6)
        self.viewer.lookat = (-5.3, 14.5, 1.0)
        if _PERF_AB:
            print(f"[conveyor_env_cfg] ⚠️ 性能 A/B 诊断开关生效: {sorted(_PERF_AB)}")
            if "no_peer" in _PERF_AB:
                self.scene.peer_robot = None
            if "no_props" in _PERF_AB:
                for _name in _PROP_NAMES:
                    setattr(self.scene, _name, None)
                self.events.drive_totes = None
            if "plain_ground" in _PERF_AB:
                self.scene.background = None
                self.events.lock_sorting_bins = None
                self.scene.ground = AssetBaseCfg(
                    prim_path="/World/GroundPlane",
                    spawn=sim_utils.GroundPlaneCfg(
                        physics_material=sim_utils.RigidBodyMaterialCfg(
                            friction_combine_mode="multiply",
                            restitution_combine_mode="multiply",
                            static_friction=1.0,
                            dynamic_friction=1.0,
                            restitution=0.0,
                        )
                    ),
                )
        # fail-fast 放在任务被实际选中时（import 期不能抛，会连坐其他任务的注册）：
        # 回退的 URDF 直转 peer 带碰撞体，会被地面弹出后无重力恒速上飘。
        if not _PEER_ROBOT_USD.exists():
            raise RuntimeError(
                f"缺少无碰撞镜像机器人产物 {_PEER_ROBOT_USD}\n先运行: python tools/build_peer_robot_usd.py"
            )
        if MIRROR_OBJECTS:
            # 基座注册的 reset_scene_to_default 会向 kinematic 镜像物体写速度，
            # CPU pipeline 下每次复位刷 ~14 条 PhysX 错误、累计 1000 条掐停仿真。
            # 换成 kinematic 感知的安全版（只写位姿；位姿反正由权威帧接管）。
            from tasks.common_event.event_manager import SimpleEvent

            mirror_reset = SimpleEvent(func=conveyor_events.reset_scene_mirror_safe)
            self.event_manager.register("reset_object_self", mirror_reset)
            self.event_manager.register("reset_all_self", mirror_reset)
            if "cpu" not in str(self.sim.device):
                # 传送带 GPU 修复的教训：GPU pipeline 下 tensor API 写位姿驱不动
                # kinematic 体，镜像物体会静默冻结在出生位。
                print(
                    "[conveyor_env_cfg] ⚠️ 镜像端跑在 GPU pipeline "
                    f"({self.sim.device})：kinematic 物体位姿写入可能不生效，"
                    "镜像物体或将冻结——请用 --device cpu"
                )
        _log_scene_layout()
