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
# re: 曾用于 env 文件引用展开，现随加载逻辑移入 sync_identity
from pathlib import Path

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass

from tasks.common_observations.dex3_state import get_robot_dex3_joint_states
from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states

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
from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    ObservationsCfg as SonicObservationsCfg,
)

from . import conveyor_events
from .zmq_scene_sync import ZmqEnvResetSyncActionCfg, ZmqSceneStateSyncActionCfg

_ASSETS_DIR = Path(__file__).resolve().parent / "scene_assets"

# ==================================================================
# 配置加载：configs/scene_sync.env → os.environ.setdefault（进程 env 永远优先）
# ==================================================================

# 配置加载与身份解析收敛到 sync_identity（单一真源）：sim_main 也从同一份实现取
# 身份。曾经的双源判定（sim_main 字面比较进程 env vs 本文件 env 文件+int 解析）会在
# 「ID 写在 env 文件」或 "00" 这类写法下裂脑——一侧按 viewer 建场景、另一侧仍按对等端
# 选 sonic_dds/domain 1。详见 sync_identity 模块 docstring。
from .sync_identity import load_scene_sync_env, resolve_host_both_robots, resolve_local_robot_id

load_scene_sync_env(verbose_tag="[conveyor_env_cfg]")


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


# 身份解析走 sync_identity 单一真源（env 文件在文件头已加载过，这里不再重复加载）。
LOCAL_ROBOT_ID = resolve_local_robot_id(verbose_tag="[conveyor_env_cfg]", load_env=False)
# viewer 模式（ID=0）：本机不发布任何状态，把 robot_1/robot_2 与物体全部作为镜像应用。
# 本机 "robot" 资产退化为停在场外的 ghost（无碰撞/无重力/合并执行器），只为满足
# SONIC EnvCfg 的 action/observation 挂载，不参与画面构图。
VIEWER_MODE = LOCAL_ROBOT_ID == 0
# host 双机器人模式（工作包 B）：ID=1 + ISAACLAB_HOST_BOTH_ROBOTS=1——robot_1 与
# robot_2 都是本机全动力学 SONIC 机器人（各一套 deploy，第二套走 rt/r2/* 话题），
# 场景里没有镜像体，sync 双发 robot_1+robot_2+物体、只发不收。
HOST_MODE = resolve_host_both_robots(verbose_tag="[conveyor_env_cfg]", load_env=False)
# viewer 的上游固定是权威端 ID=1（连它的 PUB 端口）；对等模式保持 3-ID 互指。
PEER_ROBOT_ID = 1 if VIEWER_MODE else 3 - LOCAL_ROBOT_ID
LOCAL_ROBOT_GLOBAL_NAME = "viewer" if VIEWER_MODE else f"robot_{LOCAL_ROBOT_ID}"
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
# viewer 只收不发：bind 置空让同步 term 的发布方向干净失能（PUB 扇出在权威端，
# 任意多个 viewer 都 SUB 同一个 host:15555，host 无需感知 viewer 的存在）。
SCENE_SYNC_BIND_ENDPOINT = (
    "" if VIEWER_MODE else _env_str("ISAACLAB_SCENE_SYNC_BIND_ENDPOINT", f"tcp://0.0.0.0:{_MY_PORT}")
)
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
    """对等场景同步：各发布本机机器人；物体只有权威端发布、镜像端应用。

    viewer 模式（ID=0）：发布方向整体关闭，apply 挂 robot_1/robot_2 两个镜像体。
    在权威端只发 robot_1 的过渡期，robot_2 缺席由接收侧容缺跳过（见
    ``_parse_robot_states``），等 host 侧双机器人落地后自动补齐。
    """

    if not SCENE_SYNC_ENABLED:
        return ZmqSceneStateSyncActionCfg(asset_name="robot")
    connect_endpoint = SCENE_SYNC_CONNECT_ENDPOINT
    if VIEWER_MODE:
        publish_robots = {}
        apply_robots = {"robot_1": "peer_robot", "robot_2": "peer_robot_2"}
    elif HOST_MODE:
        # host 只发不收：双机器人 + 物体全部单向广播，viewer 的容缺 apply 自动补上
        # robot_2。connect 置空让接收方向干净失能（不建 SUB socket、不刷 stale）。
        publish_robots = {"robot_1": "robot", "robot_2": "robot_2"}
        apply_robots = {}
        connect_endpoint = ""
    else:
        publish_robots = {LOCAL_ROBOT_GLOBAL_NAME: "robot"}
        apply_robots = {PEER_ROBOT_GLOBAL_NAME: "peer_robot"}
    return ZmqSceneStateSyncActionCfg(
        asset_name="robot",
        bind_endpoint=SCENE_SYNC_BIND_ENDPOINT,
        connect_endpoint=connect_endpoint,
        topic=_env_str("ISAACLAB_SCENE_SYNC_TOPIC", "scene_state"),
        local_sender_name=LOCAL_ROBOT_GLOBAL_NAME,
        publish_robots=publish_robots,
        apply_robots=apply_robots,
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
#   1 = 流水线布局：两塑料筐缩小一半（scale 0.005）放上流水线滚轮面的**入料端**，
#       由 drive_totes 事件沿 -Y 送到第二段工位停住；双机站第二段两侧
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

# 两塑料筐在流水线上的出生 y：贴着入料端排布，保持原来 0.6 m 的前后错位
# （tote1 在前，沿 -Y 先到工位）。碰撞板 y 跨度 [10.19, 18.22]，筐在 y 方向半长
# 0.1 m，所以后车最多到 18.0 左右；再往上会悬出板尾。
#
# ⚠️ 这同时是**复位落点**：整场景复位（手动 rt/reset_pose/cmd、倒地自动、对端同步）
# 走的是 mdp.reset_scene_to_default，把筐写回这里的 init_state。放在入料端才能让
# 一次复位＝重新完整流一遍。早期取 16.4/17.0 时复位只剩 2.25/2.85 m 的行程。
# 到工位 14.148 分别是 3.25 / 3.85 m；筐是被拖拽滑行不是被带动，实测速度约
# 0.244 m/s（低于 velocity_y 的 0.3，见 README「已知限制 / 待实测」），所以约 13.3 / 15.8 s
# 到位，各自越过 y_stop 约 11 mm 后被动摩擦停住（smoke 900 步实测 14.136 / 14.137）。
TOTE_SPAWN_Y_LEAD = _env_float("ISAACLAB_TOTE_SPAWN_Y_LEAD", 17.4)
TOTE_SPAWN_Y_TRAIL = _env_float("ISAACLAB_TOTE_SPAWN_Y_TRAIL", 18.0)

# 两塑料筐的初始摆放（几何推导见源分支 docs/场景布局开关-流水线与原布局切换.md）。
CART2_TOTE1_POS = [-5.35, TOTE_SPAWN_Y_LEAD, 0.775] if TOTES_ON_CONVEYOR else [CART_GROUP_X, CART_GROUP_Y, 0.3794]
CART2_TOTE2_POS = [-5.89, TOTE_SPAWN_Y_TRAIL, 0.775] if TOTES_ON_CONVEYOR else [CART_GROUP_X, CART_GROUP_Y, 0.6814]

# 双机站位（面对面）：robot_1 在 +X 侧朝 -X（yaw 180°），robot_2 在 -X 侧朝 +X（identity）。
# ⚠️ SONIC 底座任务刻意保持 identity 出生朝向（policy/world 约定）；ID=1 的 180° yaw
# 出生是否影响 deploy 行走需 Phase 1 实测，异常时先用 ISAACLAB_ROBOT_YAW_IDENTITY=1
# 兜底（两台都 identity 朝 +X，牺牲面对面布局）。
_ROBOT_YAW_IDENTITY = _env_bool("ISAACLAB_ROBOT_YAW_IDENTITY", False)
_ROBOT_1_ROT = (1.0, 0.0, 0.0, 0.0) if _ROBOT_YAW_IDENTITY else (0.0, 0.0, 0.0, 1.0)
_ROBOT_2_ROT = (1.0, 0.0, 0.0, 0.0)

LOCAL_ROBOT_POS = (
    (ROBOT_1_X, ROBOT_WORKSTATION_Y, 0.76) if LOCAL_ROBOT_ID == 1 else (ROBOT_2_X, ROBOT_WORKSTATION_Y, 0.76)
)
LOCAL_ROBOT_ROT = _ROBOT_1_ROT if LOCAL_ROBOT_ID == 1 else _ROBOT_2_ROT
if VIEWER_MODE:
    # ghost 本机机器人停到工作区外（warehouse 在 (-5,14) 一带）。它无碰撞、无重力、
    # 由 hold action source 钉在默认站姿，只为满足 EnvCfg 挂载，不该出现在镜头里。
    LOCAL_ROBOT_POS = (0.0, -30.0, 0.76)
    LOCAL_ROBOT_ROT = (1.0, 0.0, 0.0, 0.0)
PEER_ROBOT_POS = (
    (ROBOT_1_X, ROBOT_WORKSTATION_Y, 0.76) if PEER_ROBOT_ID == 1 else (ROBOT_2_X, ROBOT_WORKSTATION_Y, 0.76)
)
PEER_ROBOT_ROT = _ROBOT_1_ROT if PEER_ROBOT_ID == 1 else _ROBOT_2_ROT
# viewer 的第二镜像体：robot_2 的工位。对等模式不用（保持 None，场景里不生成）。
PEER2_ROBOT_POS = (ROBOT_2_X, ROBOT_WORKSTATION_Y, 0.76)
PEER2_ROBOT_ROT = _ROBOT_2_ROT

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
    if VIEWER_MODE:
        print(f"{tag} 本机身份: viewer（纯镜像观看端，只收不发；apply=robot_1+robot_2+物体）")
    elif HOST_MODE:
        print(
            f"{tag} 本机身份: host（双机器人权威端，只发不收；"
            "publish=robot_1+robot_2+物体，robot_2 走 rt/r2/* 话题）"
        )
    else:
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
    if TOTES_ON_CONVEYOR:
        print(
            f"{tag}   筐出生/复位落点 y: tote1={CART2_TOTE1_POS[1]:.3f} tote2={CART2_TOTE2_POS[1]:.3f}"
            f"（整场景复位写回同一位置；碰撞板尽头 y=18.220）"
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
    if VIEWER_MODE:
        # viewer 的 ghost：复用无碰撞镜像体（无重力/合并执行器/solver 1/1），
        # prim 名保持 Robot、资产名保持 robot，SONIC 的 action/observation 挂载不变。
        cfg = _make_peer_robot_cfg()
        cfg.prim_path = "{ENV_REGEX_NS}/Robot"
        cfg.init_state.pos = LOCAL_ROBOT_POS
        cfg.init_state.rot = LOCAL_ROBOT_ROT
        # SONIC 底座场景的 foot_contact 传感器挂在 Robot/.*_ankle_roll_link 上，
        # 没有 contact reporter API 会在 gym.make 时 RuntimeError。ghost 无碰撞体，
        # 打开后传感器恒零，只为满足初始化。
        cfg.spawn.activate_contact_sensors = True
        # ghost 完全不可见：viewer 画面（尤其 AR 自由视角）里不该出现第三台机器人
        # （2026-08-02 用户在 AR 里看到 3 台实测反馈）。物理照常模拟，仅隐藏视觉。
        cfg.spawn.visible = False
        return cfg
    cfg = make_sonic_robot_cfg()
    cfg.init_state.pos = LOCAL_ROBOT_POS
    cfg.init_state.rot = LOCAL_ROBOT_ROT
    return cfg


def _make_second_local_robot_cfg() -> ArticulationCfg:
    """host 专用：robot_2 的全动力学本体（与 robot 同款 SONIC 底座，站 robot_2 工位）。

    ⚠️ 不得套用 _merge_peer_actuator_groups——执行器合并只允许用于镜像体；
    主动力学机器人的执行器组划分是 SONIC 动力学对齐红线（预算吃紧也不能动这刀，
    除非先过 SONIC 跟踪回归）。
    """
    cfg = make_sonic_robot_cfg()
    cfg.prim_path = "{ENV_REGEX_NS}/Robot2"
    cfg.init_state.pos = (ROBOT_2_X, ROBOT_WORKSTATION_Y, 0.76)
    cfg.init_state.rot = _ROBOT_2_ROT
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


def _merge_peer_actuator_groups(actuators: dict) -> dict:
    """镜像体专用：把 6 组执行器合并成 1 组 Implicit。

    Isaac Lab 对每组执行器在**每个物理子步**都有一轮 Python 张量记账
    （articulation._apply_actuator_model），组数直接乘在 CPU 开销上——py-spy 实测
    该记账占主线程 39%（两台机器人合计 ~7.6ms/圈，win2 headless，2026-07-31）。
    镜像体关节每帧被 scene_state 直写，PD 只在两帧间兜底；合并时逐关节参数原样
    并入 dict，每个关节的驱动参数不变，纯减组数 6→1。
    ⚠️ 只用于镜像体；主机器人的组划分随 SONIC 动力学对齐验证走，不动。
    """

    fields = (
        "effort_limit", "velocity_limit", "effort_limit_sim", "velocity_limit_sim",
        "stiffness", "damping", "armature", "friction", "dynamic_friction",
        "viscous_friction",
    )
    exprs: list[str] = []
    merged: dict[str, dict] = {f: {} for f in fields}
    for group in actuators.values():
        exprs.extend(group.joint_names_expr)
        for f in fields:
            value = getattr(group, f, None)
            if value is None:
                continue
            if isinstance(value, dict):
                merged[f].update(value)
            else:
                for expr in group.joint_names_expr:
                    merged[f][expr] = value
    # 只设真正出现过的字段；某字段只有部分组设置时（如 hands 的摩擦三项），
    # 未匹配的关节由 Isaac Lab 参数解析回落到 USD 默认值——与原多组行为一致。
    kwargs = {f: v for f, v in merged.items() if v}
    return {"peer_all": ImplicitActuatorCfg(joint_names_expr=exprs, **kwargs)}


def _make_peer_robot_cfg() -> ArticulationCfg:
    cfg = make_sonic_robot_cfg()
    cfg.prim_path = "{ENV_REGEX_NS}/PeerRobot"
    cfg.init_state.pos = PEER_ROBOT_POS
    cfg.init_state.rot = PEER_ROBOT_ROT
    cfg.actuators = _merge_peer_actuator_groups(cfg.actuators)
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


def _make_second_peer_robot_cfg() -> ArticulationCfg:
    """viewer 专用：robot_2 的镜像体（与 peer_robot 同一份无碰撞产物）。"""
    cfg = _make_peer_robot_cfg()
    cfg.prim_path = "{ENV_REGEX_NS}/PeerRobot2"
    cfg.init_state.pos = PEER2_ROBOT_POS
    cfg.init_state.rot = PEER2_ROBOT_ROT
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
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-4.68, 14.39363, 0], rot=[0.7071, 0.0, 0.0, 0.7071]),
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
            rot=[1.0, 0.0, 0.0, 0.0],
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
    # viewer 模式下退化为场外 ghost（见 _make_local_robot_cfg）。
    robot: ArticulationCfg = _make_local_robot_cfg()

    # 对端 G1 镜像体：由 scene_state 帧驱动。viewer 模式下它镜像 robot_1；
    # host 模式下两台都是真身，不需要镜像体。
    peer_robot: ArticulationCfg | None = None if HOST_MODE else _make_peer_robot_cfg()

    # viewer 专用第二镜像体（robot_2）。对等/host 模式为 None，场景里不生成。
    peer_robot_2: ArticulationCfg | None = _make_second_peer_robot_cfg() if VIEWER_MODE else None

    # host 专用：robot_2 全动力学本体 + 它的足底接触诊断（参数照抄底座 foot_contact）。
    robot_2: ArticulationCfg | None = _make_second_local_robot_cfg() if HOST_MODE else None
    foot_contact_2: ContactSensorCfg | None = (
        ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot2/.*_ankle_roll_link",
            history_length=4,
            track_air_time=True,
            force_threshold=5.0,
            debug_vis=False,
        )
        if HOST_MODE
        else None
    )

    # 方向光制造明暗面，避免 DomeLight 均匀照明导致的"塑料感"。
    sun = AssetBaseCfg(
        prim_path="/World/sunLight",
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.9238795, 0.3826834, 0.0, 0.0)),
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
class HostConveyorActionsCfg(SonicActionsCfg):
    """host 双机器人：robot_1 三段（继承）+ robot_2 三段 + 两个零维同步 term。

    configclass 的继承字段序决定 action_manager 切片序：
    [r1_q(43), r1_dq(43), r1_tau(43), r2_q(43), r2_dq(43), r2_tau(43)] = 258 维，
    与 SonicDDSHostActionProvider 的拼接顺序逐段对应。
    """

    joint_pos_2 = mdp.JointPositionActionCfg(
        asset_name="robot_2",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=False,
        preserve_order=True,
    )
    joint_vel_2 = mdp.JointVelocityActionCfg(
        asset_name="robot_2",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=False,
        preserve_order=True,
    )
    joint_effort_2 = mdp.JointEffortActionCfg(
        asset_name="robot_2",
        joint_names=[".*"],
        scale=1.0,
        preserve_order=True,
    )
    scene_state_sync = _scene_state_sync_cfg()
    env_reset_sync = _env_reset_sync_cfg()


@configclass
class HostObservationsCfg:
    """host 双机器人观测组：每台机器人各一对 DDS 副作用 ObsTerm。

    ⚠️ 这些"观测"是 rt/lowstate 与 rt/dex3/*/state（及 rt/r2/* 对应话题）的发布
    载体——删掉任何一个，对应 deploy 就收不到状态、锁步永远握不上手。
    """

    @configclass
    class PolicyCfg(ObsGroup):
        robot_body_state = ObsTerm(
            func=get_robot_boy_joint_states,
            params={"dds_min_interval_ms": 0.0},
        )
        robot_dex3_state = ObsTerm(
            func=get_robot_dex3_joint_states,
            params={"dds_min_interval_ms": 0.0},
        )
        robot2_body_state = ObsTerm(
            func=get_robot_boy_joint_states,
            params={
                "dds_min_interval_ms": 0.0,
                "asset_name": "robot_2",
                "dds_object_name": "g129_r2",
            },
        )
        robot2_dex3_state = ObsTerm(
            func=get_robot_dex3_joint_states,
            params={
                "dds_min_interval_ms": 0.0,
                "asset_name": "robot_2",
                "dds_object_name": "dex3_r2",
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


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
    actions: ConveyorActionsCfg | HostConveyorActionsCfg = (
        HostConveyorActionsCfg() if HOST_MODE else ConveyorActionsCfg()
    )
    # host 模式换成双机器人观测组（robot_2 的两个 DDS 副作用 ObsTerm 是 rt/r2/* 状态
    # 话题的发布载体）；其余模式沿用底座观测组。
    observations: SonicObservationsCfg | HostObservationsCfg = (
        HostObservationsCfg() if HOST_MODE else SonicObservationsCfg()
    )
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
        # viewer 不开同步就是一屋子静止 ghost，没有任何意义，直接拒绝启动。
        if VIEWER_MODE and not SCENE_SYNC_ENABLED:
            raise RuntimeError(
                "viewer 模式（ISAACLAB_LOCAL_ROBOT_ID=0）必须开场景同步："
                "需要 ISAACLAB_SCENE_SYNC=1 且已安装 pyzmq"
            )
        # 回退的 URDF 直转 peer 带碰撞体，会被地面弹出后无重力恒速上飘。
        # host 模式没有镜像体，不需要该产物。
        if not HOST_MODE and not _PEER_ROBOT_USD.exists():
            raise RuntimeError(
                f"缺少无碰撞镜像机器人产物 {_PEER_ROBOT_USD}\n先运行: python tools/build_peer_robot_usd.py"
            )
        # XR 锚定重定向。⚠️ 只改 self.xr 不够——teleop_devices 构建时把 xr_cfg
        # **拷贝**了一份（2026-08-02 实测：cfg 打印挂 PeerRobot、teleop 设备实际仍用
        # Robot 路径，AR 视角锚到场外 ghost 上）。必须把 self.xr 与每个 teleop 设备
        # 持有的 xr_cfg 一起改。
        def _apply_xr_anchor(prim_name: str, tag: str) -> None:
            anchor = f"/World/envs/env_0/{prim_name}/torso_link/head_link"
            rotation = f"/World/envs/env_0/{prim_name}/pelvis"
            targets = [self.xr]
            for _dev_cfg in getattr(self.teleop_devices, "devices", {}).values():
                _dev_xr = getattr(_dev_cfg, "xr_cfg", None)
                if _dev_xr is not None and _dev_xr is not self.xr:
                    targets.append(_dev_xr)
            for _xr in targets:
                _xr.anchor_prim_path = anchor
                _xr.anchor_rotation_prim_path = rotation
            print(f"[conveyor_env_cfg] XR 锚定 -> {prim_name}（{tag}，含 {len(targets)} 份 xr_cfg）")

        # host 模式可选：XR 锚定切到 robot_2（默认锚 robot_1，即父类写的 Robot prim 路径）。
        if HOST_MODE and _env_str("ISAACLAB_XR_ANCHOR_ROBOT_ID", "1") == "2":
            _apply_xr_anchor("Robot2", "host")
        # viewer 模式（工作包 C）：本机 Robot 是场外 ghost（不可见），父类默认锚会把
        # AR 视角带到空地上。改挂镜像体：ISAACLAB_XR_ANCHOR_ROBOT_ID=1 → PeerRobot
        # （robot_1 镜像，默认），=2 → PeerRobot2（robot_2 镜像）。镜像 USD 与本体同一
        # URDF 转换，torso_link/head_link 与 pelvis 的层级一致。
        if VIEWER_MODE:
            _anchor_prim = "PeerRobot2" if _env_str("ISAACLAB_XR_ANCHOR_ROBOT_ID", "1") == "2" else "PeerRobot"
            _apply_xr_anchor(_anchor_prim, "viewer")
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
