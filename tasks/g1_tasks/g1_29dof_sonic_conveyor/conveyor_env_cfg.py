# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""SONIC G1 + warehouse 流水线场景 + ZMQ 双机同步。

场景与流水线驱动移植自 IsaacLab 分叉 feat/conveyor-loop-totes-wip 的
``pick_place/locomanipulation_g1_env_cfg.py``（tip 17e71c0a0），机器人与
DDS/XR 链路沿用本工程 ``g1_29dof_dex3_sonic`` 的 SONIC 底座。

双机形态（对等/混合权威）：
* 每台机器跑本文件的同一个任务，用 ``ISAACLAB_LOCAL_ROBOT_ID``（1/2）区分身份；
* 本机 G1 名为 ``robot``（prim ``Robot``，DDS/观测/XR 全链路与 SONIC 任务一致），
  对端 G1 是 ``peer_robot`` 镜像体（默认无碰撞 articulation；可 opt-in 为纯显示
  USD Xform/FK，两者都由 scene_state 帧驱动）；
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
from isaaclab.utils.assets import NVIDIA_NUCLEUS_DIR

from tasks.common_observations.dex3_state import get_robot_dex3_joint_states
from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states

from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    SONIC_CUBE_INITIAL_Z,
    SONIC_PACKING_TABLE_USD,
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
from .asset_variants import (
    VISUAL_ONLY_BACKGROUND_USD,
    WORKCELL_LITE_VISUAL_ONLY_BACKGROUND_USD,
    resolve_conveyor_background_usd,
)
from .background_assets import resolve_background_asset
from .contact_modes import resolve_contact_report_mode
from .contact_spawners import configure_robot_contact_reports
from .conveyor_drive import (
    BELT_COLLIDER_THICKNESS,
    BELT_TOP_Z,
    BELT_WIDTH,
    BELT_X_CENTER,
    BELT_Y_MAX,
    BELT_Y_MIN,
    CONVEYOR_NORTH_SHIFT_Y,
    resolve_conveyor_drive,
)
from . import endless_intake
from .endless_intake import resolve_endless_intake
from .peer_visual_lod import JOINT_NAMES as PEER_VISUAL_LOD_JOINT_NAMES
from .peer_visual_lod import resolve_peer_robot_mode
from .scene_layout import resolve_scene_layout
from .scene_props import resolve_scene_props
from .tote_assets import resolve_tote_asset
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

CONTACT_REPORT_MODE = resolve_contact_report_mode(os.environ)
TOTE_COLLIDER_MODE, TOTE_USD_PATH = resolve_tote_asset(_ASSETS_DIR / "props", os.environ)


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


# 布局与道具必须在同步清单之前解析：精简布局不生成的资产也不能进入
# scene_state，否则同步 term 会按旧名称查找不存在的场景实体。
SCENE_LAYOUT = resolve_scene_layout(os.environ)
TOTES_ON_CONVEYOR = SCENE_LAYOUT.totes_on_conveyor
BELT_BOX_NAMES = SCENE_LAYOUT.belt_box_names
SCENE_PROPS = resolve_scene_props(
    os.environ,
    totes_on_conveyor=TOTES_ON_CONVEYOR,
    belt_box_names=BELT_BOX_NAMES,
)
# 看不到头的入料端：西拐弯道 + X 支线×5（纯视觉资产 + kinematic 托面延伸 + 沿
# 路径距离的两段式驱动；**不新增货架**，遮挡依赖背景既有结构——Δ=0.25 整体北移
# 让支线从货架排 B 北侧擦过、回生点深藏排 B 后面：E2 全遮 / E1 缝隙残余，见
# endless_intake docstring 与 README 已知限制）。只在流水线布局且道具策略为
# layout 时生成——=0 拖车组与 legacy_props 空车都落在弯道占位内。
ENDLESS_INTAKE = resolve_endless_intake(
    os.environ,
    totes_on_conveyor=TOTES_ON_CONVEYOR,
    props_mode=SCENE_PROPS.mode,
)


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

# 显式 opt-in：默认继续使用已验过的无碰撞 articulation 镜像；visual_lod
# 改为纯 USD Xform/FK 后端，不创建镜像 articulation、刚体、碰撞、执行器或传感器。
PEER_ROBOT_MODE = resolve_peer_robot_mode(os.environ.get("ISAACLAB_PEER_ROBOT_MODE"))
PEER_VISUAL_LOD = PEER_ROBOT_MODE == "visual_lod"

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

# 进 scene_state 帧的场景物体（两端清单必须逐字一致）。默认清单直接取布局实际
# 生成的刚体：流水线布局只有两筐，推车布局是 pushcart_2+两筐；legacy_props 才恢复
# 旧的两组推车、纸箱、test_box 和打包桌三方块。
SYNC_OBJECT_NAMES = _env_str_tuple(
    "ISAACLAB_SCENE_SYNC_OBJECTS",
    SCENE_PROPS.spawned_names,
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
    apply_visual_robots = {}
    if VIEWER_MODE:
        publish_robots = {}
        if PEER_VISUAL_LOD:
            apply_robots = {}
            apply_visual_robots = {
                "robot_1": "/World/envs/env_0/PeerRobot",
                "robot_2": "/World/envs/env_0/PeerRobot2",
            }
        else:
            apply_robots = {"robot_1": "peer_robot", "robot_2": "peer_robot_2"}
    elif HOST_MODE:
        # host 只发不收：双机器人 + 物体全部单向广播，viewer 的容缺 apply 自动补上
        # robot_2。connect 置空让接收方向干净失能（不建 SUB socket、不刷 stale）。
        publish_robots = {"robot_1": "robot", "robot_2": "robot_2"}
        apply_robots = {}
        connect_endpoint = ""
    else:
        publish_robots = {LOCAL_ROBOT_GLOBAL_NAME: "robot"}
        if PEER_VISUAL_LOD:
            apply_robots = {}
            apply_visual_robots = {
                PEER_ROBOT_GLOBAL_NAME: "/World/envs/env_0/PeerRobot",
            }
        else:
            apply_robots = {PEER_ROBOT_GLOBAL_NAME: "peer_robot"}
    return ZmqSceneStateSyncActionCfg(
        asset_name="robot",
        bind_endpoint=SCENE_SYNC_BIND_ENDPOINT,
        connect_endpoint=connect_endpoint,
        topic=_env_str("ISAACLAB_SCENE_SYNC_TOPIC", "scene_state"),
        local_sender_name=LOCAL_ROBOT_GLOBAL_NAME,
        publish_robots=publish_robots,
        apply_robots=apply_robots,
        apply_visual_robots=apply_visual_robots,
        visual_robot_joint_names=PEER_VISUAL_LOD_JOINT_NAMES if apply_visual_robots else (),
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
#       (x=-4.54 / -6.7, y=14.398)，pushcart_2 空车留在 y=19.64363
#       （⚠️ robot_1 x=-4.54 是 2026-08-09 的对称化站位，不是 01cdfaf 的 -4.75；
#       所有世界 y 已含整体北移 Δ=0.25）。
#   0 = 原布局：两筐恢复原尺寸（scale 0.01）叠放回 pushcart_2 拖车顶面
#       (x=-5.62, y=19.0)；双机回到拖车两侧工位；筐被机器人搬上入料端后
#       自动流到出料段停住（作业闭环）。
#
# conveyor_collider 碰撞板常驻不随开关回退；背景 USD 里烘入的桌子/料箱平移
# 与镜像改动也不随开关回退（要回退得换 USD 文件）。
# ==================================================================
CART_GROUP_X = SCENE_LAYOUT.cart_group_x
CART_GROUP_Y = SCENE_LAYOUT.cart_group_y
ROBOT_SIDE_OFFSET = SCENE_LAYOUT.robot_side_offset

ROBOT_WORKSTATION_Y = SCENE_LAYOUT.robot_workstation_y
ROBOT_1_X = SCENE_LAYOUT.robot_1_x
ROBOT_2_X = SCENE_LAYOUT.robot_2_x

# 第二台拖车。流水线布局下是留在原工作位的空车；原布局下载着两筐顶到流水线入料端。
PUSHCART_2_POS = list(SCENE_LAYOUT.pushcart_2_pos)

# 两塑料筐在流水线上的出生 y：贴着入料端排布，保持原来 0.6 m 的前后错位
# （tote1 在前，沿 -Y 先到工位）。碰撞板 y 跨度 [10.44, 18.47]，筐在 y 方向半长
# 0.1 m，所以后车最多到 18.25 左右；再往上会悬出板尾。
#
# ⚠️ 这同时是**复位落点**：整场景复位（手动 rt/reset_pose/cmd、倒地自动、对端同步）
# 走的是 mdp.reset_scene_to_default，把筐写回这里的 init_state。放在入料端才能让
# 一次复位＝重新完整流一遍。早期取 16.4/17.0 时复位只剩 2.25/2.85 m 的行程。
# 到工位 14.398 分别是 3.25 / 3.85 m。筐是被拖拽滑行不是被带动，实测速度约
# 0.244 m/s（低于 velocity_y 的 0.3，见 README「已知限制 / 待实测」），所以约
# 13.3 / 15.8 s 到位，各自越过 y_stop 约 11 mm 后被动摩擦停住
# （smoke 900 步实测 14.136 / 14.137）。
TOTE_SPAWN_Y_LEAD = SCENE_LAYOUT.cart2_tote1_pos[1]
TOTE_SPAWN_Y_TRAIL = SCENE_LAYOUT.cart2_tote2_pos[1]

# 两塑料筐的初始摆放（几何推导见源分支 docs/场景布局开关-流水线与原布局切换.md）。
CART2_TOTE1_POS = list(SCENE_LAYOUT.cart2_tote1_pos)
CART2_TOTE2_POS = list(SCENE_LAYOUT.cart2_tote2_pos)

# 流水线纸箱队列（v61 的 ConveyorBelt_Box 同款视觉资产，见
# scene_assets/props/cart_box_d01_physics.usda）。弯道形态默认 17 个（pitch 0.75
# 铺满整条上游路径；直线回退默认 5），只排在工位 ``ROBOT_WORKSTATION_Y``
# **上游**那一段带面上——布局与校验在 scene_layout。
#
# ⚠️ 与两塑料筐时代同一条约定：出生 y 同时是**复位落点**（整场景复位走
# mdp.reset_scene_to_default，把箱子写回这里的 init_state），排在上游才能让一次
# 复位＝重新完整流一遍。
#
# ⚠️ 背景 USD 里那 10 个 ConveyorBelt_Box + 5 个 KLT_Bin 装饰物必须保持隐藏
# （warehouse-simple6_v61_visual_only.usda 里 active=false）：它们的 v48→v61 换版
# 遗留坐标让箱底 0.633 对带面 0.772（陷 14 cm），留着会和这里的真箱子视觉打架。
BELT_BOX_POSITIONS = [list(pos) for pos in SCENE_LAYOUT.belt_box_positions]
BELT_BOX_KINDS = SCENE_LAYOUT.belt_box_kinds
BELT_BOX_HALF_LENGTHS = SCENE_LAYOUT.belt_box_half_lengths
BELT_BOX_QUEUE_GAP = SCENE_LAYOUT.belt_box_queue_gap
# 箱子短边沿输送方向 Y，机器人从流水线侧面抱取时双臂跨距更小；保持资产原始朝向。
BELT_BOX_ROT = [1.0, 0.0, 0.0, 0.0]

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
# 流水线驱动参数。默认 legacy 保持历史行为；显式设置
# ISAACLAB_CONVEYOR_DRIVE_MODE=surface_velocity 才启用 PhysX 接触驱动。
# surface_velocity 比 legacy 的单机诊断语义更严格：固定只允许物体权威 ID=1 启用，
# 即便 ID=2 临时关闭同步也不会自己驱动，避免两端恢复同步后状态分叉。
# ==================================================================
# 只驱动布局实际 spawn 出来的物体：流水线布局的作业对象已换成纸箱队列，两塑料筐
# 只在推车布局（和 legacy_props 回退）里还存在。清单跟 SCENE_PROPS 走，避免事件
# 按旧名字去 env.scene 里查不存在的实体。
CONVEYOR_TOTE_NAMES = tuple(
    name for name in ("cart2_tote1", "cart2_tote2") if SCENE_PROPS.spawns(name)
)
CONVEYOR_BELT_BOX_NAMES = tuple(name for name in BELT_BOX_NAMES if SCENE_PROPS.spawns(name))
# 与 CONVEYOR_BELT_BOX_NAMES 严格同序同长——驱动事件按下标一一对应地取半长。
CONVEYOR_BELT_BOX_HALF_LENGTHS = tuple(
    half
    for name, half in zip(BELT_BOX_NAMES, BELT_BOX_HALF_LENGTHS)
    if SCENE_PROPS.spawns(name)
)
CONVEYOR_DRIVE = resolve_conveyor_drive(
    os.environ,
    object_authority=OBJECT_AUTHORITY,
    mirror_objects=MIRROR_OBJECTS,
    default_y_stop=SCENE_LAYOUT.conveyor_y_stop,
    # 移动带面在期望停位上游 handoff_offset 处结束，取作业对象沿带方向的半长。
    # 流水线布局有两种箱型，取**队首**那个（它才是停在工位上的那个）；推车布局
    # 仍是原尺寸塑料筐（0.20）。
    default_handoff_offset=(
        (BELT_BOX_HALF_LENGTHS[0] if BELT_BOX_HALF_LENGTHS else 0.19)
        if TOTES_ON_CONVEYOR
        else 0.20
    ),
)
CONVEYOR_DRIVE_MODE = CONVEYOR_DRIVE.mode
CONVEYOR_SPEED = CONVEYOR_DRIVE.speed
CONVEYOR_VELOCITY_Y = CONVEYOR_DRIVE.velocity_y
CONVEYOR_Y_RECYCLE = CONVEYOR_DRIVE.y_recycle
CONVEYOR_Y_RESPAWN = CONVEYOR_DRIVE.y_respawn
CONVEYOR_Y_STOP = CONVEYOR_DRIVE.y_stop
CONVEYOR_LEGACY_ENABLED = CONVEYOR_DRIVE.legacy_enabled
CONVEYOR_SURFACE_VELOCITY_ENABLED = CONVEYOR_DRIVE.surface_velocity_enabled
CONVEYOR_SURFACE_RECYCLE_ENABLED = CONVEYOR_DRIVE.surface_recycle_enabled
CONVEYOR_ENABLED = CONVEYOR_LEGACY_ENABLED or CONVEYOR_SURFACE_VELOCITY_ENABLED

# 循环模式（ISAACLAB_CONVEYOR_Y_STOP<=0 ⇒ y_stop=None）下纸箱同样需要回收。
# 塑料筐的 legacy 驱动自带 loop 分支会把筐传回入料端，但纸箱走的是
# drive_belt_boxes_on_conveyor——它只管排队和驱动，不搬运。若不额外开回收事件，
# legacy + 循环模式的纸箱会一路流出带尾后失去驱动、堆死在出料端。
CONVEYOR_BELT_BOX_RECYCLE_ENABLED = (
    bool(CONVEYOR_BELT_BOX_NAMES) and CONVEYOR_Y_STOP is None and CONVEYOR_ENABLED
)
CONVEYOR_RECYCLE_ENABLED = CONVEYOR_SURFACE_RECYCLE_ENABLED or CONVEYOR_BELT_BOX_RECYCLE_ENABLED

# 循环模式容量复核（17 箱铺满后新增的 fail-fast）：整列在"回生点→回收线→瞬移回
# 回生点"的环路上循环，等价一个周长 = s(y_recycle) − RESPAWN_S 的圆（默认弯道
# 形态 ≈ 15.7425 − (-3.90) = 19.64 m）。整带同速 ⇒ 出生间距永久保持，唯一会变
# 的是"回绕缺口"= 周长 − 出生跨度（17 箱 × pitch 0.75 ⇒ 跨度 12.0，缺口 ≈7.64
# m）；缺口 < 首尾半长和 + queue_gap 才会追尾。默认远够，这里拦的是以后加箱/
# 调 pitch/收 y_recycle 时的无声追尾（回收事件每 20 ms 判一次，瞬移落点抖动
# ≤ 带速×20ms ≈ 6 mm，量级不影响判据）。默认停止模式（y_stop>0）不进环路，
# 该检查不触发。
if CONVEYOR_BELT_BOX_RECYCLE_ENABLED and len(BELT_BOX_POSITIONS) > 1:
    if ENDLESS_INTAKE.enabled:
        _loop_spawn_ss = [
            endless_intake.path_s_of_point(pos[0], pos[1]) for pos in BELT_BOX_POSITIONS
        ]
        _loop_length = (
            endless_intake.path_s_of_main_y(CONVEYOR_Y_RECYCLE) - endless_intake.RESPAWN_S
        )
    else:
        # 直线回退：进度 = -y（下游递增），环路 = y_respawn → y_recycle。
        _loop_spawn_ss = [-pos[1] for pos in BELT_BOX_POSITIONS]
        _loop_length = CONVEYOR_Y_RESPAWN - CONVEYOR_Y_RECYCLE
    _loop_wrap_gap = _loop_length - (max(_loop_spawn_ss) - min(_loop_spawn_ss))
    _loop_wrap_need = (
        CONVEYOR_BELT_BOX_HALF_LENGTHS[0]
        + CONVEYOR_BELT_BOX_HALF_LENGTHS[-1]
        + BELT_BOX_QUEUE_GAP
    )
    if _loop_wrap_gap < _loop_wrap_need:
        raise ValueError(
            f"循环模式下 {len(BELT_BOX_POSITIONS)} 箱在回生环路上放不下："
            f"环路周长 {_loop_length:.3f} m − 队列跨度 "
            f"{max(_loop_spawn_ss) - min(_loop_spawn_ss):.3f} m = 回绕缺口 "
            f"{_loop_wrap_gap:.3f} m < 首尾防撞下限 {_loop_wrap_need:.3f} m；"
            "请调小 ISAACLAB_BELT_BOX_COUNT / SPAWN_PITCH 或下调 ISAACLAB_CONVEYOR_Y_RECYCLE"
        )

# 背景清理和输送机物理是两层独立选择：默认 legacy 后端沿用第一阶段的 clean
# background，但保留 ConveyorBelt02 原生物理；显式资产开关可让 legacy 也使用纯视觉
# 输送机。Surface Velocity 强制使用纯视觉输送机 adapter（按 baseline 路由：完整
# 仓库背景叠加 clean background，workcell_lite 走保持轻量工位的专用组合层），
# 从 USD 组合阶段移除原生 6 个刚体/56 个碰撞，确保任务 proxy 是唯一接触面。
_REQUESTED_BACKGROUND_MODE, _BACKGROUND_BASE_USD_PATH = resolve_background_asset(
    _ASSETS_DIR
)
BACKGROUND_USD_PATH = resolve_conveyor_background_usd(
    _ASSETS_DIR,
    baseline_path=_BACKGROUND_BASE_USD_PATH,
    drive_mode=CONVEYOR_DRIVE_MODE,
)
CONVEYOR_VISUAL_ONLY_ASSET_ENABLED = BACKGROUND_USD_PATH.name in {
    VISUAL_ONLY_BACKGROUND_USD,
    WORKCELL_LITE_VISUAL_ONLY_BACKGROUND_USD,
}
if BACKGROUND_USD_PATH == _BACKGROUND_BASE_USD_PATH:
    BACKGROUND_MODE = _REQUESTED_BACKGROUND_MODE
else:
    # workcell-lite 走保持轻量 baseline 的专用组合层；完整仓库 adapter 固定
    # subLayer clean wrapper，即使用户同时请求 legacy_v61，实际生效层仍是 clean
    # background。日志必须反映最终组合而不是被覆盖的请求值。
    if BACKGROUND_USD_PATH.name == WORKCELL_LITE_VISUAL_ONLY_BACKGROUND_USD:
        BACKGROUND_MODE = "workcell_lite+conveyor_visual_only"
    else:
        BACKGROUND_MODE = "visual_only+conveyor_visual_only"
    if CONVEYOR_DRIVE_MODE == "surface_velocity":
        BACKGROUND_MODE += "(surface_forced)"

CONVEYOR_GUIDE_THICKNESS = 0.04
CONVEYOR_GUIDE_HEIGHT = 0.12


def _make_conveyor_side_guide_cfg(prim_name: str, x: float) -> AssetBaseCfg:
    """Create an upstream low-friction guide without enclosing the grasp zone."""

    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{prim_name}",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[
                x,
                CONVEYOR_DRIVE.drive_segment.center_y,
                BELT_TOP_Z + CONVEYOR_GUIDE_HEIGHT * 0.5,
            ],
            rot=[1.0, 0.0, 0.0, 0.0],
        ),
        spawn=sim_utils.CuboidCfg(
            size=(
                CONVEYOR_GUIDE_THICKNESS,
                CONVEYOR_DRIVE.drive_segment.length,
                CONVEYOR_GUIDE_HEIGHT,
            ),
            visible=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.002,
                rest_offset=0.0,
            ),
            # Tote_B04 使用 frictionCombineMode=min，因此这里的低摩擦会主导
            # 侧壁接触，约束横向漂移但不在 Y 方向拖慢输送。
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.1,
                dynamic_friction=0.1,
                restitution=0.0,
            ),
        ),
    )


# ==================================================================
# 看不到头的入料端：A02 西拐弯道 + A05×5 X 支线（全部 AssetBaseCfg →
# scene.extras，不进 scene_props/同步清单）+ 两块 kinematic 托面延伸。
# 不新增货架——遮挡依赖背景既有结构（洞已记录在 README 已知限制）。
# ==================================================================


def _make_endless_visual_cfg(
    prim_name: str,
    usd_url: str,
    pos: tuple[float, float, float],
    yaw_deg: float = 0.0,
    scale: tuple[float, float, float] | None = None,
) -> AssetBaseCfg:
    """入口弯道视觉件：DigitalTwin 资产离线审计 coll=0/rigid=0（纯视觉），
    一律不传 rigid_props/collision_props；裸厘米 ⇒ 由调用方传 scale=0.01
    （不在薄层里写 xformOp:scale，会被 spawner 重写挤掉）。"""

    spawn = UsdFileCfg(usd_path=usd_url)
    if scale is not None:
        spawn.scale = scale
    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{prim_name}",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=list(pos),
            rot=list(endless_intake.yaw_quat(yaw_deg)),
        ),
        spawn=spawn,
    )


def _make_endless_plate_cfg(
    prim_name: str,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
) -> AssetBaseCfg:
    """弯道/支线的 kinematic 托面（与主线 conveyor_collider 同 z 同摩擦同做法）。

    托面是**静止**的：legacy 后端按固定周期覆写箱子的 root 速度（沿路径航向），
    托面只负责承重与摩擦停靠；不需要 SurfaceVelocityAPI。
    """

    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{prim_name}",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[
                (x_range[0] + x_range[1]) * 0.5,
                (y_range[0] + y_range[1]) * 0.5,
                BELT_TOP_Z - BELT_COLLIDER_THICKNESS * 0.5,
            ],
            rot=[1.0, 0.0, 0.0, 0.0],
        ),
        spawn=sim_utils.CuboidCfg(
            size=(
                x_range[1] - x_range[0],
                y_range[1] - y_range[0],
                BELT_COLLIDER_THICKNESS,
            ),
            visible=False,  # 视觉由 A02/A05 资产提供，托面只管碰撞
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.003, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.8,
                dynamic_friction=0.6,
                restitution=0.0,
            ),
        ),
    )


_ENDLESS_CURVE_USD = f"{NVIDIA_NUCLEUS_DIR}/{endless_intake.CURVE_ASSET_NVIDIA_RELPATH}"
_ENDLESS_XLEG_USD = f"{NVIDIA_NUCLEUS_DIR}/{endless_intake.XLEG_ASSET_NVIDIA_RELPATH}"
_ENDLESS_UNIT_SCALE = (
    endless_intake.CONVEYOR_UNIT_SCALE,
    endless_intake.CONVEYOR_UNIT_SCALE,
    endless_intake.CONVEYOR_UNIT_SCALE,
)


def _make_endless_xleg_cfg(index: int) -> AssetBaseCfg:
    return _make_endless_visual_cfg(
        f"ConveyorXLeg{index + 1}",
        _ENDLESS_XLEG_USD,
        endless_intake.XLEG_POSITIONS[index],
        endless_intake.XLEG_YAW_DEG,
        _ENDLESS_UNIT_SCALE,
    )


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
    print(f"{tag} 背景资产: {BACKGROUND_MODE} ({BACKGROUND_USD_PATH.name})")
    print(f"{tag} 料筐碰撞: {TOTE_COLLIDER_MODE} ({TOTE_USD_PATH.name})")
    print(f"{tag} ContactReport: {CONTACT_REPORT_MODE}")
    if TOTES_ON_CONVEYOR:
        print(
            f"{tag} 场景布局: 流水线（{len(BELT_BOX_NAMES)} 个纸箱排在工位上游、挡停放行）"
            " [ISAACLAB_TOTES_ON_CONVEYOR=1]"
        )
    else:
        print(f"{tag} 场景布局: 推车（两筐原尺寸叠在推车上） [ISAACLAB_TOTES_ON_CONVEYOR=0]")
    print(
        f"{tag} 道具策略: {SCENE_PROPS.mode} "
        f"[生成={','.join(SCENE_PROPS.spawned_names)}; "
        f"继承打包桌={'开' if SCENE_PROPS.inherited_packing_table else '关'}]"
    )
    print(
        f"{tag}   拖车/筐 x={PUSHCART_2_POS[0]:.3f} y={PUSHCART_2_POS[1]:.3f}"
        f" | robot_1 x={ROBOT_1_X:.3f} robot_2 x={ROBOT_2_X:.3f} y={ROBOT_WORKSTATION_Y:.3f}"
        f" | 流水线中线 x={BELT_X_CENTER:.3f} 入料端 y={BELT_Y_MAX:.3f}"
        f" | 整体北移 Δ={CONVEYOR_NORTH_SHIFT_Y:.2f}"
        "（支线越过货架排 B 所需；wrapper 组变换/装饰/地贴同 Δ）"
    )
    if CONVEYOR_TOTE_NAMES and TOTES_ON_CONVEYOR:
        print(
            f"{tag}   筐出生/复位落点 y: tote1={CART2_TOTE1_POS[1]:.3f} tote2={CART2_TOTE2_POS[1]:.3f}"
            f"（整场景复位写回同一位置；碰撞板 y[{BELT_Y_MIN:.3f},{BELT_Y_MAX:.3f}]）"
        )
    if ENDLESS_INTAKE.enabled:
        print(
            f"{tag}   入口形态: 看不到头（A02 西拐弯道 + A05×{endless_intake.XLEG_COUNT} X 支线，"
            "复用背景货架、零新增遮挡件）"
            f" [ISAACLAB_CONVEYOR_ENDLESS={ENDLESS_INTAKE.mode}]"
        )
        print(
            f"{tag}     弯道 pos=({endless_intake.CURVE_POS[0]:.4f},{endless_intake.CURVE_POS[1]:.4f}) "
            f"yaw={endless_intake.CURVE_YAW_DEG:.0f}° "
            f"占位 x[{endless_intake.CURVE_AABB[0][0]:.3f},{endless_intake.CURVE_AABB[0][1]:.3f}] "
            f"y[{endless_intake.CURVE_AABB[1][0]:.3f},{endless_intake.CURVE_AABB[1][1]:.3f}]"
            f"（南向母口套带头公头 {endless_intake.PLUG_DEPTH*1000:.0f}mm）"
            f" | 支线中线 y={endless_intake.BRANCH_LANE_Y:.4f} 端头 x={endless_intake.XLEG_AABBS[-1][0][0]:.3f}"
            f"（从排B北侧擦过：南缘对护板北缘净距 "
            f"{endless_intake.XLEG_AABBS[-1][1][0] - endless_intake.RACK_B_GUARD_NORTH_Y:.4f}）"
        )
        print(
            f"{tag}     托面延伸: 拐角 x[{endless_intake.CORNER_PLATE_X_RANGE[0]:.2f},"
            f"{endless_intake.CORNER_PLATE_X_RANGE[1]:.2f}]"
            f"y[{endless_intake.CORNER_PLATE_Y_RANGE[0]:.2f},{endless_intake.CORNER_PLATE_Y_RANGE[1]:.2f}]"
            f" + 支线 x[{endless_intake.BRANCH_PLATE_X_RANGE[0]:.3f},"
            f"{endless_intake.BRANCH_PLATE_X_RANGE[1]:.2f}]"
            f" | 驱动路径: 支线 +X → R={endless_intake.CORNER_RADIUS:.1f} 圆角弧 → 主线 -Y"
            "（沿路径距离 s 排队，箱子不旋转）"
        )
        print(
            f"{tag}     回生点 ({endless_intake.RESPAWN_XY[0]:.2f},{endless_intake.RESPAWN_XY[1]:.4f})"
            "（支线最深处，藏在货架排 B 后面：E2 东上角判据全遮（裕量 s=0.09）；"
            "⚠️ E1 受排 B 首层 216mm 通视缝影响无全遮解，顶面条带经缝可见——README 已知限制）"
        )
        if CONVEYOR_DRIVE_MODE == "surface_velocity":
            print(
                f"{tag}     ⚠️ surface_velocity 只驱动主线碰撞面（-Y），"
                "支线/弧段上的箱子不会被接触驱动——该组合未验证，建议 legacy 后端"
            )
    else:
        print(
            f"{tag}   入口形态: 直线带头（弯道/支线不生成）"
            f" [ISAACLAB_CONVEYOR_ENDLESS={ENDLESS_INTAKE.mode}"
            + (f"; {ENDLESS_INTAKE.disabled_reason}" if ENDLESS_INTAKE.disabled_reason else "")
            + "]"
        )
    if CONVEYOR_BELT_BOX_NAMES:
        if ENDLESS_INTAKE.enabled:
            _spawn = ", ".join(
                f"{kind.key}@({pos[0]:.2f},{pos[1]:.2f})"
                for kind, pos in zip(BELT_BOX_KINDS, BELT_BOX_POSITIONS)
            )
        else:
            _spawn = ", ".join(
                f"{kind.key}@{pos[1]:.3f}"
                for kind, pos in zip(BELT_BOX_KINDS, BELT_BOX_POSITIONS)
            )
        _sizes = " | ".join(
            f"{kind.key}: {kind.length_y}x{kind.width_x}x{kind.height_z} m, {kind.mass} kg"
            for kind in dict.fromkeys(BELT_BOX_KINDS)
        )
        if ENDLESS_INTAKE.enabled:
            print(
                f"{tag}   纸箱出生/复位落点（共 {len(BELT_BOX_POSITIONS)} 箱，"
                f"队首→上游，沿弯道路径）: {_spawn}"
                f" | z={BELT_BOX_POSITIONS[0][2]:.3f}"
            )
        else:
            print(
                f"{tag}   纸箱出生/复位落点（共 {len(BELT_BOX_POSITIONS)} 箱，队首→上游）: {_spawn}"
                f" | 车道 x={BELT_BOX_POSITIONS[0][0]:.3f} z={BELT_BOX_POSITIONS[0][2]:.3f}"
            )
        print(f"{tag}   箱型: {_sizes}")
        print(
            f"{tag}   排队净间隙 queue_gap={BELT_BOX_QUEUE_GAP:.3f} m"
            "（按各箱半长算，两种箱型混排时空隙恒定）；队首停工位，抓走后下一个自动补位"
        )
    if not CONVEYOR_DRIVE.requested_enabled:
        drive = "关 [ISAACLAB_CONVEYOR_ENABLED=0]"
    elif not CONVEYOR_ENABLED:
        drive = f"关（{CONVEYOR_DRIVE_MODE} 不在本机生效；Surface Velocity 固定由 ID=1 权威端驱动）"
    elif CONVEYOR_Y_STOP is None:
        drive = "开，一路循环不停"
    else:
        drive = f"开，目标中心 y={CONVEYOR_Y_STOP:.3f} 停住"
    print(
        f"{tag}   流水线驱动: {drive}（backend={CONVEYOR_DRIVE_MODE}，"
        f"{CONVEYOR_SPEED} m/s 沿 -Y）"
    )
    if CONVEYOR_DRIVE.stop_segment is not None:
        print(
            f"{tag}   碰撞面分区: 驱动 y[{CONVEYOR_DRIVE.drive_segment.y_min:.3f},"
            f"{CONVEYOR_DRIVE.drive_segment.y_max:.3f}] | 静态停止/抓取 "
            f"y[{CONVEYOR_DRIVE.stop_segment.y_min:.3f},{CONVEYOR_DRIVE.stop_segment.y_max:.3f}] "
            f"(handoff_offset={CONVEYOR_DRIVE.handoff_offset:.3f})"
        )


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


def _make_legacy_packing_table_cfg() -> AssetBaseCfg:
    """Recreate the inherited SONIC table only for the explicit legacy-props mode."""

    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PackingTable",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.55, 0.0, -0.3),
            rot=(0.70710678, 0.0, 0.0, -0.70710678),
        ),
        spawn=UsdFileCfg(
            usd_path=str(SONIC_PACKING_TABLE_USD),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )


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
    """塑料筐（Tote_B04），碰撞变体由 ``ISAACLAB_TOTE_COLLIDER`` 选择。

    原 0.01 → 0.6×0.4×0.3 m；流水线布局缩小一半到 0.005 → 0.3×0.2×0.15 m
    （原点仍在筐底面）。默认 compound 使用底板+四壁五个 box，保持开口语义；
    ``convex_decomposition`` 保留历史高成本碰撞以便回退。两者都绑定同一套
    2.0/1.6、combine=min 的抓取摩擦材质。
    """

    return UsdFileCfg(
        usd_path=str(TOTE_USD_PATH),
        scale=SCENE_LAYOUT.tote_scale,
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


def _make_belt_box_spawn_cfg(object_name: str, kind) -> UsdFileCfg:
    """流水线纸箱：v61 同款视觉资产 + convexHull 碰撞。

    ``kind`` 是 ``scene_layout.BeltBoxKind``，决定用哪份物理封装和质量。两种箱型
    （d01 = SM_CardBoxD_01、c01 = SM_CardBoxC_01）都是 v61 背景自带的，交错排布。

    刻意**不**就地提升背景 USD 里的 ``ConveyorBelt_Box_XX`` / ``KLT_Bin_XX``：那些
    Prim 带的是 triangle-mesh 碰撞（PhysX 对动态刚体只能退化成凸包 fallback 并刷
    警告），而且 v48→v61 换版遗留让它们的 z 对不上带面。任务层自己 spawn 才能精确
    贴面、拿到干净的凸包碰撞，并复用塑料筐那条已验过的权威/镜像分流：权威端是动态
    刚体，镜像端翻成 kinematic 只收位姿。
    """

    return UsdFileCfg(
        usd_path=str(_ASSETS_DIR / "props" / kind.asset),
        mass_props=sim_utils.MassPropertiesCfg(
            mass=_env_float(f"ISAACLAB_BELT_BOX_MASS_{kind.key.upper()}", kind.mass)
        ),
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


def _make_belt_box_cfg(index: int) -> RigidObjectCfg | None:
    """第 ``index`` 个纸箱（0 基）。数量不足或布局不生成时返回 None。"""

    name = f"belt_box_{index + 1}"
    if not SCENE_PROPS.spawns(name) or index >= len(BELT_BOX_POSITIONS):
        return None
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/BeltBox{index + 1}",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=BELT_BOX_POSITIONS[index], rot=BELT_BOX_ROT
        ),
        spawn=_make_belt_box_spawn_cfg(name, BELT_BOX_KINDS[index]),
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
        # viewer ghost 若启用足底诊断，也只在两只 ankle-roll 上挂 reporter；
        # off 模式下场景不会创建 foot_contact，ghost 不再承担传感器占位成本。
        configure_robot_contact_reports(cfg.spawn, CONTACT_REPORT_MODE)
        # ghost 完全不可见：viewer 画面（尤其 AR 自由视角）里不该出现第三台机器人
        # （2026-08-02 用户在 AR 里看到 3 台实测反馈）。物理照常模拟，仅隐藏视觉。
        cfg.spawn.visible = False
        return cfg
    cfg = make_sonic_robot_cfg()
    cfg.init_state.pos = LOCAL_ROBOT_POS
    cfg.init_state.rot = LOCAL_ROBOT_ROT
    configure_robot_contact_reports(cfg.spawn, CONTACT_REPORT_MODE)
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
    configure_robot_contact_reports(cfg.spawn, CONTACT_REPORT_MODE)
    return cfg


_PEER_ROBOT_USD = _ASSETS_DIR / "peer_robot" / "g1_43dof_peer.usd"
_PEER_VISUAL_LOD_USD = _ASSETS_DIR / "peer_robot" / "g1_43dof_visual_lod.usda"

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
        # peer 没有对应 ContactSensor；即便诊断选择 all，也不能让缺失烘焙
        # 资产的 URDF 回退重新给整台镜像机器人挂 ContactReportAPI。
        cfg.spawn.activate_contact_sensors = False
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


def _make_foot_contact_sensor(prim_name: str) -> ContactSensorCfg | None:
    """Create the two-ankle diagnostic sensor unless production selected off."""

    if CONTACT_REPORT_MODE == "off":
        return None
    return ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{prim_name}/.*_ankle_roll_link",
        history_length=4,
        track_air_time=True,
        force_threshold=5.0,
        debug_vis=False,
    )


def _make_peer_visual_lod_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/PeerRobot",
    pos: tuple[float, float, float] = PEER_ROBOT_POS,
    rot: tuple[float, float, float, float] = PEER_ROBOT_ROT,
) -> AssetBaseCfg:
    """Create a render-only G1 mirror; scene-state drives its link Xforms."""

    return AssetBaseCfg(
        prim_path=prim_path,
        init_state=AssetBaseCfg.InitialStateCfg(pos=pos, rot=rot),
        spawn=UsdFileCfg(
            usd_path=str(_PEER_VISUAL_LOD_USD),
            activate_contact_sensors=False,
        ),
    )


def _make_peer_scene_cfg() -> ArticulationCfg | AssetBaseCfg:
    if PEER_VISUAL_LOD:
        return _make_peer_visual_lod_cfg()
    return _make_peer_robot_cfg()


def _make_second_peer_scene_cfg() -> ArticulationCfg | AssetBaseCfg:
    if PEER_VISUAL_LOD:
        return _make_peer_visual_lod_cfg(
            prim_path="{ENV_REGEX_NS}/PeerRobot2",
            pos=PEER2_ROBOT_POS,
            rot=PEER2_ROBOT_ROT,
        )
    return _make_second_peer_robot_cfg()


# ==================================================================
# 场景
# ==================================================================


@configclass
class G129SonicConveyorSceneCfg(G129SonicSceneCfg):
    """SONIC 本机 G1 + warehouse 流水线工作区 + 对端镜像 G1。"""

    # warehouse 背景 USD 自带地面，去掉底座的无限地平面避免 z-fighting。
    ground = None

    # 不继承 SONIC 底座的 DomeLight；背景层也已剔除全部灯光，场景只保留
    # 本配置下方的一盏 DistantLight，避免重复照明和额外 RTX 阴影开销。
    light = None

    # 父场景的打包桌位于原点，与 (-5, 14) 的流水线工位无关。layout 策略直接不
    # 生成；legacy_props 做 A/B 时才用工厂函数恢复同一配置。
    packing_table: AssetBaseCfg | None = (
        _make_legacy_packing_table_cfg() if SCENE_PROPS.inherited_packing_table else None
    )

    # 覆盖 SONIC 底座的足底传感器：ankles/all 模式仍只读取两只脚踝，off
    # 模式连 ContactSensor 对象也不创建。reporter 的实际挂载范围由机器人
    # spawner 决定，因此默认 ankles 不再给全身每个刚体添加 ContactReportAPI。
    foot_contact: ContactSensorCfg | None = _make_foot_contact_sensor("Robot")

    background = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Background",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-4.68, 14.39363, 0], rot=[0.7071, 0.0, 0.0, 0.7071]),
        spawn=UsdFileCfg(
            # Surface 由选择器强制使用纯视觉输送机 adapter；legacy 默认保留原生
            # 输送机物理，也可通过资产开关单独做 visual-only A/B。
            usd_path=str(BACKGROUND_USD_PATH),
        ),
    )

    # legacy 默认保留背景 USD 的原生输送机物理，用于历史回退；
    # surface_velocity 通过 visual-only adapter 在组合阶段去掉原生物理，只让
    # 下列简化带面参与接触。
    # 原先的一整块 kinematic 碰撞板按 y_stop 拆为两块，无缝覆盖 y[10.19,18.22]：
    #   conveyor_collider      入料/驱动段；预置禁用的 PhysxSurfaceVelocityAPI
    #   conveyor_stop_collider 下游静态高摩擦停止/抓取段
    # legacy + ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET=1 可让两种 backend 共用
    # visual-only 资产和同一组简化碰撞几何，用于严格 driver A/B。
    # 顶面对齐滚轮顶 z≈0.772（板厚 0.04 → 中心 z=0.752），可用宽度
    # x∈[-6.07,-5.17]。y_stop<=0 的循环模式不生成静态停止段，驱动段覆盖全长。
    conveyor_collider = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/ConveyorCollider",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[
                BELT_X_CENTER,
                CONVEYOR_DRIVE.drive_segment.center_y,
                BELT_TOP_Z - BELT_COLLIDER_THICKNESS * 0.5,
            ],
            rot=[1.0, 0.0, 0.0, 0.0],
        ),
        spawn=sim_utils.CuboidCfg(
            func=conveyor_events.spawn_surface_velocity_cuboid,
            size=(
                BELT_WIDTH,
                CONVEYOR_DRIVE.drive_segment.length,
                BELT_COLLIDER_THICKNESS,
            ),
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

    conveyor_stop_collider: AssetBaseCfg | None = (
        AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/ConveyorStopCollider",
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=[
                    BELT_X_CENTER,
                    CONVEYOR_DRIVE.stop_segment.center_y,
                    BELT_TOP_Z - BELT_COLLIDER_THICKNESS * 0.5,
                ],
                rot=[1.0, 0.0, 0.0, 0.0],
            ),
            spawn=sim_utils.CuboidCfg(
                size=(
                    BELT_WIDTH,
                    CONVEYOR_DRIVE.stop_segment.length,
                    BELT_COLLIDER_THICKNESS,
                ),
                visible=False,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.003, rest_offset=0.0),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=0.8,
                    dynamic_friction=0.6,
                    restitution=0.0,
                ),
            ),
        )
        if CONVEYOR_DRIVE.stop_segment is not None
        else None
    )

    # Surface 模式用纯视觉输送机后，用两条低摩擦直导轨约束入料段横向
    # 漂移。导轨只覆盖 drive_segment，到静态停止/抓取段前结束，避免机械手从
    # ±X 方向抓筐时先撞上不可见侧壁。legacy 继续使用 A08 自带框架/滚轮碰撞。
    conveyor_guide_positive_x: AssetBaseCfg | None = (
        _make_conveyor_side_guide_cfg(
            "ConveyorGuidePositiveX",
            BELT_X_CENTER + BELT_WIDTH * 0.5 + CONVEYOR_GUIDE_THICKNESS * 0.5,
        )
        if CONVEYOR_DRIVE_MODE == "surface_velocity"
        else None
    )
    conveyor_guide_negative_x: AssetBaseCfg | None = (
        _make_conveyor_side_guide_cfg(
            "ConveyorGuideNegativeX",
            BELT_X_CENTER - BELT_WIDTH * 0.5 - CONVEYOR_GUIDE_THICKNESS * 0.5,
        )
        if CONVEYOR_DRIVE_MODE == "surface_velocity"
        else None
    )

    # ------------------------------------------------------------------
    # 看不到头的入料端（=1 且 layout 道具时生成；开关/几何见 endless_intake）：
    # A02 弯道套住带头公头向西拐 90°，五段 A05 短直段接成 X 支线，从背景货架
    # 排 B 北侧擦过、端头伸到排 B/叉车后面（Δ=0.25 整体北移换来的通道）。
    # 全部 AssetBaseCfg（scene.extras），输送机件纯视觉（coll=0/rigid=0，
    # 裸厘米 ⇒ scale=0.01）；另加两块 kinematic 托面接住弧段/支线上的箱子。
    # **零新增货架/遮挡件**（用户要求）。
    # ------------------------------------------------------------------
    conveyor_curve: AssetBaseCfg | None = (
        _make_endless_visual_cfg(
            "ConveyorCurve",
            _ENDLESS_CURVE_USD,
            endless_intake.CURVE_POS,
            endless_intake.CURVE_YAW_DEG,
            _ENDLESS_UNIT_SCALE,
        )
        if ENDLESS_INTAKE.enabled
        else None
    )
    conveyor_xleg_1: AssetBaseCfg | None = (
        _make_endless_xleg_cfg(0) if ENDLESS_INTAKE.enabled else None
    )
    conveyor_xleg_2: AssetBaseCfg | None = (
        _make_endless_xleg_cfg(1) if ENDLESS_INTAKE.enabled else None
    )
    conveyor_xleg_3: AssetBaseCfg | None = (
        _make_endless_xleg_cfg(2) if ENDLESS_INTAKE.enabled else None
    )
    conveyor_xleg_4: AssetBaseCfg | None = (
        _make_endless_xleg_cfg(3) if ENDLESS_INTAKE.enabled else None
    )
    conveyor_xleg_5: AssetBaseCfg | None = (
        _make_endless_xleg_cfg(4) if ENDLESS_INTAKE.enabled else None
    )
    conveyor_corner_plate: AssetBaseCfg | None = (
        _make_endless_plate_cfg(
            "ConveyorCornerPlate",
            endless_intake.CORNER_PLATE_X_RANGE,
            endless_intake.CORNER_PLATE_Y_RANGE,
        )
        if ENDLESS_INTAKE.enabled
        else None
    )
    conveyor_branch_plate: AssetBaseCfg | None = (
        _make_endless_plate_cfg(
            "ConveyorBranchPlate",
            endless_intake.BRANCH_PLATE_X_RANGE,
            endless_intake.BRANCH_PLATE_Y_RANGE,
        )
        if ENDLESS_INTAKE.enabled
        else None
    )

    # 纸箱推车组（外侧位 x=-6.8）：拖车 + 两纸箱 + 顶上的长条测试箱。
    # y=19.64363（=01cdfaf 原注释口径 19.39363 + 整体北移 Δ=0.25，随线同移）。
    # 只有 legacy_props 模式才生成（该模式下弯道组被 endless_intake 布局门拦下，
    # 不会与空车同场）。
    pushcart: RigidObjectCfg | None = (
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Pushcart",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[-6.8, 19.64363, 0.0], rot=[0.0, 0.0, 0.0, 1.0]
            ),
            spawn=_make_pushcart_spawn_cfg("pushcart"),
        )
        if SCENE_PROPS.spawns("pushcart")
        else None
    )
    cart_box1: RigidObjectCfg | None = (
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/CartBox1",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[-6.8, 19.64363, 0.45], rot=[0.0, 0.0, 0.0, 1.0]
            ),
            spawn=_make_graspable_cart_box_spawn_cfg("cart_box1"),
        )
        if SCENE_PROPS.spawns("cart_box1")
        else None
    )
    cart_box2: RigidObjectCfg | None = (
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/CartBox2",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[-6.8, 19.64363, 0.60], rot=[0.0, 0.0, 0.0, 1.0]
            ),
            spawn=_make_graspable_cart_box_spawn_cfg("cart_box2"),
        )
        if SCENE_PROPS.spawns("cart_box2")
        else None
    )
    test_box: RigidObjectCfg | None = (
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/TestBox",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[-6.8, 19.64363, 1.095],
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
                collision_props=sim_utils.CollisionPropertiesCfg(
                    contact_offset=0.003, rest_offset=0.0
                ),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=1.2,
                    dynamic_friction=0.9,
                    restitution=0.0,
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.76, 0.56, 0.28), roughness=0.70
                ),
            ),
        )
        if SCENE_PROPS.spawns("test_box")
        else None
    )

    # 第二台拖车与两个塑料筐（位置随 TOTES_ON_CONVEYOR 切换，见上方常量段）。
    pushcart_2: RigidObjectCfg | None = (
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Pushcart2",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=PUSHCART_2_POS, rot=[0.0, 0.0, 0.0, 1.0]
            ),
            spawn=_make_pushcart_spawn_cfg("pushcart_2"),
        )
        if SCENE_PROPS.spawns("pushcart_2")
        else None
    )
    # 两塑料筐只在推车布局（和 legacy_props 回退）里存在：流水线布局的作业对象
    # 已换成下面的纸箱队列，筐不再生成，也就不再进 scene_state 同步清单。
    cart2_tote1: RigidObjectCfg | None = (
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cart2Tote1",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=CART2_TOTE1_POS, rot=[0.0, 0.0, 0.0, 1.0]
            ),
            spawn=_make_cart2_tote_spawn_cfg("cart2_tote1"),
        )
        if SCENE_PROPS.spawns("cart2_tote1")
        else None
    )
    cart2_tote2: RigidObjectCfg | None = (
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cart2Tote2",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=CART2_TOTE2_POS, rot=[0.0, 0.0, 0.0, 1.0]
            ),
            spawn=_make_cart2_tote_spawn_cfg("cart2_tote2"),
        )
        if SCENE_PROPS.spawns("cart2_tote2")
        else None
    )

    # 流水线纸箱队列：belt_box_1 是队首（y 最小、最先到工位），编号递增向上游排。
    # configclass 需要类属性，所以字段显式声明到 BELT_BOX_MAX_COUNT 个；
    # ISAACLAB_BELT_BOX_COUNT 只能在此上限内往下减（超限在 scene_layout 里 fail-fast）。
    belt_box_1: RigidObjectCfg | None = _make_belt_box_cfg(0)
    belt_box_2: RigidObjectCfg | None = _make_belt_box_cfg(1)
    belt_box_3: RigidObjectCfg | None = _make_belt_box_cfg(2)
    belt_box_4: RigidObjectCfg | None = _make_belt_box_cfg(3)
    belt_box_5: RigidObjectCfg | None = _make_belt_box_cfg(4)
    belt_box_6: RigidObjectCfg | None = _make_belt_box_cfg(5)
    belt_box_7: RigidObjectCfg | None = _make_belt_box_cfg(6)
    belt_box_8: RigidObjectCfg | None = _make_belt_box_cfg(7)
    belt_box_9: RigidObjectCfg | None = _make_belt_box_cfg(8)
    belt_box_10: RigidObjectCfg | None = _make_belt_box_cfg(9)
    belt_box_11: RigidObjectCfg | None = _make_belt_box_cfg(10)
    belt_box_12: RigidObjectCfg | None = _make_belt_box_cfg(11)
    belt_box_13: RigidObjectCfg | None = _make_belt_box_cfg(12)
    belt_box_14: RigidObjectCfg | None = _make_belt_box_cfg(13)
    belt_box_15: RigidObjectCfg | None = _make_belt_box_cfg(14)
    belt_box_16: RigidObjectCfg | None = _make_belt_box_cfg(15)
    belt_box_17: RigidObjectCfg | None = _make_belt_box_cfg(16)

    # 底座打包桌三方块换成镜像感知版（位置/颜色与底座一致）：不同步的话
    # 两端各自模拟，任一端机器人碰一下就静默分叉。
    cube_1: RigidObjectCfg | None = (
        _make_conveyor_cube_cfg(
            "cube_1", "Cube1", (0.31243, -0.00553, SONIC_CUBE_INITIAL_Z), (0.82, 0.66, 0.36)
        )
        if SCENE_PROPS.spawns("cube_1")
        else None
    )
    cube_2: RigidObjectCfg | None = (
        _make_conveyor_cube_cfg(
            "cube_2", "Cube2", (0.31397, 0.10565, SONIC_CUBE_INITIAL_Z), (0.88, 0.72, 0.40)
        )
        if SCENE_PROPS.spawns("cube_2")
        else None
    )
    cube_3: RigidObjectCfg | None = (
        _make_conveyor_cube_cfg(
            "cube_3", "Cube3", (0.41625, 0.04810, SONIC_CUBE_INITIAL_Z), (0.76, 0.56, 0.28)
        )
        if SCENE_PROPS.spawns("cube_3")
        else None
    )

    # 本机 G1：SONIC 底座同款（DDS 驱动、名字仍是 robot/prim Robot），只挪到工位。
    # viewer 模式下退化为场外 ghost（见 _make_local_robot_cfg）。
    robot: ArticulationCfg = _make_local_robot_cfg()

    # 对端 G1 镜像体：由 scene_state 帧驱动。viewer 模式下它镜像 robot_1；
    # host 模式下两台都是真身，不需要镜像体。
    peer_robot: ArticulationCfg | AssetBaseCfg | None = None if HOST_MODE else _make_peer_scene_cfg()

    # viewer 专用第二镜像体（robot_2）。对等/host 模式为 None，场景里不生成。
    peer_robot_2: ArticulationCfg | AssetBaseCfg | None = _make_second_peer_scene_cfg() if VIEWER_MODE else None

    # host 专用：robot_2 全动力学本体 + 它的足底接触诊断（参数照抄底座 foot_contact）。
    robot_2: ArticulationCfg | None = _make_second_local_robot_cfg() if HOST_MODE else None
    foot_contact_2: ContactSensorCfg | None = (
        _make_foot_contact_sensor("Robot2") if HOST_MODE else None
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
    """背景锁定 + 互斥的 legacy / Surface Velocity 流水线事件。"""

    lock_sorting_bins = EventTerm(
        func=conveyor_events.lock_background_rigid_bodies,
        mode="startup",
        params={
            "prim_names": BACKGROUND_LOCK_PRIM_NAMES,
            "parent_path": "Background/ConveyorBelt",
            "kinematic": True,
        },
    )

    configure_surface_velocity = EventTerm(
        func=conveyor_events.configure_conveyor_surface_velocity,
        mode="startup",
        params={
            "prim_name": "ConveyorCollider",
            "velocity_y": CONVEYOR_VELOCITY_Y,
            "enabled": CONVEYOR_SURFACE_VELOCITY_ENABLED,
            # 函数内再校验一次 ID=1，防止以后改配置时意外放宽权威门禁。
            "local_robot_id": LOCAL_ROBOT_ID,
        },
    )

    drive_totes = EventTerm(
        func=conveyor_events.drive_totes_on_conveyor,
        mode="interval",
        interval_range_s=(0.02, 0.02),
        params={
            "object_names": CONVEYOR_TOTE_NAMES,
            "velocity_y": CONVEYOR_VELOCITY_Y,
            "enabled": CONVEYOR_LEGACY_ENABLED,
            "y_stop": CONVEYOR_Y_STOP,
            "y_recycle": CONVEYOR_Y_RECYCLE,
            "y_respawn": CONVEYOR_Y_RESPAWN,
        },
    )

    # 纸箱队列走带挡停的驱动：队首停在工位等抓取，后面的按 queue_pitch 排队，
    # 工位那个被拎走后下一个自动补位（判据全在当前帧位置里，无状态机）。
    # surface_velocity 后端不需要它——那边队列是后车撞前车物理涌现出来的。
    # 入口弯道生效时切两段式路径驱动（西拐）：支线 +X → 圆角弧 → 主线 -Y，排队
    # 坐标是沿路径距离 s，带面判据是主线矩形 ∪ 拐角/支线附加矩形；箱子不旋转。
    drive_belt_boxes = EventTerm(
        func=conveyor_events.drive_belt_boxes_on_conveyor,
        mode="interval",
        interval_range_s=(0.02, 0.02),
        params={
            "object_names": CONVEYOR_BELT_BOX_NAMES,
            "velocity_y": CONVEYOR_VELOCITY_Y,
            "enabled": CONVEYOR_LEGACY_ENABLED,
            "y_stop": CONVEYOR_Y_STOP,
            "queue_gap": BELT_BOX_QUEUE_GAP,
            # 两种箱型尺寸不同，排队按各自半长算净间隙，不能用统一中心距。
            "half_lengths": CONVEYOR_BELT_BOX_HALF_LENGTHS,
            "path_enabled": ENDLESS_INTAKE.enabled,
            "path_corner_center": endless_intake.CORNER_CENTER,
            "path_radius": endless_intake.CORNER_RADIUS,
            "path_s_origin_x": endless_intake.S_ORIGIN_X,
            "extra_rects": endless_intake.ON_BELT_EXTRA_RECTS,
        },
    )

    # 循环模式的回生点：弯道生效时搬到 X 支线最深处 (-16.66, 20.0534)，深藏在
    # 货架排 B（近端还有叉车）后面——E2 眼位东上角判据全遮、E1 受排 B 首层
    # 216mm 通视缝影响残余可见（README 已知限制）；直线形态维持主线 y_respawn。
    recycle_surface_totes = EventTerm(
        func=conveyor_events.recycle_totes_on_surface_conveyor,
        mode="interval",
        interval_range_s=(0.02, 0.02),
        params={
            "object_names": (*CONVEYOR_TOTE_NAMES, *CONVEYOR_BELT_BOX_NAMES),
            "enabled": CONVEYOR_RECYCLE_ENABLED,
            "y_recycle": CONVEYOR_Y_RECYCLE,
            "y_respawn": (
                endless_intake.RESPAWN_XY[1] if ENDLESS_INTAKE.enabled else CONVEYOR_Y_RESPAWN
            ),
            "respawn_x": (
                endless_intake.RESPAWN_XY[0] if ENDLESS_INTAKE.enabled else None
            ),
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

_PROP_NAMES = (
    "packing_table",
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
    *BELT_BOX_NAMES,
)


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
        # 只把实际后端注册进 interval manager。Surface 停止模式完全依靠 PhysX
        # 接触，不需要逐步事件；循环模式仅保留回收事件，绝不保留 legacy 速度覆写。
        if not CONVEYOR_LEGACY_ENABLED:
            self.events.drive_totes = None
            self.events.drive_belt_boxes = None
        if not CONVEYOR_RECYCLE_ENABLED:
            self.events.recycle_surface_totes = None
        # 清单为空的驱动事件不进 interval manager：每 20 ms 空跑一次没有意义。
        if not CONVEYOR_TOTE_NAMES:
            self.events.drive_totes = None
        if not CONVEYOR_BELT_BOX_NAMES:
            self.events.drive_belt_boxes = None
        # visual-only ConveyorBelt 已在 USD 组合阶段移除全部刚体，不再对
        # blue_sorting_bin_02 运行历史 kinematic 补丁或产生无刚体告警。
        if CONVEYOR_VISUAL_ONLY_ASSET_ENABLED:
            self.events.lock_sorting_bins = None
        # GUI 开局相机：吊在流水线正上方（x=带中线）、工位下游侧，面向流水线
        # 流动开始的方向（+Y，上游弯道/货架方向），机器人工位在画面中心——
        # 前景是双机与工位，背景是带体向弯道延伸、箱子迎面流来。
        # eye/lookat 全用动态常量，Δ/工位变了自动跟随；=0 布局下工位=拖车组 y，
        # 同一公式仍成立（相机移到带中段上方朝北看作业组）。
        # 视线通廊复核：eye z=3.2 高于弯道门柱顶 1.169 与全部箱顶，带上方
        # z∈(1.2,3.2) 无横梁（桁架在 z≥6），到工位视线无遮挡；eye y=工位-4.0
        # 在 =1 下落在带尾附近正上方（带体 y[10.44,18.47]）。
        self.viewer.eye = (BELT_X_CENTER, ROBOT_WORKSTATION_Y - 4.0, 3.2)
        self.viewer.lookat = (BELT_X_CENTER, ROBOT_WORKSTATION_Y, 1.0)
        if _PERF_AB:
            print(f"[conveyor_env_cfg] ⚠️ 性能 A/B 诊断开关生效: {sorted(_PERF_AB)}")
            if "no_peer" in _PERF_AB:
                self.scene.peer_robot = None
            if "no_props" in _PERF_AB:
                for _name in _PROP_NAMES:
                    setattr(self.scene, _name, None)
                self.events.drive_totes = None
                self.events.drive_belt_boxes = None
                self.events.recycle_surface_totes = None
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
        if not HOST_MODE and not PEER_VISUAL_LOD and not _PEER_ROBOT_USD.exists():
            raise RuntimeError(
                f"缺少无碰撞镜像机器人产物 {_PEER_ROBOT_USD}\n先运行: python tools/build_peer_robot_usd.py"
            )
        if not HOST_MODE and PEER_VISUAL_LOD and not _PEER_VISUAL_LOD_USD.exists():
            raise RuntimeError(
                f"缺少纯显示镜像机器人资产 {_PEER_VISUAL_LOD_USD}\n"
                "先运行: python tools/build_peer_visual_lod_usd.py"
            )
        if not HOST_MODE:
            print(f"[conveyor_env_cfg] 镜像机器人模式: {PEER_ROBOT_MODE}")
        # XR 锚定重定向。⚠️ 只改 self.xr 不够——teleop_devices 构建时把 xr_cfg
        # **拷贝**了一份（2026-08-02 实测：cfg 打印挂 PeerRobot、teleop 设备实际仍用
        # Robot 路径，AR 视角锚到场外 ghost 上）。必须把 self.xr 与每个 teleop 设备
        # 持有的 xr_cfg 一起改。
        def _apply_xr_anchor(prim_name: str, tag: str, extra: dict | None = None) -> None:
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
                for _k, _v in (extra or {}).items():
                    setattr(_xr, _k, _v)
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
            # 镜像体是被同步帧离散传送的（非连续物理），锚定位置裸写会以应用频率抖动
            # ——viewer 打开位置平滑（本体动力学路径保持 0=裸写不受影响）。
            # 旋转：默认 FIXED——视角旋转只听操作者自己的头，不跟机器人转身
            # （SONIC 转身会被动旋转视角，操作者实测头晕）。朝向与机器人错位时
            # 按 B/F9 recenter 把机器人 pelvis 朝向重新对到正前方；启动时自动
            # 对正一次。ISAACLAB_XR_ANCHOR_ROT_FOLLOW=1 恢复 yaw 跟随（旧手感）。
            from isaaclab.devices.openxr import XrAnchorRotationMode as _RotMode

            _extra = {
                "anchor_position_smoothing_time": float(
                    os.environ.get("ISAACLAB_XR_ANCHOR_POS_SMOOTHING", "0.15")
                ),
                "recenter_yaw_on_start": True,
            }
            if os.environ.get("ISAACLAB_XR_ANCHOR_ROT_FOLLOW", "0") != "1":
                _extra["anchor_rotation_mode"] = _RotMode.FIXED
            _apply_xr_anchor(_anchor_prim, "viewer", extra=_extra)
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
