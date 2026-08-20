# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Lightwheel KitchenRoom 中的双 G1 SONIC 递咖啡场景。

场景位姿与杯子资产选型移植自 ``~/xiaoyang_IssacLab/IsaacLab`` 的
``scene-caffe`` 分支。控制链保持本仓库既有 host 双 SONIC 约定：``robot`` 走
``rt/*``，``robot_2`` 走 ``rt/r2/*``，两台机器人都由同一个 Isaac 进程进行
全动力学仿真。本文件只提供场景实体、第二机器人的动作项和 DDS 状态观测项。
"""

from __future__ import annotations

import os

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

from tasks.common_observations.dex3_state import get_robot_dex3_joint_states
from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states
from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    G129Dex3SonicSceneCfg,
    G129SonicEnvCfg,
    make_sonic_robot_cfg,
)
from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    ActionsCfg as SonicActionsCfg,
)


def resolve_kitchen_room_usd_path() -> str:
    """返回源场景资产路径，并允许部署机用环境变量改根目录。"""

    root_dir = os.environ.get("LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR", "/home/nolo/Lightwheel_OpenSource")
    return os.path.join(root_dir, "Locomotion", "KitchenRoom", "KitchenRoom.usd")


KITCHEN_ROOM_USD_PATH = resolve_kitchen_room_usd_path()

# 以下 XY、朝向和杯子/交接区域坐标来自 scene-caffe 分支的 KitchenRoom 最终配置。
# SONIC 机器人保留其已验证的 0.76 m root 高度；源 IK 场景的固定底座高度是 0.80 m。
ROBOT_1_POS = (0.18, -0.42, 0.76)
ROBOT_1_ROT = (0.7071068, 0.0, 0.0, 0.7071068)
ROBOT_2_POS = (0.26, 0.80, 0.76)
ROBOT_2_ROT = (0.7071068, 0.0, 0.0, -0.7071068)
CUP_SPAWN_POS = (0.02, 0.04, 0.91)
HANDOVER_ZONE_POS = (0.22, 0.22, 1.00)
SERVE_ZONE_POS = (0.45, 0.46, 0.94)
VIEWER_ANCHOR_POS = HANDOVER_ZONE_POS


def _make_sonic_robot(prim_path: str, pos: tuple[float, float, float], rot: tuple[float, ...]) -> ArticulationCfg:
    cfg = make_sonic_robot_cfg()
    cfg.prim_path = prim_path
    cfg.init_state.pos = pos
    cfg.init_state.rot = rot
    return cfg


def _make_foot_contact_sensor(robot_prim_name: str) -> ContactSensorCfg:
    return ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{robot_prim_name}/.*_ankle_roll_link",
        history_length=4,
        track_air_time=True,
        force_threshold=5.0,
        debug_vis=False,
    )


def _make_anchor(name: str, pos: tuple[float, float, float]) -> AssetBaseCfg:
    """创建不可见、无碰撞的逻辑锚点，保留源任务的场景接口。"""

    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        init_state=AssetBaseCfg.InitialStateCfg(pos=pos),
        spawn=sim_utils.SphereCfg(
            radius=0.01,
            visible=False,
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
        ),
    )


@configclass
class G129DualSonicCafeSceneCfg(G129Dex3SonicSceneCfg):
    """KitchenRoom 背景、咖啡杯和两台面对面的全动力学 SONIC G1。"""

    # KitchenRoom 自带地面与碰撞体，避免再叠一层无限地平面。
    ground = None

    background = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Background",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=UsdFileCfg(usd_path=KITCHEN_ROOM_USD_PATH),
    )

    cup = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cup",
        init_state=RigidObjectCfg.InitialStateCfg(pos=CUP_SPAWN_POS, rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Objects/Mug/mug.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
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

    robot: ArticulationCfg = _make_sonic_robot("{ENV_REGEX_NS}/Robot", ROBOT_1_POS, ROBOT_1_ROT)
    robot_2: ArticulationCfg = _make_sonic_robot("{ENV_REGEX_NS}/Robot2", ROBOT_2_POS, ROBOT_2_ROT)
    foot_contact: ContactSensorCfg = _make_foot_contact_sensor("Robot")
    foot_contact_2: ContactSensorCfg = _make_foot_contact_sensor("Robot2")

    robot_spawn_a = _make_anchor("RobotSpawnA", ROBOT_1_POS)
    robot_spawn_b = _make_anchor("RobotSpawnB", ROBOT_2_POS)
    cup_spawn = _make_anchor("CupSpawn", CUP_SPAWN_POS)
    handover_zone = _make_anchor("HandoverZone", HANDOVER_ZONE_POS)
    serve_zone = _make_anchor("ServeZone", SERVE_ZONE_POS)
    viewer_anchor = _make_anchor("ViewerAnchor", VIEWER_ANCHOR_POS)


@configclass
class DualSonicActionsCfg(SonicActionsCfg):
    """既有 SONIC 三段动作后追加 robot_2 三段，顺序与 host provider 一致。"""

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


@configclass
class DualSonicObservationsCfg:
    """分别向两套 SONIC deploy 发布 body 与 Dex3 物理状态。"""

    @configclass
    class PolicyCfg(ObsGroup):
        robot_body_state = ObsTerm(func=get_robot_boy_joint_states, params={"dds_min_interval_ms": 0.0})
        robot_dex3_state = ObsTerm(func=get_robot_dex3_joint_states, params={"dds_min_interval_ms": 0.0})
        robot2_body_state = ObsTerm(
            func=get_robot_boy_joint_states,
            params={"dds_min_interval_ms": 0.0, "asset_name": "robot_2", "dds_object_name": "g129_r2"},
        )
        robot2_dex3_state = ObsTerm(
            func=get_robot_dex3_joint_states,
            params={"dds_min_interval_ms": 0.0, "asset_name": "robot_2", "dds_object_name": "dex3_r2"},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class G129DualSonicCafeEnvCfg(G129SonicEnvCfg):
    """双 SONIC 咖啡交接任务；控制周期、复位和物理参数均继承 SONIC 底座。"""

    scene: G129DualSonicCafeSceneCfg = G129DualSonicCafeSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )
    actions: DualSonicActionsCfg = DualSonicActionsCfg()
    observations: DualSonicObservationsCfg = DualSonicObservationsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = (2.6, -2.6, 1.9)
        self.viewer.lookat = VIEWER_ANCHOR_POS
