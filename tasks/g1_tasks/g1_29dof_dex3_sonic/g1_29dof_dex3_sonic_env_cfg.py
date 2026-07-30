"""Floating-base G1-29DoF scenes for Gear SONIC DDS control."""

from __future__ import annotations

import os
from pathlib import Path

import torch

import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.devices import DevicesCfg
from isaaclab.devices.openxr import OpenXRDeviceCfg, XrAnchorRotationMode, XrCfg
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from robots.g1_sonic_urdf import (
    DEX3_HAND_JOINT_NAMES,
    build_default_sonic_g1_43dof_urdf,
    default_sonic_g1_43dof_output_path,
)
from tasks.common_config import G1RobotPresets
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager
from tasks.common_observations.dex3_state import get_robot_dex3_joint_states
from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states


# Keep the active-drive gains and motor armatures identical to the released
# SONIC training setup in
# GR00T-WholeBodyControl/gear_sonic/envs/manager_env/robots/g1.py.  The policy
# was trained with these motor inertias, 10 Hz natural frequency and damping
# ratio 2.0.  Incoming LowCmd kp/kd values are applied dynamically at runtime;
# these values are also the safe startup/default gains.
SONIC_ARMATURE_5020 = 0.003609725
SONIC_ARMATURE_7520_14 = 0.010177520
SONIC_ARMATURE_7520_22 = 0.025101925
SONIC_ARMATURE_4010 = 0.00425
SONIC_NATURAL_FREQ = 10.0 * 2.0 * 3.1415926535
SONIC_DAMPING_RATIO = 2.0

SONIC_STIFFNESS_5020 = SONIC_ARMATURE_5020 * SONIC_NATURAL_FREQ**2
SONIC_STIFFNESS_7520_14 = SONIC_ARMATURE_7520_14 * SONIC_NATURAL_FREQ**2
SONIC_STIFFNESS_7520_22 = SONIC_ARMATURE_7520_22 * SONIC_NATURAL_FREQ**2
SONIC_STIFFNESS_4010 = SONIC_ARMATURE_4010 * SONIC_NATURAL_FREQ**2

SONIC_DAMPING_5020 = (
    2.0 * SONIC_DAMPING_RATIO * SONIC_ARMATURE_5020 * SONIC_NATURAL_FREQ
)
SONIC_DAMPING_7520_14 = (
    2.0 * SONIC_DAMPING_RATIO * SONIC_ARMATURE_7520_14 * SONIC_NATURAL_FREQ
)
SONIC_DAMPING_7520_22 = (
    2.0 * SONIC_DAMPING_RATIO * SONIC_ARMATURE_7520_22 * SONIC_NATURAL_FREQ
)
SONIC_DAMPING_4010 = (
    2.0 * SONIC_DAMPING_RATIO * SONIC_ARMATURE_4010 * SONIC_NATURAL_FREQ
)

DEX3_PHASE1_EFFORT_LIMITS = {
    name: 2.45 if name.endswith("thumb_0_joint") else 0.7
    for name in DEX3_HAND_JOINT_NAMES
}
DEX3_VELOCITY_LIMITS = {
    name: 3.14 if name.endswith("thumb_0_joint") else 12.0
    for name in DEX3_HAND_JOINT_NAMES
}

# Use the same current Isaac PackingTable asset as the reference
# locomanipulation task.  The repository's older collected copy is missing
# three metal-material textures and therefore emits RTX/MDL errors in XR mode.
# Isaac's asset client caches this official resource after the first load.
SONIC_PACKING_TABLE_USD = (
    f"{ISAAC_NUCLEUS_DIR}/Props/PackingTable/packing_table.usd"
)
SONIC_TABLE_TOP_Z = 0.6996
SONIC_CUBE_SIZE = (0.05, 0.05, 0.05)
SONIC_CUBE_INITIAL_Z = SONIC_TABLE_TOP_Z + 0.5 * SONIC_CUBE_SIZE[2] + 0.002


def _make_sonic_cube_cfg(
    prim_name: str,
    initial_pos: tuple[float, float, float],
    color: tuple[float, float, float],
) -> RigidObjectCfg:
    """Create one dynamic 5 cm cube using the reference scene's contact settings."""

    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{prim_name}",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=initial_pos,
            # Isaac Lab 6 asset configurations use xyzw quaternion order.
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=SONIC_CUBE_SIZE,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=False,
                disable_gravity=False,
                max_depenetration_velocity=3.0,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.003,
                rest_offset=0.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.08),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=color,
                roughness=0.70,
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.2,
                dynamic_friction=0.9,
                restitution=0.0,
            ),
        ),
    )


def _groot_root() -> Path:
    return Path(
        os.environ.get("GR00T_WBC_ROOT", "/home/nolovr/GR00T-WholeBodyControl")
    ).expanduser()


def _make_sonic_urdf_spawn(
    asset_path: Path,
    *,
    force_usd_conversion: bool,
) -> sim_utils.UrdfFileCfg:
    return sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=True,
        asset_path=str(asset_path),
        force_usd_conversion=force_usd_conversion,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=0.0,
                damping=0.0,
            )
        ),
    )


def make_sonic_robot_cfg() -> ArticulationCfg:
    """Build the phase-one 29-body + 14-Dex3 SONIC articulation."""

    cfg = G1RobotPresets.g1_29dof_dex3_wholebody(
        init_pos=(0.0, 0.0, 0.76),
        # Identity in Isaac Lab 6's xyzw quaternion convention.
        init_rot=(0.0, 0.0, 0.0, 1.0),
    )

    # Build a deterministic adapter instead of modifying GR00T's ignored data
    # files.  Missing external assets must not break Gym's eager task-package
    # import; selecting this task will then surface the missing generated path
    # directly from the URDF importer.
    asset_path = default_sonic_g1_43dof_output_path()
    try:
        asset_path = build_default_sonic_g1_43dof_urdf(_groot_root())
    except FileNotFoundError as exc:
        # 合成失败静默回退旧产物是刻意的(见上);但 /tmp 被清(如重启)后旧产物
        # 也没了,SONIC 任务必死于"asset path does not exist",在这里先把真因
        # 喊出来。仅在两头都缺时打印,不给非 SONIC 用户加噪声。
        if not asset_path.is_file():
            print(
                "[g1_29dof_dex3_sonic] ⚠️ SONIC URDF 无法合成也无旧产物,"
                f"SONIC 任务将在建环境时失败。合成失败原因: {exc}; "
                f"回退产物不存在: {asset_path}。"
                "检查 GR00T_WBC_ROOT 与 gear_sonic/data 源文件"
                "(源缺失可从 GR00T 仓库 git 历史恢复,验证: "
                "GR00T_WBC_ROOT=... python -m unittest tests.test_g1_sonic_urdf)"
            )
    cfg.spawn = _make_sonic_urdf_spawn(asset_path, force_usd_conversion=True)
    # The adapted carrier adds 14 hand joints and many hand bodies that are not
    # part of the released 29-DoF training articulation.  Keep whole-body
    # self-collision disabled for this carrier to avoid unnecessary collision
    # work and discrete arm/torso contact impulses. This does not disable hand
    # contact with external rigid objects: the URDF hand proxies still
    # participate in normal PhysX contact solving. The exact 29-DoF A/B task
    # replaces this spawn configuration and keeps its training setting.
    cfg.spawn.articulation_props.enabled_self_collisions = False

    # Match policy_parameters.hpp/default_angles and the training robot config.
    # Unlisted body joints and all Dex3 joints start at zero.
    cfg.init_state.joint_pos = {
        ".*_hip_pitch_joint": -0.312,
        ".*_knee_joint": 0.669,
        ".*_ankle_pitch_joint": -0.363,
        ".*_elbow_joint": 0.6,
        "left_shoulder_roll_joint": 0.2,
        "left_shoulder_pitch_joint": 0.2,
        "right_shoulder_roll_joint": -0.2,
        "right_shoulder_pitch_joint": 0.2,
        ".*_hand_.*_joint": 0.0,
    }

    cfg.actuators = {
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
            ],
            effort_limit_sim={
                ".*_hip_yaw_joint": 88.0,
                ".*_hip_roll_joint": 139.0,
                ".*_hip_pitch_joint": 139.0,
                ".*_knee_joint": 139.0,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 32.0,
                ".*_hip_roll_joint": 20.0,
                ".*_hip_pitch_joint": 20.0,
                ".*_knee_joint": 20.0,
            },
            stiffness={
                ".*_hip_pitch_joint": SONIC_STIFFNESS_7520_22,
                ".*_hip_roll_joint": SONIC_STIFFNESS_7520_22,
                ".*_hip_yaw_joint": SONIC_STIFFNESS_7520_14,
                ".*_knee_joint": SONIC_STIFFNESS_7520_22,
            },
            damping={
                ".*_hip_pitch_joint": SONIC_DAMPING_7520_22,
                ".*_hip_roll_joint": SONIC_DAMPING_7520_22,
                ".*_hip_yaw_joint": SONIC_DAMPING_7520_14,
                ".*_knee_joint": SONIC_DAMPING_7520_22,
            },
            armature={
                ".*_hip_pitch_joint": SONIC_ARMATURE_7520_22,
                ".*_hip_roll_joint": SONIC_ARMATURE_7520_22,
                ".*_hip_yaw_joint": SONIC_ARMATURE_7520_14,
                ".*_knee_joint": SONIC_ARMATURE_7520_22,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim=50.0,
            velocity_limit_sim=37.0,
            stiffness=2.0 * SONIC_STIFFNESS_5020,
            damping=2.0 * SONIC_DAMPING_5020,
            armature=2.0 * SONIC_ARMATURE_5020,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=["waist_roll_joint", "waist_pitch_joint"],
            effort_limit_sim=50.0,
            velocity_limit_sim=37.0,
            stiffness=2.0 * SONIC_STIFFNESS_5020,
            damping=2.0 * SONIC_DAMPING_5020,
            armature=2.0 * SONIC_ARMATURE_5020,
        ),
        "waist_yaw": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint"],
            effort_limit_sim=88.0,
            velocity_limit_sim=32.0,
            stiffness=SONIC_STIFFNESS_7520_14,
            damping=SONIC_DAMPING_7520_14,
            armature=SONIC_ARMATURE_7520_14,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
                ".*_wrist_roll_joint",
                ".*_wrist_pitch_joint",
                ".*_wrist_yaw_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_pitch_joint": 25.0,
                ".*_shoulder_roll_joint": 25.0,
                ".*_shoulder_yaw_joint": 25.0,
                ".*_elbow_joint": 25.0,
                ".*_wrist_roll_joint": 25.0,
                ".*_wrist_pitch_joint": 5.0,
                ".*_wrist_yaw_joint": 5.0,
            },
            velocity_limit_sim={
                ".*_shoulder_pitch_joint": 37.0,
                ".*_shoulder_roll_joint": 37.0,
                ".*_shoulder_yaw_joint": 37.0,
                ".*_elbow_joint": 37.0,
                ".*_wrist_roll_joint": 37.0,
                ".*_wrist_pitch_joint": 22.0,
                ".*_wrist_yaw_joint": 22.0,
            },
            stiffness={
                ".*_shoulder_pitch_joint": SONIC_STIFFNESS_5020,
                ".*_shoulder_roll_joint": SONIC_STIFFNESS_5020,
                ".*_shoulder_yaw_joint": SONIC_STIFFNESS_5020,
                ".*_elbow_joint": SONIC_STIFFNESS_5020,
                ".*_wrist_roll_joint": SONIC_STIFFNESS_5020,
                ".*_wrist_pitch_joint": SONIC_STIFFNESS_4010,
                ".*_wrist_yaw_joint": SONIC_STIFFNESS_4010,
            },
            damping={
                ".*_shoulder_pitch_joint": SONIC_DAMPING_5020,
                ".*_shoulder_roll_joint": SONIC_DAMPING_5020,
                ".*_shoulder_yaw_joint": SONIC_DAMPING_5020,
                ".*_elbow_joint": SONIC_DAMPING_5020,
                ".*_wrist_roll_joint": SONIC_DAMPING_5020,
                ".*_wrist_pitch_joint": SONIC_DAMPING_4010,
                ".*_wrist_yaw_joint": SONIC_DAMPING_4010,
            },
            armature={
                ".*_shoulder_pitch_joint": SONIC_ARMATURE_5020,
                ".*_shoulder_roll_joint": SONIC_ARMATURE_5020,
                ".*_shoulder_yaw_joint": SONIC_ARMATURE_5020,
                ".*_elbow_joint": SONIC_ARMATURE_5020,
                ".*_wrist_roll_joint": SONIC_ARMATURE_5020,
                ".*_wrist_pitch_joint": SONIC_ARMATURE_4010,
                ".*_wrist_yaw_joint": SONIC_ARMATURE_4010,
            },
        ),
        # Runtime q/dq/tau/kp/kd come from the two Dex3 HandCmd topics. The
        # non-thumb 0.7 Nm safety cap follows the working MuJoCo loop; the
        # source URDF retains its 1.4 Nm physical limit.
        "hands": ImplicitActuatorCfg(
            joint_names_expr=list(DEX3_HAND_JOINT_NAMES),
            effort_limit_sim=DEX3_PHASE1_EFFORT_LIMITS,
            velocity_limit_sim=DEX3_VELOCITY_LIMITS,
            stiffness=1.5,
            damping=0.1,
            armature=0.01,
            friction=0.1,
            dynamic_friction=0.1,
            viscous_friction=0.05,
        ),
    }
    return cfg


def make_sonic_training_robot_cfg() -> ArticulationCfg:
    """Build the exact 29-DoF URDF articulation used by SONIC training.

    This is an explicit A/B task, not the default carrier.  It removes the 14
    Dex3 joints while retaining the same body actuator and simulation timing
    configuration, so carrier/contact differences can be isolated.
    """

    cfg = make_sonic_robot_cfg()
    groot_root = _groot_root()
    asset_path = groot_root / "gear_sonic/data/assets/robot_description/urdf/g1/main.urdf"
    # Do not fail while this module is imported: the repository auto-imports
    # every task package during Gym registration, including when a non-SONIC
    # task is selected.  If this task is actually instantiated and the checkout
    # is missing, Isaac's URDF importer will report the selected path directly.

    cfg.spawn = _make_sonic_urdf_spawn(asset_path, force_usd_conversion=False)
    cfg.init_state.joint_pos.pop(".*_hand_.*_joint", None)
    cfg.actuators.pop("hands", None)
    return cfg


def zero_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The SONIC bridge scene is not an RL task; keep a zero reward term only for env compatibility."""
    return torch.zeros(env.num_envs, dtype=torch.float32, device=env.device)


@configclass
class G129Dex3SonicSceneCfg(InteractiveSceneCfg):
    """Ground, light and one floating-base G1-29DoF-Dex3 robot."""

    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
                restitution=0.0,
            )
        ),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )

    robot: ArticulationCfg = make_sonic_robot_cfg()

    # Foot-only diagnostics used by the SONIC bridge metrics. Restricting the
    # sensor to the two ankle-roll/sole bodies keeps overhead low while exposing
    # contact force, support phase and slip-related information.
    foot_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*_ankle_roll_link",
        history_length=4,
        track_air_time=True,
        force_threshold=5.0,
        debug_vis=False,
    )


@configclass
class G129SonicSceneCfg(G129Dex3SonicSceneCfg):
    """Default SONIC scene with the 43-DoF robot and one manipulation table."""

    # The reference locomanipulation robot starts at +90 degrees yaw, while the
    # SONIC bridge robot intentionally starts at identity so its policy/world
    # convention remains unchanged.  Rotate the table layout -90 degrees
    # instead: the table stays in front of the robot (+X) and its long edge
    # remains lateral to the robot.
    packing_table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PackingTable",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.55, 0.0, -0.3),
            # -90 degrees yaw in Isaac Lab 6's xyzw convention.
            rot=(0.0, 0.0, -0.70710678, 0.70710678),
        ),
        spawn=UsdFileCfg(
            usd_path=str(SONIC_PACKING_TABLE_USD),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )

    # The source locomanipulation scene contains two cubes and one long box.
    # This SONIC phase explicitly requests three equal dynamic cubes, so the
    # third cube keeps the reference object's center while using the same 5 cm
    # geometry and mass as the first two.
    cube_1 = _make_sonic_cube_cfg(
        prim_name="Cube1",
        initial_pos=(0.31243, -0.00553, SONIC_CUBE_INITIAL_Z),
        color=(0.82, 0.66, 0.36),
    )
    cube_2 = _make_sonic_cube_cfg(
        prim_name="Cube2",
        initial_pos=(0.31397, 0.10565, SONIC_CUBE_INITIAL_Z),
        color=(0.88, 0.72, 0.40),
    )
    cube_3 = _make_sonic_cube_cfg(
        prim_name="Cube3",
        initial_pos=(0.41625, 0.04810, SONIC_CUBE_INITIAL_Z),
        color=(0.76, 0.56, 0.28),
    )


@configclass
class G129TrainingSonicSceneCfg(G129Dex3SonicSceneCfg):
    """Exact released SONIC 29-DoF training articulation for A/B regression."""

    robot: ArticulationCfg = make_sonic_training_robot_cfg()


@configclass
class ActionsCfg:
    """Complete LowCmd motion targets supplied by SonicDDSActionProvider.

    The action tensor is field-major: all q targets, followed by all dq targets,
    followed by all feed-forward tau targets.  LowCmd kp/kd are persistent
    articulation properties and are written by the provider only when changed.
    """

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=False,
        preserve_order=True,
    )
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=False,
        preserve_order=True,
    )
    joint_effort = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        preserve_order=True,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # This observation also writes the 29 body states and IMU sample to the
        # existing G1 DDS shared-memory publisher.  Publish once per 50 Hz
        # environment step: wall-clock throttling aliases a slightly early
        # 19.x ms step into an unintended 25--33 Hz state stream.
        robot_body_state = ObsTerm(
            func=get_robot_boy_joint_states,
            params={"dds_min_interval_ms": 0.0},
        )
        # Publish actual q/dq/applied-torque in the same seven-joint-per-hand
        # order consumed by Gear SONIC. Dex3Hands uses this feedback to clamp
        # each outgoing position target to current_q +/- 0.25 rad.
        robot_dex3_state = ObsTerm(
            func=get_robot_dex3_joint_states,
            params={"dds_min_interval_ms": 0.0},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class TrainingObservationsCfg:
    """Body-only observation set for the exact 29-DoF regression task."""

    @configclass
    class PolicyCfg(ObsGroup):
        robot_body_state = ObsTerm(
            func=get_robot_boy_joint_states,
            params={"dds_min_interval_ms": 0.0},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    bridge_alive = RewTerm(func=zero_reward, weight=0.0)


@configclass
class TerminationsCfg:
    pass


@configclass
class EventsCfg:
    """No automatic events; resets are triggered explicitly by sim_main."""

    pass


@configclass
class G129Dex3SonicEnvCfg(ManagerBasedRLEnvCfg):
    """Minimal environment for the SONIC -> DDS -> Isaac Lab control loop."""

    scene: G129Dex3SonicSceneCfg = G129Dex3SonicSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()
    commands = None
    curriculum = None

    def __post_init__(self):
        # Released SONIC training uses 4 physics steps at 5 ms for each 50 Hz
        # policy/control update.  Keeping both values exact is important for
        # contact dynamics and the ten-frame proprioceptive history.
        self.decimation = 4
        self.episode_length_s = 24.0 * 60.0 * 60.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        # Keep the remaining PhysX fields at Isaac Lab defaults.  In particular,
        # do not retain the older bridge scene's custom bounce threshold or
        # friction correlation distance here.
        self.sim.physics_material.static_friction = 1.0
        self.sim.physics_material.dynamic_friction = 1.0
        self.sim.physics_material.friction_combine_mode = "multiply"
        self.sim.physics_material.restitution_combine_mode = "multiply"

        # sim_main uses this lightweight manager for external reset commands.
        self.event_manager = SimpleEventManager()
        reset_scene = SimpleEvent(
            func=lambda env: mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=env.device),
                reset_joint_targets=True,
            )
        )
        self.event_manager.register("reset_object_self", reset_scene)
        self.event_manager.register("reset_all_self", reset_scene)


@configclass
class G129SonicEnvCfg(G129Dex3SonicEnvCfg):
    """Default SONIC task using the adapted 43-DoF articulation."""

    scene: G129SonicSceneCfg = G129SonicSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )
    xr: XrCfg = XrCfg(
        anchor_pos=(0.0, 0.0, 0.0),
        # Identity in XrCfg's xyzw convention.  The old wxyz identity literal
        # was a 180-degree X rotation in Isaac Lab 6 and inverted the XR world.
        anchor_rot=(0.0, 0.0, 0.0, 1.0),
    )

    def __post_init__(self):
        super().__post_init__()
        # Keep PhysX's default external-force update behavior.  Enabling the
        # per-iteration mode is a TGS-specific numerical option, not an
        # equivalent of MuJoCo's Newton solver, and it showed no measured
        # benefit in the current closed loop.
        self.sim.physx.enable_external_forces_every_iteration = False

        # Isaac Sim 6 places articulated links below the Geometry scope.  Use
        # the non-instanceable physical torso as the position anchor so that
        # OpenXR can safely author its XRAnchor child.  Yaw follows the pelvis,
        # avoiding torso roll/pitch from tilting the XR world.
        self.xr.anchor_prim_path = (
            "/World/envs/env_0/Robot/Geometry/pelvis/waist_yaw_link/"
            "waist_roll_link/torso_link"
        )
        self.xr.anchor_rotation_prim_path = (
            "/World/envs/env_0/Robot/Geometry/pelvis"
        )
        self.xr.fixed_anchor_height = False
        self.xr.anchor_rotation_mode = XrAnchorRotationMode.FOLLOW_PRIM_SMOOTHED

        # Match the reference PICO/OpenXR calibration: release right-controller
        # B to align the headset's horizontal forward direction with the robot
        # pelvis yaw.  This is view recentering only, not an environment reset.
        self.xr.recenter_yaw_button = ("/user/hand/right", "b")
        self.xr.recenter_yaw_button_event = "release"
        self.xr.recenter_anchor_forward_axis = (-1.0, 0.0, 0.0)
        self.xr.recenter_headset_forward_axis = (0.0, -1.0, 0.0)
        self.xr.recenter_headset_fallback_axis = (1.0, 0.0, 0.0)

        # No retargeter is installed in this phase: OpenXR owns only the view
        # anchor and B-button recenter event.  SONIC DDS remains the sole source
        # of body and Dex3 joint commands.
        self.teleop_devices = DevicesCfg(
            devices={
                "motion_controllers": OpenXRDeviceCfg(
                    retargeters=[],
                    sim_device="cpu",
                    xr_cfg=self.xr,
                )
            }
        )


@configclass
class G129TrainingSonicEnvCfg(G129Dex3SonicEnvCfg):
    """Exact 29-DoF training-URDF regression task."""

    scene: G129TrainingSonicSceneCfg = G129TrainingSonicSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )
    observations: TrainingObservationsCfg = TrainingObservationsCfg()
