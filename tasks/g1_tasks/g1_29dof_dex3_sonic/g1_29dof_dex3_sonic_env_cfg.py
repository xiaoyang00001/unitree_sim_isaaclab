"""Floating-base G1-29DoF scenes for Gear SONIC DDS control."""

from __future__ import annotations

import os
from pathlib import Path

import torch

import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg
from isaaclab.utils import configclass

from robots.g1_sonic_urdf import (
    DEX3_HAND_JOINT_NAMES,
    build_default_sonic_g1_43dof_urdf,
    default_sonic_g1_43dof_output_path,
)
from tasks.common_config import G1RobotPresets
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager
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
        init_rot=(1.0, 0.0, 0.0, 0.0),
    )

    # Build a deterministic adapter instead of modifying GR00T's ignored data
    # files.  Missing external assets must not break Gym's eager task-package
    # import; selecting this task will then surface the missing generated path
    # directly from the URDF importer.
    asset_path = default_sonic_g1_43dof_output_path()
    try:
        asset_path = build_default_sonic_g1_43dof_urdf(_groot_root())
    except FileNotFoundError:
        pass
    cfg.spawn = _make_sonic_urdf_spawn(asset_path, force_usd_conversion=True)
    # The adapted carrier adds 14 hand joints and many hand bodies that are not
    # part of the released 29-DoF training articulation.  Keep whole-body
    # self-collision disabled for this carrier to avoid unnecessary collision
    # work and discrete arm/torso contact impulses.  The exact 29-DoF A/B task
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
        # Phase 1 keeps the 14 joints articulated and open, but does not consume
        # PICO/Dex3 commands yet.  The non-thumb 0.7 Nm safety cap follows the
        # working MuJoCo loop; the source URDF retains its 1.4 Nm physical limit.
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


@configclass
class G129SonicSceneCfg(G129Dex3SonicSceneCfg):
    """Default SONIC scene: 29 controlled body joints plus 14 held Dex3 joints."""


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
    """Default non-VR SONIC task using the adapted 43-DoF articulation."""

    scene: G129SonicSceneCfg = G129SonicSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )

    def __post_init__(self):
        super().__post_init__()
        # Keep PhysX's default external-force update behavior.  Enabling the
        # per-iteration mode is a TGS-specific numerical option, not an
        # equivalent of MuJoCo's Newton solver, and it showed no measured
        # benefit in the current closed loop.
        self.sim.physx.enable_external_forces_every_iteration = False


@configclass
class G129TrainingSonicEnvCfg(G129Dex3SonicEnvCfg):
    """Exact 29-DoF training-URDF regression task."""

    scene: G129TrainingSonicSceneCfg = G129TrainingSonicSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )
