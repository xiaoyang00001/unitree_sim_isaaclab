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

from tasks.common_config import G1RobotPresets
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager
from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states


# Keep the bridge dynamics identical to the released SONIC training setup in
# GR00T-WholeBodyControl/gear_sonic/envs/manager_env/robots/g1.py.  The policy
# was trained with these motor inertias, 10 Hz natural frequency and damping
# ratio 2.0.
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


def make_sonic_robot_cfg() -> ArticulationCfg:
    """Build the later-phase Dex3 asset with the SONIC policy's body dynamics."""

    cfg = G1RobotPresets.g1_29dof_dex3_wholebody(
        init_pos=(0.0, 0.0, 0.76),
        init_rot=(1.0, 0.0, 0.0, 0.0),
    )

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

    # Keep the released training solver iterations.  The deployed USD contains
    # 14 extra Dex3 joints and 25 extra hand bodies that are absent from the
    # training URDF; enabling whole-articulation self-collision for that asset
    # raises a 20 ms environment step to about 23 ms on the target machine.  It
    # is therefore kept disabled, matching this USD's original configuration,
    # so the 50 Hz closed loop remains real-time.
    cfg.spawn.articulation_props.enabled_self_collisions = False
    cfg.spawn.articulation_props.solver_position_iteration_count = 8
    cfg.spawn.articulation_props.solver_velocity_iteration_count = 4

    dex3_actuator = cfg.actuators["hands"]
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
        # Phase 1 keeps the 14 Dex3 joints at their default positions.
        "hands": dex3_actuator,
    }
    return cfg


def make_sonic_training_robot_cfg() -> ArticulationCfg:
    """Build the exact 29-DoF URDF articulation used by SONIC training.

    The repository's whole-body Dex3 USD gives inertial mass to several visual
    sensor helper links that have no inertial element in the released training
    URDF.  That changes the simulated robot mass by 4.22 kg.  The non-VR body
    validation therefore uses the original URDF conversion path first; the
    articulated Dex3 model remains registered separately for the later hand
    control phase.
    """

    cfg = make_sonic_robot_cfg()
    groot_root = Path(
        os.environ.get("GR00T_WBC_ROOT", "/home/nolovr/GR00T-WholeBodyControl")
    ).expanduser()
    asset_path = groot_root / "gear_sonic/data/assets/robot_description/urdf/g1/main.urdf"
    # Do not fail while this module is imported: the repository auto-imports
    # every task package during Gym registration, including when a non-SONIC
    # task is selected.  If this task is actually instantiated and the checkout
    # is missing, Isaac's URDF importer will report the selected path directly.

    cfg.spawn = sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=True,
        asset_path=str(asset_path),
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
            # Match the released SONIC training articulation exactly.  The
            # earlier 25 Hz measurement with this enabled was caused by an
            # accidentally enabled WebRTC stream, not by self-collision.
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
    """Exact released SONIC 29-DoF training articulation, without Dex3 DOFs."""

    robot: ArticulationCfg = make_sonic_training_robot_cfg()


@configclass
class ActionsCfg:
    """Absolute joint-position targets supplied by SonicDDSActionProvider."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=False,
        preserve_order=True,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # This observation also writes the 29 body states and IMU sample to the
        # existing G1 DDS shared-memory publisher.
        robot_body_state = ObsTerm(func=get_robot_boy_joint_states)

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
        # Keep the remaining PhysX fields at Isaac Lab defaults, matching the
        # released SONIC training environment.  In particular, do not retain
        # the older bridge scene's custom bounce threshold or friction
        # correlation distance here.
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
    """Non-VR 29-DoF baseline using the exact SONIC training URDF."""

    scene: G129SonicSceneCfg = G129SonicSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )
