"""Empty floating-base G1-29DoF-Dex3 scene for Gear SONIC DDS control."""

from __future__ import annotations

import torch

import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
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
                friction_combine_mode="max",
                restitution_combine_mode="min",
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

    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex3_wholebody(
        init_pos=(0.0, 0.0, 0.80),
        init_rot=(1.0, 0.0, 0.0, 0.0),
    )


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
    events = None
    commands = None
    curriculum = None

    def __post_init__(self):
        self.decimation = 1
        self.episode_length_s = 24.0 * 60.0 * 60.0
        self.sim.dt = 0.002
        self.sim.render_interval = 10
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.friction_correlation_distance = 0.00625
        self.sim.physics_material.static_friction = 1.0
        self.sim.physics_material.dynamic_friction = 1.0
        self.sim.physics_material.friction_combine_mode = "max"
        self.sim.physics_material.restitution_combine_mode = "min"

        # sim_main uses this lightweight manager for external reset commands.
        self.event_manager = SimpleEventManager()
        reset_scene = SimpleEvent(
            func=lambda env: mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=env.device),
            )
        )
        self.event_manager.register("reset_object_self", reset_scene)
        self.event_manager.register("reset_all_self", reset_scene)
