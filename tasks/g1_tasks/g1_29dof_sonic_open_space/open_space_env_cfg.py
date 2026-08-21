"""One G1 SONIC robot walking in selectable real environments.

The terrain tasks below are deliberately kept as lightweight asset choices.
``Warehouse`` is the complete-scene option: it comes from Isaac Sim's asset
library and contains the floor, walls, racks, lights, and scene structure.
"""

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import (
    G129Dex3SonicSceneCfg,
    G129SonicEnvCfg,
    make_sonic_robot_cfg,
)


LIGHTWHEEL_ROOT = os.environ.get(
    "LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR", "/home/nolo/Lightwheel_OpenSource"
)
REAL_TERRAIN_USDS = {
    "grass": os.path.join(LIGHTWHEEL_ROOT, "Locomotion", "Grass", "Grass.usd"),
    "gravel": os.path.join(
        LIGHTWHEEL_ROOT, "Locomotion", "GravelGround", "GravelGround.usd"
    ),
    "mud": os.path.join(LIGHTWHEEL_ROOT, "Locomotion", "MudGround", "MudGround.usd"),
    "slate": os.path.join(
        LIGHTWHEEL_ROOT, "Locomotion", "SLATEGround", "SLATEGround.usd"
    ),
    "snow": os.path.join(
        LIGHTWHEEL_ROOT, "Locomotion", "SnowGround", "SnowGround.usd"
    ),
}

# Official Isaac Sim resource-library scene.  Unlike the Lightwheel files
# above, this is a complete warehouse layout rather than an isolated ground
# material/patch.  The asset client resolves its referenced Props on Nucleus.
REAL_WAREHOUSE_USD = os.environ.get(
    "ISAAC_REAL_WAREHOUSE_USD",
    f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",
)

REAL_COMPLETE_SCENE_USDS = {
    "apartment": os.path.join(
        LIGHTWHEEL_ROOT, "Locomotion", "Apartment", "scene_04.usd"
    ),
    "staircase": os.path.join(
        LIGHTWHEEL_ROOT, "Locomotion", "2-StoryStaircase", "2-StoryStaircase.usd"
    ),
    "office": f"{ISAAC_NUCLEUS_DIR}/Environments/Office/office.usd",
}


def _make_real_terrain_cfg(asset_key: str) -> AssetBaseCfg:
    """Spawn one collected terrain USD as the only non-robot scene asset."""

    return AssetBaseCfg(
        prim_path="/World/envs/env_.*/RealTerrain",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=UsdFileCfg(usd_path=REAL_TERRAIN_USDS[asset_key]),
    )


def _make_complete_scene_cfg(asset_key: str, prim_name: str) -> AssetBaseCfg:
    """Spawn a complete room/building asset, including its own floor/collisions."""

    return AssetBaseCfg(
        prim_path=f"/World/envs/env_.*/{prim_name}",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=UsdFileCfg(usd_path=REAL_COMPLETE_SCENE_USDS[asset_key]),
    )


TERRAIN_ROBOT_POS = {
    "grass": (1.0, 0.0, 0.76),
    "gravel": (0.0, -5.0, 0.76),
    "mud": (0.0, 0.0, 0.76),
    "slate": (0.0, -1.0, 0.76),
    "snow": (0.0, -0.5, 0.76),
}


@configclass
class G129SonicOpenSpaceSceneCfg(G129Dex3SonicSceneCfg):
    """One SONIC G1 on the real, open GravelGround terrain.

    The terrain USD supplies the visible/collision ground.  The inherited
    procedural plane is disabled so the robot contacts only the real asset.
    """

    ground = None

    background = _make_real_terrain_cfg("gravel")

    # Exactly one dynamic robot; the only other asset is the static real terrain.
    robot: ArticulationCfg = make_sonic_robot_cfg()
    robot.init_state.pos = TERRAIN_ROBOT_POS["gravel"]


@configclass
class G129SonicGrassSceneCfg(G129SonicOpenSpaceSceneCfg):
    """Real grass terrain (compact patch)."""

    background = _make_real_terrain_cfg("grass")
    robot: ArticulationCfg = make_sonic_robot_cfg()
    robot.init_state.pos = TERRAIN_ROBOT_POS["grass"]


@configclass
class G129SonicMudSceneCfg(G129SonicOpenSpaceSceneCfg):
    """Real muddy terrain."""

    background = _make_real_terrain_cfg("mud")
    robot: ArticulationCfg = make_sonic_robot_cfg()
    robot.init_state.pos = TERRAIN_ROBOT_POS["mud"]


@configclass
class G129SonicSlateSceneCfg(G129SonicOpenSpaceSceneCfg):
    """Real slate-stone terrain."""

    background = _make_real_terrain_cfg("slate")
    robot: ArticulationCfg = make_sonic_robot_cfg()
    robot.init_state.pos = TERRAIN_ROBOT_POS["slate"]


@configclass
class G129SonicSnowSceneCfg(G129SonicOpenSpaceSceneCfg):
    """Real snow-road terrain."""

    background = _make_real_terrain_cfg("snow")
    robot: ArticulationCfg = make_sonic_robot_cfg()
    robot.init_state.pos = TERRAIN_ROBOT_POS["snow"]


@configclass
class G129SonicWarehouseSceneCfg(G129SonicOpenSpaceSceneCfg):
    """One SONIC G1 inside the complete Isaac Sim Simple Warehouse scene."""

    background = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Warehouse",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=UsdFileCfg(usd_path=REAL_WAREHOUSE_USD),
    )
    # Fresh cfg avoids mutating the inherited terrain robot class attribute.
    robot: ArticulationCfg = make_sonic_robot_cfg()
    robot.init_state.pos = (0.0, 0.0, 0.76)


@configclass
class G129SonicApartmentSceneCfg(G129SonicOpenSpaceSceneCfg):
    """Complete locally collected apartment/building scene."""

    background = _make_complete_scene_cfg("apartment", "Apartment")
    robot: ArticulationCfg = make_sonic_robot_cfg()
    # (0, 0) is occupied by the kitchen island in scene_04.usd; use the
    # carpeted living-room floor so the robot stands on the actual ground.
    robot.init_state.pos = (-2.1, -0.15, 0.76)


@configclass
class G129SonicStaircaseSceneCfg(G129SonicOpenSpaceSceneCfg):
    """Complete two-story loft and staircase scene with an open ground area."""

    background = _make_complete_scene_cfg("staircase", "TwoStoryStaircase")
    robot: ArticulationCfg = make_sonic_robot_cfg()
    robot.init_state.pos = (0.0, -12.0, 0.76)


@configclass
class G129SonicOfficeSceneCfg(G129SonicOpenSpaceSceneCfg):
    """Official Isaac Sim office environment with one SONIC G1."""

    background = _make_complete_scene_cfg("office", "Office")
    robot: ArticulationCfg = make_sonic_robot_cfg()
    # The official office asset's main floor is around this open corridor.
    robot.init_state.pos = (-8.0, 21.0, 0.76)


@configclass
class G129SonicOpenSpaceEnvCfg(G129SonicEnvCfg):
    """SONIC bridge environment backed by the single-robot open-space scene."""

    scene: G129SonicOpenSpaceSceneCfg = G129SonicOpenSpaceSceneCfg(
        num_envs=1,
        env_spacing=0.0,
        replicate_physics=True,
    )

    def __post_init__(self):
        super().__post_init__()
        # The gravel patch is an outdoor scan with no enclosing landmarks;
        # keep the camera close enough to show the SONIC robot and ground
        # texture instead of leaving the robot as a tiny horizon silhouette.
        self.viewer.eye = (3.5, -0.5, 2.8)
        self.viewer.lookat = (0.0, -5.0, 0.95)


@configclass
class G129SonicGrassEnvCfg(G129SonicEnvCfg):
    scene: G129SonicGrassSceneCfg = G129SonicGrassSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = (-4.0, 5.0, 3.0)
        self.viewer.lookat = TERRAIN_ROBOT_POS["grass"]


@configclass
class G129SonicMudEnvCfg(G129SonicEnvCfg):
    scene: G129SonicMudSceneCfg = G129SonicMudSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = (-4.0, 5.0, 3.0)
        self.viewer.lookat = TERRAIN_ROBOT_POS["mud"]


@configclass
class G129SonicSlateEnvCfg(G129SonicEnvCfg):
    scene: G129SonicSlateSceneCfg = G129SonicSlateSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = (-4.0, 5.0, 3.0)
        self.viewer.lookat = TERRAIN_ROBOT_POS["slate"]


@configclass
class G129SonicSnowEnvCfg(G129SonicEnvCfg):
    scene: G129SonicSnowSceneCfg = G129SonicSnowSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = (-4.0, 5.0, 3.0)
        self.viewer.lookat = TERRAIN_ROBOT_POS["snow"]


@configclass
class G129SonicWarehouseEnvCfg(G129SonicEnvCfg):
    """SONIC bridge environment using the complete warehouse background."""

    scene: G129SonicWarehouseSceneCfg = G129SonicWarehouseSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )

    def __post_init__(self):
        super().__post_init__()
        # Keep the warehouse context in frame while making the lone robot
        # large enough to inspect from the active Kit viewport.
        self.viewer.eye = (3.8, -5.0, 2.8)
        self.viewer.lookat = (0.0, 0.0, 1.0)


@configclass
class G129SonicApartmentEnvCfg(G129SonicEnvCfg):
    scene: G129SonicApartmentSceneCfg = G129SonicApartmentSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )

    def __post_init__(self):
        super().__post_init__()
        # The apartment asset has a solid roof/upper shell.  Use an interior
        # eye point so the viewport sees the room and the robot instead of the
        # exterior roof from the inherited high-angle view.
        self.viewer.eye = (-4.0, -0.8, 2.2)
        self.viewer.lookat = (-2.1, -0.15, 0.95)


@configclass
class G129SonicStaircaseEnvCfg(G129SonicEnvCfg):
    scene: G129SonicStaircaseSceneCfg = G129SonicStaircaseSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = (8.0, -14.0, 6.0)
        self.viewer.lookat = (0.0, -2.0, 1.0)


@configclass
class G129SonicOfficeEnvCfg(G129SonicEnvCfg):
    scene: G129SonicOfficeSceneCfg = G129SonicOfficeSceneCfg(
        num_envs=1, env_spacing=0.0, replicate_physics=True
    )

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = (-3.0, 15.0, 5.0)
        self.viewer.lookat = (-8.0, 21.0, 1.0)
