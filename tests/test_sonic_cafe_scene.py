import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CFG_PATH = ROOT / "tasks/g1_tasks/g1_29dof_sonic_cafe/cafe_env_cfg.py"
INIT_PATH = ROOT / "tasks/g1_tasks/g1_29dof_sonic_cafe/__init__.py"
SIM_MAIN_PATH = ROOT / "sim_main.py"
CONVEYOR_CFG_PATH = ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
PIPELINE_VIEWER_PATH = ROOT / "run_pipeline_viewer.bat"
PIPELINE_VIEWER_AR_PATH = ROOT / "run_pipeline_viewer_ar.bat"
CAFE_VIEWER_PATH = ROOT / "run_cafe_viewer.bat"
CAFE_VIEWER_AR_PATH = ROOT / "run_cafe_viewer_ar.bat"
CAFE_VIEWER_AR_ROBOT2_PATH = ROOT / "run_cafe_viewer_ar_robot2.bat"


class TestSonicCafeScene(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg_source = CFG_PATH.read_text(encoding="utf-8")
        cls.cfg_tree = ast.parse(cls.cfg_source)

    def test_only_scene_assets_and_poses_are_cafe_specific(self):
        self.assertIn('"Locomotion", "KitchenRoom", "KitchenRoom.usd"', self.cfg_source)
        self.assertIn("ROBOT_1_POS = (0.18, -0.42, 0.76)", self.cfg_source)
        self.assertIn("ROBOT_2_POS = (0.26, 0.80, 0.76)", self.cfg_source)
        self.assertIn("CUP_SPAWN_POS = (0.02, 0.04, 0.91)", self.cfg_source)
        self.assertIn("/Objects/Mug/mug.usd", self.cfg_source)

    def test_kitchen_mode_defaults_proxy_and_rejects_invalid_values(self):
        self.assertIn('KITCHEN_MODE_ENV = "ISAACLAB_CAFE_KITCHEN_MODE"', self.cfg_source)
        self.assertIn('KITCHEN_MODE_STATIC = "static_background"', self.cfg_source)
        self.assertIn('KITCHEN_MODE_PROXY = "proxy_background"', self.cfg_source)
        self.assertIn('KITCHEN_MODE_LEGACY = "legacy_dynamic"', self.cfg_source)
        self.assertIn("(KITCHEN_MODE_STATIC, KITCHEN_MODE_PROXY, KITCHEN_MODE_LEGACY)", self.cfg_source)
        self.assertIn("os.environ.get(KITCHEN_MODE_ENV, KITCHEN_MODE_PROXY)", self.cfg_source)
        self.assertIn("or KITCHEN_MODE_PROXY", self.cfg_source)
        self.assertIn("mode not in _KITCHEN_MODES", self.cfg_source)
        self.assertIn("raise ValueError", self.cfg_source)
        self.assertIn("KITCHEN_ROOM_MODE = resolve_kitchen_room_mode()", self.cfg_source)

    def test_static_kitchen_spawner_preserves_collision_contract(self):
        self.assertIn("root_prim.Load(Usd.LoadWithDescendants)", self.cfg_source)
        self.assertIn("Usd.TraverseInstanceProxies()", self.cfg_source)
        self.assertIn("prim.SetInstanceable(False)", self.cfg_source)
        self.assertIn("with Usd.EditContext(stage, stage.GetSessionLayer()):", self.cfg_source)
        self.assertIn("GetJointEnabledAttr().Set(False)", self.cfg_source)
        self.assertIn("_remove_api(usd_articulations, UsdPhysics.ArticulationRootAPI", self.cfg_source)
        self.assertIn("_remove_api(physx_articulations, PhysxSchema.PhysxArticulationAPI", self.cfg_source)
        self.assertIn("_remove_api(usd_rigid_bodies, UsdPhysics.RigidBodyAPI", self.cfg_source)
        self.assertIn("_remove_api(physx_rigid_bodies, PhysxSchema.PhysxRigidBodyAPI", self.cfg_source)
        self.assertIn("if not strip_collisions and collision_before != collision_after:", self.cfg_source)
        self.assertIn("collisions_preserved=", self.cfg_source)
        self.assertIn("cfg.func = _spawn_kitchen_room", self.cfg_source)

    def test_proxy_kitchen_replaces_composed_collisions_with_two_static_boxes(self):
        self.assertIn("strip_collisions: bool = False", self.cfg_source)
        self.assertIn("_remove_api(physx_collisions, PhysxSchema.PhysxCollisionAPI", self.cfg_source)
        self.assertIn("_remove_api(mesh_collisions, UsdPhysics.MeshCollisionAPI", self.cfg_source)
        self.assertIn("_remove_api(usd_collisions, UsdPhysics.CollisionAPI", self.cfg_source)
        self.assertIn("if strip_collisions and (collision_after or remaining_collision_apis):", self.cfg_source)
        self.assertIn("CAFE_FLOOR_PROXY_POS = (0.0, 0.052293800, -0.010499233)", self.cfg_source)
        self.assertIn("CAFE_FLOOR_PROXY_SIZE = (5.275517464, 4.987211227, 0.020000000)", self.cfg_source)
        self.assertIn("CAFE_ISLAND_PROXY_POS = (0.213442038, 0.225252678, 0.429156477)", self.cfg_source)
        self.assertIn("CAFE_ISLAND_PROXY_SIZE = (1.149627462, 0.761342592, 0.858449757)", self.cfg_source)
        self.assertIn('"CafeFloorProxy"', self.cfg_source)
        self.assertIn('"CafeIslandProxy"', self.cfg_source)
        self.assertIn("spawn=sim_utils.CuboidCfg(", self.cfg_source)
        self.assertIn("visible=False", self.cfg_source)
        self.assertIn("collision_enabled=True", self.cfg_source)

    def test_robot_factories_are_reused_from_conveyor(self):
        self.assertIn("_make_conveyor_local_robot_cfg", self.cfg_source)
        self.assertIn("_make_conveyor_additional_local_robot_cfg(2)", self.cfg_source)
        self.assertIn("_make_conveyor_peer_scene_cfg", self.cfg_source)
        self.assertIn("_make_conveyor_additional_peer_scene_cfg(2)", self.cfg_source)
        self.assertNotIn("_make_conveyor_second_local_robot_cfg", self.cfg_source)
        self.assertNotIn("_make_conveyor_second_peer_scene_cfg", self.cfg_source)
        self.assertNotIn("standby_robot_", self.cfg_source)
        self.assertNotIn("make_sonic_robot_cfg", self.cfg_source)

    def test_scene_uses_the_same_dynamic_topology_as_conveyor(self):
        scene_class = next(
            node for node in self.cfg_tree.body
            if isinstance(node, ast.ClassDef) and node.name == "G129DualSonicCafeSceneCfg"
        )
        assignments = {
            target.id for node in scene_class.body
            if isinstance(node, ast.AnnAssign) and isinstance((target := node.target), ast.Name)
        }
        self.assertTrue({
            "robot", "robot_2", "peer_robot", "peer_robot_2",
            "foot_contact", "foot_contact_2",
        } <= assignments)
        self.assertIn("None if HOST_MODE else _make_primary_peer_scene_cfg()", self.cfg_source)
        self.assertIn("_make_second_peer_scene_cfg() if VIEWER_MODE else None", self.cfg_source)
        self.assertIn("_make_second_local_robot_cfg() if HOST_MODE else None", self.cfg_source)
        for extra_name in (
            "robot_3", "robot_4", "robot_5",
            "peer_robot_3", "peer_robot_4", "peer_robot_5",
            "standby_robot_1", "standby_robot_2", "standby_robot_3",
        ):
            self.assertNotIn(extra_name, assignments)

    def test_viewer_uses_conveyor_full_fidelity_peer_asset(self):
        conveyor_source = CONVEYOR_CFG_PATH.read_text(encoding="utf-8")
        self.assertIn('"peer_robot" / "g1_43dof_peer.usd"', conveyor_source)
        self.assertIn("def _make_peer_scene_cfg()", conveyor_source)
        self.assertIn("if PEER_VISUAL_LOD:", conveyor_source)
        self.assertIn("return _make_peer_robot_cfg()", conveyor_source)
        self.assertIn("return _make_additional_peer_robot_cfg(robot_id)", conveyor_source)

    def test_windows_viewers_reuse_pipeline_and_force_articulation(self):
        desktop_source = CAFE_VIEWER_PATH.read_text(encoding="utf-8")
        ar_source = CAFE_VIEWER_AR_PATH.read_text(encoding="utf-8")
        ar_robot2_source = CAFE_VIEWER_AR_ROBOT2_PATH.read_text(encoding="utf-8")
        for source in (desktop_source, ar_source):
            self.assertIn('set "ISAACLAB_PEER_ROBOT_MODE=articulation"', source)
            self.assertNotIn("visual_lod", source)
            self.assertIn("--task Isaac-G1-29DoF-Sonic-Cafe", source)
        self.assertIn('call "%~dp0run_pipeline_viewer.bat"', desktop_source)
        self.assertIn('call "%~dp0run_pipeline_viewer_ar.bat"', ar_source)
        self.assertIn('set "ISAACLAB_XR_ANCHOR_ROBOT_ID=2"', ar_robot2_source)
        self.assertIn('call "%~dp0run_cafe_viewer_ar.bat"', ar_robot2_source)
        self.assertNotIn(
            "ISAACLAB_PEER_ROBOT_MODE",
            PIPELINE_VIEWER_PATH.read_text(encoding="utf-8"),
        )
        self.assertNotIn(
            "ISAACLAB_PEER_ROBOT_MODE",
            PIPELINE_VIEWER_AR_PATH.read_text(encoding="utf-8"),
        )

    def test_actions_include_scene_and_reset_sync(self):
        self.assertIn("class CafeActionsCfg(ConveyorActionsCfg)", self.cfg_source)
        self.assertIn("class HostCafeActionsCfg(HostConveyorActionsCfg)", self.cfg_source)
        self.assertIn("HostObservationsCfg()", self.cfg_source)
        self.assertIn("ViewerObservationsCfg()", self.cfg_source)
        self.assertIn("if VIEWER_MODE", self.cfg_source)
        self.assertEqual(self.cfg_source.count("scene_state_sync = _scene_state_sync_cfg()"), 2)
        self.assertNotIn("joint_pos_2 = mdp.", self.cfg_source)
        self.assertIn('SYNC_OBJECT_NAMES = ("cup",)', self.cfg_source)

    def test_cup_authority_and_mirror_are_explicit(self):
        self.assertIn("mirror = MIRROR_OBJECTS", self.cfg_source)
        self.assertIn("kinematic_enabled=mirror", self.cfg_source)
        self.assertIn("disable_gravity=mirror", self.cfg_source)
        self.assertIn("if MIRROR_OBJECTS:", self.cfg_source)
        self.assertIn("SimpleEvent(func=reset_scene_mirror_safe)", self.cfg_source)
        self.assertIn("reset_scene_mirror_safe", self.cfg_source)

    def test_task_is_registered_and_uses_shared_sync_identity(self):
        task_id = "Isaac-G1-29DoF-Sonic-Cafe"
        self.assertIn(task_id, INIT_PATH.read_text(encoding="utf-8"))
        sim_source = SIM_MAIN_PATH.read_text(encoding="utf-8")
        self.assertIn(task_id, sim_source)
        self.assertIn('"Isaac-G1-29DoF-Sonic-Conveyor"', sim_source)
        self.assertIn('"Isaac-G1-29DoF-Sonic-Cafe"', sim_source)
        self.assertIn("if args_cli.task in scene_sync_sonic_task_names:", sim_source)
        self.assertIn(
            'apply_g1_sonic_visual_materials("/World/envs/env_0/PeerRobot")',
            sim_source,
        )
        self.assertNotIn(
            'is_scene_sync_host = args_cli.task == "Isaac-G1-29DoF-Sonic-Cafe"',
            sim_source,
        )


if __name__ == "__main__":
    unittest.main()
