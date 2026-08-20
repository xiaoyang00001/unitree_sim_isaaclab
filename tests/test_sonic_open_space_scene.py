import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CFG_PATH = ROOT / "tasks/g1_tasks/g1_29dof_sonic_open_space/open_space_env_cfg.py"
INIT_PATH = ROOT / "tasks/g1_tasks/g1_29dof_sonic_open_space/__init__.py"
TASKS_INIT_PATH = ROOT / "tasks/g1_tasks/__init__.py"
SIM_MAIN_PATH = ROOT / "sim_main.py"


class TestSonicOpenSpaceScene(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg_source = CFG_PATH.read_text(encoding="utf-8")
        cls.cfg_tree = ast.parse(cls.cfg_source)

    def test_scene_uses_real_open_ground_asset(self):
        for asset_name in ("Grass.usd", "GravelGround.usd", "MudGround.usd", "SLATEGround.usd", "SnowGround.usd"):
            self.assertIn(asset_name, self.cfg_source)
        self.assertIn("UsdFileCfg", self.cfg_source)
        self.assertIn("warehouse.usd", self.cfg_source)
        self.assertIn("Simple_Warehouse", self.cfg_source)
        self.assertIn("2-StoryStaircase.usd", self.cfg_source)
        self.assertIn("scene_04.usd", self.cfg_source)
        self.assertIn("Environments/Office/office.usd", self.cfg_source)
        self.assertIn("ground = None", self.cfg_source)
        self.assertNotIn("GroundPlaneCfg", self.cfg_source)

    def test_scene_has_one_robot_and_no_background_props(self):
        scene_class = next(
            node
            for node in self.cfg_tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "G129SonicOpenSpaceSceneCfg"
        )
        assignments = {
            target.id
            for node in scene_class.body
            if isinstance(node, ast.AnnAssign)
            and isinstance((target := node.target), ast.Name)
        }
        self.assertEqual(assignments & {"robot", "robot_2"}, {"robot"})
        self.assertNotIn("robot_2", assignments)
        self.assertNotIn("packing_table", assignments)
        self.assertNotIn("background", assignments)

    def test_task_is_registered_and_routed_as_single_robot_sonic(self):
        task_ids = (
            "Isaac-G1-29DoF-Sonic-OpenSpace",
            "Isaac-G1-29DoF-Sonic-Grass",
            "Isaac-G1-29DoF-Sonic-Gravel",
            "Isaac-G1-29DoF-Sonic-Mud",
            "Isaac-G1-29DoF-Sonic-Slate",
            "Isaac-G1-29DoF-Sonic-Snow",
            "Isaac-G1-29DoF-Sonic-Warehouse",
            "Isaac-G1-29DoF-Sonic-Apartment",
            "Isaac-G1-29DoF-Sonic-Staircase",
            "Isaac-G1-29DoF-Sonic-Office",
        )
        init_source = INIT_PATH.read_text(encoding="utf-8")
        for task_id in task_ids:
            self.assertIn(task_id, init_source)
        self.assertIn("g1_29dof_sonic_open_space", TASKS_INIT_PATH.read_text(encoding="utf-8"))
        sim_source = SIM_MAIN_PATH.read_text(encoding="utf-8")
        for task_id in task_ids:
            self.assertIn(task_id, sim_source)
            self.assertIn(task_id, sim_source[sim_source.index("sonic_dex3_task_names") :])


if __name__ == "__main__":
    unittest.main()
