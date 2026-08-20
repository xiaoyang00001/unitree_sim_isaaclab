import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CFG_PATH = ROOT / "tasks/g1_tasks/g1_29dof_sonic_cafe/cafe_env_cfg.py"
INIT_PATH = ROOT / "tasks/g1_tasks/g1_29dof_sonic_cafe/__init__.py"
SIM_MAIN_PATH = ROOT / "sim_main.py"


class TestSonicCafeScene(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg_source = CFG_PATH.read_text(encoding="utf-8")
        cls.cfg_tree = ast.parse(cls.cfg_source)

    def test_source_scene_and_positions_are_ported(self):
        self.assertIn('"Locomotion", "KitchenRoom", "KitchenRoom.usd"', self.cfg_source)
        self.assertIn("ROBOT_1_POS = (0.18, -0.42, 0.76)", self.cfg_source)
        self.assertIn("ROBOT_2_POS = (0.26, 0.80, 0.76)", self.cfg_source)
        self.assertIn("CUP_SPAWN_POS = (0.02, 0.04, 0.91)", self.cfg_source)
        self.assertIn("/Objects/Mug/mug.usd", self.cfg_source)

    def test_scene_contains_two_dynamic_sonic_robots(self):
        scene_class = next(
            node
            for node in self.cfg_tree.body
            if isinstance(node, ast.ClassDef) and node.name == "G129DualSonicCafeSceneCfg"
        )
        assignments = {
            target.id
            for node in scene_class.body
            if isinstance(node, ast.AnnAssign) and isinstance((target := node.target), ast.Name)
        }
        self.assertTrue({"robot", "robot_2", "foot_contact", "foot_contact_2"} <= assignments)

    def test_host_action_term_order_matches_existing_provider(self):
        action_class = next(
            node
            for node in self.cfg_tree.body
            if isinstance(node, ast.ClassDef) and node.name == "DualSonicActionsCfg"
        )
        appended_terms = [
            node.targets[0].id
            for node in action_class.body
            if isinstance(node, ast.Assign) and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
        ]
        self.assertEqual(appended_terms, ["joint_pos_2", "joint_vel_2", "joint_effort_2"])

    def test_task_is_registered_and_routed_as_dual_sonic(self):
        task_id = "Isaac-G1-29DoF-Sonic-Cafe"
        self.assertIn(task_id, INIT_PATH.read_text(encoding="utf-8"))
        sim_source = SIM_MAIN_PATH.read_text(encoding="utf-8")
        self.assertGreaterEqual(sim_source.count(task_id), 3)
        self.assertIn('is_scene_sync_host = args_cli.task == "Isaac-G1-29DoF-Sonic-Cafe"', sim_source)


if __name__ == "__main__":
    unittest.main()
