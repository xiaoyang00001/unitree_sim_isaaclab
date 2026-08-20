import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MULTI_CFG = ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/multi_scene_env_cfg.py"
CONVEYOR_INIT = ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/__init__.py"
SIM_MAIN = ROOT / "sim_main.py"


class ConveyorMultiSceneStaticTest(unittest.TestCase):
    def test_multi_scene_module_is_valid_python_and_has_all_complete_assets(self) -> None:
        source = MULTI_CFG.read_text(encoding="utf-8")
        ast.parse(source)
        for key in ("warehouse", "apartment", "staircase", "office"):
            self.assertIn(f'"{key}": MultiRobotSceneSpec', source)
        self.assertIn("_NON_CONVEYOR_FIELDS", source)
        self.assertIn('"conveyor_collider"', source)
        self.assertIn('*(f"belt_box_{index}"', source)
        self.assertIn("ISAAC_REAL_WAREHOUSE_USD", source)

    def test_conveyor_warehouse_is_room_only(self) -> None:
        source = MULTI_CFG.read_text(encoding="utf-8")
        init_source = CONVEYOR_INIT.read_text(encoding="utf-8")
        self.assertIn("G129SonicMultiWarehouseEnvCfg", init_source)
        self.assertIn('background_prim_name="Warehouse"', source)
        self.assertIn('"warehouse": REAL_WAREHOUSE_USD', source)

    def test_every_robot_role_is_declared_and_not_removed(self) -> None:
        source = MULTI_CFG.read_text(encoding="utf-8")
        for field in (
            '"robot": local_robot',
            '"peer_robot": peer_robot',
            '"peer_robot_2": peer_robot_2',
            '"robot_2": robot_2',
            '"standby_robot_1":',
            '"standby_robot_2":',
            '"standby_robot_3":',
        ):
            self.assertIn(field, source)
        self.assertIn("ISAACLAB_CONVEYOR_VISIBLE_ROBOTS", (
            ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/robot_visibility.py"
        ).read_text(encoding="utf-8"))

    def test_registry_and_sim_main_route_new_ids_as_conveyor_tasks(self) -> None:
        init_source = CONVEYOR_INIT.read_text(encoding="utf-8")
        sim_source = SIM_MAIN.read_text(encoding="utf-8")
        for suffix in ("Warehouse", "Apartment", "Staircase", "Office"):
            task_id = f"Isaac-G1-29DoF-Sonic-Conveyor-{suffix}"
            self.assertIn(task_id, init_source)
            self.assertIn(task_id, sim_source)
        self.assertIn("if args_cli.task in conveyor_scene_task_names:", sim_source)

    def test_apartment_group_is_not_spawned_inside_the_known_collision_wall(self) -> None:
        source = MULTI_CFG.read_text(encoding="utf-8")
        self.assertIn('"robot_1": (-2.5, 5.5, 0.76)', source)

    def test_staircase_reuses_the_stable_warehouse_formation(self) -> None:
        source = MULTI_CFG.read_text(encoding="utf-8")
        self.assertIn('"robot_1": (0.0, 0.0, 0.76)', source)
        self.assertIn('"robot_2": (0.0, 2.0, 0.76)', source)
        self.assertNotIn('"robot_1": (0.0, -12.0, 0.76)', source)


if __name__ == "__main__":
    unittest.main()
