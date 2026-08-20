import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/robot_visibility.py"
SPEC = importlib.util.spec_from_file_location("robot_visibility_test_module", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ConveyorRobotVisibilityTest(unittest.TestCase):
    def test_default_shows_global_robot_one_only(self) -> None:
        self.assertEqual(
            MODULE.resolve_visible_robot_names({}),
            frozenset({"robot_1"}),
        )

    def test_all_and_none_modes_keep_global_names(self) -> None:
        self.assertEqual(
            MODULE.resolve_visible_robot_names(
                {MODULE.VISIBLE_ROBOTS_ENV: "all"}
            ),
            frozenset(MODULE.ALL_ROBOT_NAMES),
        )
        self.assertEqual(
            MODULE.resolve_visible_robot_names(
                {MODULE.VISIBLE_ROBOTS_ENV: "none"}
            ),
            frozenset(),
        )

    def test_viewer_and_peer_aliases_are_identity_based(self) -> None:
        # The scene factories map PeerRobot/Robot to these global names.  The
        # policy itself must remain independent of the local process identity.
        self.assertEqual(
            MODULE.resolve_visible_robot_names(
                {MODULE.VISIBLE_ROBOTS_ENV: "r1,standby2"}
            ),
            frozenset({"robot_1", "standby_robot_2"}),
        )

    def test_unknown_tokens_fall_back_safely(self) -> None:
        self.assertEqual(
            MODULE.resolve_visible_robot_names(
                {MODULE.VISIBLE_ROBOTS_ENV: "not-a-robot"}
            ),
            frozenset({"robot_1"}),
        )


if __name__ == "__main__":
    unittest.main()
