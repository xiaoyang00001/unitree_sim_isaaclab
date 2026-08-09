from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


_LAYOUT_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_layout.py"
)
_LAYOUT_SPEC = importlib.util.spec_from_file_location(
    "conveyor_scene_layout_for_test", _LAYOUT_MODULE_PATH
)
assert _LAYOUT_SPEC is not None and _LAYOUT_SPEC.loader is not None
_LAYOUT_MODULE = importlib.util.module_from_spec(_LAYOUT_SPEC)
sys.modules[_LAYOUT_SPEC.name] = _LAYOUT_MODULE
_LAYOUT_SPEC.loader.exec_module(_LAYOUT_MODULE)
resolve_scene_layout = _LAYOUT_MODULE.resolve_scene_layout


class ConveyorSceneLayoutTest(unittest.TestCase):
    def test_conveyor_layout_is_the_default(self) -> None:
        layout = resolve_scene_layout({})

        self.assertTrue(layout.totes_on_conveyor)
        self.assertEqual(layout.pushcart_2_pos, (-5.4, 19.39363, 0.0))
        self.assertEqual(layout.cart2_tote1_pos, (-5.35, 17.4, 0.775))
        self.assertEqual(layout.cart2_tote2_pos, (-5.89, 18.0, 0.775))
        self.assertEqual(layout.tote_scale, (0.005, 0.005, 0.005))
        self.assertEqual(layout.robot_1_x, -4.75)
        self.assertEqual(layout.robot_2_x, -6.7)
        # 13.548 = 14.148 - 0.600，补偿 v61 换版时工位家具整体 -Y 平移；
        # 不补偿的话两台机器人开局插进 blue_sorting_bin。
        self.assertEqual(layout.robot_workstation_y, 13.548)
        self.assertEqual(layout.conveyor_y_stop, 13.548)

    def test_zero_selects_stacked_full_size_totes_on_pushcart(self) -> None:
        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})

        self.assertFalse(layout.totes_on_conveyor)
        self.assertEqual(layout.pushcart_2_pos, (-5.62, 18.75, 0.0))
        self.assertEqual(layout.cart2_tote1_pos, (-5.62, 18.75, 0.3794))
        self.assertEqual(layout.cart2_tote2_pos, (-5.62, 18.75, 0.6814))
        self.assertEqual(layout.tote_scale, (0.01, 0.01, 0.01))
        self.assertEqual(layout.robot_1_x, -4.82)
        self.assertEqual(layout.robot_2_x, -6.42)
        self.assertEqual(layout.robot_workstation_y, 18.75)
        self.assertEqual(layout.conveyor_y_stop, 11.5)

    def test_existing_position_overrides_are_preserved(self) -> None:
        layout = resolve_scene_layout(
            {
                "ISAACLAB_TOTES_ON_CONVEYOR": "false",
                "ISAACLAB_CART_GROUP_X": "-5.5",
                "ISAACLAB_CART_GROUP_Y": "18.9",
                "ISAACLAB_ROBOT_SIDE_OFFSET": "0.7",
                "ISAACLAB_ROBOT_2_X": "-6.9",
                "ISAACLAB_CONVEYOR_Y_STOP": "12.25",
            }
        )

        self.assertEqual(layout.pushcart_2_pos, (-5.5, 18.9, 0.0))
        self.assertEqual(layout.robot_1_x, -4.8)
        self.assertEqual(layout.robot_2_x, -6.9)
        self.assertEqual(layout.robot_workstation_y, 18.9)
        self.assertEqual(layout.conveyor_y_stop, 12.25)


if __name__ == "__main__":
    unittest.main()
