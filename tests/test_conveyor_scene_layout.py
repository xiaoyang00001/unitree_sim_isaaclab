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
CONVEYOR_NORTH_SHIFT_Y = _LAYOUT_MODULE.CONVEYOR_NORTH_SHIFT_Y

_DRIVE_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_drive.py"
)
_DRIVE_SPEC = importlib.util.spec_from_file_location(
    "conveyor_drive_for_layout_test", _DRIVE_MODULE_PATH
)
assert _DRIVE_SPEC is not None and _DRIVE_SPEC.loader is not None
_DRIVE_MODULE = importlib.util.module_from_spec(_DRIVE_SPEC)
sys.modules[_DRIVE_SPEC.name] = _DRIVE_MODULE
_DRIVE_SPEC.loader.exec_module(_DRIVE_MODULE)


class ConveyorSceneLayoutTest(unittest.TestCase):
    def test_conveyor_layout_is_the_default(self) -> None:
        layout = resolve_scene_layout({})

        self.assertTrue(layout.totes_on_conveyor)
        # 全部含流水线整体北移 Δ=3.55（19.39363/17.4/18.0 + Δ）。
        self.assertEqual(layout.pushcart_2_pos, (-5.4, 22.94363, 0.0))
        self.assertEqual(layout.cart2_tote1_pos, (-5.35, 20.95, 0.775))
        self.assertEqual(layout.cart2_tote2_pos, (-5.89, 21.55, 0.775))
        self.assertEqual(layout.tote_scale, (0.005, 0.005, 0.005))
        self.assertEqual(layout.robot_1_x, -4.75)
        self.assertEqual(layout.robot_2_x, -6.7)
        self.assertEqual(layout.robot_workstation_y, 17.698)
        self.assertEqual(layout.conveyor_y_stop, 17.698)

    def test_zero_selects_stacked_full_size_totes_on_pushcart(self) -> None:
        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})

        self.assertFalse(layout.totes_on_conveyor)
        # =0 的作业组同样整体 +Δ：18.75 → 22.30，z 叠放高度不变。
        self.assertEqual(layout.pushcart_2_pos, (-5.62, 22.3, 0.0))
        self.assertEqual(layout.cart2_tote1_pos, (-5.62, 22.3, 0.3794))
        self.assertEqual(layout.cart2_tote2_pos, (-5.62, 22.3, 0.6814))
        self.assertEqual(layout.tote_scale, (0.01, 0.01, 0.01))
        self.assertEqual(layout.robot_1_x, -4.82)
        self.assertEqual(layout.robot_2_x, -6.42)
        self.assertEqual(layout.robot_workstation_y, 22.3)
        self.assertEqual(layout.conveyor_y_stop, 15.05)

    def test_existing_position_overrides_are_preserved(self) -> None:
        layout = resolve_scene_layout(
            {
                "ISAACLAB_TOTES_ON_CONVEYOR": "false",
                "ISAACLAB_CART_GROUP_X": "-5.5",
                "ISAACLAB_CART_GROUP_Y": "20.9",
                "ISAACLAB_ROBOT_SIDE_OFFSET": "0.7",
                "ISAACLAB_ROBOT_2_X": "-6.9",
                "ISAACLAB_CONVEYOR_Y_STOP": "16.25",
            }
        )

        self.assertEqual(layout.pushcart_2_pos, (-5.5, 20.9, 0.0))
        self.assertEqual(layout.robot_1_x, -4.8)
        self.assertEqual(layout.robot_2_x, -6.9)
        self.assertEqual(layout.robot_workstation_y, 20.9)
        self.assertEqual(layout.conveyor_y_stop, 16.25)


class ConveyorLayoutNorthShiftTest(unittest.TestCase):
    """北移量在 scene_layout / conveyor_drive 两处副本必须一致。"""

    def test_shift_constant_matches_conveyor_drive(self) -> None:
        self.assertEqual(CONVEYOR_NORTH_SHIFT_Y, _DRIVE_MODULE.CONVEYOR_NORTH_SHIFT_Y)

    def test_conveyor_layout_spawn_points_stay_on_the_shifted_belt(self) -> None:
        layout = resolve_scene_layout({})
        belt_max = _DRIVE_MODULE.BELT_Y_MAX
        belt_min = _DRIVE_MODULE.BELT_Y_MIN

        # 后筐后缘（半长 0.10）不能悬出带尾；工位仍在带上。
        self.assertLess(layout.cart2_tote2_pos[1] + 0.10, belt_max)
        self.assertGreater(layout.cart2_tote1_pos[1], layout.robot_workstation_y)
        self.assertGreater(layout.robot_workstation_y, belt_min)
        # 行程长度不随整体平移改变。
        self.assertAlmostEqual(
            layout.cart2_tote1_pos[1] - layout.robot_workstation_y, 3.252, places=6
        )
        self.assertAlmostEqual(
            layout.cart2_tote2_pos[1] - layout.robot_workstation_y, 3.852, places=6
        )

    def test_pushcart_layout_group_stays_between_the_belt_end_and_the_wall(self) -> None:
        """=0 作业组北移后仍夹在带端与 +Y 墙之间，不必按布局分叉 Δ。"""

        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})
        belt_max = _DRIVE_MODULE.BELT_Y_MAX  # 21.77
        wall_face_y = 23.606
        cart_half_len_y = 0.412  # pushcart spawn scale 0.5 后实测 y 跨度 0.824

        cart_south = layout.cart_group_y - cart_half_len_y
        cart_north = layout.cart_group_y + cart_half_len_y
        self.assertGreater(cart_south, belt_max)          # 不压到带面
        self.assertLess(cart_north, wall_face_y - 0.20)   # 离墙留余量
        self.assertAlmostEqual(cart_south - belt_max, 0.118, places=3)
        self.assertAlmostEqual(wall_face_y - cart_north, 0.894, places=3)
        # 出料段停止点仍落在带上（handoff 0.20）。
        self.assertLess(_DRIVE_MODULE.BELT_Y_MIN, layout.conveyor_y_stop + 0.20)
        self.assertLess(layout.conveyor_y_stop + 0.20, belt_max)


if __name__ == "__main__":
    unittest.main()
