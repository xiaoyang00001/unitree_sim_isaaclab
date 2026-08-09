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
        self.assertEqual(layout.robot_workstation_y, 14.148)
        self.assertEqual(layout.conveyor_y_stop, 14.148)

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


class BeltBoxLayoutTest(unittest.TestCase):
    """纸箱队列只排在工位上游那一段带面上（"流水线前段"）。"""

    def test_default_conveyor_layout_spawns_five_boxes_upstream(self) -> None:
        layout = resolve_scene_layout({})

        self.assertEqual(layout.belt_box_count, 5)
        self.assertEqual(
            layout.belt_box_names,
            ("belt_box_1", "belt_box_2", "belt_box_3", "belt_box_4", "belt_box_5"),
        )
        self.assertEqual(layout.belt_box_queue_pitch, 0.45)
        self.assertEqual(
            layout.belt_box_positions,
            (
                (-5.62, 15.6, 0.775),
                (-5.62, 16.2, 0.775),
                (-5.62, 16.8, 0.775),
                (-5.62, 17.4, 0.775),
                (-5.62, 18.0, 0.775),
            ),
        )

    def test_every_box_sits_fully_upstream_of_the_workstation_and_on_the_belt(self) -> None:
        layout = resolve_scene_layout({})
        half_len = _LAYOUT_MODULE.BELT_BOX_LENGTH_Y * 0.5

        for name, (_x, y, _z) in zip(layout.belt_box_names, layout.belt_box_positions):
            with self.subTest(name):
                self.assertGreater(y - half_len, layout.conveyor_y_stop, "压到工位下游了")
                self.assertLessEqual(y + half_len, _LAYOUT_MODULE.BELT_BOX_BELT_Y_MAX)

    def test_boxes_are_ordered_lead_first_and_never_overlap(self) -> None:
        """belt_box_1 是队首（y 最小、最先到工位），相邻箱子不得重叠。"""

        layout = resolve_scene_layout({})
        ys = [pos[1] for pos in layout.belt_box_positions]

        self.assertEqual(ys, sorted(ys))
        for lead, follow in zip(ys, ys[1:]):
            self.assertGreaterEqual(follow - lead, _LAYOUT_MODULE.BELT_BOX_LENGTH_Y)

    def test_settled_queue_still_fits_on_the_belt(self) -> None:
        """停稳后队列占据 y_stop + k*queue_pitch，最后一个不能悬出带尾。"""

        layout = resolve_scene_layout({})
        tail = layout.conveyor_y_stop + layout.belt_box_queue_pitch * (layout.belt_box_count - 1)

        self.assertLessEqual(
            tail + _LAYOUT_MODULE.BELT_BOX_LENGTH_Y * 0.5,
            _LAYOUT_MODULE.BELT_BOX_BELT_Y_MAX,
        )

    def test_count_and_geometry_are_overridable(self) -> None:
        layout = resolve_scene_layout(
            {
                "ISAACLAB_BELT_BOX_COUNT": "3",
                "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "16.0",
                "ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.7",
                "ISAACLAB_BELT_BOX_QUEUE_PITCH": "0.5",
                "ISAACLAB_BELT_BOX_LANE_X": "-5.617",
            }
        )

        self.assertEqual(layout.belt_box_count, 3)
        self.assertEqual(layout.belt_box_queue_pitch, 0.5)
        self.assertEqual(
            layout.belt_box_positions,
            ((-5.617, 16.0, 0.775), (-5.617, 16.7, 0.775), (-5.617, 17.4, 0.775)),
        )

    def test_zero_count_yields_no_boxes(self) -> None:
        layout = resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "0"})

        self.assertEqual(layout.belt_box_count, 0)
        self.assertEqual(layout.belt_box_positions, ())
        self.assertEqual(layout.belt_box_names, ())

    def test_pushcart_layout_spawns_no_belt_boxes(self) -> None:
        """推车布局的作业闭环仍是两塑料筐。"""

        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})

        self.assertEqual(layout.belt_box_count, 0)
        self.assertEqual(layout.belt_box_names, ())

    def test_count_above_the_scene_cfg_limit_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "超过上限"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "6"})

    def test_negative_count_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "必须非负"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "-1"})

    def test_queue_pitch_below_box_length_fails_fast(self) -> None:
        """排队间距小于箱长 = 队列自穿模，必须启动时就拦住。"""

        with self.assertRaisesRegex(ValueError, "不能小于箱长"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_QUEUE_PITCH": "0.2"})

    def test_spawn_pitch_below_queue_pitch_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "不能小于排队间距"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.3"})

    def test_lead_box_on_the_workstation_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "队首压在工位"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "14.2"})

    def test_queue_running_off_the_belt_tail_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "队尾悬出带面"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "17.0"})

    def test_lead_bound_follows_a_custom_workstation(self) -> None:
        """y_stop 被覆盖时，队首下界跟着走——不能拿默认 14.148 硬判。"""

        with self.assertRaisesRegex(ValueError, "队首压在工位"):
            resolve_scene_layout(
                {
                    "ISAACLAB_CONVEYOR_Y_STOP": "16.0",
                    "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "15.6",
                }
            )


if __name__ == "__main__":
    unittest.main()
