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
    """纸箱队列只排在工位上游那一段带面上（"流水线前段"），两种箱型交错。"""

    def test_default_conveyor_layout_spawns_five_interleaved_boxes(self) -> None:
        layout = resolve_scene_layout({})

        self.assertEqual(layout.belt_box_count, 5)
        self.assertEqual(
            layout.belt_box_names,
            ("belt_box_1", "belt_box_2", "belt_box_3", "belt_box_4", "belt_box_5"),
        )
        # 默认 pattern d01,c01 循环 → 交错排布。
        self.assertEqual(
            [kind.key for kind in layout.belt_box_kinds],
            ["d01", "c01", "d01", "c01", "d01"],
        )
        self.assertEqual(layout.belt_box_queue_gap, 0.07)
        self.assertEqual(
            layout.belt_box_positions,
            (
                (-5.62, 14.95, 0.775),
                (-5.62, 15.70, 0.775),
                (-5.62, 16.45, 0.775),
                (-5.62, 17.20, 0.775),
                (-5.62, 17.95, 0.775),
            ),
        )
        self.assertEqual(layout.belt_box_half_lengths, (0.19, 0.25, 0.19, 0.25, 0.19))

    def test_both_kinds_come_from_the_v61_background_assets(self) -> None:
        """两种箱型就是 v61 背景里 ConveyorBelt_Box / KLT_Bin 引的那两个视觉资产。"""

        kinds = _LAYOUT_MODULE.BELT_BOX_KINDS
        self.assertEqual(kinds["d01"].asset, "cart_box_d01_physics.usda")
        self.assertEqual(kinds["c01"].asset, "cart_box_c01_physics.usda")
        self.assertEqual(
            (kinds["d01"].length_y, kinds["d01"].width_x, kinds["d01"].height_z),
            (0.38, 0.25, 0.1487),
        )
        self.assertEqual(
            (kinds["c01"].length_y, kinds["c01"].width_x, kinds["c01"].height_z),
            (0.50, 0.50, 0.25),
        )

    def test_pattern_is_configurable_and_cycles(self) -> None:
        single = resolve_scene_layout({"ISAACLAB_BELT_BOX_PATTERN": "c01"})
        self.assertEqual([k.key for k in single.belt_box_kinds], ["c01"] * 5)

        flipped = resolve_scene_layout({"ISAACLAB_BELT_BOX_PATTERN": " C01 , D01 "})
        self.assertEqual(
            [k.key for k in flipped.belt_box_kinds], ["c01", "d01", "c01", "d01", "c01"]
        )

    def test_unknown_pattern_key_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "未知箱型"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_PATTERN": "d01,nope"})

    def test_every_box_sits_fully_upstream_of_the_workstation_and_on_the_belt(self) -> None:
        layout = resolve_scene_layout({})

        for name, kind, (_x, y, _z) in zip(
            layout.belt_box_names, layout.belt_box_kinds, layout.belt_box_positions
        ):
            with self.subTest(name):
                half = kind.half_length_y
                self.assertGreater(y - half, layout.conveyor_y_stop, "压到工位下游了")
                self.assertLessEqual(y + half, _LAYOUT_MODULE.BELT_BOX_BELT_Y_MAX)

    def test_boxes_are_ordered_lead_first_and_never_overlap_at_spawn(self) -> None:
        """belt_box_1 是队首（y 最小、最先到工位），相邻箱子出生就不得互穿。"""

        layout = resolve_scene_layout({})
        ys = [pos[1] for pos in layout.belt_box_positions]
        halves = layout.belt_box_half_lengths

        self.assertEqual(ys, sorted(ys))
        for index in range(len(ys) - 1):
            gap = (ys[index + 1] - halves[index + 1]) - (ys[index] + halves[index])
            self.assertGreaterEqual(gap, 0.0, f"box {index} 与 {index + 1} 出生就互穿")

    def test_settled_queue_still_fits_on_the_belt(self) -> None:
        """整带节拍下队列保持出生间距整体平移，停稳队尾必须仍在带面内。

        停稳位一定比出生位更靠下游（队首从 y_lead 走到更小的 y_stop），所以这条
        由出生位校验蕴含；这里显式钉住，防止以后换回积放语义时无声失守。
        """

        layout = resolve_scene_layout({})
        halves = layout.belt_box_half_lengths
        ys = [pos[1] for pos in layout.belt_box_positions]
        pitch = ys[1] - ys[0]
        settled_tail = layout.conveyor_y_stop + pitch * (layout.belt_box_count - 1)

        self.assertLessEqual(settled_tail + halves[-1], _LAYOUT_MODULE.BELT_BOX_BELT_Y_MAX)
        self.assertLess(settled_tail, ys[-1], "停稳队尾应比出生队尾更靠下游")

    def test_count_and_geometry_are_overridable(self) -> None:
        layout = resolve_scene_layout(
            {
                "ISAACLAB_BELT_BOX_COUNT": "3",
                "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "16.0",
                "ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.7",
                "ISAACLAB_BELT_BOX_QUEUE_GAP": "0.1",
                "ISAACLAB_BELT_BOX_LANE_X": "-5.617",
            }
        )

        self.assertEqual(layout.belt_box_count, 3)
        self.assertEqual(layout.belt_box_queue_gap, 0.1)
        self.assertEqual(
            layout.belt_box_positions,
            ((-5.617, 16.0, 0.775), (-5.617, 16.7, 0.775), (-5.617, 17.4, 0.775)),
        )

    def test_spacing_is_wide_enough_to_read_as_separate_boxes(self) -> None:
        """默认间距要让相邻箱子之间留出肉眼可辨的空隙（不是挨在一起）。"""

        layout = resolve_scene_layout({})
        ys = [pos[1] for pos in layout.belt_box_positions]
        halves = layout.belt_box_half_lengths
        for index in range(len(ys) - 1):
            clear = (ys[index + 1] - halves[index + 1]) - (ys[index] + halves[index])
            self.assertGreaterEqual(clear, 0.25, f"box {index}/{index + 1} 挨太近")

    def test_zero_count_yields_no_boxes(self) -> None:
        layout = resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "0"})

        self.assertEqual(layout.belt_box_count, 0)
        self.assertEqual(layout.belt_box_positions, ())
        self.assertEqual(layout.belt_box_kinds, ())
        self.assertEqual(layout.belt_box_names, ())

    def test_pushcart_layout_spawns_no_belt_boxes(self) -> None:
        """推车布局的作业闭环仍是两塑料筐。"""

        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})

        self.assertEqual(layout.belt_box_count, 0)
        self.assertEqual(layout.belt_box_names, ())
        self.assertEqual(layout.belt_box_kinds, ())

    def test_count_above_the_scene_cfg_limit_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "超过上限"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "6"})

    def test_negative_count_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "必须非负"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "-1"})

    def test_negative_queue_gap_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "不能为负"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_QUEUE_GAP": "-0.01"})

    def test_spawn_pitch_too_small_for_the_actual_pair_fails_fast(self) -> None:
        """出生间距按**相邻两箱各自的半长**校验，而不是一个统一箱长。"""

        with self.assertRaisesRegex(ValueError, "放不下相邻的 d01/c01"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.3"})

        # 全 d01 时 0.40 够（0.19+0.19=0.38），但交错时放不下 d01/c01（要 0.44）。
        resolve_scene_layout(
            {"ISAACLAB_BELT_BOX_PATTERN": "d01", "ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.40"}
        )
        with self.assertRaisesRegex(ValueError, "放不下相邻的"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.40"})

    def test_lead_box_on_the_workstation_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "队首压在工位"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "14.2"})

    def test_queue_running_off_the_belt_tail_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "悬出带面"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "17.0"})

    def test_enlarging_the_pitch_without_room_fails_fast(self) -> None:
        """间距、行程、数量此消彼长：只调大 pitch 而不让出行程就会被拦住。"""

        with self.assertRaisesRegex(ValueError, "悬出带面"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "1.0"})

        # 同时把队首往工位方向下调就能放下。
        layout = resolve_scene_layout(
            {
                "ISAACLAB_BELT_BOX_SPAWN_PITCH": "1.0",
                "ISAACLAB_BELT_BOX_COUNT": "3",
                "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "15.0",
            }
        )
        self.assertEqual([pos[1] for pos in layout.belt_box_positions], [15.0, 16.0, 17.0])

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
