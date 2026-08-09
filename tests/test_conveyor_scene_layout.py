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

_EI_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/endless_intake.py"
)
_EI_SPEC = importlib.util.spec_from_file_location(
    "endless_intake_for_layout_test", _EI_MODULE_PATH
)
assert _EI_SPEC is not None and _EI_SPEC.loader is not None
_EI_MODULE = importlib.util.module_from_spec(_EI_SPEC)
sys.modules[_EI_SPEC.name] = _EI_MODULE
_EI_SPEC.loader.exec_module(_EI_MODULE)

# 弯道形态（默认）下的关键路径量：与 endless_intake 交叉推导。
_S_STOP_DEFAULT = _EI_MODULE.path_s_of_main_y(14.148)
_S_LEAD_DEFAULT = _LAYOUT_MODULE.BELT_BOX_DEFAULT_S_LEAD
_ENDLESS_OFF = {"ISAACLAB_CONVEYOR_ENDLESS": "off"}


class ConveyorSceneLayoutTest(unittest.TestCase):
    def test_conveyor_layout_is_the_default(self) -> None:
        layout = resolve_scene_layout({})

        self.assertTrue(layout.totes_on_conveyor)
        self.assertEqual(layout.pushcart_2_pos, (-5.4, 19.39363, 0.0))
        self.assertEqual(layout.cart2_tote1_pos, (-5.35, 17.4, 0.775))
        self.assertEqual(layout.cart2_tote2_pos, (-5.89, 18.0, 0.775))
        self.assertEqual(layout.tote_scale, (0.005, 0.005, 0.005))
        # 两台对称分站带两侧（中线 -5.62 ± 1.08）；robot_1 历史值 -4.75 比对面近
        # 0.21，一台贴带一台离远，2026-08-09 对称化。
        self.assertEqual(layout.robot_1_x, -4.54)
        self.assertAlmostEqual(
            abs(layout.robot_1_x - (-5.62)), abs(layout.robot_2_x - (-5.62)), places=6
        )
        self.assertEqual(layout.robot_2_x, -6.7)
        self.assertEqual(layout.robot_workstation_y, 14.148)
        self.assertEqual(layout.conveyor_y_stop, 14.148)

    def test_zero_selects_stacked_full_size_totes_on_pushcart(self) -> None:
        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})

        self.assertFalse(layout.totes_on_conveyor)
        # =0 的作业组回 Δ=0 基线 (-5.62, 18.75)，z 叠放高度不变。
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
        """=0 作业组夹在带端与 +Y 墙之间（Δ=0 基线）。"""

        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})
        belt_max = _DRIVE_MODULE.BELT_Y_MAX  # 18.22
        wall_face_y = 23.606
        cart_half_len_y = 0.412  # pushcart spawn scale 0.5 后实测 y 跨度 0.824

        cart_south = layout.cart_group_y - cart_half_len_y
        cart_north = layout.cart_group_y + cart_half_len_y
        self.assertGreater(cart_south, belt_max)          # 不压到带面
        self.assertLess(cart_north, wall_face_y - 0.20)   # 离墙留余量
        self.assertAlmostEqual(cart_south - belt_max, 0.118, places=3)
        self.assertAlmostEqual(wall_face_y - cart_north, 4.444, places=3)
        # 出料段停止点仍落在带上（handoff 0.20）。
        self.assertLess(_DRIVE_MODULE.BELT_Y_MIN, layout.conveyor_y_stop + 0.20)
        self.assertLess(layout.conveyor_y_stop + 0.20, belt_max)


class BeltBoxLayoutTest(unittest.TestCase):
    """纸箱队列默认沿西拐入口弯道路径排（队尾伸上 X 支线），两种箱型交错。"""

    def test_default_conveyor_layout_spawns_five_boxes_along_the_path(self) -> None:
        layout = resolve_scene_layout({})

        self.assertEqual(layout.belt_box_count, 5)
        self.assertEqual(
            layout.belt_box_names,
            ("belt_box_1", "belt_box_2", "belt_box_3", "belt_box_4", "belt_box_5"),
        )
        # 默认 pattern d01,parcel_a02 循环 → 小纸箱与白色软包裹交错。
        self.assertEqual(
            [kind.key for kind in layout.belt_box_kinds],
            ["d01", "parcel_a02", "d01", "parcel_a02", "d01"],
        )
        self.assertEqual(layout.belt_box_queue_gap, 0.07)
        # 出生点 = 沿路径 s_lead − k·pitch 的路径点（与 endless_intake 交叉核对）。
        for index, (x, y, z) in enumerate(layout.belt_box_positions):
            expected = _EI_MODULE.path_point(_S_LEAD_DEFAULT - 0.75 * index)
            with self.subTest(index=index):
                self.assertAlmostEqual(x, expected[0], places=9)
                self.assertAlmostEqual(y, expected[1], places=9)
                self.assertEqual(z, 0.775)
        # 队首刚拐出弯 0.121 m、整箱上主线；队尾伸上 X 支线（西侧无全遮窗，
        # 出生可见是 README 已知限制，弧上 3 箱是刻意的"绕弯而来"揭示点）。
        self.assertAlmostEqual(layout.belt_box_positions[0][0], -5.62, places=9)
        self.assertAlmostEqual(layout.belt_box_positions[0][1], 18.2825, places=3)
        self.assertAlmostEqual(layout.belt_box_positions[4][0], -7.70, places=6)
        self.assertAlmostEqual(
            layout.belt_box_positions[4][1], _EI_MODULE.BRANCH_LANE_Y, places=9
        )
        self.assertEqual(layout.belt_box_half_lengths, (0.19, 0.2263, 0.19, 0.2263, 0.19))

    def test_endless_off_falls_back_to_the_straight_head(self) -> None:
        """三态开关 off / legacy_props 回退：沿主车道直排（历史行为）。"""

        for environ in (_ENDLESS_OFF, {"ISAACLAB_CONVEYOR_PROPS": "legacy_props"}):
            with self.subTest(environ=environ):
                layout = resolve_scene_layout(environ)
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

    def test_two_cardbox_rollback_pattern_still_works(self) -> None:
        """软包裹只是默认 pattern 的替换项：d01,c01 双纸箱形态必须随时能切回。"""

        layout = resolve_scene_layout({"ISAACLAB_BELT_BOX_PATTERN": "d01,c01"})
        self.assertEqual(
            [k.key for k in layout.belt_box_kinds], ["d01", "c01", "d01", "c01", "d01"]
        )
        self.assertEqual(layout.belt_box_half_lengths, (0.19, 0.25, 0.19, 0.25, 0.19))

    def test_all_three_parcel_variants_are_registered(self) -> None:
        layout = resolve_scene_layout(
            {"ISAACLAB_BELT_BOX_PATTERN": "parcel_a01,parcel_a02,parcel_a03"}
        )
        self.assertEqual(
            [k.key for k in layout.belt_box_kinds],
            ["parcel_a01", "parcel_a02", "parcel_a03", "parcel_a01", "parcel_a02"],
        )

    def test_unknown_pattern_key_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "未知箱型"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_PATTERN": "d01,nope"})

    def test_every_box_sits_fully_upstream_of_the_workstation_and_on_the_path(self) -> None:
        layout = resolve_scene_layout({})

        for name, kind, (x, y, _z) in zip(
            layout.belt_box_names, layout.belt_box_kinds, layout.belt_box_positions
        ):
            with self.subTest(name):
                half = kind.half_length_y
                s = _EI_MODULE.path_s_of_point(x, y)
                self.assertLess(s + half, _S_STOP_DEFAULT, "压到工位下游了")
                self.assertGreaterEqual(s - half, _EI_MODULE.PATH_S_MIN)

    def test_boxes_are_ordered_lead_first_and_never_overlap_at_spawn(self) -> None:
        """belt_box_1 是队首（s 最大、最先到工位），相邻箱子出生就不得互穿。"""

        layout = resolve_scene_layout({})
        ss = [_EI_MODULE.path_s_of_point(x, y) for x, y, _z in layout.belt_box_positions]
        halves = layout.belt_box_half_lengths

        self.assertEqual(ss, sorted(ss, reverse=True))
        for index in range(len(ss) - 1):
            gap = (ss[index] - halves[index]) - (ss[index + 1] + halves[index + 1])
            self.assertGreaterEqual(gap, 0.0, f"box {index} 与 {index + 1} 出生就互穿")

    def test_settled_queue_still_fits_on_the_path(self) -> None:
        """整带节拍下队列保持出生间距整体平移，停稳队尾必须仍在路径带面内。

        停稳位一定比出生位更靠下游（队首从 s_lead 走到更大的 s_stop），所以这条
        由出生位校验蕴含；这里显式钉住，防止以后换回积放语义时无声失守。
        """

        layout = resolve_scene_layout({})
        halves = layout.belt_box_half_lengths
        settled_tail_s = _S_STOP_DEFAULT - 0.75 * (layout.belt_box_count - 1)

        # 停稳队尾在主线段上（早已拐出支线），且不越出带面南端。
        self.assertGreater(settled_tail_s, _EI_MODULE.S_ARC_END)
        _x, tail_y = _EI_MODULE.path_point(settled_tail_s)
        self.assertGreater(tail_y - halves[-1], _DRIVE_MODULE.BELT_Y_MIN)
        self.assertGreater(
            settled_tail_s, _EI_MODULE.path_s_of_point(*layout.belt_box_positions[4][:2])
        )

    def test_count_and_geometry_are_overridable(self) -> None:
        """Y_LEAD 仍可用（主线段 y 自动换算成 s）；弯道形态下 LANE_X 不生效。"""

        layout = resolve_scene_layout(
            {
                "ISAACLAB_BELT_BOX_COUNT": "3",
                "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "16.0",
                "ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.7",
                "ISAACLAB_BELT_BOX_QUEUE_GAP": "0.1",
            }
        )

        self.assertEqual(layout.belt_box_count, 3)
        self.assertEqual(layout.belt_box_queue_gap, 0.1)
        # 队首在主线 y=16.0，向上游按路径距离 0.7 排——三个都还在主线段上。
        for index, (x, y, _z) in enumerate(layout.belt_box_positions):
            with self.subTest(index=index):
                self.assertAlmostEqual(x, -5.62, places=9)
                self.assertAlmostEqual(y, 16.0 + 0.7 * index, places=6)

    def test_s_lead_override_wins_and_reaches_the_branch(self) -> None:
        layout = resolve_scene_layout(
            {
                "ISAACLAB_BELT_BOX_COUNT": "2",
                "ISAACLAB_BELT_BOX_SPAWN_S_LEAD": "5.06",
                "ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.75",
                # S_LEAD 显式给出时 Y_LEAD 被忽略。
                "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "14.95",
            }
        )
        for index, s in ((0, 5.06), (1, 4.31)):
            expected = _EI_MODULE.path_point(s)
            with self.subTest(index=index):
                self.assertAlmostEqual(layout.belt_box_positions[index][0], expected[0], places=9)
                self.assertAlmostEqual(layout.belt_box_positions[index][1], expected[1], places=9)

    def test_lane_x_override_only_applies_to_the_straight_fallback(self) -> None:
        environ = {"ISAACLAB_BELT_BOX_LANE_X": "-5.617", **_ENDLESS_OFF}
        layout = resolve_scene_layout(environ)
        self.assertTrue(all(pos[0] == -5.617 for pos in layout.belt_box_positions))
        # 弯道形态：x 由路径决定，LANE_X 静默不生效（README 有说明）。
        layout = resolve_scene_layout({"ISAACLAB_BELT_BOX_LANE_X": "-5.617"})
        self.assertAlmostEqual(layout.belt_box_positions[0][0], -5.62, places=9)

    def test_spacing_is_wide_enough_to_read_as_separate_boxes(self) -> None:
        """默认间距要让相邻箱子之间留出肉眼可辨的空隙（不是挨在一起）。"""

        layout = resolve_scene_layout({})
        ss = [_EI_MODULE.path_s_of_point(x, y) for x, y, _z in layout.belt_box_positions]
        halves = layout.belt_box_half_lengths
        for index in range(len(ss) - 1):
            clear = (ss[index] - halves[index]) - (ss[index + 1] + halves[index + 1])
            self.assertGreaterEqual(clear, 0.25, f"box {index}/{index + 1} 挨太近")

    def test_path_constants_match_endless_intake(self) -> None:
        """scene_layout 的路径抄本必须与 endless_intake 真源逐值一致。"""

        self.assertEqual(_LAYOUT_MODULE.BELT_BOX_BRANCH_LANE_Y, _EI_MODULE.BRANCH_LANE_Y)
        self.assertEqual(_LAYOUT_MODULE.BELT_BOX_CORNER_RADIUS, _EI_MODULE.CORNER_RADIUS)
        self.assertEqual(_LAYOUT_MODULE.BELT_BOX_PATH_S_ORIGIN_X, _EI_MODULE.S_ORIGIN_X)
        self.assertEqual(_LAYOUT_MODULE.BELT_BOX_PATH_S_MIN, _EI_MODULE.PATH_S_MIN)
        self.assertEqual(_LAYOUT_MODULE.BELT_BOX_LANE_X, _EI_MODULE.MAIN_LANE_X)

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

        with self.assertRaisesRegex(ValueError, "放不下相邻的 d01/parcel_a02"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.3"})

        # 全 d01 时 0.40 够（0.19+0.19=0.38），但混排时放不下 d01/parcel_a02（要 0.4163）。
        resolve_scene_layout(
            {"ISAACLAB_BELT_BOX_PATTERN": "d01", "ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.40"}
        )
        with self.assertRaisesRegex(ValueError, "放不下相邻的"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.40"})

    def test_lead_box_on_the_workstation_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "压在工位"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "14.2"})
        with self.assertRaisesRegex(ValueError, "压在工位"):
            resolve_scene_layout(
                {"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "14.2", **_ENDLESS_OFF}
            )

    def test_queue_running_off_the_path_start_fails_fast(self) -> None:
        # 弯道形态：队尾越过支线滚筒可用端（s_min=-0.30）。
        with self.assertRaisesRegex(ValueError, "悬出支线带面"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_S_LEAD": "2.5"})
        # 直线回退：队尾悬出带面 y_max（历史判据原样保留）。
        with self.assertRaisesRegex(ValueError, "悬出带面"):
            resolve_scene_layout(
                {"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "17.0", **_ENDLESS_OFF}
            )

    def test_lead_beyond_the_corner_needs_the_s_knob(self) -> None:
        """Y_LEAD 只接受主线段 y；想把队首排上弧段/支线必须换 S_LEAD。"""

        with self.assertRaisesRegex(ValueError, "不在主线段上"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "19.0"})

    def test_workstation_must_stay_on_the_main_segment(self) -> None:
        with self.assertRaisesRegex(ValueError, "拐角切线"):
            resolve_scene_layout({"ISAACLAB_CONVEYOR_Y_STOP": "19.0"})

    def test_enlarging_the_pitch_without_room_fails_fast(self) -> None:
        """间距、行程、数量此消彼长：只调大 pitch 而不让出空间就会被拦住。

        弯道形态下支线多给了 ~4.4 m 排队长度，pitch=2.0 都放得下（这是弯道方案
        的收益之一）；2.1 会顶穿支线可用端。直线回退维持旧的 1.0 拦截。
        """

        resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "2.0"})
        with self.assertRaisesRegex(ValueError, "悬出支线带面"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "2.1"})
        with self.assertRaisesRegex(ValueError, "悬出带面"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "1.0", **_ENDLESS_OFF})

        # 同时把队首往工位方向下调就能放下（Y_LEAD 在主线段上自动换算成 s）。
        layout = resolve_scene_layout(
            {
                "ISAACLAB_BELT_BOX_SPAWN_PITCH": "1.0",
                "ISAACLAB_BELT_BOX_COUNT": "3",
                "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "15.0",
            }
        )
        for index, pos in enumerate(layout.belt_box_positions):
            self.assertAlmostEqual(pos[1], 15.0 + 1.0 * index, places=6)

    def test_lead_bound_follows_a_custom_workstation(self) -> None:
        """y_stop 被覆盖时，队首下界跟着走——不能拿默认 14.148 硬判。"""

        with self.assertRaisesRegex(ValueError, "压在工位"):
            resolve_scene_layout(
                {
                    "ISAACLAB_CONVEYOR_Y_STOP": "16.0",
                    "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "15.6",
                }
            )


if __name__ == "__main__":
    unittest.main()
