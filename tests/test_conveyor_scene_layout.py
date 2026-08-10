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
# 工位 y=14.398（=14.148+Δ）；s 参数化对 Δ 平移不变，s_stop 仍 ≈12.1945。
_S_STOP_DEFAULT = _EI_MODULE.path_s_of_main_y(14.398)
_S_LEAD_DEFAULT = _LAYOUT_MODULE.BELT_BOX_DEFAULT_S_LEAD
_SLOTS_DEFAULT = _LAYOUT_MODULE.BELT_BOX_DEFAULT_SLOT_S
_ENDLESS_OFF = {"ISAACLAB_CONVEYOR_ENDLESS": "off"}


class ConveyorSceneLayoutTest(unittest.TestCase):
    def test_conveyor_layout_is_the_default(self) -> None:
        layout = resolve_scene_layout({})

        self.assertTrue(layout.totes_on_conveyor)
        self.assertEqual(layout.pushcart_2_pos, (-5.4, 19.64363, 0.0))
        self.assertEqual(layout.cart2_tote1_pos, (-5.35, 17.65, 0.775))
        self.assertEqual(layout.cart2_tote2_pos, (-5.89, 18.25, 0.775))
        self.assertEqual(layout.tote_scale, (0.005, 0.005, 0.005))
        # 两台对称分站带两侧（中线 -5.62 ± 1.08）；robot_1 历史值 -4.75 比对面近
        # 0.21，一台贴带一台离远，2026-08-09 对称化。
        self.assertEqual(layout.robot_1_x, -4.54)
        self.assertAlmostEqual(
            abs(layout.robot_1_x - (-5.62)), abs(layout.robot_2_x - (-5.62)), places=6
        )
        self.assertEqual(layout.robot_2_x, -6.7)
        self.assertEqual(layout.robot_workstation_y, 14.398)
        self.assertEqual(layout.conveyor_y_stop, 14.398)

    def test_zero_selects_stacked_full_size_totes_on_pushcart(self) -> None:
        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})

        self.assertFalse(layout.totes_on_conveyor)
        # =0 的作业组随 Δ=0.25 到 (-5.62, 19.0)，z 叠放高度不变。
        self.assertEqual(layout.pushcart_2_pos, (-5.62, 19.0, 0.0))
        self.assertEqual(layout.cart2_tote1_pos, (-5.62, 19.0, 0.3794))
        self.assertEqual(layout.cart2_tote2_pos, (-5.62, 19.0, 0.6814))
        self.assertEqual(layout.tote_scale, (0.01, 0.01, 0.01))
        self.assertEqual(layout.robot_1_x, -4.82)
        self.assertEqual(layout.robot_2_x, -6.42)
        self.assertEqual(layout.robot_workstation_y, 19.0)
        self.assertEqual(layout.conveyor_y_stop, 11.75)

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


class StandbyRobotLayoutTest(unittest.TestCase):
    """流水线布局最多增加三台分散、非面对面的纯显示机器人。"""

    def test_conveyor_layout_adds_three_scattered_standby_robots(self) -> None:
        layout = resolve_scene_layout({})

        self.assertLessEqual(len(layout.standby_robot_poses), 3)
        self.assertEqual(
            tuple((pose.pos, pose.rot) for pose in layout.standby_robot_poses),
            (
                (
                    (-12.7, 18.9534, 0.76),
                    (0.40455358, 0.0, 0.0, 0.91451430),
                ),
                ((-6.7, 17.2, 0.76), (1.0, 0.0, 0.0, 0.0)),
                (
                    (-9.5, 21.15, 0.76),
                    (0.70710678, 0.0, 0.0, -0.70710678),
                ),
            ),
        )
        # 三台分处支线队尾、主线上游和支线中段，不在同一横截面相向站立。
        self.assertEqual(len({pose.pos[1] for pose in layout.standby_robot_poses}), 3)
        self.assertEqual(len({pose.rot for pose in layout.standby_robot_poses}), 3)

    def test_pushcart_layout_does_not_add_standby_robots(self) -> None:
        layout = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})

        self.assertEqual(layout.standby_robot_poses, ())

    def test_existing_two_robot_workstations_are_unchanged(self) -> None:
        conveyor = resolve_scene_layout({})
        pushcart = resolve_scene_layout({"ISAACLAB_TOTES_ON_CONVEYOR": "0"})

        self.assertEqual(
            (conveyor.robot_1_x, conveyor.robot_2_x, conveyor.robot_workstation_y),
            (-4.54, -6.7, 14.398),
        )
        self.assertEqual(
            (pushcart.robot_1_x, pushcart.robot_2_x, pushcart.robot_workstation_y),
            (-4.82, -6.42, 19.0),
        )


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
        belt_max = _DRIVE_MODULE.BELT_Y_MAX  # 18.47
        wall_face_y = 23.606
        cart_half_len_y = 0.412  # pushcart spawn scale 0.5 后实测 y 跨度 0.824

        cart_south = layout.cart_group_y - cart_half_len_y
        cart_north = layout.cart_group_y + cart_half_len_y
        self.assertGreater(cart_south, belt_max)          # 不压到带面
        self.assertLess(cart_north, wall_face_y - 0.20)   # 离墙留余量
        self.assertAlmostEqual(cart_south - belt_max, 0.118, places=3)  # Δ 不变量
        self.assertAlmostEqual(wall_face_y - cart_north, 4.194, places=3)
        # 出料段停止点仍落在带上（handoff 0.20）。
        self.assertLess(_DRIVE_MODULE.BELT_Y_MIN, layout.conveyor_y_stop + 0.20)
        self.assertLess(layout.conveyor_y_stop + 0.20, belt_max)


class BeltBoxLayoutTest(unittest.TestCase):
    """纸箱队列默认沿西拐入口弯道路径**铺满**（pitch 0.75 × 17 槽），两种箱型交错。"""

    def test_drop_root_zones_cover_both_robot_sorting_bins(self) -> None:
        """两个接收区分别位于 robot_2/robot_1 身后的蓝色分拣框内部。"""

        self.assertEqual(
            _LAYOUT_MODULE.BELT_BOX_DROP_ROOT_ZONES,
            (
                (-7.068025, -6.465390, 15.099451, 15.995693, 0.42, 0.56),
                (-4.778759, -4.167826, 15.096080, 15.999063, 0.42, 0.56),
            ),
        )
        # 接收区保守沿用 0.25 m 内缩；当前 d01/d02 最大路径半长只有 0.19 m，
        # 所有可选箱型进入该区时，箱底根都不会只擦到框沿。
        for kind in _LAYOUT_MODULE.BELT_BOX_KINDS.values():
            self.assertLessEqual(kind.length_y, 0.50)
            self.assertLessEqual(kind.width_x, 0.50)
            self.assertLessEqual(kind.height_z, 0.25)
            self.assertLessEqual(kind.half_queue_extent, 0.25)

    def test_default_conveyor_layout_fills_seventeen_slots_along_the_path(self) -> None:
        layout = resolve_scene_layout({})

        self.assertEqual(layout.belt_box_count, 17)
        self.assertEqual(
            layout.belt_box_names,
            tuple(f"belt_box_{i + 1}" for i in range(17)),
        )
        # 默认 pattern d01,d02 循环 → 原箱与压皱箱交错（首尾都是 d01）。
        self.assertEqual(
            [kind.key for kind in layout.belt_box_kinds],
            ["d01" if i % 2 == 0 else "d02" for i in range(17)],
        )
        self.assertEqual(layout.belt_box_queue_gap, 0.07)
        # 槽位序列 = 队首 8.06 为锚、pitch 0.75 向上游铺满（最深 -3.94）。
        self.assertEqual(len(_SLOTS_DEFAULT), 17)
        for index, s in enumerate(_SLOTS_DEFAULT):
            self.assertAlmostEqual(s, 8.06 - 0.75 * index, places=9)
        # 出生点 = 显式槽位序列的路径点（与 endless_intake 交叉核对）。
        for index, (x, y, z) in enumerate(layout.belt_box_positions):
            expected = _EI_MODULE.path_point(_SLOTS_DEFAULT[index])
            with self.subTest(index=index):
                self.assertAlmostEqual(x, expected[0], places=9)
                self.assertAlmostEqual(y, expected[1], places=9)
                self.assertEqual(z, 0.775)
        # 队首刚拐出弯 0.121 m、整箱上主线；弧上 3 箱维持"绕弯而来"的揭示点；
        # 其余 13 箱一路排上 X 支线，队尾深藏在货架排 B 后面。
        self.assertAlmostEqual(layout.belt_box_positions[0][0], -5.62, places=9)
        self.assertAlmostEqual(layout.belt_box_positions[0][1], 18.5325, places=3)
        self.assertAlmostEqual(layout.belt_box_positions[16][0], -16.70, places=6)
        self.assertAlmostEqual(
            layout.belt_box_positions[16][1], _EI_MODULE.BRANCH_LANE_Y, places=9
        )
        self.assertEqual(
            layout.belt_box_half_lengths,
            (0.19,) * 17,
        )

    def test_deepest_slot_clears_the_roller_end_and_deepens_the_e2_hide(self) -> None:
        """最深槽位 -3.94 的两条红线：滚筒端净距 ≥50 mm、E2 遮挡只强不弱。

        * 对支线滚筒可用端 PATH_S_MIN=-4.27 的净距按 d01/d02 路径半长 0.19
          算：140 mm ≥ 50 mm；
        * 比旧深藏位 -3.90（=RESPAWN_S，回生点**不动**）更深 ⇒ E2"东上角"全遮
          边界 s≤-3.81 的裕量从 90 mm 加深到 130 mm，结论沿用不需重算。
        """

        deepest = _SLOTS_DEFAULT[-1]
        self.assertAlmostEqual(deepest, -3.94, places=9)
        worst_half = _LAYOUT_MODULE.BELT_BOX_KINDS["d02"].half_queue_extent
        self.assertAlmostEqual(worst_half, 0.19, places=9)
        self.assertGreaterEqual(
            (deepest - worst_half) - _EI_MODULE.PATH_S_MIN, 0.05 - 1e-9
        )
        # 更深于回生点（E2 全遮沿用依据）；回生点自身不随槽位改动。
        self.assertLess(deepest, _EI_MODULE.RESPAWN_S)
        self.assertLessEqual(deepest, -3.81)
        self.assertAlmostEqual(_EI_MODULE.RESPAWN_S, -3.90, places=9)

    def test_count_override_takes_the_prefix_from_the_lead(self) -> None:
        """COUNT<17 取序列前缀（从队首往上游数），不再有队尾深藏特例。"""

        layout = resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "5"})

        self.assertEqual(layout.belt_box_count, 5)
        for index, (x, y, _z) in enumerate(layout.belt_box_positions):
            expected = _EI_MODULE.path_point(8.06 - 0.75 * index)
            with self.subTest(index=index):
                self.assertAlmostEqual(x, expected[0], places=9)
                self.assertAlmostEqual(y, expected[1], places=9)

    def test_loop_mode_wrap_gap_fits_the_full_queue(self) -> None:
        """循环模式复核：17 箱在回生→回收环路上循环不追尾。

        环路等价周长 = s(y_recycle) − RESPAWN_S ≈ 19.64 m；整带同速 ⇒ 出生间距
        永久保持，唯一约束是回绕缺口 = 周长 − 队列跨度（12.0 m）≈ 7.64 m 必须
        ≥ 首尾防撞下限（最坏半长和 + queue_gap）。与 conveyor_env_cfg 的运行时
        fail-fast 同一算式（那边 import Isaac 不可单测，这里锁默认值）。
        """

        layout = resolve_scene_layout({})
        ss = [_EI_MODULE.path_s_of_point(x, y) for x, y, _z in layout.belt_box_positions]
        span = max(ss) - min(ss)
        self.assertAlmostEqual(span, 12.0, places=6)

        s_recycle = _EI_MODULE.path_s_of_main_y(_DRIVE_MODULE.DEFAULT_Y_RECYCLE)
        loop_length = s_recycle - _EI_MODULE.RESPAWN_S
        self.assertAlmostEqual(loop_length, 19.6425, places=3)

        wrap_gap = loop_length - span
        worst_half = max(
            _LAYOUT_MODULE.BELT_BOX_KINDS["d02"].half_queue_extent,
            *layout.belt_box_half_lengths,
        )
        need = 2 * worst_half + layout.belt_box_queue_gap
        # 留 ≥1 m 余量：回收事件 20ms 判定的瞬移落点抖动与弧段拖滑漂移都吃不掉它。
        self.assertGreaterEqual(wrap_gap, need + 1.0)

    def test_endless_off_falls_back_to_the_straight_head(self) -> None:
        """三态开关 off / legacy_props 回退：沿主车道直排（历史行为）。"""

        for environ in (_ENDLESS_OFF, {"ISAACLAB_CONVEYOR_PROPS": "legacy_props"}):
            with self.subTest(environ=environ):
                layout = resolve_scene_layout(environ)
                self.assertEqual(
                    layout.belt_box_positions,
                    (
                        (-5.62, 15.20, 0.775),
                        (-5.62, 15.95, 0.775),
                        (-5.62, 16.70, 0.775),
                        (-5.62, 17.45, 0.775),
                        (-5.62, 18.20, 0.775),
                    ),
                )

    def test_both_cardbox_kinds_have_equal_size_and_distinct_visual_assets(self) -> None:
        """D01/D02 横向尺寸相同，但必须引用不同轮廓的视觉资产。"""

        kinds = _LAYOUT_MODULE.BELT_BOX_KINDS
        self.assertEqual(kinds["d01"].asset, "cart_box_d01_physics.usda")
        self.assertEqual(kinds["d02"].asset, "cart_box_d02_physics.usda")
        self.assertEqual(
            (kinds["d01"].length_y, kinds["d01"].width_x, kinds["d01"].height_z),
            (0.25, 0.38, 0.1487),
        )
        self.assertEqual(
            (kinds["d02"].length_y, kinds["d02"].width_x, kinds["d02"].height_z),
            (0.25, 0.38, 0.1663),
        )

    def test_pattern_is_configurable_and_cycles(self) -> None:
        single = resolve_scene_layout({"ISAACLAB_BELT_BOX_PATTERN": "d02"})
        self.assertEqual([k.key for k in single.belt_box_kinds], ["d02"] * 17)

        flipped = resolve_scene_layout({"ISAACLAB_BELT_BOX_PATTERN": " D02 , D01 "})
        self.assertEqual(
            [k.key for k in flipped.belt_box_kinds],
            ["d02" if i % 2 == 0 else "d01" for i in range(17)],
        )

    def test_default_two_cardbox_pattern_can_be_selected_explicitly(self) -> None:
        """显式指定 d01,d02 与默认双纸箱形态一致。"""

        layout = resolve_scene_layout({"ISAACLAB_BELT_BOX_PATTERN": "d01,d02"})
        self.assertEqual(
            [k.key for k in layout.belt_box_kinds],
            ["d01" if i % 2 == 0 else "d02" for i in range(17)],
        )
        self.assertEqual(layout.belt_box_half_lengths, (0.19,) * 17)

    def test_all_three_parcel_variants_are_registered(self) -> None:
        layout = resolve_scene_layout(
            {"ISAACLAB_BELT_BOX_PATTERN": "parcel_a01,parcel_a02,parcel_a03"}
        )
        cycle = ("parcel_a01", "parcel_a02", "parcel_a03")
        self.assertEqual(
            [k.key for k in layout.belt_box_kinds],
            [cycle[i % 3] for i in range(17)],
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
        17 箱跨度 12.0 ⇒ 停稳队尾 s = 12.1945 − 12.0 ≈ 0.19，仍深在支线段上
        ——要等前面被逐一取走才逐格拐出来，这是"看不到头"的刻意语义。
        """

        layout = resolve_scene_layout({})
        halves = layout.belt_box_half_lengths
        spawn_spread = _SLOTS_DEFAULT[0] - _SLOTS_DEFAULT[-1]
        self.assertAlmostEqual(spawn_spread, 12.0, places=9)
        settled_tail_s = _S_STOP_DEFAULT - spawn_spread

        # 停稳队尾仍在支线滚筒可用带上（不越出西端），且比出生位更靠下游。
        self.assertGreaterEqual(
            settled_tail_s - halves[-1], _EI_MODULE.PATH_S_MIN
        )
        tail_x, tail_y = _EI_MODULE.path_point(settled_tail_s)
        self.assertAlmostEqual(tail_y, _EI_MODULE.BRANCH_LANE_Y, places=9)
        self.assertGreater(
            settled_tail_s,
            _EI_MODULE.path_s_of_point(*layout.belt_box_positions[-1][:2]),
        )
        # 全列 17 箱的停稳位全部仍在带面/托面覆盖内：主线段不越过带尾 y_min，
        # 弧段/支线段不越出滚筒可用端 s_min。
        for index in range(layout.belt_box_count):
            settled = _S_STOP_DEFAULT - (_SLOTS_DEFAULT[0] - _SLOTS_DEFAULT[index])
            self.assertGreaterEqual(settled - halves[index], _EI_MODULE.PATH_S_MIN)
            _x, y = _EI_MODULE.path_point(settled)
            if settled >= _EI_MODULE.S_ARC_END:  # 主线段
                self.assertGreater(y - halves[index], _DRIVE_MODULE.BELT_Y_MIN)

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
            resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "18"})

    def test_negative_count_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "必须非负"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_COUNT": "-1"})

    def test_negative_queue_gap_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "不能为负"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_QUEUE_GAP": "-0.01"})

    def test_spawn_pitch_too_small_for_the_actual_pair_fails_fast(self) -> None:
        """出生间距按**相邻两箱各自的半长**校验，而不是一个统一箱长。"""

        with self.assertRaisesRegex(ValueError, "放不下相邻的 d01/d02"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.37"})

        # D01/D02 横向尺寸相同，0.40 对全 D01 和默认混排都够（所需 0.19+0.19=0.38）。
        resolve_scene_layout(
            {"ISAACLAB_BELT_BOX_PATTERN": "d01", "ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.40"}
        )
        resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.40"})

    def test_lead_box_on_the_workstation_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "压在工位"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "14.2"})
        with self.assertRaisesRegex(ValueError, "压在工位"):
            resolve_scene_layout(
                {"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "14.2", **_ENDLESS_OFF}
            )

    def test_queue_running_off_the_path_start_fails_fast(self) -> None:
        # 弯道形态：队尾越过支线滚筒可用端（s_min=-4.27，支线延到五段后大幅西移）。
        with self.assertRaisesRegex(ValueError, "悬出支线带面"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_S_LEAD": "-1.5"})
        # 直线回退：队尾悬出带面 y_max（历史判据原样保留）。
        with self.assertRaisesRegex(ValueError, "悬出带面"):
            resolve_scene_layout(
                {"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD": "17.3", **_ENDLESS_OFF}
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

        弯道形态默认 17 箱铺满，等距 pitch 上限 ≈0.758（队尾缘顶到支线可用端
        s_min=-4.27）：0.75 放得下，0.76 顶穿。显式 COUNT=5 时空间富余，等距
        pitch=3.0 仍放得下（历史容量口径）、3.1 顶穿。直线回退维持旧的 1.0 拦截。
        """

        resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.75"})
        with self.assertRaisesRegex(ValueError, "悬出支线带面"):
            resolve_scene_layout({"ISAACLAB_BELT_BOX_SPAWN_PITCH": "0.76"})
        resolve_scene_layout(
            {"ISAACLAB_BELT_BOX_COUNT": "5", "ISAACLAB_BELT_BOX_SPAWN_PITCH": "3.0"}
        )
        with self.assertRaisesRegex(ValueError, "悬出支线带面"):
            resolve_scene_layout(
                {"ISAACLAB_BELT_BOX_COUNT": "5", "ISAACLAB_BELT_BOX_SPAWN_PITCH": "3.1"}
            )
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
