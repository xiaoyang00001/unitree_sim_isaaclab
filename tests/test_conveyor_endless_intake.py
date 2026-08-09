"""endless_intake（西拐入口弯道 + X 支线，复用已有货架）的零 Kit 单测。

锁四件事：①三态开关与布局门（=0 / legacy_props 都不许生成）；②几何常量与
conveyor_drive 真源的交叉一致（车道、带面、板厚）；③路径参数化与插接/占位/
周边间隙（对带头、对货架排 B、对北墙、对交通锥）；④托面对弧段/支线的全覆盖。
"""

from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path


def _load(name: str, relpath: str):
    path = Path(__file__).resolve().parents[1] / relpath
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_EI = _load(
    "conveyor_endless_intake_for_test",
    "tasks/g1_tasks/g1_29dof_sonic_conveyor/endless_intake.py",
)
_DRIVE = _load(
    "conveyor_drive_for_endless_test",
    "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_drive.py",
)


class EndlessIntakeModeTest(unittest.TestCase):
    def test_auto_is_the_default_and_enables_on_the_conveyor_layout(self) -> None:
        cfg = _EI.resolve_endless_intake({}, totes_on_conveyor=True, props_mode="layout")
        self.assertEqual(cfg.mode, "auto")
        self.assertTrue(cfg.requested)
        self.assertTrue(cfg.enabled)
        self.assertEqual(cfg.disabled_reason, "")

    def test_on_and_off_are_explicit(self) -> None:
        on = _EI.resolve_endless_intake(
            {"ISAACLAB_CONVEYOR_ENDLESS": "on"}, totes_on_conveyor=True, props_mode="layout"
        )
        off = _EI.resolve_endless_intake(
            {"ISAACLAB_CONVEYOR_ENDLESS": "off"}, totes_on_conveyor=True, props_mode="layout"
        )
        self.assertTrue(on.enabled)
        self.assertFalse(off.requested)
        self.assertFalse(off.enabled)
        self.assertIn("off", off.disabled_reason)

    def test_invalid_mode_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "ISAACLAB_CONVEYOR_ENDLESS"):
            _EI.resolve_endless_intake(
                {"ISAACLAB_CONVEYOR_ENDLESS": "banana"},
                totes_on_conveyor=True,
                props_mode="layout",
            )

    def test_pushcart_layout_never_generates_the_curve(self) -> None:
        """=0 的拖车组 (-5.62, 18.75) 约 83% 落在弯道占位内，必须被布局门拦下。"""

        cfg = _EI.resolve_endless_intake(
            {"ISAACLAB_CONVEYOR_ENDLESS": "on"}, totes_on_conveyor=False, props_mode="layout"
        )
        self.assertTrue(cfg.requested)
        self.assertFalse(cfg.enabled)
        self.assertIn("=0", cfg.disabled_reason)

    def test_legacy_props_never_generates_the_curve(self) -> None:
        """legacy_props 在 =1 下的空车 pushcart_2 (-5.4, 19.39363) 也在弯道足迹里。"""

        cfg = _EI.resolve_endless_intake(
            {}, totes_on_conveyor=True, props_mode="legacy_props"
        )
        self.assertTrue(cfg.requested)
        self.assertFalse(cfg.enabled)
        self.assertIn("legacy_props", cfg.disabled_reason)


class EndlessIntakeGeometryCrossTest(unittest.TestCase):
    """与 conveyor_drive 真源常量的交叉断言（两模块都是单文件加载）。"""

    def test_lane_constants_match_conveyor_drive(self) -> None:
        self.assertEqual(_EI.MAIN_LANE_X, _DRIVE.BELT_X_CENTER)
        self.assertAlmostEqual(_EI.LANE_HALF_WIDTH, _DRIVE.BELT_WIDTH / 2.0, places=9)
        self.assertEqual(_EI.BELT_TOP_Z, _DRIVE.BELT_TOP_Z)
        self.assertEqual(_EI.COLLIDER_THICKNESS, _DRIVE.BELT_COLLIDER_THICKNESS)
        # 带头实测面与碰撞板常量的收口差 ≤3 mm（插接必须用实测面）。
        self.assertLess(abs(_EI.BELT_HEAD_FACE_Y - _DRIVE.BELT_Y_MAX), 0.003)

    def test_curve_female_port_plugs_over_the_belt_head(self) -> None:
        """弯道南向母口套主线带头公头 15 mm（正常插接，非东拐的公对公对顶）。"""

        curve_south = _EI.CURVE_AABB[1][0]
        overlap = _EI.BELT_HEAD_FACE_Y - curve_south
        self.assertAlmostEqual(overlap, _EI.PLUG_DEPTH, places=4)
        # 南口车道中线对齐公头实测中心（与驱动常量 -5.62 差 3 mm，同源偏差）。
        self.assertLess(abs(_EI.CURVE_POS[0] + 1.4959 - _EI.BELT_HEAD_MALE_CENTER_X), 1e-6)

    def test_xleg_chain_plugs_male_into_female_at_each_joint(self) -> None:
        """弯道西公头插段1母口、段间公插母，各 15 mm（AABB 重叠口径 ±0.5 mm）。"""

        aabbs = (_EI.CURVE_AABB, *_EI.XLEG_AABBS)
        for index in range(len(aabbs) - 1):
            east = aabbs[index]
            west = aabbs[index + 1]
            with self.subTest(joint=index):
                overlap = west[0][1] - east[0][0]
                self.assertGreater(overlap, 0.0145)
                self.assertLess(overlap, 0.0160)

    def test_xleg_segments_share_the_branch_lane(self) -> None:
        for tx, ty, tz in _EI.XLEG_POSITIONS:
            with self.subTest(tx=tx):
                self.assertAlmostEqual(ty, _EI.CURVE_POS[1], places=9)
                self.assertAlmostEqual(tz, 0.003, places=9)
                self.assertAlmostEqual(ty - 0.5014, _EI.BRANCH_LANE_Y, places=9)

    def test_curve_and_xleg_stay_clear_of_the_north_wall(self) -> None:
        for name, aabb in (("curve", _EI.CURVE_AABB), *(
            (f"xleg{i+1}", a) for i, a in enumerate(_EI.XLEG_AABBS)
        )):
            with self.subTest(name):
                self.assertGreater(_EI.WALL_FACE_Y - aabb[1][1], 3.0)

    def test_branch_tail_respects_the_existing_rack_row(self) -> None:
        """三段是硬上限：端头距排 B 端护板 0.318 m；南北向与货架排 x 分离。"""

        tail_west = _EI.XLEG_AABBS[-1][0][0]
        gap = tail_west - _EI.RACK_B_GUARD_EAST_X
        self.assertGreater(gap, 0.25)
        self.assertLess(gap, 0.40)
        # 支线 AABB 南缘与排 B 北缘有 y 重叠（0.118），但 x 区间完全分离。
        self.assertLess(_EI.XLEG_AABBS[-1][1][0], _EI.RACK_B_NORTH_Y)
        self.assertGreater(tail_west, _EI.RACK_B_GUARD_EAST_X)
        # 托面条带同样不越过护板。
        self.assertGreater(_EI.BRANCH_PLATE_X_RANGE[0], _EI.RACK_B_GUARD_EAST_X + 0.3)

    def test_cone4_clears_the_branch(self) -> None:
        (cx0, cx1), (cy0, _cy1) = _EI.CONE_4_AABB
        xleg3 = _EI.XLEG_AABBS[-1]
        x_overlap = min(cx1, xleg3[0][1]) - max(cx0, xleg3[0][0])
        y_gap = cy0 - xleg3[1][1]
        # x 上几乎相切（允许有重叠），但 y 净距必须 > 0.8。
        self.assertGreater(y_gap, 0.8)
        self.assertTrue(x_overlap < 0.05 or y_gap > 0.0)

    def test_layout_zero_conflicts_are_real(self) -> None:
        """=0 拖车组与 legacy_props 空车确实落在弯道 xy 占位内——布局门的依据。"""

        (x0, x1), (y0, y1), _ = _EI.CURVE_AABB
        for px, py in ((-5.62, 18.75), (-5.4, 19.39363)):
            with self.subTest((px, py)):
                self.assertTrue(x0 < px < x1)
                self.assertTrue(y0 < py < y1)


class EndlessIntakePathTest(unittest.TestCase):
    def test_corner_center_is_derived_from_the_two_lanes(self) -> None:
        cx, cy = _EI.CORNER_CENTER
        self.assertAlmostEqual(cx, _EI.MAIN_LANE_X - _EI.CORNER_RADIUS, places=9)
        self.assertAlmostEqual(cy, _EI.BRANCH_LANE_Y - _EI.CORNER_RADIUS, places=9)

    def test_path_points_at_the_segment_boundaries(self) -> None:
        self.assertEqual(_EI.path_point(0.0), (_EI.S_ORIGIN_X, _EI.BRANCH_LANE_Y))
        x, y = _EI.path_point(_EI.S_ARC_START)
        self.assertAlmostEqual(x, _EI.CORNER_CENTER[0], places=9)
        self.assertAlmostEqual(y, _EI.BRANCH_LANE_Y, places=9)
        x, y = _EI.path_point(_EI.S_ARC_END)
        self.assertAlmostEqual(x, _EI.MAIN_LANE_X, places=9)
        self.assertAlmostEqual(y, _EI.CORNER_CENTER[1], places=9)

    def test_path_heading_turns_from_plus_x_to_minus_y(self) -> None:
        self.assertEqual(_EI.path_heading(0.0), (1.0, 0.0))
        hx, hy = _EI.path_heading(_EI.S_ARC_START + _EI.CORNER_RADIUS * math.pi / 4)
        self.assertAlmostEqual(hx, math.sqrt(0.5), places=6)
        self.assertAlmostEqual(hy, -math.sqrt(0.5), places=6)
        self.assertEqual(_EI.path_heading(_EI.S_ARC_END + 1.0), (0.0, -1.0))

    def test_s_roundtrips_through_the_point_mapping(self) -> None:
        for s in (-0.2, 0.0, 0.7, _EI.S_ARC_START, 6.2, 7.5, _EI.S_ARC_END, 9.0, 12.19):
            with self.subTest(s=s):
                x, y = _EI.path_point(s)
                self.assertAlmostEqual(_EI.path_s_of_point(x, y), s, places=6)

    def test_workstation_maps_onto_the_main_segment(self) -> None:
        s_stop = _EI.path_s_of_main_y(14.148)
        self.assertGreater(s_stop, _EI.S_ARC_END)
        self.assertAlmostEqual(s_stop, 12.194515, places=4)
        x, y = _EI.path_point(s_stop)
        self.assertAlmostEqual(x, _EI.MAIN_LANE_X, places=9)
        self.assertAlmostEqual(y, 14.148, places=9)

    def test_recycle_line_maps_downstream_of_the_workstation(self) -> None:
        self.assertGreater(_EI.path_s_of_main_y(10.6), _EI.path_s_of_main_y(14.148))

    def test_respawn_point_is_on_the_deepest_xleg_rollers(self) -> None:
        x, y = _EI.RESPAWN_XY
        self.assertEqual(y, _EI.BRANCH_LANE_Y)
        lo, hi = _EI.XLEG_ROLLER_X_RANGES[-1]
        self.assertTrue(lo <= x <= hi)
        # PATH_S_MIN 同样不越过段 3 滚筒可用端。
        px, _ = _EI.path_point(_EI.PATH_S_MIN)
        self.assertGreaterEqual(px, lo - 1e-9)


class EndlessIntakePlatesTest(unittest.TestCase):
    def test_plates_extend_the_main_collider_without_gaps(self) -> None:
        # 拐角补块南缘正好接主线碰撞板末端（带头）。
        self.assertAlmostEqual(_EI.CORNER_PLATE_Y_RANGE[0], _DRIVE.BELT_Y_MAX, places=6)
        # 拐角补块与支线条带在 x 上无缝相接。
        self.assertAlmostEqual(
            _EI.BRANCH_PLATE_X_RANGE[1], _EI.CORNER_PLATE_X_RANGE[0], places=9
        )
        # 支线条带以车道中线为中心、宽度=主线碰撞板宽度。
        y0, y1 = _EI.BRANCH_PLATE_Y_RANGE
        self.assertAlmostEqual((y0 + y1) / 2.0, _EI.BRANCH_LANE_Y, places=9)
        self.assertAlmostEqual(y1 - y0, _DRIVE.BELT_WIDTH, places=9)

    def test_plates_stay_inside_the_visual_belts(self) -> None:
        self.assertLess(_EI.CORNER_PLATE_Y_RANGE[1], _EI.CURVE_AABB[1][1])
        self.assertLess(_EI.BRANCH_PLATE_Y_RANGE[1], _EI.XLEG_AABBS[0][1][1])
        self.assertGreater(_EI.BRANCH_PLATE_Y_RANGE[0], _EI.XLEG_AABBS[0][1][0])
        # 支线条带西端不越过段 3 滚筒可用端。
        self.assertGreaterEqual(
            _EI.BRANCH_PLATE_X_RANGE[0], _EI.XLEG_ROLLER_X_RANGES[-1][0] - 0.001
        )

    def test_arc_swath_is_fully_supported_by_the_plates(self) -> None:
        """弧段车道 ±LANE_HALF_WIDTH 的扫掠范围必须落在托面并集内（箱子不悬空）。"""

        cx, cy = _EI.CORNER_CENTER
        for theta_deg in range(0, 91, 5):
            theta = math.radians(theta_deg)
            for r in (
                _EI.CORNER_RADIUS - _EI.LANE_HALF_WIDTH,
                _EI.CORNER_RADIUS + _EI.LANE_HALF_WIDTH,
            ):
                x = cx + r * math.sin(theta)
                y = cy + r * math.cos(theta)
                if y <= _DRIVE.BELT_Y_MAX:
                    continue  # 已在主线碰撞板覆盖范围内
                with self.subTest(theta=theta_deg, r=r):
                    self.assertTrue(
                        _EI.CORNER_PLATE_X_RANGE[0] - 1e-6
                        <= x
                        <= _EI.CORNER_PLATE_X_RANGE[1] + 1e-6
                        or _EI.BRANCH_PLATE_X_RANGE[0] - 1e-6
                        <= x
                        <= _EI.BRANCH_PLATE_X_RANGE[1] + 1e-6
                    )
                    self.assertLessEqual(y, _EI.CORNER_PLATE_Y_RANGE[1] + 1e-6)

    def test_extra_rects_pad_the_plates_by_ten_centimetres(self) -> None:
        for rect, (xr, yr) in zip(
            _EI.ON_BELT_EXTRA_RECTS,
            (
                (_EI.CORNER_PLATE_X_RANGE, _EI.CORNER_PLATE_Y_RANGE),
                (_EI.BRANCH_PLATE_X_RANGE, _EI.BRANCH_PLATE_Y_RANGE),
            ),
        ):
            with self.subTest(rect=rect):
                self.assertAlmostEqual(rect[0], xr[0] - 0.10, places=9)
                self.assertAlmostEqual(rect[1], xr[1] + 0.10, places=9)
                self.assertAlmostEqual(rect[2], yr[0] - 0.10, places=9)
                self.assertAlmostEqual(rect[3], yr[1] + 0.10, places=9)

    def test_yaw_quat_is_a_unit_z_rotation(self) -> None:
        w, x, y, z = _EI.yaw_quat(-90.0)
        self.assertAlmostEqual(w, math.sqrt(0.5), places=9)
        self.assertEqual((x, y), (0.0, 0.0))
        self.assertAlmostEqual(z, -math.sqrt(0.5), places=9)


if __name__ == "__main__":
    unittest.main()
