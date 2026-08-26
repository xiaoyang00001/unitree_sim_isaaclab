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
        """=0 的拖车组 (-5.62, 19.0) 约 76% 落在窄弯道内，必须被布局门拦下。"""

        cfg = _EI.resolve_endless_intake(
            {"ISAACLAB_CONVEYOR_ENDLESS": "on"}, totes_on_conveyor=False, props_mode="layout"
        )
        self.assertTrue(cfg.requested)
        self.assertFalse(cfg.enabled)
        self.assertIn("=0", cfg.disabled_reason)

    def test_legacy_props_never_generates_the_curve(self) -> None:
        """legacy_props 在 =1 下的空车 pushcart_2 (-5.4, 19.64363) 也在弯道足迹里。"""

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
                self.assertAlmostEqual(ty, _EI.XLEG_POS_Y, places=9)
                self.assertAlmostEqual(tz, 0.003, places=9)
                self.assertAlmostEqual(
                    ty - _EI.XLEG_SOURCE_LANE_CENTER_X * _EI.NARROW_WIDTH_SCALE,
                    _EI.BRANCH_LANE_Y,
                    places=9,
                )

    def test_xleg_visual_width_is_scaled_to_the_narrow_lane(self) -> None:
        self.assertEqual(_EI.NARROW_LANE_WIDTH, _DRIVE.BELT_WIDTH)
        self.assertAlmostEqual(_EI.NARROW_WIDTH_SCALE, 2.0 / 3.0, places=9)
        expected_scale = (_EI.CONVEYOR_UNIT_SCALE * 2.0 / 3.0, 0.01, 0.01)
        for actual, expected in zip(_EI.XLEG_SCALE, expected_scale, strict=True):
            self.assertAlmostEqual(actual, expected, places=12)
        # 托面在窄化后的 A05 外框内保持两侧近似相等的视觉余量。
        visual_y0, visual_y1 = _EI.XLEG_VISUAL_Y_RANGE
        support_y0, support_y1 = _EI.BRANCH_PLATE_Y_RANGE
        self.assertGreater(support_y0, visual_y0)
        self.assertLess(support_y1, visual_y1)
        self.assertAlmostEqual(support_y0 - visual_y0, visual_y1 - support_y1, places=4)

    def test_curve_and_xleg_stay_clear_of_the_north_wall(self) -> None:
        # 窄支线北缘约 20.437、窄弯道北缘 20.390，均距墙超过 2.9 m。
        for name, aabb in (("curve", _EI.CURVE_AABB), *(
            (f"xleg{i+1}", a) for i, a in enumerate(_EI.XLEG_AABBS)
        )):
            with self.subTest(name):
                self.assertGreater(_EI.WALL_FACE_Y - aabb[1][1], 2.9)

    def test_branch_skims_north_of_the_existing_rack_row(self) -> None:
        """Δ=0.25 北移让支线从排 B **北侧**通过；窄化后 y 净距约 275 mm。

        约束绑定项是端护板北缘 19.3946（mesh 实测）；架板北缘 19.3457、钢架柱
        北缘 19.3073 更靠南，净距只会更大。五段端头 -17.038 已越过护板东缘
        -13.386 约 3.65 m——旧版"三段是硬上限"的前提（南北向贴着排 B 走）已随
        北移失效。
        """

        south = _EI.XLEG_AABBS[-1][1][0]
        gap = south - _EI.RACK_B_GUARD_NORTH_Y
        self.assertGreaterEqual(gap, 0.05)
        self.assertAlmostEqual(gap, 0.2751, places=3)
        # 端护板北缘在架板/钢架柱之北（绑定项排序），排 B 本体更不构成约束。
        self.assertGreater(_EI.RACK_B_GUARD_NORTH_Y, _EI.RACK_B_NORTH_Y)
        # 托面条带南缘离护板北缘 358.8 mm（x 现已重叠，靠 y 让开）。
        self.assertGreaterEqual(
            _EI.BRANCH_PLATE_Y_RANGE[0] - _EI.RACK_B_GUARD_NORTH_Y, 0.15
        )
        # 支线整体（含托面）都在排 B 北缘以北。
        self.assertGreater(south, _EI.RACK_B_NORTH_Y)

    def test_branch_clears_the_forklift_fork_tips(self) -> None:
        """段 4/5 与叉车 x 重叠：窄化后南缘对货叉尖净距约 349 mm。"""

        south = _EI.XLEG_AABBS[-1][1][0]
        # x 确有重叠（叉车在段 4/5 下方），才有必要验 y 净距。
        overlap = min(_EI.FORKLIFT_X_RANGE[1], _EI.XLEG_AABBS[-1][0][1]) - max(
            _EI.FORKLIFT_X_RANGE[0], _EI.XLEG_AABBS[-1][0][0]
        )
        self.assertGreater(overlap, 0.5)
        gap = south - _EI.FORKLIFT_FORK_NORTH_Y
        self.assertGreaterEqual(gap, 0.15)
        self.assertAlmostEqual(gap, 0.3487, places=3)

    def test_cone4_clears_the_branch(self) -> None:
        (cx0, cx1), (cy0, _cy1) = _EI.CONE_4_AABB
        xleg3 = _EI.XLEG_AABBS[2]
        x_overlap = min(cx1, xleg3[0][1]) - max(cx0, xleg3[0][0])
        y_gap = cy0 - xleg3[1][1]
        # x 上几乎相切（允许有重叠），窄化后 y 净距约 0.793 m。
        self.assertGreater(y_gap, 0.55)
        self.assertTrue(x_overlap < 0.05 or y_gap > 0.0)

    def test_layout_zero_conflicts_are_real(self) -> None:
        """=0 拖车组与 legacy_props 空车确实落在弯道 xy 占位内——布局门的依据。

        两组道具都走 _shifted 管线随 Δ 同移，相对几何与 Δ=0 完全一致。
        """

        (x0, x1), (y0, y1), _ = _EI.CURVE_AABB
        for px, py in ((-5.62, 19.0), (-5.4, 19.64363)):
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
        # 北移基准 14.398 再沿世界 -Y 下移 0.30 m，默认工位 y=14.098。
        s_stop = _EI.path_s_of_main_y(14.098)
        self.assertGreater(s_stop, _EI.S_ARC_END)
        self.assertAlmostEqual(s_stop, 12.494515, places=4)
        x, y = _EI.path_point(s_stop)
        self.assertAlmostEqual(x, _EI.MAIN_LANE_X, places=9)
        self.assertAlmostEqual(y, 14.098, places=9)

    def test_recycle_line_maps_downstream_of_the_workstation(self) -> None:
        self.assertGreater(_EI.path_s_of_main_y(10.85), _EI.path_s_of_main_y(14.098))

    def test_respawn_point_is_on_the_deepest_xleg_rollers(self) -> None:
        x, y = _EI.RESPAWN_XY
        self.assertEqual(y, _EI.BRANCH_LANE_Y)
        lo, hi = _EI.XLEG_ROLLER_X_RANGES[-1]
        self.assertTrue(lo <= x <= hi)
        # PATH_S_MIN 同样不越过段 5 滚筒可用端。
        px, _ = _EI.path_point(_EI.PATH_S_MIN)
        self.assertGreaterEqual(px, lo - 1e-9)
        # 最大箱型 c01（半长 0.25）在回生点的西缘距滚筒西端还有 ~124 mm。
        self.assertGreaterEqual(x - 0.25 - lo, 0.12)

    def test_respawn_slot_hides_behind_rack_row_b_for_eye_e2(self) -> None:
        """回生点的遮挡口径（数值来自本轮 mesh 级射线核算，几何关系锁死）。

        E2=(-6.70, 工位y+0.3, 1.6) 的"东上角"判据全遮边界是箱心 x≤-16.57
        （s≤-3.81）；RESPAWN_S=-3.90 留 90 mm 裕量。E1 无全遮解（排 B 首层
        216 mm 通视缝），只锁"深藏在排 B x 覆盖段内"。
        """

        x, _y = _EI.RESPAWN_XY
        self.assertLessEqual(x, -16.57)
        self.assertAlmostEqual(-16.57 - x, 0.09, places=6)
        # 回生点在排 B 的 x 覆盖段内（排 B 连排到 -29.58，护板东缘 -13.386）。
        self.assertLess(x, _EI.RACK_B_GUARD_EAST_X)
        # 通视缝确实存在于箱体高度带内（E1 残余可见的根因，README 已知限制）。
        seam_lo, seam_hi = _EI.RACK_B_FIRST_TIER_SEAM_Z
        self.assertLess(seam_lo, 1.0223)  # 箱顶最高 c01
        self.assertGreater(seam_hi - seam_lo, 0.2)


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
