from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


_TASK_DIR = (
    Path(__file__).resolve().parents[1] / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
)


def _load(module_name: str, relative_path: str):
    """按 test_conveyor_scene_layout 的套路单文件加载，不 import Isaac Lab。"""

    spec = importlib.util.spec_from_file_location(module_name, _TASK_DIR / relative_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_SHROUD = _load("conveyor_shroud_config_for_test", "shroud_config.py")
_DRIVE = _load("conveyor_drive_for_shroud_test", "conveyor_drive.py")

resolve_conveyor_shroud = _SHROUD.resolve_conveyor_shroud

_INFEED_ASSET_PATH = (
    _TASK_DIR / "scene_assets/props" / _SHROUD.SHROUD_INFEED_ASSET_FILENAME
)
_OUTFEED_ASSET_PATH = (
    _TASK_DIR / "scene_assets/props" / _SHROUD.SHROUD_OUTFEED_ASSET_FILENAME
)
_SCANNER_S3_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/"
    "NVIDIA/Assets/DigitalTwin/Assets/Warehouse/Equipment/Security/BaggageScanner_A/"
    "SecurityBaggageScanner_A01_01.usd"
)


class ConveyorShroudModeTest(unittest.TestCase):
    def test_auto_is_the_default_and_enables_the_shroud(self) -> None:
        placement = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        self.assertEqual(placement.mode, "auto")
        self.assertTrue(placement.enabled)

    def test_on_and_off_are_explicit(self) -> None:
        on = resolve_conveyor_shroud(
            {"ISAACLAB_CONVEYOR_SHROUD": " ON "}, totes_on_conveyor=True
        )
        off = resolve_conveyor_shroud(
            {"ISAACLAB_CONVEYOR_SHROUD": "off"}, totes_on_conveyor=True
        )

        self.assertEqual((on.mode, on.enabled), ("on", True))
        self.assertEqual((off.mode, off.enabled), ("off", False))

    def test_off_still_reports_where_the_shroud_would_go(self) -> None:
        """关掉时也算出坐标，启动日志才能自曝"本该在哪"，排障不用改代码。"""

        off = resolve_conveyor_shroud(
            {"ISAACLAB_CONVEYOR_SHROUD": "off"}, totes_on_conveyor=True
        )
        on = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        self.assertEqual(off.pos, on.pos)
        self.assertEqual(off.rot, on.rot)

    def test_invalid_mode_raises_and_lists_the_choices(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            resolve_conveyor_shroud(
                {"ISAACLAB_CONVEYOR_SHROUD": "enabled"}, totes_on_conveyor=True
            )

        message = str(ctx.exception)
        self.assertIn("ISAACLAB_CONVEYOR_SHROUD", message)
        for choice in ("auto", "on", "off"):
            self.assertIn(choice, message)

    def test_invalid_face_override_raises(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            resolve_conveyor_shroud(
                {"ISAACLAB_CONVEYOR_SHROUD_FACE_Y": "front"}, totes_on_conveyor=True
            )

        self.assertIn("ISAACLAB_CONVEYOR_SHROUD_FACE_Y", str(ctx.exception))


class ConveyorShroudPlacementTest(unittest.TestCase):
    def test_conveyor_layout_parks_the_shroud_against_the_positive_y_wall(self) -> None:
        placement = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        self.assertEqual(placement.port, "infeed")
        self.assertEqual(placement.rot, (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(placement.yaw_degrees, 0.0)
        self.assertEqual(placement.design_scale, 1.40)
        self.assertEqual(
            placement.asset_filename, _SHROUD.SHROUD_INFEED_ASSET_FILENAME
        )
        # +Y 端面 = 墙面 23.606 − 墙缝 0.05；机身中心 = 端面 − 放大后半长 1.783936。
        self.assertAlmostEqual(placement.face_y, 23.556, places=5)
        self.assertAlmostEqual(placement.pos[0], -5.62, places=5)
        self.assertAlmostEqual(placement.pos[1], 21.772064, places=6)
        self.assertAlmostEqual(placement.pos[2], -0.429897, places=6)
        self.assertAlmostEqual(placement.y_span[0], 19.988128, places=6)
        self.assertAlmostEqual(placement.y_span[1], 23.556, places=5)
        self.assertAlmostEqual(placement.x_span[0], -6.210254, places=6)
        self.assertAlmostEqual(placement.x_span[1], -5.029746, places=6)
        self.assertAlmostEqual(placement.top_z, 1.672987, places=6)
        # 机身放大后宽 1.1805，略宽于流水线实测最宽处 1.15114。
        self.assertGreater(placement.x_span[1] - placement.x_span[0], 1.15114)
        # 机身顶不能捅穿 3.10 m 的墙。
        self.assertLess(placement.top_z, 3.10)

    def test_pushcart_layout_keeps_the_unscaled_shroud_outside_the_outfeed_port(self) -> None:
        placement = resolve_conveyor_shroud(
            {"ISAACLAB_TOTES_ON_CONVEYOR": "0"}, totes_on_conveyor=False
        )

        self.assertEqual(placement.port, "outfeed")
        # 绕 Z 转 180°，让**完整的**那半段滚筒床朝着流水线（近端那半在入料端薄层
        # 里被隐藏，所以出料端换用没有隐藏清单的另一份薄层）。
        self.assertEqual(placement.rot, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(placement.yaw_degrees, 180.0)
        self.assertEqual(
            placement.asset_filename, _SHROUD.SHROUD_OUTFEED_ASSET_FILENAME
        )
        # =0 必须钉死 scale=1.0：放大后半长 1.784，贴带尾会整块插进
        # blue_sorting_bin_03（北移后 y[12.307,13.642]）。
        self.assertEqual(placement.design_scale, 1.0)
        self.assertAlmostEqual(placement.face_y, 13.21, places=5)
        self.assertAlmostEqual(placement.pos[1], 11.93576, places=5)
        self.assertAlmostEqual(placement.y_span[0], 10.66152, places=5)
        self.assertAlmostEqual(placement.y_span[1], 13.21, places=5)
        # 落地机柜仍必须整体避开北移后的蓝料箱堆。
        self.assertLess(placement.cabinet_y_span[1], 12.307)

    def test_infeed_shroud_swallows_the_belt_end_and_outfeed_stays_clear(self) -> None:
        """⚠️ 判据方向与旧版**相反**：入料端现在就是要让带体插进机身。"""

        infeed = resolve_conveyor_shroud({}, totes_on_conveyor=True)
        outfeed = resolve_conveyor_shroud({}, totes_on_conveyor=False)
        belt_max = _DRIVE.BELT_Y_MAX  # 21.77
        belt_min = _DRIVE.BELT_Y_MIN  # 13.74

        self.assertTrue(infeed.belt_penetrates)
        # 带端必须落在机身内部……
        self.assertLess(infeed.y_span[0], belt_max)
        self.assertGreater(infeed.y_span[1], belt_max)
        # ……而且要越过近端条帘、停在不透明机柜内部（端头断面才藏得住）。
        self.assertGreater(belt_max, infeed.curtain_y_span[0])
        self.assertLess(belt_max, infeed.curtain_y_span[1])
        self.assertGreater(belt_max, infeed.cabinet_y_span[0])
        self.assertLess(belt_max, infeed.cabinet_y_span[1])
        self.assertAlmostEqual(belt_max - infeed.curtain_y_span[0], 0.41235, places=4)

        # 出料端维持"整机在带体外侧"的老语义。
        self.assertFalse(outfeed.belt_penetrates)
        self.assertLessEqual(outfeed.y_span[1], belt_min)

    def test_infeed_shroud_is_parked_against_the_wall_without_touching_it(self) -> None:
        infeed = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        self.assertAlmostEqual(
            _SHROUD.WALL_FACE_Y - infeed.y_span[1], _SHROUD.WALL_CLEARANCE, places=6
        )
        self.assertGreater(_SHROUD.WALL_CLEARANCE, 0.0)

    def test_north_shift_constant_matches_conveyor_drive(self) -> None:
        self.assertEqual(
            _SHROUD.CONVEYOR_NORTH_SHIFT_Y, _DRIVE.CONVEYOR_NORTH_SHIFT_Y
        )

    def test_shift_upper_bound_keeps_the_belt_legs_out_of_the_cabinet(self) -> None:
        """Δ 的硬上限：流水线最北一组支腿站不能扎穿机柜侧板。

        支腿站外偏 0.576 > 洞口半宽 0.442，只能靠"停在机柜端面之外"来保证。
        北移前最北一组立柱到 y≈17.70，北移后 21.25，机柜近端面 21.2816。
        """

        infeed = resolve_conveyor_shroud({}, totes_on_conveyor=True)
        leg_station_north_y = 17.70 + _SHROUD.CONVEYOR_NORTH_SHIFT_Y

        self.assertLess(leg_station_north_y, infeed.cabinet_y_span[0])
        self.assertLess(infeed.cabinet_y_span[0] - leg_station_north_y, 0.10)

    def test_face_override_translates_the_whole_shroud(self) -> None:
        default = resolve_conveyor_shroud({}, totes_on_conveyor=True)
        moved = resolve_conveyor_shroud(
            {"ISAACLAB_CONVEYOR_SHROUD_FACE_Y": "23.856"}, totes_on_conveyor=True
        )

        self.assertAlmostEqual(moved.face_y, 23.856, places=5)
        self.assertAlmostEqual(moved.pos[1] - default.pos[1], 0.30, places=5)
        self.assertAlmostEqual(moved.y_span[0] - default.y_span[0], 0.30, places=5)
        self.assertEqual(moved.pos[0], default.pos[0])
        self.assertEqual(moved.pos[2], default.pos[2])

    def test_machine_tunnel_floor_is_sunk_just_under_the_belt_plane(self) -> None:
        """滚筒床已被隐藏，这条现在保证的是"隧道地板刚好低于带面"。

        资产的 cartframe 顶面 native z 恰好 == ASSET_ROLLER_TOP_Z，正好对齐会与
        流水线滚筒顶共面 z-fighting，所以多压 SHROUD_ZFIGHT_SINK。
        """

        for totes_on_conveyor in (True, False):
            with self.subTest(totes_on_conveyor=totes_on_conveyor):
                placement = resolve_conveyor_shroud(
                    {}, totes_on_conveyor=totes_on_conveyor
                )
                roller_top_world = (
                    placement.pos[2]
                    + _SHROUD.ASSET_ROLLER_TOP_Z * placement.design_scale
                )
                self.assertAlmostEqual(
                    roller_top_world,
                    _DRIVE.BELT_TOP_Z - _SHROUD.SHROUD_ZFIGHT_SINK,
                    places=6,
                )
                self.assertLess(roller_top_world, _DRIVE.BELT_TOP_Z)
        self.assertGreater(_SHROUD.SHROUD_ZFIGHT_SINK, 0.0)

    def test_curtain_opening_clears_both_tote_lanes(self) -> None:
        placement = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        # 条帘下沿只允许轻扎进带面（<10 mm，帘子搭在带上的观感），上沿要给
        # 半尺寸料筐（高 0.15）留出净空。
        self.assertLess(_DRIVE.BELT_TOP_Z - placement.opening_z[0], 0.010)
        self.assertGreater(placement.opening_z[1] - _DRIVE.BELT_TOP_Z, 0.15)
        # ⭐ 放大的验收点：洞口必须容得下两条筐道的横向总跨 0.84
        # （x=-5.35/-5.89、筐宽 0.30）。原尺寸只有 0.632，装不下。
        self.assertGreater(placement.opening_width_x, _SHROUD.TOTE_LANE_SPAN_X)
        self.assertAlmostEqual(placement.opening_width_x, 0.884202, places=5)
        lane_margin = (placement.opening_width_x - _SHROUD.TOTE_LANE_SPAN_X) / 2.0
        self.assertGreater(lane_margin, 0.02)

    def test_spawn_scale_carries_the_centimetre_to_metre_factor(self) -> None:
        placement = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        expected = _SHROUD.SHROUD_UNIT_SCALE * placement.design_scale
        self.assertEqual(placement.scale, (expected, expected, expected))
        self.assertAlmostEqual(expected, 0.014, places=6)


class ConveyorShroudBeltConstantsTest(unittest.TestCase):
    """shroud_config 为了零相对 import 复制了带面常量，这里防止两边漂移。"""

    def test_belt_constants_match_conveyor_drive(self) -> None:
        self.assertEqual(
            _SHROUD.CONVEYOR_NORTH_SHIFT_Y, _DRIVE.CONVEYOR_NORTH_SHIFT_Y
        )
        self.assertEqual(_SHROUD.BELT_X_CENTER, _DRIVE.BELT_X_CENTER)
        self.assertEqual(_SHROUD.BELT_TOP_Z, _DRIVE.BELT_TOP_Z)
        self.assertEqual(_SHROUD.BELT_Y_MIN, _DRIVE.BELT_Y_MIN)
        self.assertEqual(_SHROUD.BELT_Y_MAX, _DRIVE.BELT_Y_MAX)


class ConveyorShroudAssetLayerTest(unittest.TestCase):
    def test_both_layers_exist_and_declare_the_stage_metadata(self) -> None:
        for path, default_prim in (
            (_INFEED_ASSET_PATH, "InfeedScannerVisual"),
            (_OUTFEED_ASSET_PATH, "OutfeedScannerVisual"),
        ):
            with self.subTest(layer=path.name):
                self.assertTrue(path.is_file(), path)
                text = path.read_text(encoding="utf-8")

                self.assertTrue(text.startswith("#usda 1.0"))
                for token in (
                    f'defaultPrim = "{default_prim}"',
                    "metersPerUnit = 1",
                    'upAxis = "Z"',
                    "doc = ",
                ):
                    self.assertIn(token, text)

    def test_both_layers_reference_the_official_asset_over_s3(self) -> None:
        for path in (_INFEED_ASSET_PATH, _OUTFEED_ASSET_PATH):
            with self.subTest(layer=path.name):
                text = path.read_text(encoding="utf-8")

                self.assertIn(f"@{_SCANNER_S3_URL}@", text)
                # 本地资产包只是 S3 前缀的镜像，写死本地盘路径换机就断。
                self.assertNotIn("/media/", text)

    def test_layers_author_no_physics_at_all(self) -> None:
        """被引用资产 coll=0 / rigid=0，薄层也不许自己加回来。

        只看 stage metadata 之后的实际 authored 内容，header 的 doc 里会提到这些
        schema 名字（说明的是"已确认没有"），不能拿来当命中。
        """

        for path in (_INFEED_ASSET_PATH, _OUTFEED_ASSET_PATH):
            with self.subTest(layer=path.name):
                body = path.read_text(encoding="utf-8").split('"""\n)', 1)[-1]

                for token in ("apiSchemas", "physics:", "physxCollision", "physxRigidBody"):
                    self.assertNotIn(token, body)

    def test_infeed_layer_hides_exactly_the_near_end_roller_bed(self) -> None:
        """被带体吞掉的那半段外露滚筒床必须显式隐藏，不能靠"刚好被挡住"。

        近端 roller/rollercover 10..18 是资产里唯一按端分离的 Prim（局部
        y[-1.2135,-0.6428]）；源资产 68 个 Gprim 无 instanceable，over 正常生效。
        """

        body = _INFEED_ASSET_PATH.read_text(encoding="utf-8").split('"""\n)', 1)[-1]

        self.assertEqual(len(_SHROUD.HIDDEN_SUBMESH_NAMES), 18)
        for name in _SHROUD.HIDDEN_SUBMESH_NAMES:
            self.assertIn(f'over "{name}"', body)
        self.assertEqual(body.count('token visibility = "invisible"'), 18)
        # 远端（靠墙）那半段必须留着，它读起来就是"出料段顶到墙上"。
        for index in range(1, 10):
            for kind in ("roller", "rollercover"):
                self.assertNotIn(
                    f'over "sm_securitybaggagescanner_a01_{kind}{index:02d}_01"', body
                )

    def test_outfeed_layer_keeps_its_roller_bed_visible(self) -> None:
        """=0 的机身在带尾外侧，滚筒床要留着当"料筐的去处"。"""

        body = _OUTFEED_ASSET_PATH.read_text(encoding="utf-8").split('"""\n)', 1)[-1]

        self.assertNotIn("visibility", body)
        self.assertNotIn("over ", body)

    def test_hidden_list_is_empty_for_the_outfeed_placement(self) -> None:
        infeed = resolve_conveyor_shroud({}, totes_on_conveyor=True)
        outfeed = resolve_conveyor_shroud({}, totes_on_conveyor=False)

        self.assertEqual(infeed.hidden_submesh_names, _SHROUD.HIDDEN_SUBMESH_NAMES)
        self.assertEqual(outfeed.hidden_submesh_names, ())

    def test_env_cfg_spawns_the_shroud_without_collision_or_rigid_props(self) -> None:
        """静态自检：遮挡罩必须走 AssetBaseCfg，且 spawn 不带任何物理属性。"""

        source = (_TASK_DIR / "conveyor_env_cfg.py").read_text(encoding="utf-8")
        start = source.index("def _make_conveyor_shroud_cfg(")
        factory = source[start : source.index("\ndef ", start + 1)]

        self.assertIn("def _make_conveyor_shroud_cfg() -> AssetBaseCfg:", factory)
        self.assertNotIn("RigidObjectCfg", factory)
        # docstring 里会解释"刻意不传 collision_props/rigid_props"，只查实际代码。
        code = factory[factory.index("return AssetBaseCfg(") :]
        self.assertNotIn("collision_props", code)
        self.assertNotIn("rigid_props", code)


if __name__ == "__main__":
    unittest.main()
