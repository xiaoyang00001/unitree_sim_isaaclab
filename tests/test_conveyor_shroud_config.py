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

_ASSET_PATH = _TASK_DIR / "scene_assets/props" / _SHROUD.SHROUD_ASSET_FILENAME
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
    def test_conveyor_layout_puts_the_shroud_outside_the_infeed_port(self) -> None:
        placement = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        self.assertEqual(placement.port, "infeed")
        self.assertEqual(placement.rot, (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(placement.yaw_degrees, 0.0)
        self.assertAlmostEqual(placement.face_y, 18.30, places=5)
        self.assertAlmostEqual(placement.pos[0], -5.62, places=5)
        self.assertAlmostEqual(placement.pos[1], 19.57424, places=5)
        self.assertAlmostEqual(placement.pos[2], -0.079355, places=6)
        self.assertAlmostEqual(placement.y_span[0], 18.30, places=5)
        self.assertAlmostEqual(placement.y_span[1], 20.84848, places=5)

    def test_pushcart_layout_puts_the_shroud_outside_the_outfeed_port(self) -> None:
        placement = resolve_conveyor_shroud(
            {"ISAACLAB_TOTES_ON_CONVEYOR": "0"}, totes_on_conveyor=False
        )

        self.assertEqual(placement.port, "outfeed")
        # 绕 Z 转 180°，让机身同一个端面在两种布局下都朝着流水线。
        self.assertEqual(placement.rot, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(placement.yaw_degrees, 180.0)
        self.assertAlmostEqual(placement.face_y, 9.66, places=5)
        self.assertAlmostEqual(placement.pos[1], 8.38576, places=5)
        self.assertAlmostEqual(placement.y_span[0], 7.11152, places=5)
        self.assertAlmostEqual(placement.y_span[1], 9.66, places=5)

    def test_shroud_never_cuts_into_the_belt_body(self) -> None:
        infeed = resolve_conveyor_shroud({}, totes_on_conveyor=True)
        outfeed = resolve_conveyor_shroud({}, totes_on_conveyor=False)

        # 入料端带体到 18.222，末尾带尾 10.188：罩子必须整体在带体外侧。
        self.assertGreaterEqual(infeed.y_span[0], 18.23)
        self.assertLessEqual(outfeed.y_span[1], 10.188)

    def test_face_override_translates_the_whole_shroud(self) -> None:
        default = resolve_conveyor_shroud({}, totes_on_conveyor=True)
        moved = resolve_conveyor_shroud(
            {"ISAACLAB_CONVEYOR_SHROUD_FACE_Y": "18.60"}, totes_on_conveyor=True
        )

        self.assertAlmostEqual(moved.face_y, 18.60, places=5)
        self.assertAlmostEqual(moved.pos[1] - default.pos[1], 0.30, places=5)
        self.assertAlmostEqual(moved.y_span[0] - default.y_span[0], 0.30, places=5)
        self.assertEqual(moved.pos[0], default.pos[0])
        self.assertEqual(moved.pos[2], default.pos[2])

    def test_machine_roller_surface_is_sunk_onto_the_belt_plane(self) -> None:
        placement = resolve_conveyor_shroud({}, totes_on_conveyor=True)
        design_scale = _SHROUD.SHROUD_DESIGN_SCALE

        roller_top_world = (
            placement.pos[2] + _SHROUD.ASSET_ROLLER_TOP_Z * design_scale
        )
        self.assertAlmostEqual(roller_top_world, _DRIVE.BELT_TOP_Z, places=6)

    def test_curtain_opening_clears_the_belt_and_a_tote(self) -> None:
        placement = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        # 条帘下沿不能压进带面，上沿要给半尺寸料筐（高 0.15）留出净空。
        self.assertGreater(placement.opening_z[0], _DRIVE.BELT_TOP_Z)
        self.assertGreater(placement.opening_z[1] - _DRIVE.BELT_TOP_Z, 0.15)
        self.assertGreater(placement.opening_width_x, 0.30)

    def test_spawn_scale_carries_the_centimetre_to_metre_factor(self) -> None:
        placement = resolve_conveyor_shroud({}, totes_on_conveyor=True)

        expected = _SHROUD.SHROUD_UNIT_SCALE * _SHROUD.SHROUD_DESIGN_SCALE
        self.assertEqual(placement.scale, (expected, expected, expected))


class ConveyorShroudBeltConstantsTest(unittest.TestCase):
    """shroud_config 为了零相对 import 复制了带面常量，这里防止两边漂移。"""

    def test_belt_constants_match_conveyor_drive(self) -> None:
        self.assertEqual(_SHROUD.BELT_X_CENTER, _DRIVE.BELT_X_CENTER)
        self.assertEqual(_SHROUD.BELT_TOP_Z, _DRIVE.BELT_TOP_Z)
        self.assertEqual(_SHROUD.BELT_Y_MIN, _DRIVE.BELT_Y_MIN)
        self.assertEqual(_SHROUD.BELT_Y_MAX, _DRIVE.BELT_Y_MAX)


class ConveyorShroudAssetLayerTest(unittest.TestCase):
    def test_layer_exists_and_declares_the_stage_metadata(self) -> None:
        self.assertTrue(_ASSET_PATH.is_file(), _ASSET_PATH)
        text = _ASSET_PATH.read_text(encoding="utf-8")

        self.assertTrue(text.startswith("#usda 1.0"))
        for token in (
            'defaultPrim = "InfeedScannerVisual"',
            "metersPerUnit = 1",
            'upAxis = "Z"',
            "doc = ",
        ):
            self.assertIn(token, text)

    def test_layer_references_the_official_asset_over_s3(self) -> None:
        text = _ASSET_PATH.read_text(encoding="utf-8")

        self.assertIn(f"@{_SCANNER_S3_URL}@", text)
        # 本地资产包只是 S3 前缀的镜像，写死本地盘路径换机就断。
        self.assertNotIn("/media/", text)

    def test_layer_authors_no_physics_at_all(self) -> None:
        """被引用资产 coll=0 / rigid=0，薄层也不许自己加回来。

        只看 stage metadata 之后的实际 authored 内容，header 的 doc 里会提到这些
        schema 名字（说明的是"已确认没有"），不能拿来当命中。
        """

        text = _ASSET_PATH.read_text(encoding="utf-8")
        body = text.split('"""\n)', 1)[-1]

        for token in ("apiSchemas", "physics:", "physxCollision", "physxRigidBody"):
            self.assertNotIn(token, body)

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
