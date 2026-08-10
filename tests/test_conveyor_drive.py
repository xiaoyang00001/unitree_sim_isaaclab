from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_drive.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "conveyor_drive_for_test", _MODULE_PATH
)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _MODULE
_SPEC.loader.exec_module(_MODULE)

BELT_Y_MAX = _MODULE.BELT_Y_MAX
BELT_Y_MIN = _MODULE.BELT_Y_MIN
CONVEYOR_NORTH_SHIFT_Y = _MODULE.CONVEYOR_NORTH_SHIFT_Y
resolve_conveyor_drive = _MODULE.resolve_conveyor_drive


class ConveyorDriveConfigTest(unittest.TestCase):
    def test_legacy_is_default_and_preserves_authority_offline_behavior(self) -> None:
        config = resolve_conveyor_drive(
            {}, object_authority=False, mirror_objects=False, default_y_stop=14.148
        )

        self.assertEqual(config.mode, "legacy")
        self.assertTrue(config.legacy_enabled)
        self.assertFalse(config.surface_velocity_enabled)
        self.assertFalse(config.surface_recycle_enabled)
        self.assertEqual(config.velocity_y, -0.3)

    def test_surface_velocity_is_strictly_gated_to_object_authority(self) -> None:
        environ = {"ISAACLAB_CONVEYOR_DRIVE_MODE": "surface_velocity"}

        authority = resolve_conveyor_drive(
            environ, object_authority=True, mirror_objects=False, default_y_stop=14.148
        )
        standalone_id2 = resolve_conveyor_drive(
            environ, object_authority=False, mirror_objects=False, default_y_stop=14.148
        )
        mirrored_id2 = resolve_conveyor_drive(
            environ, object_authority=False, mirror_objects=True, default_y_stop=14.148
        )

        self.assertTrue(authority.surface_velocity_enabled)
        self.assertFalse(authority.legacy_enabled)
        self.assertFalse(standalone_id2.surface_velocity_enabled)
        self.assertFalse(mirrored_id2.surface_velocity_enabled)

    def test_backends_are_mutually_exclusive_in_all_modes(self) -> None:
        cases = (
            ({}, True, False),
            ({"ISAACLAB_CONVEYOR_DRIVE_MODE": "legacy"}, False, False),
            ({"ISAACLAB_CONVEYOR_DRIVE_MODE": "surface_velocity"}, True, False),
            ({"ISAACLAB_CONVEYOR_DRIVE_MODE": "surface_velocity"}, False, True),
            ({"ISAACLAB_CONVEYOR_ENABLED": "0"}, True, False),
        )
        for environ, authority, mirror in cases:
            with self.subTest(environ=environ, authority=authority, mirror=mirror):
                config = resolve_conveyor_drive(
                    environ,
                    object_authority=authority,
                    mirror_objects=mirror,
                    default_y_stop=14.148,
                )
                self.assertFalse(
                    config.legacy_enabled and config.surface_velocity_enabled
                )

    def test_default_surface_split_preserves_total_belt_and_stop_center(self) -> None:
        config = resolve_conveyor_drive(
            {"ISAACLAB_CONVEYOR_DRIVE_MODE": "surface_velocity"},
            object_authority=True,
            mirror_objects=False,
            default_y_stop=14.148,
        )

        self.assertIsNotNone(config.stop_segment)
        assert config.stop_segment is not None
        self.assertAlmostEqual(config.stop_segment.y_min, BELT_Y_MIN)
        self.assertAlmostEqual(config.stop_segment.y_max, 14.248)
        self.assertAlmostEqual(config.drive_segment.y_min, 14.248)
        self.assertAlmostEqual(config.drive_segment.y_max, BELT_Y_MAX)
        self.assertAlmostEqual(
            config.stop_segment.length + config.drive_segment.length,
            BELT_Y_MAX - BELT_Y_MIN,
        )

    def test_zero_stop_selects_full_length_loop_surface(self) -> None:
        config = resolve_conveyor_drive(
            {
                "ISAACLAB_CONVEYOR_DRIVE_MODE": "surface_velocity",
                "ISAACLAB_CONVEYOR_Y_STOP": "0",
            },
            object_authority=True,
            mirror_objects=False,
            default_y_stop=14.148,
        )

        self.assertIsNone(config.y_stop)
        self.assertIsNone(config.stop_segment)
        self.assertTrue(config.surface_recycle_enabled)
        self.assertEqual(config.drive_segment.y_min, BELT_Y_MIN)
        self.assertEqual(config.drive_segment.y_max, BELT_Y_MAX)

    def test_full_size_tote_can_use_a_larger_default_handoff(self) -> None:
        config = resolve_conveyor_drive(
            {},
            object_authority=True,
            mirror_objects=False,
            default_y_stop=11.5,
            default_handoff_offset=0.20,
        )

        self.assertEqual(config.handoff_offset, 0.20)
        self.assertAlmostEqual(config.drive_segment.y_min, 11.70)

    def test_invalid_mode_and_out_of_range_partition_fail_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "legacy\\|surface_velocity"):
            resolve_conveyor_drive(
                {"ISAACLAB_CONVEYOR_DRIVE_MODE": "warp"},
                object_authority=True,
                mirror_objects=False,
                default_y_stop=14.148,
            )
        with self.assertRaisesRegex(ValueError, "停止分区超出"):
            resolve_conveyor_drive(
                {"ISAACLAB_CONVEYOR_Y_STOP": "18.45"},
                object_authority=True,
                mirror_objects=False,
                default_y_stop=14.148,
            )

    def test_non_finite_drive_geometry_values_fail_fast(self) -> None:
        for name, value in (
            ("ISAACLAB_CONVEYOR_SPEED", "nan"),
            ("ISAACLAB_CONVEYOR_Y_STOP", "inf"),
            ("ISAACLAB_CONVEYOR_SURFACE_HANDOFF_OFFSET", "-inf"),
            ("ISAACLAB_CONVEYOR_Y_RECYCLE", "nan"),
            ("ISAACLAB_CONVEYOR_Y_RESPAWN", "inf"),
        ):
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, f"{name} 必须是有限数"):
                    resolve_conveyor_drive(
                        {name: value},
                        object_authority=True,
                        mirror_objects=False,
                        default_y_stop=14.148,
                    )


class ConveyorNorthShiftTest(unittest.TestCase):
    """流水线整体北移 Δ=0.25：X 支线越过货架排 B 所需，wrapper 是几何落地处。"""

    def test_delta_is_the_rack_row_b_bypass_shift(self) -> None:
        # Δ=0.25 由"支线机身南缘 19.4779 对排 B 端护板北缘 19.3946 留 ≥50mm"
        # 约束决定（最小可行 0.2167，取 50mm 整倍数）；带体世界 y[10.438, 18.472]。
        self.assertAlmostEqual(CONVEYOR_NORTH_SHIFT_Y, 0.25, places=6)

    def test_belt_constants_are_the_measured_base_plus_delta(self) -> None:
        self.assertAlmostEqual(_MODULE.BELT_Y_MIN_BASE, 10.19, places=6)
        self.assertAlmostEqual(_MODULE.BELT_Y_MAX_BASE, 18.22, places=6)
        self.assertAlmostEqual(BELT_Y_MIN, 10.44, places=6)
        self.assertAlmostEqual(BELT_Y_MAX, 18.47, places=6)
        # 带长不随整体平移改变。
        self.assertAlmostEqual(BELT_Y_MAX - BELT_Y_MIN, 8.03, places=6)

    def test_loop_mode_recycle_and_respawn_defaults_follow_the_shift(self) -> None:
        config = resolve_conveyor_drive(
            {"ISAACLAB_CONVEYOR_Y_STOP": "0"},
            object_authority=True,
            mirror_objects=False,
            default_y_stop=14.398,
        )

        self.assertAlmostEqual(config.y_recycle, 10.85, places=6)
        self.assertAlmostEqual(config.y_respawn, 18.25, places=6)
        self.assertGreater(config.y_recycle, BELT_Y_MIN)
        self.assertLess(config.y_respawn, BELT_Y_MAX)

    def test_the_background_layer_authors_the_same_belt_shift(self) -> None:
        """clean wrapper 必须把 Δ 写进 ConveyorBelt 组变换（世界 y +Δ ≡ 背景局部
        x +Δ），且按 v61 的 TRS-with-orient 形式回写并显式重述 xformOpOrder——
        只写 translate 不写 order 时 matrix/TRS 混写会静默失效（东拐版实测坑）。
        """

        layer = (
            Path(__file__).resolve().parents[1]
            / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets"
            / "warehouse-simple6_v61_visual_only.usda"
        ).read_text(encoding="utf-8")

        import re

        # 只看 over "ConveyorBelt" 自身的直接属性区（到第一个嵌套 over 为止）：
        # 组内子 prim（打包桌/桌面板修正等）本来就合法地 author 各自的 translate。
        head = re.search(
            r'over "ConveyorBelt"\n    \{\n(.*?)(?=        over "|    \})',
            layer,
            flags=re.S,
        )
        self.assertIsNotNone(head)
        body = head.group(1)
        shift = CONVEYOR_NORTH_SHIFT_Y
        self.assertIn(
            f"double3 xformOp:translate = ({shift:g}, 0, 0.58)", body
        )
        self.assertIn(
            'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]',
            body,
        )
        # 三条地面标线是 /Root 下的兄弟 prim，不跟组走，必须各自 +Δ。
        self.assertIn(f'over "FloorZone_KeepClear"\n    {{\n        double3 xformOp:translate = ({shift:g}, 0.94, 0.01)', layer)
        self.assertIn(f'over "FloorZone_Robot"\n    {{\n        double3 xformOp:translate = ({4.5 + shift:g}, 1.5, 0.01)', layer)
        self.assertIn(f'over "Stripe_Conv1"\n    {{\n        double3 xformOp:translate = ({1.0 + shift:g}, 2, 0.01)', layer)
        # 15 件带面装饰物（active=false 但保留回退路径）各自 +Δ：抽两件锚点验证。
        self.assertIn(f"double3 xformOp:translate = ({-2.5 + shift:g}, 0.937, 0.633)", layer)
        self.assertIn(f"double3 xformOp:translate = ({4.19 + shift:g}, 0.937, 0.633)", layer)


if __name__ == "__main__":
    unittest.main()
