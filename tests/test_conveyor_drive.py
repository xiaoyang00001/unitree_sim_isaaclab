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
            {}, object_authority=False, mirror_objects=False, default_y_stop=17.698
        )

        self.assertEqual(config.mode, "legacy")
        self.assertTrue(config.legacy_enabled)
        self.assertFalse(config.surface_velocity_enabled)
        self.assertFalse(config.surface_recycle_enabled)
        self.assertEqual(config.velocity_y, -0.3)

    def test_surface_velocity_is_strictly_gated_to_object_authority(self) -> None:
        environ = {"ISAACLAB_CONVEYOR_DRIVE_MODE": "surface_velocity"}

        authority = resolve_conveyor_drive(
            environ, object_authority=True, mirror_objects=False, default_y_stop=17.698
        )
        standalone_id2 = resolve_conveyor_drive(
            environ, object_authority=False, mirror_objects=False, default_y_stop=17.698
        )
        mirrored_id2 = resolve_conveyor_drive(
            environ, object_authority=False, mirror_objects=True, default_y_stop=17.698
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
                    default_y_stop=17.698,
                )
                self.assertFalse(
                    config.legacy_enabled and config.surface_velocity_enabled
                )

    def test_default_surface_split_preserves_total_belt_and_stop_center(self) -> None:
        config = resolve_conveyor_drive(
            {"ISAACLAB_CONVEYOR_DRIVE_MODE": "surface_velocity"},
            object_authority=True,
            mirror_objects=False,
            default_y_stop=17.698,
        )

        self.assertIsNotNone(config.stop_segment)
        assert config.stop_segment is not None
        self.assertAlmostEqual(config.stop_segment.y_min, BELT_Y_MIN)
        self.assertAlmostEqual(config.stop_segment.y_max, 17.798)
        self.assertAlmostEqual(config.drive_segment.y_min, 17.798)
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
            default_y_stop=17.698,
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
            default_y_stop=15.05,
            default_handoff_offset=0.20,
        )

        self.assertEqual(config.handoff_offset, 0.20)
        self.assertAlmostEqual(config.drive_segment.y_min, 15.25)

    def test_invalid_mode_and_out_of_range_partition_fail_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "legacy\\|surface_velocity"):
            resolve_conveyor_drive(
                {"ISAACLAB_CONVEYOR_DRIVE_MODE": "warp"},
                object_authority=True,
                mirror_objects=False,
                default_y_stop=17.698,
            )
        with self.assertRaisesRegex(ValueError, "停止分区超出"):
            resolve_conveyor_drive(
                {"ISAACLAB_CONVEYOR_Y_STOP": "21.75"},
                object_authority=True,
                mirror_objects=False,
                default_y_stop=17.698,
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
                        default_y_stop=17.698,
                    )


class ConveyorNorthShiftTest(unittest.TestCase):
    """流水线整体北移 Δ 的真源断言（几何位移写在背景 clean wrapper 里）。"""

    def test_delta_matches_the_wall_anchored_scanner_derivation(self) -> None:
        # 墙面 23.606 − 墙缝 0.05 − 放大后半长 1.27424*1.40 = 机身中心 21.772064；
        # 北移前带体实测到 18.2223 ⇒ Δ = 3.54966，取 3.55。
        self.assertAlmostEqual(CONVEYOR_NORTH_SHIFT_Y, 3.55, places=6)
        machine_centre_y = 23.606 - 0.05 - 1.27424 * 1.40
        self.assertAlmostEqual(machine_centre_y, 21.772064, places=6)
        self.assertLess(abs((18.2223 + CONVEYOR_NORTH_SHIFT_Y) - machine_centre_y), 0.001)

    def test_belt_constants_are_the_measured_base_plus_delta(self) -> None:
        self.assertAlmostEqual(_MODULE.BELT_Y_MIN_BASE, 10.19, places=6)
        self.assertAlmostEqual(_MODULE.BELT_Y_MAX_BASE, 18.22, places=6)
        self.assertAlmostEqual(BELT_Y_MIN, 13.74, places=6)
        self.assertAlmostEqual(BELT_Y_MAX, 21.77, places=6)
        # 带长不随整体平移改变。
        self.assertAlmostEqual(BELT_Y_MAX - BELT_Y_MIN, 8.03, places=6)

    def test_loop_mode_recycle_and_respawn_defaults_follow_the_shift(self) -> None:
        config = resolve_conveyor_drive(
            {"ISAACLAB_CONVEYOR_Y_STOP": "0"},
            object_authority=True,
            mirror_objects=False,
            default_y_stop=17.698,
        )

        self.assertAlmostEqual(config.y_recycle, 14.15, places=6)
        self.assertAlmostEqual(config.y_respawn, 21.55, places=6)
        self.assertGreater(config.y_recycle, BELT_Y_MIN)
        self.assertLess(config.y_respawn, BELT_Y_MAX)

    def test_the_background_layer_authors_the_same_delta(self) -> None:
        """位移写在 clean wrapper 的 over "ConveyorBelt"；数值必须与常量一致。

        ⚠️ 世界 y +Δ ≡ 背景局部 x +Δ（背景挂载时绕 Z 转 +90°），且 z=0.58 是整组
        既有抬升量，写丢会让整条线连同工位家具下沉。
        """

        layer = (
            Path(__file__).resolve().parents[1]
            / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets"
            / "warehouse-simple6_v61_visual_only.usda"
        ).read_text(encoding="utf-8")

        self.assertIn(
            f"double3 xformOp:translate = ({CONVEYOR_NORTH_SHIFT_Y:g}, 0, 0.58)", layer
        )
        self.assertIn(
            'uniform token[] xformOpOrder = ["xformOp:translate", '
            '"xformOp:orient", "xformOp:scale"]',
            layer,
        )
        # 地贴是 ConveyorBelt 的兄弟，不会被组平移带走，必须各自 +Δ。
        self.assertIn(
            f'over "FloorZone_KeepClear"\n    {{\n        double3 xformOp:translate '
            f'= ({CONVEYOR_NORTH_SHIFT_Y:g}, 0.94, 0.01)',
            layer,
        )
        self.assertIn(f"({4.5 + CONVEYOR_NORTH_SHIFT_Y:g}, 1.5, 0.01)", layer)
        self.assertIn(f"({1.0 + CONVEYOR_NORTH_SHIFT_Y:g}, 2, 0.01)", layer)


if __name__ == "__main__":
    unittest.main()
