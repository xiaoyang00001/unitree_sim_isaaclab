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
                {"ISAACLAB_CONVEYOR_Y_STOP": "18.2"},
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


if __name__ == "__main__":
    unittest.main()
