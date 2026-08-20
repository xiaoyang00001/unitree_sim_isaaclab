from __future__ import annotations

import importlib.util
import re
import sys
import tempfile
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_PROPS_DIR = _TASK_DIR / "scene_assets/props"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_TOTE = _load_module("conveyor_tote_assets_for_test", _TASK_DIR / "tote_assets.py")
_CONTACT = _load_module(
    "conveyor_contact_modes_for_test", _TASK_DIR / "contact_modes.py"
)


class ToteCompoundAssetTest(unittest.TestCase):
    def test_compound_is_default_and_legacy_remains_available(self) -> None:
        mode, path = _TOTE.resolve_tote_asset(_PROPS_DIR, {})
        self.assertEqual(mode, "compound")
        self.assertEqual(path.name, "tote_b04_compound_physics.usda")

        mode, path = _TOTE.resolve_tote_asset(
            _PROPS_DIR, {_TOTE.TOTE_COLLIDER_ENV: " convex_decomposition "}
        )
        self.assertEqual(mode, "convex_decomposition")
        self.assertEqual(path.name, "tote_b04_physics.usda")

    def test_unknown_or_missing_tote_asset_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "可选值"):
            _TOTE.resolve_tote_asset(
                _PROPS_DIR, {_TOTE.TOTE_COLLIDER_ENV: "mesh"}
            )
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                _TOTE.resolve_tote_asset(Path(directory), {})

    def test_compound_has_exactly_five_open_container_boxes(self) -> None:
        text = (_PROPS_DIR / "tote_b04_compound_physics.usda").read_text(
            encoding="utf-8"
        )
        names = re.findall(r'^\s*def Cube "([^"]+)"', text, re.MULTILINE)
        self.assertEqual(
            names,
            [
                "Bottom",
                "WallPositiveX",
                "WallNegativeX",
                "WallPositiveY",
                "WallNegativeY",
            ],
        )
        self.assertEqual(text.count('prepend apiSchemas = ["PhysicsCollisionAPI"'), 5)
        self.assertNotIn("convexDecomposition", text)
        self.assertIn('over "Tote_B04_01"', text)
        self.assertIn("bool physics:collisionEnabled = false", text)
        self.assertIn('token visibility = "invisible"', text)

    def test_box_dimensions_cover_outer_shell_but_leave_top_open(self) -> None:
        text = (_PROPS_DIR / "tote_b04_compound_physics.usda").read_text(
            encoding="utf-8"
        )
        expected = {
            "Bottom": ((60.0, 40.0, 2.5), (0.0, 0.0, 1.25)),
            "WallPositiveX": ((2.5, 40.0, 30.0), (28.75, 0.0, 15.0)),
            "WallNegativeX": ((2.5, 40.0, 30.0), (-28.75, 0.0, 15.0)),
            "WallPositiveY": ((55.0, 2.5, 30.0), (0.0, 18.75, 15.0)),
            "WallNegativeY": ((55.0, 2.5, 30.0), (0.0, -18.75, 15.0)),
        }
        for name, (scale, translation) in expected.items():
            block = re.search(
                rf'def Cube "{name}".*?double3 xformOp:scale = \(([^)]+)\)'
                rf'.*?double3 xformOp:translate = \(([^)]+)\)',
                text,
                re.DOTALL,
            )
            self.assertIsNotNone(block, name)
            assert block is not None
            parsed_scale = tuple(float(item.strip()) for item in block.group(1).split(","))
            parsed_translation = tuple(
                float(item.strip()) for item in block.group(2).split(",")
            )
            self.assertEqual(parsed_scale, scale)
            self.assertEqual(parsed_translation, translation)


class ContactReportModeTest(unittest.TestCase):
    def test_ankles_is_default_and_all_modes_are_explicit(self) -> None:
        self.assertEqual(_CONTACT.resolve_contact_report_mode({}), "ankles")
        for mode in _CONTACT.CONTACT_REPORT_MODES:
            self.assertEqual(
                _CONTACT.resolve_contact_report_mode(
                    {_CONTACT.CONTACT_REPORT_MODE_ENV: f" {mode.upper()} "}
                ),
                mode,
            )

    def test_unknown_contact_mode_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "可选值"):
            _CONTACT.resolve_contact_report_mode(
                {_CONTACT.CONTACT_REPORT_MODE_ENV: "feet-ish"}
            )

    def test_spawner_contract_targets_only_ankle_roll_bodies(self) -> None:
        source = (_TASK_DIR / "contact_spawners.py").read_text(encoding="utf-8")
        self.assertIn('_ANKLE_SUFFIX = "_ankle_roll_link"', source)
        self.assertIn("if len(matched) != 2", source)
        self.assertIn("spawn_cfg.activate_contact_sensors = False", source)

    def test_peer_urdf_fallback_cannot_restore_full_body_reporters(self) -> None:
        source = (_TASK_DIR / "conveyor_env_cfg.py").read_text(encoding="utf-8")
        fallback = source.split("else:\n        # 回退：URDF 直转", maxsplit=1)[1]
        fallback = fallback.split("    return cfg", maxsplit=1)[0]
        self.assertIn("cfg.spawn.activate_contact_sensors = False", fallback)


class ContactHistoryLengthTest(unittest.TestCase):
    def test_history_four_is_default_and_zero_is_explicit_candidate(self) -> None:
        self.assertEqual(_CONTACT.resolve_contact_history_length({}), 4)
        for length in (0, 4):
            self.assertEqual(
                _CONTACT.resolve_contact_history_length(
                    {_CONTACT.CONTACT_HISTORY_LENGTH_ENV: f" {length} "}
                ),
                length,
            )

    def test_noncanonical_or_unsupported_history_fails_fast(self) -> None:
        for value in ("", "-1", "1", "2", "3", "5", "4.0", "true"):
            with self.subTest(value=value), self.assertRaisesRegex(
                ValueError, "可选值: 0, 4"
            ):
                _CONTACT.resolve_contact_history_length(
                    {_CONTACT.CONTACT_HISTORY_LENGTH_ENV: value}
                )

    def test_scene_wires_history_without_weakening_contact_contract(self) -> None:
        source = (_TASK_DIR / "conveyor_env_cfg.py").read_text(encoding="utf-8")
        sensor_factory = source.split(
            "def _make_foot_contact_sensor", maxsplit=1
        )[1].split("def _make_peer_visual_lod_cfg", maxsplit=1)[0]
        self.assertIn("history_length=CONTACT_HISTORY_LENGTH", sensor_factory)
        self.assertIn("track_air_time=True", sensor_factory)
        self.assertIn("force_threshold=5.0", sensor_factory)
        self.assertIn("debug_vis=False", sensor_factory)

        env_post_init = source.split(
            "class G129SonicConveyorEnvCfg", maxsplit=1
        )[1].split("# EOF", maxsplit=1)[0]
        self.assertIn(
            'CONTACT_REPORT_MODE != "off"',
            env_post_init,
        )
        self.assertIn("CONTACT_HISTORY_LENGTH == 0", env_post_init)
        self.assertIn("self.scene.lazy_sensor_update is not True", env_post_init)
        self.assertIn("ContactSensor effective history", source)

    def test_off_mode_bypasses_lazy_history_gate_and_all_hosts_share_factory(self) -> None:
        source = (_TASK_DIR / "conveyor_env_cfg.py").read_text(encoding="utf-8")
        env_post_init = source.split(
            "class G129SonicConveyorEnvCfg", maxsplit=1
        )[1].split("# EOF", maxsplit=1)[0]
        gate = re.search(
            r"if \(\s*CONTACT_REPORT_MODE != \"off\"\s*"
            r"and CONTACT_HISTORY_LENGTH == 0\s*"
            r"and self\.scene\.lazy_sensor_update is not True\s*\):",
            env_post_init,
        )
        self.assertIsNotNone(gate)

        scene_cfg = source.split(
            "class G129SonicConveyorSceneCfg", maxsplit=1
        )[1].split("# MDP", maxsplit=1)[0]
        for suffix in ("", "2", "3", "4", "5"):
            with self.subTest(robot=suffix or "1"):
                self.assertIn(
                    f'_make_foot_contact_sensor("Robot{suffix}")', scene_cfg
                )
        self.assertEqual(scene_cfg.count("_make_foot_contact_sensor("), 5)


if __name__ == "__main__":
    unittest.main()
