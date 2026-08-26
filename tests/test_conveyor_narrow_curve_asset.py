"""A02 窄弯道资源、中心线形变和场景接线的零 Kit 契约测试。"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_ASSET_DIR = _TASK_DIR / "scene_assets"
_MANIFEST_PATH = _ASSET_DIR / "conveyor_belt_a02_narrow_visual.manifest.json"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_EI = _load("conveyor_endless_intake_for_narrow_curve_test", _TASK_DIR / "endless_intake.py")
_GENERATOR = _load(
    "conveyor_narrow_curve_generator_for_test",
    _REPO_ROOT / "tools/build_narrow_conveyor_curve.py",
)
_MANIFEST = json.loads(_MANIFEST_PATH.read_text(encoding="utf-8"))


class _Vec3d(tuple):
    def __new__(cls, *values: float):
        return super().__new__(cls, values)


class _FakeGf:
    Vec3d = _Vec3d


class NarrowCurveAssetContractTest(unittest.TestCase):
    def test_manifest_and_generated_asset_are_present(self) -> None:
        self.assertEqual(_EI.CURVE_ASSET_FILENAME, _MANIFEST["output_asset"])
        asset_path = _ASSET_DIR / _EI.CURVE_ASSET_FILENAME
        self.assertTrue(asset_path.is_file())
        self.assertGreater(asset_path.stat().st_size, 1024 * 1024)
        self.assertEqual(_MANIFEST["source_mesh_count"], 57)
        self.assertEqual(_MANIFEST["source_point_count"], 145122)
        self.assertRegex(_MANIFEST["source_geometry_sha256"], r"^[0-9a-f]{64}$")

    def test_width_contract_is_090_to_060_metres(self) -> None:
        source = _MANIFEST["source_lane_width_source_units"]
        target = _MANIFEST["target_lane_width_source_units"]
        self.assertEqual((source, target), (90.0, 60.0))
        self.assertAlmostEqual(target / source, _EI.NARROW_WIDTH_SCALE, places=12)
        self.assertEqual(_EI.NARROW_LANE_WIDTH, 0.60)

    def test_runtime_uses_the_local_narrow_curve_wrapper(self) -> None:
        source = (_TASK_DIR / "conveyor_env_cfg.py").read_text(encoding="utf-8")
        self.assertIn(
            "_ENDLESS_CURVE_USD = str(_ASSETS_DIR / endless_intake.CURVE_ASSET_FILENAME)",
            source,
        )

    def test_deformation_keeps_centerline_and_scales_only_transverse_offset(self) -> None:
        spec = _GENERATOR._curve_spec(_MANIFEST)
        cx, cy, radius = spec.center_x, spec.center_y, spec.radius

        samples = (
            # 中心线本身不动。
            ((cx + 20.0, cy + radius, 76.93), (cx + 20.0, cy + radius, 76.93)),
            ((cx - radius, cy - 20.0, 76.93), (cx - radius, cy - 20.0, 76.93)),
            # 两条直线和圆弧的 45 cm 横向偏差都收成 30 cm。
            ((cx + 20.0, cy + radius + 45.0, 76.93), (cx + 20.0, cy + radius + 30.0, 76.93)),
            ((cx - radius + 45.0, cy - 20.0, 76.93), (cx - radius + 30.0, cy - 20.0, 76.93)),
        )
        for point, expected in samples:
            with self.subTest(point=point):
                actual = _GENERATOR._deform_root_point(point, spec, _FakeGf)
                for value, expected_value in zip(actual, expected, strict=True):
                    self.assertAlmostEqual(value, expected_value, places=9)

        theta = 3.0 * math.pi / 4.0
        unit = (math.cos(theta), math.sin(theta))
        arc_source = (
            cx + (radius + 45.0) * unit[0],
            cy + (radius + 45.0) * unit[1],
            76.93,
        )
        arc_expected = (
            cx + (radius + 30.0) * unit[0],
            cy + (radius + 30.0) * unit[1],
            76.93,
        )
        actual = _GENERATOR._deform_root_point(arc_source, spec, _FakeGf)
        for value, expected_value in zip(actual, arc_expected, strict=True):
            self.assertAlmostEqual(value, expected_value, places=9)

    def test_asset_centerline_maps_to_unchanged_runtime_path(self) -> None:
        center_x, center_y = _MANIFEST["centerline_source_units"]["corner_center_xy"]
        radius = _MANIFEST["centerline_source_units"]["radius"]
        scale = _EI.CONVEYOR_UNIT_SCALE

        # yaw=-90°：局部 (x, y) -> 世界 (t.x+y*s, t.y-x*s)。资产接口实测中线
        # 与路径真源仅保留历史 3 mm 装配偏差，收窄不再引入任何新偏移。
        world_corner = (
            _EI.CURVE_POS[0] + center_y * scale,
            _EI.CURVE_POS[1] - center_x * scale,
        )
        self.assertLess(abs(world_corner[0] - _EI.CORNER_CENTER[0]), 0.0031)
        self.assertAlmostEqual(world_corner[1], _EI.CORNER_CENTER[1], places=9)
        self.assertAlmostEqual(radius * scale, _EI.CORNER_RADIUS, places=9)

        main_lane_x = _EI.CURVE_POS[0] + (center_y + radius) * scale
        branch_lane_y = _EI.CURVE_POS[1] - (center_x - radius) * scale
        self.assertAlmostEqual(main_lane_x, _EI.BELT_HEAD_MALE_CENTER_X, places=9)
        self.assertAlmostEqual(branch_lane_y, _EI.BRANCH_LANE_Y, places=9)

    def test_world_aabb_matches_the_generated_narrow_mesh(self) -> None:
        self.assertEqual(
            _EI.CURVE_AABB,
            ((-7.1130, -5.2334), (18.4573, 20.3898), (0.003, 1.1693)),
        )


if __name__ == "__main__":
    unittest.main()
