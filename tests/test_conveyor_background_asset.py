from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_ASSETS_DIR = _TASK_DIR / "scene_assets"
_MODULE_PATH = _TASK_DIR / "background_assets.py"
_MODULE_SPEC = importlib.util.spec_from_file_location(
    "conveyor_background_assets_for_test", _MODULE_PATH
)
assert _MODULE_SPEC is not None and _MODULE_SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_MODULE_SPEC)
sys.modules[_MODULE_SPEC.name] = _MODULE
_MODULE_SPEC.loader.exec_module(_MODULE)


class ConveyorBackgroundAssetTest(unittest.TestCase):
    def test_visual_only_background_is_the_default(self) -> None:
        mode, path = _MODULE.resolve_background_asset(_ASSETS_DIR, {})

        self.assertEqual(mode, "visual_only")
        self.assertEqual(path.name, "warehouse-simple6_v61_visual_only.usda")
        self.assertTrue(path.is_file())

    def test_legacy_background_remains_available_for_ab(self) -> None:
        mode, path = _MODULE.resolve_background_asset(
            _ASSETS_DIR,
            {_MODULE.BACKGROUND_MODE_ENV: " legacy_v61 "},
        )

        self.assertEqual(mode, "legacy_v61")
        self.assertEqual(path.name, "warehouse-simple6_v61.usd")
        self.assertTrue(path.is_file())

    def test_workcell_lite_background_is_explicitly_opt_in(self) -> None:
        mode, path = _MODULE.resolve_background_asset(
            _ASSETS_DIR,
            {_MODULE.BACKGROUND_MODE_ENV: " workcell_lite "},
        )

        self.assertEqual(mode, "workcell_lite")
        self.assertEqual(path.name, "conveyor_workcell_lite.usd")
        self.assertTrue(path.is_file())

    def test_unknown_background_mode_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "可选值"):
            _MODULE.resolve_background_asset(
                _ASSETS_DIR,
                {_MODULE.BACKGROUND_MODE_ENV: "typo"},
            )

    def test_visual_only_layer_covers_all_dynamic_decorations(self) -> None:
        layer = (_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda").read_text(
            encoding="utf-8"
        )

        self.assertIn("@./warehouse-simple6_v61.usd@", layer)
        for index in range(10):
            self.assertEqual(layer.count(f'over "ConveyorBelt_Box_{index:02d}"'), 1)
        for index in range(5):
            self.assertEqual(layer.count(f'over "KLT_Bin_{index:02d}"'), 1)

        # 15 个根装饰物 + 15 个纸箱引用子 Prim 都显式去掉物理 API 并双重禁用。
        self.assertEqual(layer.count("delete apiSchemas"), 30)
        self.assertEqual(layer.count("bool physics:collisionEnabled = 0"), 30)
        self.assertEqual(layer.count("bool physics:rigidBodyEnabled = 0"), 30)


if __name__ == "__main__":
    unittest.main()
