from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_ASSET_DIR = _TASK_DIR / "scene_assets"
_VARIANT_MODULE_PATH = _TASK_DIR / "asset_variants.py"
_VARIANT_SPEC = importlib.util.spec_from_file_location(
    "conveyor_asset_variants_for_test", _VARIANT_MODULE_PATH
)
assert _VARIANT_SPEC is not None and _VARIANT_SPEC.loader is not None
_VARIANT_MODULE = importlib.util.module_from_spec(_VARIANT_SPEC)
sys.modules[_VARIANT_SPEC.name] = _VARIANT_MODULE
_VARIANT_SPEC.loader.exec_module(_VARIANT_MODULE)
_GENERATOR_MODULE_PATH = _REPO_ROOT / "tools/build_conveyor_visual_only_usd.py"
_GENERATOR_SPEC = importlib.util.spec_from_file_location(
    "build_conveyor_visual_only_usd_for_test", _GENERATOR_MODULE_PATH
)
assert _GENERATOR_SPEC is not None and _GENERATOR_SPEC.loader is not None
_GENERATOR_MODULE = importlib.util.module_from_spec(_GENERATOR_SPEC)
sys.modules[_GENERATOR_SPEC.name] = _GENERATOR_MODULE
_GENERATOR_SPEC.loader.exec_module(_GENERATOR_MODULE)


class ConveyorVisualOnlyAssetTest(unittest.TestCase):
    def test_original_asset_remains_the_default_and_visual_only_is_opt_in(self) -> None:
        resolve = _VARIANT_MODULE.resolve_conveyor_background_usd
        expected_baseline = (
            "warehouse-simple6_v61_visual_only.usda"
            if (_ASSET_DIR / "warehouse-simple6_v61_visual_only.usda").is_file()
            else "warehouse-simple6_v61.usd"
        )

        self.assertEqual(
            resolve(_ASSET_DIR, {}).name,
            expected_baseline,
        )
        self.assertEqual(
            resolve(
                _ASSET_DIR,
                {_VARIANT_MODULE.CONVEYOR_VISUAL_ONLY_ENV: "true"},
            ).name,
            "warehouse-simple6_v61_conveyor_visual_only.usda",
        )
        self.assertEqual(
            resolve(
                _ASSET_DIR,
                {_VARIANT_MODULE.CONVEYOR_VISUAL_ONLY_ENV: "0"},
            ).name,
            expected_baseline,
        )

    def test_explicit_baseline_is_preserved_until_visual_only_is_selected(self) -> None:
        resolve = _VARIANT_MODULE.resolve_conveyor_background_usd
        legacy = _ASSET_DIR / _VARIANT_MODULE.LEGACY_BACKGROUND_USD
        adapter = _ASSET_DIR / _VARIANT_MODULE.VISUAL_ONLY_BACKGROUND_USD

        self.assertEqual(resolve(_ASSET_DIR, {}, baseline_path=legacy), legacy)
        self.assertEqual(
            resolve(
                _ASSET_DIR,
                {_VARIANT_MODULE.CONVEYOR_VISUAL_ONLY_ENV: "1"},
                baseline_path=legacy,
            ),
            adapter,
        )

    def test_surface_override_forces_visual_only_even_when_env_disables_it(self) -> None:
        resolve = _VARIANT_MODULE.resolve_conveyor_background_usd
        legacy = _ASSET_DIR / _VARIANT_MODULE.LEGACY_BACKGROUND_USD

        self.assertEqual(
            resolve(
                _ASSET_DIR,
                {_VARIANT_MODULE.CONVEYOR_VISUAL_ONLY_ENV: "0"},
                baseline_path=legacy,
                drive_mode="surface_velocity",
            ).name,
            _VARIANT_MODULE.VISUAL_ONLY_BACKGROUND_USD,
        )

    def test_integrated_clean_background_rejects_stale_adapter(self) -> None:
        resolve = _VARIANT_MODULE.resolve_conveyor_background_usd
        with tempfile.TemporaryDirectory() as tmp:
            assets_dir = Path(tmp)
            (assets_dir / _VARIANT_MODULE.CLEAN_BACKGROUND_USD).write_text(
                "#usda 1.0\n", encoding="utf-8"
            )
            adapter = assets_dir / _VARIANT_MODULE.VISUAL_ONLY_BACKGROUND_USD
            adapter.write_text(
                "subLayers = [@./warehouse-simple6_v61.usd@]\n",
                encoding="utf-8",
            )
            enabled = {_VARIANT_MODULE.CONVEYOR_VISUAL_ONLY_ENV: "1"}

            with self.assertRaisesRegex(RuntimeError, "会绕过背景物理清理层"):
                resolve(assets_dir, enabled)

            adapter.write_text(
                "subLayers = [@./warehouse-simple6_v61_visual_only.usda@]\n",
                encoding="utf-8",
            )
            self.assertEqual(resolve(assets_dir, enabled), adapter)

    def test_manifest_pins_source_and_records_every_authored_override(self) -> None:
        source = _ASSET_DIR / "ConveyorBelt02.usd"
        layer = _ASSET_DIR / "ConveyorBelt02_visual_only.usda"
        manifest_path = _ASSET_DIR / "ConveyorBelt02_visual_only.manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        layer_text = layer.read_text(encoding="utf-8")

        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        self.assertEqual(manifest["source_sha256"], digest)
        self.assertEqual(manifest["source"], source.name)
        self.assertEqual(manifest["source_default_prim"], "/Root")
        self.assertEqual(len(manifest["rigid_body_overrides"]), 6)
        self.assertEqual(len(manifest["collision_overrides"]), 56)
        self.assertLess(layer.stat().st_size, 32 * 1024)

        self.assertEqual(
            layer_text,
            _GENERATOR_MODULE._render_visual_only_layer(manifest, 0.01, "Z"),
        )
        self.assertEqual(
            layer_text.count("bool physics:rigidBodyEnabled = false"),
            len(manifest["rigid_body_overrides"]),
        )
        self.assertEqual(
            layer_text.count("bool physics:collisionEnabled = false"),
            len(manifest["collision_overrides"]),
        )

    def test_warehouse_adapter_replaces_payload_without_editing_v61(self) -> None:
        adapter = _ASSET_DIR / "warehouse-simple6_v61_conveyor_visual_only.usda"
        text = adapter.read_text(encoding="utf-8")

        self.assertEqual(
            text,
            _GENERATOR_MODULE._render_warehouse_adapter(
                "ConveyorBelt02_visual_only.usda",
                _GENERATOR_MODULE._resolve_warehouse_base_name(_ASSET_DIR),
            ),
        )
        warehouse_base = _GENERATOR_MODULE._resolve_warehouse_base_name(_ASSET_DIR)
        self.assertIn(f"subLayers = [@./{warehouse_base}@]", text)
        self.assertIn("delete payload = @./ConveyorBelt02.usd@", text)
        self.assertIn(
            "prepend payload = @./ConveyorBelt02_visual_only.usda@",
            text,
        )
        self.assertTrue((_ASSET_DIR / warehouse_base).is_file())
        self.assertTrue((_ASSET_DIR / "ConveyorBelt02_visual_only.usda").is_file())


if __name__ == "__main__":
    unittest.main()
