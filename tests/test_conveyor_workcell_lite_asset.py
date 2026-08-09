from __future__ import annotations

import hashlib
import json
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
_ASSET_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets"
_MANIFEST_PATH = _ASSET_DIR / "conveyor_workcell_lite.manifest.json"
_MANIFEST = json.loads(_MANIFEST_PATH.read_text(encoding="utf-8"))
_LAYER_PATH = _ASSET_DIR / _MANIFEST["output_asset"]
_ENV_CFG_PATH = (
    _REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
)


class ConveyorWorkcellLiteAssetTest(unittest.TestCase):
    def test_source_fingerprint_and_inventory_contract_are_versioned(self) -> None:
        source = _ASSET_DIR / _MANIFEST["source_asset"]
        digest = hashlib.sha256(source.read_bytes()).hexdigest()

        self.assertEqual(digest, _MANIFEST["source_asset_sha256"])
        # 2569 = 3017 - 300(BA01 纸箱垛) - 148(TB04 料筐垛)：clean wrapper 用
        # active=false 把这两垛靠墙装饰物移出组合。TB04 是 148 而非 50×3=150，
        # 第 8 列只有 L0——按实际存在枚举，不用笛卡尔积。
        self.assertEqual(_MANIFEST["source_root_child_count"], 2569)
        self.assertIn("ConveyorBelt", _MANIFEST["keep_root_exact"])
        self.assertEqual(
            _MANIFEST["omit_pseudoroot_prims"],
            ["/PhysicsScene", "/Render", "/NavMesh"],
        )

    def test_layer_is_a_small_selective_reference_usd(self) -> None:
        layer = _LAYER_PATH.read_text(encoding="utf-8")

        self.assertTrue(layer.startswith("#usda 1.0\n"))
        self.assertLess(_LAYER_PATH.stat().st_size, 32 * 1024)
        self.assertNotIn("subLayers", layer)
        self.assertNotIn("active = false", layer)
        self.assertEqual(layer.count("prepend references = @./warehouse-simple6_v61_visual_only.usda@</Root/"), 63)

    def test_required_workcell_roots_are_referenced(self) -> None:
        layer = _LAYER_PATH.read_text(encoding="utf-8")

        for name in (
            "ConveyorBelt",
            "GroundPlane",
            "SM_floor47",
            "SM_floor58",
            "FloorZone_Robot",
            "FloorZone_KeepClear",
            "Stripe_Walk1",
            "Stripe_Walk2",
            "Stripe_Conv1",
        ):
            self.assertIn(f"</Root/{name}>", layer)
        self.assertIn("</Root/SM_WallA_", layer)

    def test_heavy_and_runtime_only_subtrees_are_absent(self) -> None:
        layer = _LAYER_PATH.read_text(encoding="utf-8")

        for name in (
            "Camera",
            "PhysicsScene",
            "RenderProduct",
            "RenderSettings",
            "RenderVar",
            "NavMesh",
            "DistantLight",
            "RectLight",
            "SM_LampCeilingA",
            "SM_Rack",
            "KLT_Bins",
            "forklift",
            "SM_PushcartA",
            "WorkTable",
        ):
            self.assertNotIn(name, layer)

    def test_task_disables_inherited_dome_light(self) -> None:
        source = _ENV_CFG_PATH.read_text(encoding="utf-8")
        scene_class = source.split("class G129SonicConveyorSceneCfg", 1)[1]
        scene_class = scene_class.split("class G129SonicConveyorEventCfg", 1)[0]
        self.assertIn("\n    light = None\n", scene_class)

    def test_audit_contract_requires_material_composition_reduction(self) -> None:
        limits = _MANIFEST["audit_limits"]

        self.assertLessEqual(limits["max_lite_active_prims"], 2000)
        self.assertLessEqual(limits["max_lite_used_layers"], 100)
        # 相对削减量随源自身瘦身而下降（清理 BA01+TB04 后 Kit 实测 22168），是次要
        # 约束，留足余量即可；防止白名单被误扩的硬约束是 max_lite_active_prims。
        self.assertGreaterEqual(limits["min_active_prim_reduction"], 20000)
        self.assertGreaterEqual(limits["min_used_layer_reduction"], 1600)
        self.assertTrue(_MANIFEST["remote_dependency_prefix"].startswith("https://"))


if __name__ == "__main__":
    unittest.main()
