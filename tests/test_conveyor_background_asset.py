from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


try:  # pxr（usd-core）不需要 Kit，但缺失时也不该让整组选择器测试挂掉。
    from pxr import Usd, UsdGeom

    _HAS_PXR = True
except ModuleNotFoundError:  # pragma: no cover
    _HAS_PXR = False


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

    @unittest.skipUnless(_HAS_PXR, "需要 pxr（usd-core）做离线组合审计")
    def test_packing_table_top_is_not_floating_into_the_sorting_bin(self) -> None:
        """C02_01 的桌面板必须坐在桌体上，不能浮起来穿进分拣料箱。

        源资产 ConveyorBelt02.usd 给这块板单独 author 了 translate.z=23.9065（局部
        单位，× z 缩放 0.0043086922 = 0.103 m），于是它浮在桌体上方、从下面穿进
        blue_sorting_bin_02 十厘米。对照组 C02_03 的同名板 translate 是 (0,0,0)，
        四块几何完全对齐——0 才是正确值。clean wrapper 把 z 分量改回 0。
        """

        stage = Usd.Stage.Open(
            str(_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda"), Usd.Stage.LoadAll
        )
        self.assertIsNotNone(stage)
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        )

        def z_range(path: str) -> tuple[float, float]:
            prim = stage.GetPrimAtPath(path)
            self.assertTrue(prim, path)
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            self.assertFalse(box.IsEmpty(), path)
            return box.GetMin()[2], box.GetMax()[2]

        belt = "/Root/ConveyorBelt"
        top_01 = z_range(
            f"{belt}/SM_HeavyDutyPackingTable_C02_01/SM_HeavyDutyPackingTable_C02_01"
            "/Geometry/M_HeavyDutyPackingTable_C01_TableTop"
        )
        top_03 = z_range(
            f"{belt}/SM_HeavyDutyPackingTable_C02_03/SM_HeavyDutyPackingTable_C02_03"
            "/Geometry/M_HeavyDutyPackingTable_C01_TableTop"
        )
        bin_02 = z_range(f"{belt}/blue_sorting_bin_02")

        # 桌面板不得侵入料箱：板顶 ≤ 料箱底。
        self.assertLessEqual(
            top_01[1], bin_02[0] + 1e-4, f"桌面板顶 {top_01[1]} 穿进料箱底 {bin_02[0]}"
        )
        # 与对照组那张桌子的同名板等高。
        self.assertAlmostEqual(top_01[1], top_03[1], places=4)
        self.assertAlmostEqual(top_01[0], top_03[0], places=4)

    @unittest.skipUnless(_HAS_PXR, "需要 pxr（usd-core）做离线组合审计")
    def test_belt_decorations_are_removed_from_composition_entirely(self) -> None:
        """15 个传送带装饰物必须整体失活，而不只是去掉物理。

        它们的摆位是 v48→v61 换版遗留（纸箱底 z=0.633 对带面顶 0.772，陷 14 cm；
        料箱悬空 12~22 cm），任务又已经在同一条中线上 spawn 真刚体纸箱队列，
        留着就是视觉打架。物理清理保留在原地当双保险，见该层头部注释。
        """

        stage = Usd.Stage.Open(
            str(_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda")
        )
        self.assertIsNotNone(stage)

        decorations = [f"ConveyorBelt_Box_{index:02d}" for index in range(10)]
        decorations += [f"KLT_Bin_{index:02d}" for index in range(5)]
        for name in decorations:
            with self.subTest(name):
                prim = stage.GetPrimAtPath(f"/Root/{name}")
                self.assertTrue(prim, f"{name} 应仍被 author（只是失活）")
                self.assertFalse(prim.IsActive(), f"{name} 仍在组合里")

        # 默认遍历只走 active 分支——Kit 也是照这条路实例化的。
        still_composed = [
            prim.GetName()
            for prim in stage.Traverse()
            if prim.GetName().startswith(("ConveyorBelt_Box_", "KLT_Bin_"))
        ]
        self.assertEqual(still_composed, [])


if __name__ == "__main__":
    unittest.main()
