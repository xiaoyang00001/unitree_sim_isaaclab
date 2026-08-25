from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


try:  # pxr（usd-core）不需要 Kit，但缺失时也不该让整组选择器测试挂掉。
    from pxr import Gf, Usd, UsdGeom

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

    def test_blue_sorting_bin_02_authors_in_place_half_turn(self) -> None:
        """02 应在叶子 mesh 自身原点翻转，不能旋转偏置很大的根 Prim。"""

        layer = (_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda").read_text(
            encoding="utf-8"
        )
        bin_02 = layer.split('over "blue_sorting_bin_02"', 1)[1]

        # 根矩阵保留 v48 的原始平移与缩放，避免可见料箱被甩离工作位。
        self.assertIn(
            "(-1.9383017274054972, -0.35331139354023133, "
            "-0.24833856360426954, 1.0)",
            bin_02,
        )
        # v61 叶子原角度 179.65359452632015°，精确减去 180°。
        self.assertEqual(
            layer.count(
                "double3 xformOp:rotateXYZ = (0.0, 0.0, -0.34640547367985)"
            ),
            1,
        )

    def test_robot_2_workcell_authors_robot_relative_half_turn(self) -> None:
        """robot_2 的支撑台和 bin_01 应共用机器人相对半周变换。"""

        layer = (_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda").read_text(
            encoding="utf-8"
        )
        table_03 = layer.split('over "SM_HeavyDutyPackingTable_C02_03"', 1)[1].split(
            'over "blue_sorting_bin_01"', 1
        )[0]
        bin_01 = layer.split('over "blue_sorting_bin_01"', 1)[1].split(
            'over "blue_sorting_bin_02"', 1
        )[0]

        self.assertIn(
            "double3 xformOp:translate = (-0.6452017853480621, "
            "2.086707303107458, -0.577029550733144)",
            table_03,
        )
        self.assertIn(
            "float3 xformOp:rotateXYZ = (0.0, 0.0, 180.0)",
            table_03,
        )
        self.assertIn(
            "(-7.311237476209021, 12.802697097255098, "
            "-0.6416343162044926, 1.0)",
            bin_01,
        )
        self.assertNotIn(
            "double3 xformOp:rotateXYZ = (0.0, 0.0, 180.0)",
            bin_01,
        )

    @unittest.skipUnless(_HAS_PXR, "需要 pxr（usd-core）做离线组合审计")
    def test_robot_2_workcell_switches_sides_with_clearance(self) -> None:
        """桌箱应相对 robot_2 换边，且箱底贴桌、不侵入流水线。"""

        stage = Usd.Stage.Open(
            str(_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda"),
            Usd.Stage.LoadNone,
        )
        self.assertIsNotNone(stage)
        stage.Load("/Root/ConveyorBelt")
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        )

        def bounds(name: str) -> Gf.Range3d:
            prim = stage.GetPrimAtPath(f"/Root/ConveyorBelt/{name}")
            self.assertTrue(prim, name)
            return cache.ComputeWorldBound(prim).ComputeAlignedRange()

        robot_2_table = bounds("SM_HeavyDutyPackingTable_C02_03")
        robot_2_bin = bounds("blue_sorting_bin_01")
        table_center = robot_2_table.GetMidpoint()
        bin_center = robot_2_bin.GetMidpoint()

        for axis in range(2):
            self.assertAlmostEqual(table_center[axis], bin_center[axis], places=9)
        self.assertAlmostEqual(
            robot_2_table.GetMax()[2], robot_2_bin.GetMin()[2], places=9
        )

        # 背景挂载在 world=(-4.68, 14.39363) 并绕 Z +90°：
        # world_x=-4.68-asset_y，world_y=14.39363+asset_x。
        world_center = Gf.Vec3d(
            -4.68 - bin_center[1], 14.39363 + bin_center[0], bin_center[2]
        )
        expected_world_center = Gf.Vec3d(
            -6.766707303107462, 13.998428214651938, 0.6152765818312004
        )
        for actual, expected in zip(world_center, expected_world_center):
            self.assertAlmostEqual(actual, expected, places=9)
        self.assertAlmostEqual(
            15.148 - (14.39363 + robot_2_bin.GetMax()[0]),
            0.4514505780354008,
            places=9,
        )

        belt_west_asset_y = max(
            bounds(name).GetMax()[1]
            for name in (
                "ConveyorBelt_A08_06",
                "ConveyorBelt_A08_07",
                "ConveyorBelt_A08_08",
            )
        )
        self.assertGreater(robot_2_table.GetMin()[1] - belt_west_asset_y, 0.05)
        self.assertGreater(robot_2_bin.GetMin()[1] - belt_west_asset_y, 0.02)

    @unittest.skipUnless(_HAS_PXR, "需要 pxr（usd-core）做离线组合审计")
    def test_sorting_bins_open_toward_their_robots(self) -> None:
        """robot_2 桌箱换边后，两只料箱的缺口仍分别朝向所属机器人。"""

        stage = Usd.Stage.Open(
            str(_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda"),
            Usd.Stage.LoadNone,
        )
        self.assertIsNotNone(stage)
        stage.Load("/Root/ConveyorBelt")
        cache = UsdGeom.XformCache()

        def mesh_pose(name: str) -> tuple[Gf.Vec3d, Gf.Vec3d]:
            path = (
                f"/Root/ConveyorBelt/{name}/{name}/Geometry/"
                "sm_bin_20x25x05cm_a01_01/"
                "sm_bin_a05_20x25x05cm_pr_v_nvd_a01_01"
            )
            prim = stage.GetPrimAtPath(path)
            self.assertTrue(prim, path)
            transform = cache.GetLocalToWorldTransform(prim)
            origin = transform.Transform(Gf.Vec3d(0.0))
            opening = transform.TransformDir(Gf.Vec3d(0.0, -1.0, 0.0)).GetNormalized()
            return origin, opening

        origin_01, opening_01 = mesh_pose("blue_sorting_bin_01")
        origin_02, opening_02 = mesh_pose("blue_sorting_bin_02")

        robot_2 = Gf.Vec3d(0.75437, 2.02, 0.0)
        robot_1 = Gf.Vec3d(0.00437, -0.14, 0.0)
        toward_robot_2 = Gf.Vec3d(
            robot_2[0] - origin_01[0], robot_2[1] - origin_01[1], 0.0
        ).GetNormalized()
        toward_robot_1 = Gf.Vec3d(
            robot_1[0] - origin_02[0], robot_1[1] - origin_02[1], 0.0
        ).GetNormalized()

        self.assertLess(opening_01 * opening_02, -0.9999)
        self.assertGreater(opening_01 * toward_robot_2, 0.998)
        self.assertGreater(opening_02 * toward_robot_1, 0.998)
        expected_origin_01 = Gf.Vec3d(
            -0.3952017853480603, 2.0867072621063463, 0.4312765846209089
        )
        expected_origin_02 = Gf.Vec3d(
            1.1539417853480611, -0.20670726210634566, 0.43127658462090845
        )
        for actual, expected in zip(origin_01, expected_origin_01):
            self.assertAlmostEqual(actual, expected, places=9)
        for actual, expected in zip(origin_02, expected_origin_02):
            self.assertAlmostEqual(actual, expected, places=9)

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
