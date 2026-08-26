from __future__ import annotations

import importlib.util
import math
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

_LAYOUT_PATH = _TASK_DIR / "scene_layout.py"
_LAYOUT_SPEC = importlib.util.spec_from_file_location(
    "conveyor_scene_layout_for_background_test", _LAYOUT_PATH
)
assert _LAYOUT_SPEC is not None and _LAYOUT_SPEC.loader is not None
_LAYOUT_MODULE = importlib.util.module_from_spec(_LAYOUT_SPEC)
sys.modules[_LAYOUT_SPEC.name] = _LAYOUT_MODULE
_LAYOUT_SPEC.loader.exec_module(_LAYOUT_MODULE)

_WORKCELL_DECORATIVE_BOXES = tuple(
    f"CardBoxC_{row:02d}_{column:02d}" for row in range(3) for column in range(6)
) + tuple(
    f"CardBoxD_{row:02d}_{column:02d}" for row in range(4) for column in range(6)
)
_GROUND_REMINDER_PROPS = tuple(f"Cone_{index}" for index in range(1, 5))
_FLOOR_MARKERS = (
    "SM_FloorDecal_Keepclear6_446",
    "SM_FloorDecal_RecRed1X15",
    "SM_FloorDecal_RecRed1X16",
    *(f"SM_FloorDecal_StripeFull_4m{index}" for index in range(74, 82)),
    "FloorZone_KeepClear",
    "FloorZone_Robot",
    "Stripe_Walk1",
    "Stripe_Walk2",
    "Stripe_Conv1",
)


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

    def test_workcell_decorations_are_removed(self) -> None:
        """工位箱堆、四个交通锥和 16 个地面标识应退出 USD 组合。"""

        layer = (_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda").read_text(
            encoding="utf-8"
        )

        for name in (*_WORKCELL_DECORATIVE_BOXES, *_GROUND_REMINDER_PROPS, *_FLOOR_MARKERS):
            with self.subTest(name=name):
                self.assertEqual(layer.count(f'over "{name}" (active = false)'), 1)
        self.assertEqual(
            layer.count('over "warehouse_extras_01" (active = false)'),
            1,
        )

    def test_three_main_belt_segments_are_narrowed_around_their_centerline(self) -> None:
        layer = (_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda").read_text(
            encoding="utf-8"
        )

        self.assertEqual(layer.count("float3 xformOp:scale = (1, 0.6666667, 1)"), 3)
        for segment in ("ConveyorBelt_A08_06", "ConveyorBelt_A08_07", "ConveyorBelt_A08_08"):
            with self.subTest(segment=segment):
                block = layer.split(f'over "{segment}"', 1)[1]
                self.assertIn("float3 xformOp:scale = (1, 0.6666667, 1)", block)

    def test_robot_2_workcell_keeps_its_side_and_orientation_while_shifting(self) -> None:
        """robot_2 桌箱保留侧向/朝向，并作为一组向窄带内移、向下游下移。"""

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
            "double3 xformOp:translate = (1.3539417853480621, "
            "1.886707303107458, -0.577029550733144)",
            table_03,
        )
        self.assertIn(
            "float3 xformOp:rotateXYZ = (0.0, 0.0, 0.0)",
            table_03,
        )
        self.assertNotIn(
            "float3 xformOp:rotateXYZ = (0.0, 0.0, 180.0)",
            table_03,
        )
        self.assertIn(
            "(8.01997747620902, -8.82928249104018, "
            "-0.6416343162044926, 1.0)",
            bin_01,
        )

    def test_blue_sorting_bin_02_authors_in_place_half_turn(self) -> None:
        """02 应在叶子 mesh 自身原点翻转，不能旋转偏置很大的根 Prim。"""

        layer = (_ASSETS_DIR / "warehouse-simple6_v61_visual_only.usda").read_text(
            encoding="utf-8"
        )
        bin_02 = layer.split('over "blue_sorting_bin_02"', 1)[1]

        # 根矩阵局部 Y +0.20 m 靠近窄带、局部 X -0.30 m 移向世界 -Y；
        # 旋转/缩放和偏置补偿保持不变。
        self.assertIn(
            "(-2.2383017274054972, -0.15331139354023133, "
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

    @unittest.skipUnless(_HAS_PXR, "需要 pxr（usd-core）做离线组合审计")
    def test_both_workcell_tables_and_bins_follow_the_inward_and_downstream_shifts(self) -> None:
        """两套桌箱向内 0.20 m、向世界 -Y 0.30 m，并保持箱底完整坐在桌面上。"""

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

        def world_x_range(name: str) -> tuple[float, float]:
            local = bounds(name)
            # 背景挂载 pos=(-4.68, 14.39363) / yaw=+90°：world_x=-4.68-local_y。
            return -4.68 - local.GetMax()[1], -4.68 - local.GetMin()[1]

        def world_y_range(name: str) -> tuple[float, float]:
            local = bounds(name)
            return 14.39363 + local.GetMin()[0], 14.39363 + local.GetMax()[0]

        east_table = bounds("SM_HeavyDutyPackingTable_C02_01").GetMidpoint()
        east_bin = bounds("blue_sorting_bin_02").GetMidpoint()
        west_table = bounds("SM_HeavyDutyPackingTable_C02_03").GetMidpoint()
        west_bin = bounds("blue_sorting_bin_01").GetMidpoint()

        self.assertAlmostEqual(east_table[1], east_bin[1], places=6)
        self.assertAlmostEqual(west_table[1], west_bin[1], places=6)
        self.assertAlmostEqual(
            bounds("SM_HeavyDutyPackingTable_C02_01").GetMax()[2],
            bounds("blue_sorting_bin_02").GetMin()[2],
            places=9,
        )
        self.assertAlmostEqual(
            bounds("SM_HeavyDutyPackingTable_C02_03").GetMax()[2],
            bounds("blue_sorting_bin_01").GetMin()[2],
            places=9,
        )
        self.assertAlmostEqual(west_bin[0] - east_bin[0], 0.75, places=6)

        east_bin_world_x = -4.68 - east_bin[1]
        west_bin_world_x = -4.68 - west_bin[1]
        self.assertAlmostEqual(east_bin_world_x, -4.673292696892543, places=6)
        self.assertAlmostEqual(west_bin_world_x, -6.566707303107458, places=6)
        # 两只收纳箱恢复到各自机器人外侧约 66.7 mm，而不是窄带改造后遗留的 266.7 mm。
        self.assertAlmostEqual(east_bin_world_x - (-4.74), 0.066707303107457, places=6)
        self.assertAlmostEqual(-6.50 - west_bin_world_x, 0.066707303107458, places=6)

        east_bin_world_y = 14.39363 + east_bin[0]
        west_bin_world_y = 14.39363 + west_bin[0]
        self.assertAlmostEqual(east_bin_world_y, 15.24757178534806, places=6)
        self.assertAlmostEqual(west_bin_world_y, 15.997571785348062, places=6)
        # 桌箱与对应机器人刚性同移，仍保持约 1.1496 m 的纵向关系。
        self.assertAlmostEqual(east_bin_world_y - 14.098, 1.14957178534806, places=6)
        self.assertAlmostEqual(west_bin_world_y - 14.848, 1.149571785348062, places=6)
        west_near_y, _west_far_y = world_y_range("blue_sorting_bin_01")
        self.assertLess(west_near_y, 15.30)

        # 从组合后的 A08_07 网格直接抽取 z>0.85 m 的北端高架，再按 D02 半尺寸做
        # Minkowski 膨胀。紧凑的 0.30 m 布局不支持直接斜送：robot_2 必须先向西侧
        # 回撤到 x≈-6.25，再转向收纳箱；该折线路径应保留至少 0.20 m 正交净距。
        frame_mesh_path = (
            "/Root/ConveyorBelt/ConveyorBelt_A08_07/ConveyorBelt_A08_07/"
            "SM_ConveyorBelt_A08_02"
        )
        frame_mesh_prim = stage.GetPrimAtPath(frame_mesh_path)
        self.assertTrue(frame_mesh_prim, frame_mesh_path)
        frame_mesh = UsdGeom.Mesh(frame_mesh_prim)
        frame_xform = UsdGeom.XformCache().GetLocalToWorldTransform(frame_mesh_prim)
        high_frame_xy = []
        for point in frame_mesh.GetPointsAttr().Get():
            local_point = frame_xform.Transform(Gf.Vec3d(point))
            world_point = (
                -4.68 - local_point[1],
                14.39363 + local_point[0],
                local_point[2],
            )
            if world_point[2] > 0.85 and 15.5 < world_point[1] < 15.75:
                high_frame_xy.append(world_point[:2])
        self.assertTrue(high_frame_xy)

        frame_min_x = min(point[0] for point in high_frame_xy)
        frame_min_y = min(point[1] for point in high_frame_xy)
        layout = _LAYOUT_MODULE.resolve_scene_layout({})
        d02 = layout.belt_box_kinds[1]
        start_y = layout.robot_2_workstation_y
        target_x, target_y = west_bin_world_x, west_bin_world_y
        expanded_corner_x = frame_min_x - d02.width_x * 0.5
        expanded_corner_y = frame_min_y - d02.length_y * 0.5

        waypoint_x, waypoint_y = -6.25, start_y
        # 第一段保持在高架膨胀 AABB 南侧；第二段从 waypoint 斜送到收纳箱。
        self.assertGreaterEqual(expanded_corner_y - waypoint_y, 0.60)
        dogleg_x, dogleg_y = target_x - waypoint_x, target_y - waypoint_y
        dogleg_signed_clearance = (
            dogleg_y * (expanded_corner_x - waypoint_x)
            - dogleg_x * (expanded_corner_y - waypoint_y)
        ) / math.hypot(dogleg_x, dogleg_y)
        self.assertGreaterEqual(dogleg_signed_clearance, 0.20)

        belt_ranges = [
            world_x_range(name)
            for name in ("ConveyorBelt_A08_06", "ConveyorBelt_A08_07", "ConveyorBelt_A08_08")
        ]
        belt_min = min(item[0] for item in belt_ranges)
        belt_max = max(item[1] for item in belt_ranges)
        east_min, _east_max = world_x_range("blue_sorting_bin_02")
        _west_min, west_max = world_x_range("blue_sorting_bin_01")
        self.assertGreaterEqual(east_min - belt_max, 0.004)
        self.assertGreaterEqual(belt_min - west_max, 0.014)

    @unittest.skipUnless(_HAS_PXR, "需要 pxr（usd-core）做离线组合审计")
    def test_blue_sorting_bins_open_toward_the_same_robot_side(self) -> None:
        """组合后两只料箱的可见中心不漂移，局部 -Y 缺口方向保持一致。"""

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

        _, opening_01 = mesh_pose("blue_sorting_bin_01")
        origin_02, opening_02 = mesh_pose("blue_sorting_bin_02")

        self.assertGreater(opening_01 * opening_02, 0.9999)
        expected_origin_02 = Gf.Vec3d(
            0.8539417853480611, -0.00670726210634566, 0.43127658462090845
        )
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
