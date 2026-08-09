"""流水线纸箱物理封装 ``cart_box_d01_physics.usda`` 的契约体检。

纸箱是 v61 ``ConveyorBelt_Box_XX`` 的同款视觉资产（SM_CardBoxD_01）。任务不直接
提升背景 Prim，而是自己包一层：背景那份带 triangle-mesh 碰撞（PhysX 对动态刚体
只能退化成凸包 fallback 并刷警告），摆位也是 v48→v61 换版遗留。这里锁住封装层的
关键不变量——碰撞盒尺寸必须与视觉资产缩放后的包围盒一致，否则箱子会浮在带面上
或陷进去，而这种错位在画面上很难一眼看出来。
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_PROPS_DIR = _TASK_DIR / "scene_assets/props"
_ASSET_PATH = _PROPS_DIR / "cart_box_d01_physics.usda"

# SM_CardBoxD_01.usd：mesh extent (-19,-12.5,0)~(19,12.5,14.875)，scale 0.01。
_BOX_HALF_X = 0.19
_BOX_HALF_Y = 0.125
_BOX_HEIGHT = 0.149


class BeltBoxPhysicsAssetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.text = _ASSET_PATH.read_text(encoding="utf-8")

    def test_asset_exists_and_declares_stage_metadata(self) -> None:
        self.assertTrue(_ASSET_PATH.is_file(), _ASSET_PATH)
        self.assertTrue(self.text.startswith("#usda 1.0\n"))
        self.assertIn('defaultPrim = "CartBoxD01"', self.text)
        self.assertIn("metersPerUnit = 1", self.text)
        self.assertIn('upAxis = "Z"', self.text)

    def test_visual_comes_from_the_same_asset_v61_uses(self) -> None:
        """必须引用 SM_CardBoxD_01，而不是 cart_box 那份 D_05。

        只看 references 行——两份 wrapper 同构，docstring 里会互相提及对方。
        """

        references = re.findall(r"references = @([^@]+)@", self.text)
        self.assertEqual(len(references), 1, references)
        self.assertTrue(references[0].endswith("SM_CardBoxD_01.usd"), references[0])

    def test_referenced_visual_mesh_loses_its_own_collision(self) -> None:
        """原资产自带 triangle-mesh 碰撞，必须删掉，否则和本层凸包重复。"""

        self.assertRegex(
            self.text,
            r'over "SM_CardBoxD_01"\s*\(\s*\n\s*delete apiSchemas = \[[^\]]*'
            r'"PhysicsCollisionAPI"[^\]]*"PhysicsMeshCollisionAPI"[^\]]*\]',
        )

    def test_root_is_a_dynamic_rigid_body(self) -> None:
        self.assertIn('prepend apiSchemas = ["PhysicsRigidBodyAPI"', self.text)
        self.assertIn("bool physics:rigidBodyEnabled = 1", self.text)
        self.assertIn("bool physics:kinematicEnabled = 0", self.text)

    def test_collider_is_an_invisible_convex_hull(self) -> None:
        self.assertIn('uniform token physics:approximation = "convexHull"', self.text)
        self.assertIn("bool physics:collisionEnabled = 1", self.text)
        self.assertIn('token visibility = "invisible"', self.text)
        self.assertNotIn("convexDecomposition", self.text)

    def test_collider_box_matches_the_scaled_visual_extent(self) -> None:
        """8 个角点必须正好张成 0.38 × 0.25 × 0.149 m，且原点在箱底面。

        原点在底面是布局代码的前提：出生 z 直接取带面高度 + 3 mm 沉降余量。
        """

        block = re.search(r"point3f\[\] points = \[(.*?)\]", self.text, re.S)
        self.assertIsNotNone(block, "找不到 convexHull 的 points")
        points = [
            tuple(float(value) for value in match)
            for match in re.findall(
                r"\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)",
                block.group(1),
            )
        ]

        self.assertEqual(len(points), 8)
        xs = {round(p[0], 6) for p in points}
        ys = {round(p[1], 6) for p in points}
        zs = {round(p[2], 6) for p in points}
        self.assertEqual(xs, {-_BOX_HALF_X, _BOX_HALF_X})
        self.assertEqual(ys, {-_BOX_HALF_Y, _BOX_HALF_Y})
        self.assertEqual(zs, {0.0, _BOX_HEIGHT}, "原点必须在箱底面")

    def test_collider_dimensions_agree_with_the_layout_constants(self) -> None:
        """碰撞盒与 scene_layout 的箱尺寸常量必须是同一套数，否则排队间距会算错。"""

        import importlib.util
        import sys

        spec = importlib.util.spec_from_file_location(
            "conveyor_scene_layout_for_box_asset_test", _TASK_DIR / "scene_layout.py"
        )
        assert spec is not None and spec.loader is not None
        layout = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = layout
        spec.loader.exec_module(layout)

        # 箱子绕 Z 转 90° 摆放：视觉长边 (0.38) 沿输送方向 Y，短边 (0.25) 横向。
        self.assertAlmostEqual(layout.BELT_BOX_LENGTH_Y, _BOX_HALF_X * 2, places=6)
        self.assertAlmostEqual(layout.BELT_BOX_WIDTH_X, _BOX_HALF_Y * 2, places=6)
        self.assertAlmostEqual(layout.BELT_BOX_HEIGHT_Z, 0.1487, places=6)

    def test_grasp_friction_material_is_bound_to_the_collider(self) -> None:
        self.assertIn("float physics:staticFriction = 1.4", self.text)
        self.assertIn("float physics:dynamicFriction = 1.1", self.text)
        self.assertIn(
            "rel material:binding:physics = </CartBoxD01/GraspPhysicsMaterial>",
            self.text,
        )


if __name__ == "__main__":
    unittest.main()
