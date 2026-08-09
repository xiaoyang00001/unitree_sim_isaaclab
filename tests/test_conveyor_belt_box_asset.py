"""两种流水线纸箱物理封装的契约体检。

箱型都取自 v61 背景：`ConveyorBelt_Box_XX` 引 SM_CardBoxD_01、`KLT_Bin_XX` 引
SM_CardBoxC_01。任务不直接提升那些背景 Prim，而是各包一层：背景那份带
triangle-mesh 碰撞（PhysX 对动态刚体只能退化成凸包 fallback 并刷警告），摆位也是
v48→v61 换版遗留。这里锁住封装层的关键不变量——碰撞盒尺寸必须与视觉资产缩放后的
包围盒一致，且必须与 scene_layout 的箱型表逐值吻合，否则箱子会浮在带面上或陷进去、
排队间距也会算错，而这两种错位在画面上都很难一眼看出来。
"""

from __future__ import annotations

import importlib.util
import re
import sys
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_PROPS_DIR = _TASK_DIR / "scene_assets/props"

_SPEC = importlib.util.spec_from_file_location(
    "conveyor_scene_layout_for_box_asset_test", _TASK_DIR / "scene_layout.py"
)
assert _SPEC is not None and _SPEC.loader is not None
_LAYOUT = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _LAYOUT
_SPEC.loader.exec_module(_LAYOUT)

# key -> (defaultPrim, 视觉资产名, 半宽x, 半宽y, 高)。数值取自本机 Isaac 5.1 资产包实测：
# SM_CardBoxD_01 extent (-19,-12.5,0)~(19,12.5,14.875) × scale 0.01
# SM_CardBoxC_01 extent (-25,-25,0)~(25,25,25)         × scale 0.01
_EXPECTED = {
    "d01": ("CartBoxD01", "SM_CardBoxD_01", 0.19, 0.125, 0.149),
    "c01": ("CartBoxC01", "SM_CardBoxC_01", 0.25, 0.25, 0.25),
}


class BeltBoxPhysicsAssetTest(unittest.TestCase):
    def _text(self, key: str) -> str:
        return (_PROPS_DIR / _LAYOUT.BELT_BOX_KINDS[key].asset).read_text(encoding="utf-8")

    def test_every_kind_in_the_table_has_a_physics_wrapper(self) -> None:
        self.assertEqual(set(_LAYOUT.BELT_BOX_KINDS), set(_EXPECTED))
        for key, kind in _LAYOUT.BELT_BOX_KINDS.items():
            with self.subTest(key):
                self.assertTrue((_PROPS_DIR / kind.asset).is_file(), kind.asset)

    def test_stage_metadata_and_default_prim(self) -> None:
        for key, (default_prim, _visual, *_rest) in _EXPECTED.items():
            with self.subTest(key):
                text = self._text(key)
                self.assertTrue(text.startswith("#usda 1.0\n"))
                self.assertIn(f'defaultPrim = "{default_prim}"', text)
                self.assertIn("metersPerUnit = 1", text)
                self.assertIn('upAxis = "Z"', text)

    def test_each_wrapper_references_exactly_its_own_visual_asset(self) -> None:
        """只看 references 行——几份 wrapper 同构，docstring 里会互相提及。"""

        for key, (_prim, visual, *_rest) in _EXPECTED.items():
            with self.subTest(key):
                references = re.findall(r"references = @([^@]+)@", self._text(key))
                self.assertEqual(len(references), 1, references)
                self.assertTrue(references[0].endswith(f"{visual}.usd"), references[0])

    def test_referenced_visual_mesh_loses_its_own_collision(self) -> None:
        """原资产自带 triangle-mesh 碰撞，必须删掉，否则和本层凸包重复。"""

        for key, (_prim, visual, *_rest) in _EXPECTED.items():
            with self.subTest(key):
                self.assertRegex(
                    self._text(key),
                    rf'over "{visual}"\s*\(\s*\n\s*delete apiSchemas = \[[^\]]*'
                    r'"PhysicsCollisionAPI"[^\]]*"PhysicsMeshCollisionAPI"[^\]]*\]',
                )

    def test_root_is_a_dynamic_rigid_body(self) -> None:
        for key in _EXPECTED:
            with self.subTest(key):
                text = self._text(key)
                self.assertIn('prepend apiSchemas = ["PhysicsRigidBodyAPI"', text)
                self.assertIn("bool physics:rigidBodyEnabled = 1", text)
                self.assertIn("bool physics:kinematicEnabled = 0", text)

    def test_collider_is_an_invisible_convex_hull(self) -> None:
        for key in _EXPECTED:
            with self.subTest(key):
                text = self._text(key)
                self.assertIn('uniform token physics:approximation = "convexHull"', text)
                self.assertIn("bool physics:collisionEnabled = 1", text)
                self.assertIn('token visibility = "invisible"', text)
                self.assertNotIn("convexDecomposition", text)

    def test_collider_box_matches_the_scaled_visual_extent(self) -> None:
        """8 个角点必须正好张成资产尺寸，且原点在箱底面。

        原点在底面是布局代码的前提：出生 z 直接取带面高度 + 3 mm 沉降余量。
        """

        for key, (_prim, _visual, half_x, half_y, height) in _EXPECTED.items():
            with self.subTest(key):
                block = re.search(r"point3f\[\] points = \[(.*?)\]", self._text(key), re.S)
                self.assertIsNotNone(block, "找不到 convexHull 的 points")
                assert block is not None
                points = re.findall(
                    r"\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)", block.group(1)
                )
                self.assertEqual(len(points), 8)
                xs = {round(float(p[0]), 6) for p in points}
                ys = {round(float(p[1]), 6) for p in points}
                zs = {round(float(p[2]), 6) for p in points}
                self.assertEqual(xs, {-half_x, half_x})
                self.assertEqual(ys, {-half_y, half_y})
                self.assertEqual(zs, {0.0, height}, "原点必须在箱底面")

    def test_collider_dimensions_agree_with_the_layout_kind_table(self) -> None:
        """碰撞盒与 scene_layout 的箱型表必须是同一套数，否则排队间距会算错。

        箱子绕 Z 转 90° 摆放：视觉的 X 尺寸变成沿输送方向的 ``length_y``，
        视觉的 Y 尺寸变成横向的 ``width_x``。
        """

        for key, (_prim, _visual, half_x, half_y, height) in _EXPECTED.items():
            with self.subTest(key):
                kind = _LAYOUT.BELT_BOX_KINDS[key]
                self.assertAlmostEqual(kind.length_y, half_x * 2, places=6)
                self.assertAlmostEqual(kind.width_x, half_y * 2, places=6)
                self.assertAlmostEqual(kind.half_length_y, half_x, places=6)
                # 高度容差放宽到 mm：碰撞盒取整到 0.149，视觉包围盒是 0.1487。
                self.assertAlmostEqual(kind.height_z, height, places=2)

    def test_both_kinds_fit_the_belt_width(self) -> None:
        """最宽的箱型也要放得进带面，否则会一直蹭侧导轨。"""

        for key, kind in _LAYOUT.BELT_BOX_KINDS.items():
            with self.subTest(key):
                self.assertLessEqual(kind.width_x, _LAYOUT.BELT_BOX_BELT_WIDTH)

    def test_the_two_kinds_are_visually_distinguishable(self) -> None:
        """交错排布的意义在于一眼能区分——尺寸必须有明显差异。"""

        d01 = _LAYOUT.BELT_BOX_KINDS["d01"]
        c01 = _LAYOUT.BELT_BOX_KINDS["c01"]
        self.assertGreater(c01.length_y - d01.length_y, 0.1)
        self.assertGreater(c01.height_z - d01.height_z, 0.05)

    def test_grasp_friction_material_is_bound_to_the_collider(self) -> None:
        for key, (default_prim, *_rest) in _EXPECTED.items():
            with self.subTest(key):
                text = self._text(key)
                self.assertIn("float physics:staticFriction = 1.4", text)
                self.assertIn("float physics:dynamicFriction = 1.1", text)
                self.assertIn(
                    f"rel material:binding:physics = </{default_prim}/GraspPhysicsMaterial>",
                    text,
                )


if __name__ == "__main__":
    unittest.main()
