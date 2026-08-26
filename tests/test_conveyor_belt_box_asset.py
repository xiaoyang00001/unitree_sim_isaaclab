"""流水线纸箱与连续路径托面的物理资产契约体检。

C01/C02、D01~D05 都取自 Simple Warehouse canonical 纸箱；v61 背景里实际摆出的
`ConveyorBelt_Box_XX` / `KLT_Bin_XX` 只是其中 D01/C01 的实例。任务不直接提升背景
Prim，而是为每种独立外观包一层：官方 mesh 带 triangle-mesh 碰撞（PhysX 对动态刚体
只能退化成凸包 fallback 并刷警告），背景摆位还有 v48→v61 换版遗留。这里锁住封装层
的关键不变量——碰撞盒尺寸必须与视觉资产缩放后的包围盒一致，且必须与 scene_layout
的箱型表逐值吻合，否则箱子会浮在带面上或陷进去、排队间距也会算错。
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
_ENV_CFG_PATH = _TASK_DIR / "conveyor_env_cfg.py"
_PATH_SUPPORT_PATH = _PROPS_DIR / "conveyor_path_support_physics.usda"

_SPEC = importlib.util.spec_from_file_location(
    "conveyor_scene_layout_for_box_asset_test", _TASK_DIR / "scene_layout.py"
)
assert _SPEC is not None and _SPEC.loader is not None
_LAYOUT = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _LAYOUT
_SPEC.loader.exec_module(_LAYOUT)

# key -> (defaultPrim, 视觉资产名, 碰撞半宽x, 碰撞半宽y, 碰撞高)。规则箱取
# Isaac 5.1 官方视觉包围盒；D03 的视觉 AABB 关于原点不对称，wrapper 按两轴最大
# 绝对值向两侧扩成保守平底盒，因此这里锁的是任务实际使用的碰撞尺寸。
_EXPECTED = {
    "c01": ("CartBoxC01", "SM_CardBoxC_01", 0.25, 0.25, 0.25),
    "c02": ("CartBoxC02", "SM_CardBoxC_02", 0.25, 0.25, 0.311),
    "d01": ("CartBoxD01", "SM_CardBoxD_01", 0.19, 0.125, 0.149),
    "d02": ("CartBoxD02", "SM_CardBoxD_02", 0.19, 0.125, 0.1663),
    "d03": ("CartBoxD03", "SM_CardBoxD_03", 0.2063, 0.1303, 0.2656),
    "d04": ("CartBoxD04", "SM_CardBoxD_04", 0.19, 0.125, 0.149),
    "d05": ("CartBoxD05", "SM_CardBoxD_05", 0.19, 0.125, 0.149),
}

# 软包裹是自包含的程序化资产（IsaacLab 分叉 feat/pickplace-parcel-assets 生成），
# 不走"引用官方视觉 + 外挂碰撞盒"的 wrapper 结构：视觉网格 Bag 自己带 convexHull
# 碰撞。key -> defaultPrim。
_PARCEL_EXPECTED = {
    "parcel_a01": "ParcelSoftA01",
    "parcel_a02": "ParcelSoftA02",
    "parcel_a03": "ParcelSoftA03",
}


class BeltBoxPhysicsAssetTest(unittest.TestCase):
    def _text(self, key: str) -> str:
        return (_PROPS_DIR / _LAYOUT.BELT_BOX_KINDS[key].asset).read_text(encoding="utf-8")

    def test_every_kind_in_the_table_has_a_physics_asset(self) -> None:
        self.assertEqual(set(_LAYOUT.BELT_BOX_KINDS), set(_EXPECTED) | set(_PARCEL_EXPECTED))
        for key, kind in _LAYOUT.BELT_BOX_KINDS.items():
            with self.subTest(key):
                self.assertTrue((_PROPS_DIR / kind.asset).is_file(), kind.asset)

    def test_spawn_rotation_keeps_the_short_edge_in_the_grasp_direction(self) -> None:
        """资产保持原始朝向，Y 向短边就是机器人双臂需要跨过的宽度。"""

        text = _ENV_CFG_PATH.read_text(encoding="utf-8")
        self.assertRegex(text, r"BELT_BOX_ROT\s*=\s*\[1\.0, 0\.0, 0\.0, 0\.0\]")

    def test_d02_has_a_blue_tint_and_high_contrast_top_label(self) -> None:
        """第二箱型必须在远处也能与棕色 D01 明显区分。"""

        text = self._text("d02")
        self.assertIn("inputs:BaseColor_Tint = (0.15, 0.48, 1.0, 1.0)", text)
        self.assertIn("float inputs:Desaturation = 0.85", text)
        self.assertIn('def Mesh "TopLabel"', text)
        self.assertIn('def Material "LabelMaterial"', text)
        self.assertIn("rel material:binding = </CartBoxD02/LabelMaterial>", text)

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

        箱子保持资产原始朝向：视觉的 Y 尺寸沿输送方向成为 ``length_y``，
        视觉的 X 尺寸横跨带面成为 ``width_x``，让机器人抱取时跨过较短边。
        """

        for key, (_prim, _visual, half_x, half_y, height) in _EXPECTED.items():
            with self.subTest(key):
                kind = _LAYOUT.BELT_BOX_KINDS[key]
                self.assertAlmostEqual(kind.length_y, half_y * 2, places=6)
                self.assertAlmostEqual(kind.width_x, half_x * 2, places=6)
                self.assertAlmostEqual(kind.half_length_y, half_y, places=6)
                self.assertAlmostEqual(kind.half_queue_extent, max(half_x, half_y), places=6)
                # 高度容差放宽到 mm：碰撞盒取整到 0.149，视觉包围盒是 0.1487。
                self.assertAlmostEqual(kind.height_z, height, places=2)

    def test_all_kinds_fit_the_belt_width(self) -> None:
        """最宽的箱型也要放得进带面，否则会一直蹭侧导轨。"""

        for key, kind in _LAYOUT.BELT_BOX_KINDS.items():
            with self.subTest(key):
                self.assertLessEqual(kind.width_x, _LAYOUT.BELT_BOX_BELT_WIDTH)

    def test_the_two_cardboxes_share_grasp_dimensions_but_use_distinct_visuals(self) -> None:
        """D02 靠压皱轮廓区分，但抓取与排队所用的横向尺寸必须与 D01 一致。"""

        d01 = _LAYOUT.BELT_BOX_KINDS["d01"]
        d02 = _LAYOUT.BELT_BOX_KINDS["d02"]
        self.assertNotEqual(d01.asset, d02.asset)
        self.assertEqual(
            (d01.length_y, d01.width_x, d01.mass),
            (d02.length_y, d02.width_x, d02.mass),
        )
        self.assertGreater(d02.height_z, d01.height_z)

    def test_parcel_assets_are_self_contained_rigid_bodies(self) -> None:
        """软包裹：刚体 + convexHull + 自包含，物理约定与纸箱一致。

        ⚠️ "软"只是视觉造型（枕形鼓包+褶皱），物理必须是刚体——CPU pipeline 不支持
        deformable，而 conveyor 场景固定跑 --device cpu。这条钉住"没人把它换成真软体"。
        """

        for key, default_prim in _PARCEL_EXPECTED.items():
            with self.subTest(key):
                text = self._text(key)
                self.assertTrue(text.startswith("#usda 1.0\n"))
                self.assertIn(f'defaultPrim = "{default_prim}"', text)
                self.assertIn("metersPerUnit = 1", text)
                self.assertIn('upAxis = "Z"', text)
                # 根是动态刚体（无 kinematic、无 deformable）。
                self.assertIn('"PhysicsRigidBodyAPI"', text)
                self.assertNotIn("Deformable", text)
                # 碰撞是平底"雪橇"盒（SledCollider），不是枕形 Bag 自身的凸包：
                # 枕形凸包滑动时轻微摇晃耗能，实测比平底纸箱慢 ~3%/程，混排时
                # 队列间距逐程漂移。Bag 的碰撞被显式禁用。
                self.assertIn('def Mesh "SledCollider"', text)
                self.assertIn('uniform token physics:approximation = "convexHull"', text)
                self.assertIn("bool physics:collisionEnabled = 0", text)
                self.assertNotIn("convexDecomposition", text)
                # 自包含：无任何外部 references/payload。
                self.assertNotIn("references = @", text)
                self.assertNotIn("payload", text)
                # 抓取摩擦已归一到纸箱同一套约定。源分支的 1.6/1.2 combine=max 会
                # 压过带面摩擦（pair 取 max），legacy 驱动每 20ms 写一次速度、写入
                # 间隙靠摩擦减速，μd 近两倍 ⇒ 包裹比纸箱慢 ~35%，整带停时短停
                # 0.24~0.28 m（2026-08-09 实测）。multiply 下 pair 与纸箱逐值一致。
                self.assertIn("float physics:staticFriction = 1.4", text)
                self.assertIn("float physics:dynamicFriction = 1.1", text)
                self.assertIn('frictionCombineMode = "multiply"', text)
                self.assertNotIn('frictionCombineMode = "max"', text)

    def test_parcel_dimensions_agree_with_the_layout_kind_table(self) -> None:
        """箱型表里软包裹的尺寸必须与资产实测包围盒一致（排队半长靠它算）。"""

        expected_dims = {
            "parcel_a01": (0.3013, 0.4017, 0.0801),
            "parcel_a02": (0.3520, 0.4526, 0.0968),
            "parcel_a03": (0.2414, 0.3218, 0.0603),
        }
        for key, (length, width, height) in expected_dims.items():
            with self.subTest(key):
                kind = _LAYOUT.BELT_BOX_KINDS[key]
                self.assertAlmostEqual(kind.length_y, length, places=4)
                self.assertAlmostEqual(kind.width_x, width, places=4)
                self.assertAlmostEqual(kind.height_z, height, places=4)

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


class ConveyorPathSupportPhysicsAssetTest(unittest.TestCase):
    def _text(self) -> str:
        return _PATH_SUPPORT_PATH.read_text(encoding="utf-8")

    def _mesh_topology(
        self,
    ) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
        text = self._text()
        points_block = re.search(r"point3f\[\] points = \[(.*?)\]", text, re.S)
        counts_block = re.search(r"int\[\] faceVertexCounts = \[(.*?)\]", text, re.S)
        indices_block = re.search(r"int\[\] faceVertexIndices = \[(.*?)\]", text, re.S)
        self.assertIsNotNone(points_block, "找不到托面 points")
        self.assertIsNotNone(counts_block, "找不到托面 faceVertexCounts")
        self.assertIsNotNone(indices_block, "找不到托面 faceVertexIndices")
        assert points_block is not None
        assert counts_block is not None
        assert indices_block is not None

        number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
        points = [
            tuple(float(value) for value in match)
            for match in re.findall(
                rf"\(\s*({number})\s*,\s*({number})\s*,\s*({number})\s*\)",
                points_block.group(1),
            )
        ]
        counts = [int(value) for value in re.findall(r"\d+", counts_block.group(1))]
        indices = [int(value) for value in re.findall(r"\d+", indices_block.group(1))]
        self.assertTrue(points)
        self.assertTrue(counts)
        self.assertTrue(indices)
        self.assertEqual(set(counts), {3}, "连续托面只允许三角面")
        self.assertEqual(len(indices), sum(counts))
        self.assertEqual(len(indices) % 3, 0)
        self.assertTrue(all(index < len(points) for index in indices))
        faces = [tuple(indices[offset : offset + 3]) for offset in range(0, len(indices), 3)]
        return points, faces

    def test_support_is_one_static_collision_surface(self) -> None:
        """托面只能提供静态接触，不能意外变成会受重力影响的刚体。"""

        self.assertTrue(_PATH_SUPPORT_PATH.is_file())
        text = self._text()
        self.assertTrue(text.startswith("#usda 1.0\n"))
        self.assertIn('defaultPrim = "ConveyorPathSupport"', text)
        self.assertIn("metersPerUnit = 1", text)
        self.assertIn('upAxis = "Z"', text)
        self.assertEqual(text.count('def Mesh "SupportSurface"'), 1)
        self.assertEqual(text.count('"PhysicsCollisionAPI"'), 1)
        self.assertIn("bool physics:collisionEnabled = 1", text)
        self.assertIn('uniform token physics:approximation = "none"', text)
        self.assertIn('token visibility = "invisible"', text)
        self.assertNotIn("PhysicsRigidBodyAPI", text)
        self.assertNotIn("PhysicsMassAPI", text)
        self.assertNotIn("physics:rigidBodyEnabled", text)
        self.assertNotIn("physics:kinematicEnabled", text)

    def test_support_faces_are_horizontal_and_upward_only(self) -> None:
        """所有碰撞面共面且朝上，因此网格内部不可能藏有卡包裹的竖直端面。"""

        points, faces = self._mesh_topology()
        self.assertEqual({point[2] for point in points}, {0.0})
        self.assertEqual(len(set(points)), len(points), "同坐标重复顶点会把连续托面暗中切开")
        for face in faces:
            with self.subTest(face=face):
                a, b, c = (points[index] for index in face)
                signed_double_area = (b[0] - a[0]) * (c[1] - a[1]) - (
                    b[1] - a[1]
                ) * (c[0] - a[0])
                self.assertGreater(signed_double_area, 0.0, "面必须非退化且法向朝 +Z")

    def test_support_uses_the_same_sixty_centimetre_width_on_both_straights(self) -> None:
        points, _faces = self._mesh_topology()

        main_x = sorted({x for x, y, _z in points if y == -8.03})
        branch_y = sorted({y for x, y, _z in points if x == -11.4142})
        self.assertEqual(main_x, [-0.30, 0.30])
        self.assertEqual(branch_y, [1.2834, 1.8834])
        self.assertAlmostEqual(main_x[1] - main_x[0], 0.60, places=9)
        self.assertAlmostEqual(branch_y[1] - branch_y[0], 0.60, places=9)

    def test_support_triangles_form_one_seam_free_manifold(self) -> None:
        """三角形须按共享边连成一个无洞平面，不能退回多块对接托面。"""

        _points, faces = self._mesh_topology()
        edge_faces: dict[tuple[int, int], list[int]] = {}
        for face_index, (a, b, c) in enumerate(faces):
            for edge in ((a, b), (b, c), (c, a)):
                edge_faces.setdefault(tuple(sorted(edge)), []).append(face_index)

        self.assertTrue(all(len(owners) in (1, 2) for owners in edge_faces.values()))
        neighbours = [set() for _ in faces]
        for owners in edge_faces.values():
            if len(owners) == 2:
                left, right = owners
                neighbours[left].add(right)
                neighbours[right].add(left)
        visited_faces = {0}
        pending_faces = [0]
        while pending_faces:
            current = pending_faces.pop()
            for neighbour in neighbours[current] - visited_faces:
                visited_faces.add(neighbour)
                pending_faces.append(neighbour)
        self.assertEqual(len(visited_faces), len(faces), "全部三角形必须经共享边连通")

        boundary_edges = [edge for edge, owners in edge_faces.items() if len(owners) == 1]
        boundary_neighbours: dict[int, set[int]] = {}
        for left, right in boundary_edges:
            boundary_neighbours.setdefault(left, set()).add(right)
            boundary_neighbours.setdefault(right, set()).add(left)
        self.assertTrue(boundary_neighbours)
        self.assertTrue(
            all(len(neighbours) == 2 for neighbours in boundary_neighbours.values()),
            "外边界必须是闭环",
        )
        start = next(iter(boundary_neighbours))
        visited_boundary = {start}
        pending_boundary = [start]
        while pending_boundary:
            current = pending_boundary.pop()
            for neighbour in boundary_neighbours[current] - visited_boundary:
                visited_boundary.add(neighbour)
                pending_boundary.append(neighbour)
        self.assertEqual(
            visited_boundary,
            set(boundary_neighbours),
            "只能有一个外边界闭环，内部不得留下孔洞或断开的托面",
        )


if __name__ == "__main__":
    unittest.main()
