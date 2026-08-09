from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_ASSET_DIR = _TASK_DIR / "scene_assets"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_VARIANTS = _load_module(
    "conveyor_asset_variants_for_workcell_test", _TASK_DIR / "asset_variants.py"
)
_BACKGROUNDS = _load_module(
    "conveyor_background_assets_for_workcell_test", _TASK_DIR / "background_assets.py"
)
_GENERATOR = _load_module(
    "build_conveyor_visual_only_usd_for_workcell_test",
    _REPO_ROOT / "tools/build_conveyor_visual_only_usd.py",
)

try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    _HAS_PXR = True
except ModuleNotFoundError:
    Sdf = Usd = UsdGeom = UsdPhysics = None  # type: ignore[assignment]
    _HAS_PXR = False


_WORKCELL_ADAPTER_NAME = "conveyor_workcell_lite_conveyor_visual_only.usda"
_VALID_WAREHOUSE_ADAPTER_TEXT = (
    "#usda 1.0\nsubLayers = [@./warehouse-simple6_v61_visual_only.usda@]\n"
)
# 场景坐标系约定：v61 及其所有派生层都是 Z-up 米制。subLayers 不继承 stage
# metadata，adapter 必须自己声明，否则单独打开会落到 USD 默认的 Y-up / 厘米。
_SCENE_METERS_PER_UNIT = 1.0
_SCENE_UP_AXIS = "Z"


def _write_valid_fixture(assets_dir: Path) -> None:
    """写出一套满足静态校验的最小 mock 资产组。"""

    (assets_dir / _VARIANTS.CLEAN_BACKGROUND_USD).write_text(
        "#usda 1.0\n", encoding="utf-8"
    )
    (assets_dir / _VARIANTS.WORKCELL_LITE_BACKGROUND_USD).write_text(
        "#usda 1.0\n", encoding="utf-8"
    )
    (assets_dir / _VARIANTS.VISUAL_ONLY_BACKGROUND_USD).write_text(
        _VALID_WAREHOUSE_ADAPTER_TEXT, encoding="utf-8"
    )
    (assets_dir / _WORKCELL_ADAPTER_NAME).write_text(
        _GENERATOR._render_workcell_lite_adapter(
            _VARIANTS.VISUAL_ONLY_BACKGROUND_USD,
            _SCENE_METERS_PER_UNIT,
            _SCENE_UP_AXIS,
        ),
        encoding="utf-8",
    )


class WorkcellVisualOnlyAdapterRoutingTest(unittest.TestCase):
    def test_workcell_baseline_routes_to_workcell_adapter(self) -> None:
        resolve = _VARIANTS.resolve_conveyor_background_usd
        baseline = _ASSET_DIR / _VARIANTS.WORKCELL_LITE_BACKGROUND_USD

        selected = resolve(
            _ASSET_DIR,
            {_VARIANTS.CONVEYOR_VISUAL_ONLY_ENV: "1"},
            baseline_path=baseline,
        )
        self.assertEqual(selected.name, _WORKCELL_ADAPTER_NAME)

    def test_workcell_plus_surface_velocity_never_falls_back_to_full_warehouse(
        self,
    ) -> None:
        resolve = _VARIANTS.resolve_conveyor_background_usd
        baseline = _ASSET_DIR / _VARIANTS.WORKCELL_LITE_BACKGROUND_USD

        # 环境显式关掉资产开关也一样：Surface Velocity 强制纯视觉输送机，
        # 且必须保持轻量工位 baseline，而不是退回完整 clean-v61。
        selected = resolve(
            _ASSET_DIR,
            {_VARIANTS.CONVEYOR_VISUAL_ONLY_ENV: "0"},
            baseline_path=baseline,
            drive_mode="surface_velocity",
        )
        self.assertEqual(selected.name, _WORKCELL_ADAPTER_NAME)

    def test_workcell_baseline_without_visual_only_is_returned_unchanged(self) -> None:
        resolve = _VARIANTS.resolve_conveyor_background_usd
        baseline = _ASSET_DIR / _VARIANTS.WORKCELL_LITE_BACKGROUND_USD

        self.assertEqual(
            resolve(
                _ASSET_DIR,
                {_VARIANTS.CONVEYOR_VISUAL_ONLY_ENV: "0"},
                baseline_path=baseline,
                drive_mode="legacy",
            ),
            baseline,
        )

    def test_full_backgrounds_still_route_to_warehouse_adapter(self) -> None:
        resolve = _VARIANTS.resolve_conveyor_background_usd
        for baseline_name in (
            _VARIANTS.CLEAN_BACKGROUND_USD,
            _VARIANTS.LEGACY_BACKGROUND_USD,
        ):
            selected = resolve(
                _ASSET_DIR,
                {_VARIANTS.CONVEYOR_VISUAL_ONLY_ENV: "1"},
                baseline_path=_ASSET_DIR / baseline_name,
            )
            self.assertEqual(selected.name, _VARIANTS.VISUAL_ONLY_BACKGROUND_USD)

    def test_routing_keys_match_background_selector_module(self) -> None:
        # 路由按 baseline 文件名匹配，而 baseline 由 background_assets 产生；
        # 两个模块互不 import，文件名漂移会静默退回完整仓库 adapter，必须
        # 用跨模块相等断言钉住。
        filenames = _BACKGROUNDS.BACKGROUND_ASSET_FILENAMES
        self.assertEqual(
            filenames["workcell_lite"], _VARIANTS.WORKCELL_LITE_BACKGROUND_USD
        )
        self.assertEqual(filenames["visual_only"], _VARIANTS.CLEAN_BACKGROUND_USD)
        self.assertEqual(filenames["legacy_v61"], _VARIANTS.LEGACY_BACKGROUND_USD)

    def test_env_cfg_consumes_both_adapters_and_workcell_mode_label(self) -> None:
        # conveyor_env_cfg 无法脱离 Isaac import，按套件先例用源码文本钉住
        # 双 adapter 标志位与 workcell 模式日志两处，防止合并冲突静默回退。
        source = (_TASK_DIR / "conveyor_env_cfg.py").read_text(encoding="utf-8")

        self.assertIn(
            "CONVEYOR_VISUAL_ONLY_ASSET_ENABLED = BACKGROUND_USD_PATH.name in {\n"
            "    VISUAL_ONLY_BACKGROUND_USD,\n"
            "    WORKCELL_LITE_VISUAL_ONLY_BACKGROUND_USD,\n"
            "}",
            source,
        )
        self.assertIn(
            'BACKGROUND_MODE = "workcell_lite+conveyor_visual_only"', source
        )


class WorkcellVisualOnlyAdapterFailFastTest(unittest.TestCase):
    def _resolve_workcell(self, assets_dir: Path):
        return _VARIANTS.resolve_conveyor_background_usd(
            assets_dir,
            {_VARIANTS.CONVEYOR_VISUAL_ONLY_ENV: "1"},
            baseline_path=assets_dir / _VARIANTS.WORKCELL_LITE_BACKGROUND_USD,
        )

    def test_valid_fixture_resolves(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            assets_dir = Path(tmp)
            _write_valid_fixture(assets_dir)
            self.assertEqual(
                self._resolve_workcell(assets_dir).name, _WORKCELL_ADAPTER_NAME
            )

    def test_missing_workcell_adapter_fails_fast(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            assets_dir = Path(tmp)
            _write_valid_fixture(assets_dir)
            (assets_dir / _WORKCELL_ADAPTER_NAME).unlink()

            with self.assertRaisesRegex(FileNotFoundError, "缺少专用组合层"):
                self._resolve_workcell(assets_dir)

    def test_wrong_sublayer_fails_fast(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            assets_dir = Path(tmp)
            _write_valid_fixture(assets_dir)
            # 过期形态：组合层错误叠加完整 clean wrapper，会丢失轻量工位。
            (assets_dir / _WORKCELL_ADAPTER_NAME).write_text(
                "#usda 1.0\n"
                f"subLayers = [@./{_VARIANTS.CLEAN_BACKGROUND_USD}@]\n"
                f"{_VARIANTS.WORKCELL_LITE_ADAPTER_DELETE_REFERENCE}\n"
                f"{_VARIANTS.WORKCELL_LITE_ADAPTER_PREPEND_REFERENCE}\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "未以轻量工位为 baseline"):
                self._resolve_workcell(assets_dir)

    def test_missing_reference_swap_fails_fast(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            assets_dir = Path(tmp)
            _write_valid_fixture(assets_dir)
            # 过期形态：subLayer 正确但没有把 ConveyorBelt 换成纯视觉子树。
            (assets_dir / _WORKCELL_ADAPTER_NAME).write_text(
                "#usda 1.0\n"
                f"subLayers = [@./{_VARIANTS.WORKCELL_LITE_BACKGROUND_USD}@]\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "原生输送机碰撞仍会进入组合"):
                self._resolve_workcell(assets_dir)

    def test_stale_warehouse_adapter_dependency_fails_fast(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            assets_dir = Path(tmp)
            _write_valid_fixture(assets_dir)
            # 组合层引用完整 adapter 的 ConveyorBelt 子树，后者过期指向
            # legacy v61 时必须一并拦截。
            (assets_dir / _VARIANTS.VISUAL_ONLY_BACKGROUND_USD).write_text(
                "#usda 1.0\nsubLayers = [@./warehouse-simple6_v61.usd@]\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "会绕过背景物理清理层"):
                self._resolve_workcell(assets_dir)


class WorkcellVisualOnlyAdapterArtifactTest(unittest.TestCase):
    def test_checked_in_adapter_matches_generator_and_resolves(self) -> None:
        adapter = _ASSET_DIR / _WORKCELL_ADAPTER_NAME
        text = adapter.read_text(encoding="utf-8")

        self.assertEqual(
            text,
            _GENERATOR._render_workcell_lite_adapter(
                _VARIANTS.VISUAL_ONLY_BACKGROUND_USD,
                _SCENE_METERS_PER_UNIT,
                _SCENE_UP_AXIS,
            ),
        )
        self.assertIn(
            f"subLayers = [@./{_VARIANTS.WORKCELL_LITE_BACKGROUND_USD}@]", text
        )
        # stage metadata 必须显式声明：subLayers 不继承，缺了单独打开就是 Y-up 躺倒。
        self.assertIn(f'upAxis = "{_SCENE_UP_AXIS}"', text)
        self.assertIn(f"metersPerUnit = {_SCENE_METERS_PER_UNIT!r}", text)
        self.assertIn(_VARIANTS.WORKCELL_LITE_ADAPTER_DELETE_REFERENCE, text)
        self.assertIn(_VARIANTS.WORKCELL_LITE_ADAPTER_PREPEND_REFERENCE, text)
        # 组合层不允许自带 payload 语义：payload 替换必须发生在被引用的完整
        # adapter layer stack 内，这里再出现 payload 说明结构被人为改动。
        self.assertNotIn("payload", text)

    def test_checked_in_assets_pass_variant_fail_fast(self) -> None:
        selected = _VARIANTS.resolve_conveyor_background_usd(
            _ASSET_DIR,
            {},
            baseline_path=_ASSET_DIR / _VARIANTS.WORKCELL_LITE_BACKGROUND_USD,
            drive_mode="surface_velocity",
        )
        self.assertEqual(selected, _ASSET_DIR / _WORKCELL_ADAPTER_NAME)


@unittest.skipUnless(_HAS_PXR, "需要 pxr（usd-core）做离线组合审计")
class WorkcellVisualOnlyComposedStageTest(unittest.TestCase):
    """离线组合审计：不启动 Isaac Sim，直接用 pxr 打开组合 stage。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.manifest = json.loads(
            (_ASSET_DIR / "conveyor_workcell_lite.manifest.json").read_text(
                encoding="utf-8"
            )
        )
        cls.stage = Usd.Stage.Open(
            str(_ASSET_DIR / _WORKCELL_ADAPTER_NAME), Usd.Stage.LoadAll
        )
        assert cls.stage

    def _enabled(self, prim, attr: str) -> bool:
        attribute = prim.GetAttribute(attr)
        if not attribute:
            return True
        value = attribute.Get()
        return True if value is None else bool(value)

    def _audit_physics(self, prims) -> tuple[list[str], list[str]]:
        rigid, collision = [], []
        for prim in prims:
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and self._enabled(
                prim, "physics:rigidBodyEnabled"
            ):
                rigid.append(str(prim.GetPath()))
            if prim.HasAPI(UsdPhysics.CollisionAPI) and self._enabled(
                prim, "physics:collisionEnabled"
            ):
                collision.append(str(prim.GetPath()))
        return rigid, collision

    def test_conveyor_belt_subtree_has_zero_rigid_zero_collision(self) -> None:
        belt = self.stage.GetPrimAtPath("/Root/ConveyorBelt")
        self.assertTrue(belt and belt.IsActive() and belt.IsLoaded())

        # instanceable Prim 会让默认遍历漏检其内部物理，且跨 instance 的
        # enabled=false 覆盖无效——出现即视为审计失效。
        self.assertEqual(
            [
                str(prim.GetPath())
                for prim in Usd.PrimRange.AllPrims(belt)
                if prim.IsInstance()
            ],
            [],
        )

        rigid, collision = self._audit_physics(Usd.PrimRange.AllPrims(belt))
        self.assertEqual(rigid, [])
        self.assertEqual(collision, [])

    def test_only_static_boundary_collision_survives(self) -> None:
        rigid, collision = self._audit_physics(self.stage.TraverseAll())
        # 地贴（FloorZone_*/Stripe_*）引用的官方 SM_FloorDecal 资产带静态碰撞，
        # Kit 能解析 S3 时会进入组合，属于保留的静态边界；离线时天然缺席。
        boundary = tuple(
            f"/Root/{prefix}"
            for prefix in tuple(self.manifest["keep_root_prefixes"])
            + ("GroundPlane", "FloorZone_", "Stripe_")
        )

        self.assertEqual(rigid, [])
        self.assertEqual(
            [path for path in collision if not path.startswith(boundary)], []
        )
        # 只验"不多"抓不住上游把边界碰撞整体清空的回归（机器人会穿地/出界），
        # 同时验"仍在"：地面碰撞面必须健在，外墙碰撞必须非空。
        self.assertIn("/Root/GroundPlane/CollisionPlane", collision)
        wall_prefixes = tuple(
            f"/Root/{prefix}" for prefix in self.manifest["keep_root_prefixes"]
        )
        self.assertGreater(
            sum(1 for path in collision if path.startswith(wall_prefixes)), 0
        )

    def test_lite_contract_prims_and_forbidden_types(self) -> None:
        for path in self.manifest["required_active_prims"]:
            prim = self.stage.GetPrimAtPath(path)
            self.assertTrue(prim and prim.IsActive(), f"缺少必需 Prim: {path}")
        for path in self.manifest["required_inactive_prims"]:
            prim = self.stage.GetPrimAtPath(path)
            self.assertFalse(
                bool(prim) and prim.IsActive(), f"被裁 Prim 回到组合: {path}"
            )

        # TraverseAll+IsActive 口径：默认 Traverse() 会跳过 S3 未解析时仅存的
        # over Prim，审计范围随远端可解析性漂移。
        forbidden = set(self.manifest["forbidden_active_type_names"])
        offenders = [
            f"{prim.GetPath()}:{prim.GetTypeName()}"
            for prim in self.stage.TraverseAll()
            if prim.IsActive() and prim.GetTypeName() in forbidden
        ]
        self.assertEqual(offenders, [])

    def test_root_children_stay_within_keep_whitelist(self) -> None:
        keep_exact = set(self.manifest["keep_root_exact"])
        keep_prefixes = tuple(self.manifest["keep_root_prefixes"])
        unexpected = sorted(
            prim.GetName()
            for prim in self.stage.GetPrimAtPath("/Root").GetAllChildren()
            if prim.GetName() not in keep_exact
            and not prim.GetName().startswith(keep_prefixes)
        )
        self.assertEqual(unexpected, [])

    def test_stage_stays_within_lite_budgets(self) -> None:
        # 计数口径含 S3 未解析时的 over Prim（离线实测 829，Kit 下会随远端
        # 解析上浮），与生成器一致；门槛 2000 两种环境都远未触顶。
        limits = self.manifest["audit_limits"]
        active = sum(1 for prim in self.stage.TraverseAll() if prim.IsActive())
        layers = len(self.stage.GetUsedLayers())

        self.assertLessEqual(active, limits["max_lite_active_prims"])
        self.assertLessEqual(layers, limits["max_lite_used_layers"])

    def test_belt_keeps_v61_local_xform(self) -> None:
        v61 = Sdf.Layer.FindOrOpen(str(_ASSET_DIR / _VARIANTS.LEGACY_BACKGROUND_USD))
        self.assertIsNotNone(v61)
        spec = v61.GetPrimAtPath("/Root/ConveyorBelt")
        translate_spec = spec.properties.get("xformOp:translate")
        self.assertIsNotNone(translate_spec, "v61 不再给 ConveyorBelt 提供本地位移")

        # 比较生效的本地变换而不是属性作者值：xformOpOrder 被覆盖时
        # translate 属性仍可读到旧值但已不生效，输送机会错位。
        belt = self.stage.GetPrimAtPath("/Root/ConveyorBelt")
        belt_transform = UsdGeom.Xformable(belt).GetLocalTransformation()
        self.assertEqual(
            belt_transform.ExtractTranslation(), translate_spec.default
        )

        warehouse_stage = Usd.Stage.Open(
            str(_ASSET_DIR / _VARIANTS.VISUAL_ONLY_BACKGROUND_USD),
            Usd.Stage.LoadNone,
        )
        warehouse_stage.Load("/Root/ConveyorBelt")
        reference_belt = warehouse_stage.GetPrimAtPath("/Root/ConveyorBelt")
        self.assertEqual(
            belt_transform,
            UsdGeom.Xformable(reference_belt).GetLocalTransformation(),
        )

    def test_belt_visuals_match_full_warehouse_adapter(self) -> None:
        def signature(root):
            return sorted(
                (str(prim.GetPath()), prim.GetTypeName())
                for prim in Usd.PrimRange(root)
                if prim.IsA(UsdGeom.Imageable)
            )

        warehouse_stage = Usd.Stage.Open(
            str(_ASSET_DIR / _VARIANTS.VISUAL_ONLY_BACKGROUND_USD),
            Usd.Stage.LoadNone,
        )
        self.assertTrue(warehouse_stage)
        warehouse_stage.Load("/Root/ConveyorBelt")

        self.assertEqual(
            signature(self.stage.GetPrimAtPath("/Root/ConveyorBelt")),
            signature(warehouse_stage.GetPrimAtPath("/Root/ConveyorBelt")),
        )


if __name__ == "__main__":
    unittest.main()
