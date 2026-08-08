#!/usr/bin/env python3
"""生成并验证 ConveyorBelt02 的任务专用 visual-only 覆盖层。

原始 ``ConveyorBelt02.usd`` 是 45 MiB 的 crate，除三段 A08 输送机外还包含
工作台、分拣箱和装饰箱组。任务已经用独立 ``ConveyorCollider`` 提供碰撞，若
继续保留 payload 自带的 RigidBody/Collision API，会同时存在两套物理语义。

本脚本不改写原始 crate，也不复制网格。它生成一个很小的 USDA payload 包装层，
对源资产中所有当前有效的刚体和碰撞逐 Prim 写入 ``enabled = false`` 覆盖，并记录
源文件哈希和审计路径。``--check`` 会验证生成物未过期、视觉 Prim 集合未改变、
原始 payload 可解析，且 composed stage 中没有仍然有效的刚体或碰撞。

需要在能启动 Isaac Sim 5.1 的 Python 环境中运行，例如：

    conda activate env_isaaclab
    python tools/build_conveyor_visual_only_usd.py
    python tools/build_conveyor_visual_only_usd.py --check
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable


_REPO_ROOT = Path(__file__).resolve().parents[1]
_ASSET_DIR = (
    _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets"
)
_DEFAULT_SOURCE = _ASSET_DIR / "ConveyorBelt02.usd"
_DEFAULT_OUTPUT = _ASSET_DIR / "ConveyorBelt02_visual_only.usda"
_DEFAULT_MANIFEST = _ASSET_DIR / "ConveyorBelt02_visual_only.manifest.json"
_DEFAULT_ADAPTER = _ASSET_DIR / "warehouse-simple6_v61_conveyor_visual_only.usda"
_DEFAULT_WORKCELL_ADAPTER = (
    _ASSET_DIR / "conveyor_workcell_lite_conveyor_visual_only.usda"
)
_LEGACY_WAREHOUSE_SOURCE_NAME = "warehouse-simple6_v61.usd"
_CLEAN_WAREHOUSE_SOURCE_NAME = "warehouse-simple6_v61_visual_only.usda"
_WORKCELL_LITE_SOURCE_NAME = "conveyor_workcell_lite.usd"
_WORKCELL_LITE_MANIFEST_NAME = "conveyor_workcell_lite.manifest.json"
_GENERATOR_RELATIVE_PATH = "tools/build_conveyor_visual_only_usd.py"


@dataclass
class _OverrideNode:
    rigid_body: bool = False
    collision: bool = False
    children: dict[str, "_OverrideNode"] = field(default_factory=dict)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="只校验已提交的生成物，不写文件",
    )
    parser.add_argument("--source", type=Path, default=_DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=_DEFAULT_OUTPUT)
    parser.add_argument("--manifest", type=Path, default=_DEFAULT_MANIFEST)
    parser.add_argument("--adapter", type=Path, default=_DEFAULT_ADAPTER)
    parser.add_argument(
        "--workcell-adapter", type=Path, default=_DEFAULT_WORKCELL_ADAPTER
    )
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _effective_enabled(prim: Any, attribute_name: str) -> bool:
    attribute = prim.GetAttribute(attribute_name)
    if not attribute:
        return True
    value = attribute.Get()
    return True if value is None else bool(value)


def _reject_instances(prims: Iterable[Any], scope: str) -> None:
    """instanceable Prim 会让整条审计链静默漏检：默认遍历不进入 instance
    内部，而跨 instance 边界写的 enabled=false 覆盖在组合中无效。与其漏检，
    不如在生成/校验阶段直接拒绝，等人工确认后再扩展机制。"""

    instanced = [str(prim.GetPath()) for prim in prims if prim.IsInstance()]
    if instanced:
        raise RuntimeError(
            f"{scope}包含 instanceable Prim，visual-only 覆盖无法跨 instance "
            f"边界禁用物理，请先人工审计：{instanced[:10]}"
        )


def _collect_visual_signature(stage: Any, usd_geom: Any) -> list[tuple[str, str]]:
    return sorted(
        (str(prim.GetPath()), prim.GetTypeName())
        for prim in stage.TraverseAll()
        if prim.IsA(usd_geom.Imageable)
    )


def _audit_source(stage: Any, usd_physics: Any, usd_geom: Any, source: Path) -> dict[str, Any]:
    default_prim = stage.GetDefaultPrim()
    if not default_prim or default_prim.GetPath() != "/Root":
        raise RuntimeError(
            f"源资产 defaultPrim 应为 /Root，实际为 {default_prim.GetPath() if default_prim else '<none>'}"
        )

    _reject_instances(stage.TraverseAll(), "源资产")

    rigid_body_paths: list[str] = []
    collision_paths: list[str] = []
    prim_count = 0
    for prim in stage.TraverseAll():
        prim_count += 1
        if prim.HasAPI(usd_physics.RigidBodyAPI) and _effective_enabled(
            prim, "physics:rigidBodyEnabled"
        ):
            rigid_body_paths.append(str(prim.GetPath()))
        if prim.HasAPI(usd_physics.CollisionAPI) and _effective_enabled(
            prim, "physics:collisionEnabled"
        ):
            collision_paths.append(str(prim.GetPath()))

    if not rigid_body_paths or not collision_paths:
        raise RuntimeError(
            "源资产未审计到有效刚体或碰撞；资产结构可能已变化，请先人工确认"
        )

    return {
        "schema_version": 1,
        "generator": _GENERATOR_RELATIVE_PATH,
        "source": source.name,
        "source_sha256": _sha256(source),
        "source_default_prim": str(default_prim.GetPath()),
        "source_prim_count": prim_count,
        "source_visual_prim_count": len(_collect_visual_signature(stage, usd_geom)),
        "rigid_body_overrides": sorted(rigid_body_paths),
        "collision_overrides": sorted(collision_paths),
    }


def _build_override_tree(
    rigid_body_paths: Iterable[str], collision_paths: Iterable[str]
) -> _OverrideNode:
    root = _OverrideNode()
    rigid_body_set = set(rigid_body_paths)
    collision_set = set(collision_paths)
    for path in sorted(rigid_body_set | collision_set):
        # USD SdfPath 始终使用正斜杠，不能交给宿主 Path 解析，否则 Windows
        # 会把根路径转换成反斜杠并误判。
        parts = path.split("/")
        if len(parts) < 3 or parts[:2] != ["", "Root"]:
            raise RuntimeError(f"覆盖路径不在 /Root 下：{path}")
        node = root
        for name in parts[2:]:
            node = node.children.setdefault(name, _OverrideNode())
        node.rigid_body = path in rigid_body_set
        node.collision = path in collision_set
    return root


def _render_children(node: _OverrideNode, indent: int) -> list[str]:
    lines: list[str] = []
    prefix = " " * indent
    for name in sorted(node.children):
        child = node.children[name]
        lines.append(f'{prefix}over {json.dumps(name)}')
        lines.append(f"{prefix}{{")
        if child.rigid_body:
            lines.append(f"{prefix}    bool physics:rigidBodyEnabled = false")
        if child.collision:
            lines.append(f"{prefix}    bool physics:collisionEnabled = false")
        if (child.rigid_body or child.collision) and child.children:
            lines.append("")
        lines.extend(_render_children(child, indent + 4))
        lines.append(f"{prefix}}}")
    return lines


def _render_visual_only_layer(manifest: dict[str, Any], meters_per_unit: float, up_axis: str) -> str:
    tree = _build_override_tree(
        manifest["rigid_body_overrides"], manifest["collision_overrides"]
    )
    rigid_count = len(manifest["rigid_body_overrides"])
    collision_count = len(manifest["collision_overrides"])
    lines = [
        "#usda 1.0",
        "(",
        '    defaultPrim = "Root"',
        f"    metersPerUnit = {meters_per_unit!r}",
        f'    upAxis = "{up_axis}"',
        ")",
        "",
        'def Xform "Root" (',
        "    customData = {",
        f'        string generatedBy = "{manifest["generator"]}"',
        f'        string sourceSha256 = "{manifest["source_sha256"]}"',
        f"        int disabledCollisionCount = {collision_count}",
        f"        int disabledRigidBodyCount = {rigid_count}",
        "    }",
        f"    prepend payload = @./{manifest['source']}@",
        ")",
        "{",
        "",
    ]
    lines.extend(_render_children(tree, 4))
    lines.extend(["}", ""])
    return "\n".join(lines)


def _resolve_warehouse_base_name(asset_dir: Path) -> str:
    """优先叠加根层物理已清理的 wrapper；独立分支回退 legacy v61。"""

    if (asset_dir / _CLEAN_WAREHOUSE_SOURCE_NAME).is_file():
        return _CLEAN_WAREHOUSE_SOURCE_NAME
    return _LEGACY_WAREHOUSE_SOURCE_NAME


def _render_warehouse_adapter(output_name: str, warehouse_base_name: str) -> str:
    return "\n".join(
        [
            "#usda 1.0",
            "(",
            '    defaultPrim = "Root"',
            f"    subLayers = [@./{warehouse_base_name}@]",
            ")",
            "",
            'over "Root"',
            "{",
            '    over "ConveyorBelt" (',
            "        delete payload = @./ConveyorBelt02.usd@",
            f"        prepend payload = @./{output_name}@",
            "    )",
            "    {",
            "    }",
            "}",
            "",
        ]
    )


def _render_workcell_lite_adapter(warehouse_adapter_name: str) -> str:
    """渲染 workcell-lite 背景专用的 conveyor visual-only 组合层。

    workcell-lite 的 ``/Root/ConveyorBelt`` 经 reference 取自 clean wrapper，
    payload 定义在引用目标的 layer stack 内部，上层 ``delete payload`` 跨
    reference arc 无效。因此这里把 reference 重定向到完整仓库 adapter 的同名
    子树：payload 替换在那个 layer stack 内生效，v61 对该 Prim 的 xform 与
    子 Prim 覆盖全部保留，且不会把仓库其它 Prim 带回组合。
    """

    return "\n".join(
        [
            "#usda 1.0",
            "(",
            '    defaultPrim = "Root"',
            f"    subLayers = [@./{_WORKCELL_LITE_SOURCE_NAME}@]",
            ")",
            "",
            'over "Root"',
            "{",
            '    over "ConveyorBelt" (',
            "        delete references = "
            f"@./{_CLEAN_WAREHOUSE_SOURCE_NAME}@</Root/ConveyorBelt>",
            "        prepend references = "
            f"@./{warehouse_adapter_name}@</Root/ConveyorBelt>",
            "    )",
            "    {",
            "    }",
            "}",
            "",
        ]
    )


def _expected_manifest_text(manifest: dict[str, Any]) -> str:
    return json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"


def _assert_file_matches(path: Path, expected: str) -> None:
    if not path.is_file():
        raise RuntimeError(f"缺少生成物：{path}")
    actual = path.read_text(encoding="utf-8")
    if actual != expected:
        raise RuntimeError(
            f"生成物已过期：{path}\n请运行 python {_GENERATOR_RELATIVE_PATH}"
        )


def _write_if_changed(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file() and path.read_text(encoding="utf-8") == content:
        return
    path.write_text(content, encoding="utf-8")


def _find_enabled_physics_in(
    prims: Iterable[Any], usd_physics: Any
) -> tuple[list[str], list[str]]:
    rigid_body_paths: list[str] = []
    collision_paths: list[str] = []
    for prim in prims:
        if prim.HasAPI(usd_physics.RigidBodyAPI) and _effective_enabled(
            prim, "physics:rigidBodyEnabled"
        ):
            rigid_body_paths.append(str(prim.GetPath()))
        if prim.HasAPI(usd_physics.CollisionAPI) and _effective_enabled(
            prim, "physics:collisionEnabled"
        ):
            collision_paths.append(str(prim.GetPath()))
    return sorted(rigid_body_paths), sorted(collision_paths)


def _find_enabled_physics(stage: Any, usd_physics: Any) -> tuple[list[str], list[str]]:
    return _find_enabled_physics_in(stage.TraverseAll(), usd_physics)


def _validate_composed_layers(
    source_stage: Any,
    output: Path,
    adapter: Path,
    source: Path,
    usd: Any,
    usd_physics: Any,
    usd_geom: Any,
) -> None:
    visual_stage = usd.Stage.Open(str(output), usd.Stage.LoadAll)
    if not visual_stage:
        raise RuntimeError(f"无法打开 visual-only 层：{output}")
    if visual_stage.GetDefaultPrim().GetPath() != "/Root":
        raise RuntimeError("visual-only 层 defaultPrim 不是 /Root")

    used_real_paths = {
        Path(layer.realPath).resolve()
        for layer in visual_stage.GetUsedLayers()
        if layer.realPath
    }
    if source.resolve() not in used_real_paths:
        raise RuntimeError("visual-only 层没有解析到原始 ConveyorBelt02.usd payload")

    source_visuals = _collect_visual_signature(source_stage, usd_geom)
    visual_only_visuals = _collect_visual_signature(visual_stage, usd_geom)
    if visual_only_visuals != source_visuals:
        raise RuntimeError("visual-only 层改变了源资产的视觉 Prim 集合或类型")

    rigid_body_paths, collision_paths = _find_enabled_physics(
        visual_stage, usd_physics
    )
    if rigid_body_paths or collision_paths:
        raise RuntimeError(
            "visual-only 层仍有有效物理语义："
            f"rigid={rigid_body_paths}, collision={collision_paths}"
        )

    # adapter 的组成验证只加载 ConveyorBelt payload；其它仓库资产不是本产物的职责。
    adapter_stage = usd.Stage.Open(str(adapter), usd.Stage.LoadNone)
    if not adapter_stage:
        raise RuntimeError(f"无法打开仓库 adapter：{adapter}")
    adapter_stage.Load("/Root/ConveyorBelt")
    conveyor_root = adapter_stage.GetPrimAtPath("/Root/ConveyorBelt")
    if not conveyor_root or not conveyor_root.IsLoaded():
        raise RuntimeError("仓库 adapter 未能加载 /Root/ConveyorBelt")
    _reject_instances(
        usd.PrimRange.AllPrims(conveyor_root), "仓库 adapter 的 ConveyorBelt 子树"
    )
    adapter_rigid, adapter_collision = _find_enabled_physics_in(
        usd.PrimRange.AllPrims(conveyor_root), usd_physics
    )
    if adapter_rigid or adapter_collision:
        raise RuntimeError(
            "仓库 adapter 的 ConveyorBelt 子树仍有有效物理语义："
            f"rigid={adapter_rigid}, collision={adapter_collision}"
        )


def _validate_workcell_adapter(
    workcell_adapter: Path,
    warehouse_adapter: Path,
    usd: Any,
    usd_geom: Any,
    usd_physics: Any,
    sdf: Any,
) -> None:
    """验证 workcell-lite 组合层：输送机纯视觉化且轻量工位契约未被破坏。"""

    manifest_path = workcell_adapter.parent / _WORKCELL_LITE_MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    stage = usd.Stage.Open(str(workcell_adapter), usd.Stage.LoadAll)
    if not stage:
        raise RuntimeError(f"无法打开 workcell-lite 组合层：{workcell_adapter}")
    if stage.GetDefaultPrim().GetPath() != "/Root":
        raise RuntimeError("workcell-lite 组合层 defaultPrim 不是 /Root")

    belt = stage.GetPrimAtPath("/Root/ConveyorBelt")
    if not belt or not belt.IsActive() or not belt.IsLoaded():
        raise RuntimeError("workcell-lite 组合层未能组合出 /Root/ConveyorBelt")

    _reject_instances(usd.PrimRange.AllPrims(belt), "workcell-lite ConveyorBelt 子树")

    belt_rigid, belt_collision = _find_enabled_physics_in(
        usd.PrimRange.AllPrims(belt), usd_physics
    )
    if belt_rigid or belt_collision:
        raise RuntimeError(
            "workcell-lite 组合层的 ConveyorBelt 子树仍有有效物理语义："
            f"rigid={belt_rigid}, collision={belt_collision}"
        )

    # 全 stage 不允许任何刚体；碰撞只允许轻量工位刻意保留的静态边界——地面、
    # 工位地贴（FloorZone_*/Stripe_* 引用的官方 SM_FloorDecal 资产带静态碰撞，
    # Kit 能解析 S3 时会进入组合，与完整场景行为一致；离线 pxr 解析不到时天然
    # 缺席）与仓库外墙（keep_root_prefixes，当前即 SM_WallA_）。名单之外的根级
    # Prim 已由下方 keep 白名单检查兜底，这里的前缀匹配不会放走新增子树。
    stage_rigid, stage_collision = _find_enabled_physics(stage, usd_physics)
    boundary_prefixes = tuple(
        f"/Root/{prefix}"
        for prefix in tuple(manifest["keep_root_prefixes"])
        + ("GroundPlane", "FloorZone_", "Stripe_")
    )
    leaked_collision = [
        path for path in stage_collision if not path.startswith(boundary_prefixes)
    ]
    if stage_rigid or leaked_collision:
        raise RuntimeError(
            "workcell-lite 组合层出现静态边界以外的有效物理语义："
            f"rigid={stage_rigid}, collision={leaked_collision}"
        )

    # 只验"不多"会在上游把边界碰撞整体清空时静默通过；地面/外墙碰撞丢失意味着
    # 机器人和道具会穿地、出界，必须同时验"仍在"。
    ground_collision_path = "/Root/GroundPlane/CollisionPlane"
    wall_prefixes = tuple(f"/Root/{p}" for p in manifest["keep_root_prefixes"])
    wall_collision_count = sum(
        1 for path in stage_collision if path.startswith(wall_prefixes)
    )
    if ground_collision_path not in stage_collision or wall_collision_count == 0:
        raise RuntimeError(
            "workcell-lite 组合层丢失了刻意保留的静态边界碰撞："
            f"ground={ground_collision_path in stage_collision}, "
            f"wall collisions={wall_collision_count}"
        )

    # 视觉保真：组合层的输送机子树必须与完整仓库 adapter 的同名子树一致。
    warehouse_stage = usd.Stage.Open(str(warehouse_adapter), usd.Stage.LoadNone)
    if not warehouse_stage:
        raise RuntimeError(f"无法打开仓库 adapter：{warehouse_adapter}")
    warehouse_stage.Load("/Root/ConveyorBelt")
    reference_belt = warehouse_stage.GetPrimAtPath("/Root/ConveyorBelt")

    def _belt_visual_signature(root: Any) -> list[tuple[str, str]]:
        return sorted(
            (str(prim.GetPath()), prim.GetTypeName())
            for prim in usd.PrimRange(root)
            if prim.IsA(usd_geom.Imageable)
        )

    if _belt_visual_signature(belt) != _belt_visual_signature(reference_belt):
        raise RuntimeError(
            "workcell-lite 组合层的 ConveyorBelt 视觉 Prim 集合与完整仓库 adapter 不一致"
        )

    # v61 对 ConveyorBelt 的本地 xform 必须在 reference 重定向后存活，否则输送机
    # 会从工位错位。比较生效的本地变换矩阵而不是单个属性作者值——xformOpOrder
    # 被覆盖时 translate 属性仍可读到旧值但已不生效。
    belt_transform = usd_geom.Xformable(belt).GetLocalTransformation()
    reference_transform = usd_geom.Xformable(reference_belt).GetLocalTransformation()
    if belt_transform != reference_transform:
        raise RuntimeError(
            "workcell-lite 组合层的 ConveyorBelt 本地变换与完整仓库 adapter 不一致："
            f"{belt_transform} != {reference_transform}"
        )
    v61_layer = sdf.Layer.FindOrOpen(
        str(workcell_adapter.parent / _LEGACY_WAREHOUSE_SOURCE_NAME)
    )
    if v61_layer:
        belt_spec = v61_layer.GetPrimAtPath("/Root/ConveyorBelt")
        translate_spec = (
            belt_spec.properties.get("xformOp:translate") if belt_spec else None
        )
        if translate_spec is not None:
            expected_translate = translate_spec.default
            actual_translate = belt_transform.ExtractTranslation()
            if actual_translate != expected_translate:
                raise RuntimeError(
                    "workcell-lite 组合层丢失了 v61 的 ConveyorBelt 位移："
                    f"expected={expected_translate}, actual={actual_translate}"
                )

    # 轻量工位契约：必需 Prim 保持 active，被裁 Prim 与运行时类型保持缺席。
    def _effectively_active(path: str) -> bool:
        prim = stage.GetPrimAtPath(path)
        return bool(prim) and prim.IsActive()

    missing_required = [
        path
        for path in manifest["required_active_prims"]
        if not _effectively_active(path)
    ]
    returned_pruned = [
        path
        for path in manifest["required_inactive_prims"]
        if _effectively_active(path)
    ]
    if missing_required or returned_pruned:
        raise RuntimeError(
            "workcell-lite 组合层破坏了轻量工位契约："
            f"inactive_kept={missing_required}, active_pruned={returned_pruned}"
        )

    # TraverseAll+IsActive 口径：默认 Traverse() 会跳过 S3 层未解析时仅存的
    # over Prim，导致审计范围随远端可解析性漂移；物理审计同款口径已对齐。
    forbidden_types = set(manifest["forbidden_active_type_names"])
    forbidden_type_prims = [
        f"{prim.GetPath()}:{prim.GetTypeName()}"
        for prim in stage.TraverseAll()
        if prim.IsActive() and prim.GetTypeName() in forbidden_types
    ]
    if forbidden_type_prims:
        raise RuntimeError(
            f"workcell-lite 组合层出现被禁止的运行时 Prim 类型：{forbidden_type_prims}"
        )

    # 根级白名单：组合不允许把清单之外的仓库道具带回场景。GetAllChildren
    # 连 over-only 子 Prim 一起检查，防止经 adapter 覆盖悄悄挂新子树。
    keep_exact = set(manifest["keep_root_exact"])
    keep_prefixes = tuple(manifest["keep_root_prefixes"])
    unexpected_roots = sorted(
        prim.GetName()
        for prim in stage.GetPrimAtPath("/Root").GetAllChildren()
        if prim.GetName() not in keep_exact
        and not prim.GetName().startswith(keep_prefixes)
    )
    if unexpected_roots:
        raise RuntimeError(
            f"workcell-lite 组合层引入了白名单之外的根级 Prim：{unexpected_roots}"
        )

    # 计数口径说明：TraverseAll+IsActive 含 S3 未解析时的 over Prim（本机离线
    # 实测 829），比 lite 生成器在 Kit 下的 Traverse() 口径更宽，门槛仍远未触顶。
    limits = manifest["audit_limits"]
    active_prims = sum(1 for prim in stage.TraverseAll() if prim.IsActive())
    used_layers = len(stage.GetUsedLayers())
    failures = []
    if active_prims > limits["max_lite_active_prims"]:
        failures.append(
            f"active prims {active_prims} > {limits['max_lite_active_prims']}"
        )
    if used_layers > limits["max_lite_used_layers"]:
        failures.append(f"used layers {used_layers} > {limits['max_lite_used_layers']}")
    if failures:
        raise RuntimeError("workcell-lite 组合层超出轻量门槛：" + "; ".join(failures))


def main() -> int:
    args = _parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    manifest_path = args.manifest.resolve()
    adapter = args.adapter.resolve()
    workcell_adapter = args.workcell_adapter.resolve()
    if not source.is_file():
        raise FileNotFoundError(f"缺少源资产：{source}")
    if workcell_adapter.parent != adapter.parent:
        raise ValueError(
            "workcell adapter 与仓库 adapter 必须在同一目录，组合层用相对路径引用后者："
            f"{workcell_adapter.parent} != {adapter.parent}"
        )

    simulation_app = None
    try:
        try:
            from pxr import Sdf, Usd, UsdGeom, UsdPhysics
        except ModuleNotFoundError:
            from isaacsim import SimulationApp

            # 成功路径使用 Kit fast shutdown；失败路径在下方先输出 traceback，
            # 再以明确非零码退出，避免 close() 把异常覆盖成退出码 0。
            simulation_app = SimulationApp(
                {"headless": True, "fast_shutdown": True}
            )
            from pxr import Sdf, Usd, UsdGeom, UsdPhysics

        source_stage = Usd.Stage.Open(str(source), Usd.Stage.LoadAll)
        if not source_stage:
            raise RuntimeError(f"无法打开源资产：{source}")
        manifest = _audit_source(source_stage, UsdPhysics, UsdGeom, source)
        layer_text = _render_visual_only_layer(
            manifest,
            float(source_stage.GetMetadata("metersPerUnit")),
            str(source_stage.GetMetadata("upAxis")),
        )
        manifest_text = _expected_manifest_text(manifest)
        adapter_text = _render_warehouse_adapter(
            output.name, _resolve_warehouse_base_name(adapter.parent)
        )
        # workcell-lite 是集成分支产物；独立功能分支没有它时跳过组合层，
        # 保持本生成器在两类检出上都可用。
        workcell_source = workcell_adapter.parent / _WORKCELL_LITE_SOURCE_NAME
        workcell_adapter_text = (
            _render_workcell_lite_adapter(adapter.name)
            if workcell_source.is_file()
            else None
        )

        if args.check:
            _assert_file_matches(output, layer_text)
            _assert_file_matches(manifest_path, manifest_text)
            _assert_file_matches(adapter, adapter_text)
            if workcell_adapter_text is not None:
                _assert_file_matches(workcell_adapter, workcell_adapter_text)
        else:
            _write_if_changed(output, layer_text)
            _write_if_changed(manifest_path, manifest_text)
            _write_if_changed(adapter, adapter_text)
            if workcell_adapter_text is not None:
                _write_if_changed(workcell_adapter, workcell_adapter_text)

        _validate_composed_layers(
            source_stage,
            output,
            adapter,
            source,
            Usd,
            UsdPhysics,
            UsdGeom,
        )
        if workcell_adapter_text is not None:
            _validate_workcell_adapter(
                workcell_adapter,
                adapter,
                Usd,
                UsdGeom,
                UsdPhysics,
                Sdf,
            )
        workcell_status = (
            "ok"
            if workcell_adapter_text is not None
            else f"skipped (no {_WORKCELL_LITE_SOURCE_NAME})"
        )
        print(
            "[conveyor_visual_only] PASS: "
            f"disabled rigid={len(manifest['rigid_body_overrides'])}, "
            f"collision={len(manifest['collision_overrides'])}, "
            f"visual prims={manifest['source_visual_prim_count']}, "
            f"workcell adapter={workcell_status}",
            flush=True,
        )
    except BaseException as error:
        if simulation_app is not None:
            traceback.print_exc()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(130 if isinstance(error, KeyboardInterrupt) else 1)
        raise

    if simulation_app is not None:
        # Isaac Sim 5.1 的 graceful shutdown 在并行 Kit 进程存在时可能长时间
        # 卡住；一次性构建工具使用官方 fast shutdown，输出已在上方 flush。
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
