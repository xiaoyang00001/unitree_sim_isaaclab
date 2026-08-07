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
_LEGACY_WAREHOUSE_SOURCE_NAME = "warehouse-simple6_v61.usd"
_CLEAN_WAREHOUSE_SOURCE_NAME = "warehouse-simple6_v61_visual_only.usda"
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


def _find_enabled_physics(stage: Any, usd_physics: Any) -> tuple[list[str], list[str]]:
    rigid_body_paths: list[str] = []
    collision_paths: list[str] = []
    for prim in stage.TraverseAll():
        if prim.HasAPI(usd_physics.RigidBodyAPI) and _effective_enabled(
            prim, "physics:rigidBodyEnabled"
        ):
            rigid_body_paths.append(str(prim.GetPath()))
        if prim.HasAPI(usd_physics.CollisionAPI) and _effective_enabled(
            prim, "physics:collisionEnabled"
        ):
            collision_paths.append(str(prim.GetPath()))
    return sorted(rigid_body_paths), sorted(collision_paths)


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
    adapter_rigid: list[str] = []
    adapter_collision: list[str] = []
    for prim in usd.PrimRange(conveyor_root):
        if prim.HasAPI(usd_physics.RigidBodyAPI) and _effective_enabled(
            prim, "physics:rigidBodyEnabled"
        ):
            adapter_rigid.append(str(prim.GetPath()))
        if prim.HasAPI(usd_physics.CollisionAPI) and _effective_enabled(
            prim, "physics:collisionEnabled"
        ):
            adapter_collision.append(str(prim.GetPath()))
    if adapter_rigid or adapter_collision:
        raise RuntimeError(
            "仓库 adapter 的 ConveyorBelt 子树仍有有效物理语义："
            f"rigid={adapter_rigid}, collision={adapter_collision}"
        )


def main() -> int:
    args = _parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    manifest_path = args.manifest.resolve()
    adapter = args.adapter.resolve()
    if not source.is_file():
        raise FileNotFoundError(f"缺少源资产：{source}")

    simulation_app = None
    try:
        try:
            from pxr import Usd, UsdGeom, UsdPhysics
        except ModuleNotFoundError:
            from isaacsim import SimulationApp

            # 成功路径使用 Kit fast shutdown；失败路径在下方先输出 traceback，
            # 再以明确非零码退出，避免 close() 把异常覆盖成退出码 0。
            simulation_app = SimulationApp(
                {"headless": True, "fast_shutdown": True}
            )
            from pxr import Usd, UsdGeom, UsdPhysics

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

        if args.check:
            _assert_file_matches(output, layer_text)
            _assert_file_matches(manifest_path, manifest_text)
            _assert_file_matches(adapter, adapter_text)
        else:
            _write_if_changed(output, layer_text)
            _write_if_changed(manifest_path, manifest_text)
            _write_if_changed(adapter, adapter_text)

        _validate_composed_layers(
            source_stage,
            output,
            adapter,
            source,
            Usd,
            UsdPhysics,
            UsdGeom,
        )
        print(
            "[conveyor_visual_only] PASS: "
            f"disabled rigid={len(manifest['rigid_body_overrides'])}, "
            f"collision={len(manifest['collision_overrides'])}, "
            f"visual prims={manifest['source_visual_prim_count']}",
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
