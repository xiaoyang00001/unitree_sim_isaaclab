#!/usr/bin/env python3
"""生成并校验保持中心线不变的 0.60 m A02 窄弯道视觉覆写层。

A02 是一体式 90° 弯道，根节点单轴缩放只能收窄一个接口，XY 等比缩放又会
改变弯曲半径并让两个接口错位。本生成器改为在资产根坐标中逐顶点处理：先求
顶点到“主线半直线 + 四分之一圆弧 + 支线半直线”中心线的最近点，再只把横向
偏差缩为 2/3。这样中心线上的点、弧半径与两个接口的纵向位置严格不动，材质、
拓扑、UV 和法线继续来自 NVIDIA 源资产。

需要在 Isaac Lab 环境中运行：

    python tools/build_narrow_conveyor_curve.py
    python tools/build_narrow_conveyor_curve.py --check
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import tempfile
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets"
MANIFEST_PATH = ASSET_DIR / "conveyor_belt_a02_narrow_visual.manifest.json"


@dataclass(frozen=True)
class CurveSpec:
    center_x: float
    center_y: float
    radius: float
    transverse_scale: float


@dataclass
class DeformedMesh:
    path: str
    points: Any
    extent: Any


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="只校验源资产契约和已生成的覆写层，不写文件。",
    )
    return parser.parse_args()


def _curve_spec(manifest: dict[str, Any]) -> CurveSpec:
    center_x, center_y = manifest["centerline_source_units"]["corner_center_xy"]
    return CurveSpec(
        center_x=float(center_x),
        center_y=float(center_y),
        radius=float(manifest["centerline_source_units"]["radius"]),
        transverse_scale=(
            float(manifest["target_lane_width_source_units"])
            / float(manifest["source_lane_width_source_units"])
        ),
    )


def _closest_centerline_point(x: float, y: float, spec: CurveSpec) -> tuple[float, float]:
    """返回 L 路径中心线上离 ``(x, y)`` 最近的点（均为 A02 源资产单位）。"""

    cx, cy, radius = spec.center_x, spec.center_y, spec.radius

    # 主线从圆弧南东切点沿源资产 +X 伸出；支线从西北切点沿 -Y 伸出。
    main = (max(x, cx), cy + radius)
    branch = (cx - radius, min(y, cy))

    theta = math.atan2(y - cy, x - cx)
    theta = min(max(theta, math.pi * 0.5), math.pi)
    arc = (cx + radius * math.cos(theta), cy + radius * math.sin(theta))

    candidates = (main, arc, branch)
    return min(candidates, key=lambda point: (x - point[0]) ** 2 + (y - point[1]) ** 2)


def _deform_root_point(point: Any, spec: CurveSpec, gf: Any) -> Any:
    center_x, center_y = _closest_centerline_point(float(point[0]), float(point[1]), spec)
    scale = spec.transverse_scale
    return gf.Vec3d(
        center_x + (float(point[0]) - center_x) * scale,
        center_y + (float(point[1]) - center_y) * scale,
        float(point[2]),
    )


def _source_meshes(stage: Any, usd_geom: Any) -> list[Any]:
    return sorted(
        (prim for prim in stage.TraverseAll() if prim.IsA(usd_geom.Mesh)),
        key=lambda prim: str(prim.GetPath()),
    )


def _source_geometry_fingerprint(stage: Any, usd_geom: Any) -> tuple[str, int, int]:
    """锁定会影响覆写结果的路径、根相对矩阵和原始点坐标。"""

    root = stage.GetDefaultPrim()
    cache = usd_geom.XformCache()
    digest = hashlib.sha256()
    mesh_count = 0
    point_count = 0
    for prim in _source_meshes(stage, usd_geom):
        mesh_count += 1
        path = str(prim.GetPath())
        points = usd_geom.Mesh(prim).GetPointsAttr().Get()
        if points is None:
            raise RuntimeError(f"源 Mesh 没有 points：{path}")
        point_count += len(points)
        relative, resets = cache.ComputeRelativeTransform(prim, root)
        digest.update(path.encode("utf-8"))
        digest.update(b"\0")
        digest.update(b"1" if resets else b"0")
        for row in range(4):
            for column in range(4):
                digest.update(struct.pack("<d", float(relative[row][column])))
        for point in points:
            digest.update(struct.pack("<fff", *map(float, point)))
    return digest.hexdigest(), mesh_count, point_count


def _deformed_meshes(
    stage: Any,
    spec: CurveSpec,
    gf: Any,
    vt: Any,
    usd_geom: Any,
) -> list[DeformedMesh]:
    root = stage.GetDefaultPrim()
    cache = usd_geom.XformCache()
    deformed: list[DeformedMesh] = []

    for prim in _source_meshes(stage, usd_geom):
        mesh = usd_geom.Mesh(prim)
        source_points = mesh.GetPointsAttr().Get()
        if source_points is None:
            raise RuntimeError(f"源 Mesh 没有 points：{prim.GetPath()}")
        if mesh.GetPointsAttr().GetNumTimeSamples() != 0:
            raise RuntimeError(f"暂不支持带 points 时间采样的 Mesh：{prim.GetPath()}")

        local_to_root, resets = cache.ComputeRelativeTransform(prim, root)
        if resets:
            raise RuntimeError(
                f"Mesh 中途重置变换栈，无法安全写根坐标形变：{prim.GetPath()}"
            )
        root_to_local = local_to_root.GetInverse()

        point_values = []
        min_x = min_y = min_z = math.inf
        max_x = max_y = max_z = -math.inf
        for source_point in source_points:
            root_point = local_to_root.Transform(gf.Vec3d(source_point))
            narrowed_root_point = _deform_root_point(root_point, spec, gf)
            narrowed_local_point = root_to_local.Transform(narrowed_root_point)
            narrowed = gf.Vec3f(narrowed_local_point)
            point_values.append(narrowed)
            min_x = min(min_x, float(narrowed[0]))
            min_y = min(min_y, float(narrowed[1]))
            min_z = min(min_z, float(narrowed[2]))
            max_x = max(max_x, float(narrowed[0]))
            max_y = max(max_y, float(narrowed[1]))
            max_z = max(max_z, float(narrowed[2]))

        points = vt.Vec3fArray(point_values)
        extent = vt.Vec3fArray(
            [gf.Vec3f(min_x, min_y, min_z), gf.Vec3f(max_x, max_y, max_z)]
        )
        deformed.append(DeformedMesh(str(prim.GetPath()), points, extent))
    return deformed


def _validate_centerline_contract(spec: CurveSpec, gf: Any) -> None:
    """中心线与 0.90→0.60 横向收缩必须由纯数学契约锁住。"""

    cx, cy, radius = spec.center_x, spec.center_y, spec.radius
    centerline_samples = (
        gf.Vec3d(cx + 25.0, cy + radius, 76.93),
        gf.Vec3d(cx - radius / math.sqrt(2.0), cy + radius / math.sqrt(2.0), 76.93),
        gf.Vec3d(cx - radius, cy - 25.0, 76.93),
    )
    for point in centerline_samples:
        narrowed = _deform_root_point(point, spec, gf)
        if narrowed != point:
            raise RuntimeError(f"中心线点发生漂移：{tuple(point)} -> {tuple(narrowed)}")

    half_source = 45.0
    half_target = 30.0
    main_center_y = cy + radius
    for offset in (-half_source, half_source):
        point = gf.Vec3d(cx + 10.0, main_center_y + offset, 76.93)
        narrowed = _deform_root_point(point, spec, gf)
        actual = float(narrowed[1]) - main_center_y
        expected = math.copysign(half_target, offset)
        if not math.isclose(actual, expected, abs_tol=1.0e-9):
            raise RuntimeError(f"主线横向缩放错误：expected={expected}, actual={actual}")

    branch_center_x = cx - radius
    for offset in (-half_source, half_source):
        point = gf.Vec3d(branch_center_x + offset, cy - 10.0, 76.93)
        narrowed = _deform_root_point(point, spec, gf)
        actual = float(narrowed[0]) - branch_center_x
        expected = math.copysign(half_target, offset)
        if not math.isclose(actual, expected, abs_tol=1.0e-9):
            raise RuntimeError(f"支线横向缩放错误：expected={expected}, actual={actual}")


def _validate_source_contract(
    stage: Any,
    manifest: dict[str, Any],
    usd_geom: Any,
) -> None:
    fingerprint, mesh_count, point_count = _source_geometry_fingerprint(stage, usd_geom)
    failures = []
    if fingerprint != manifest["source_geometry_sha256"]:
        failures.append(
            "source geometry sha256 "
            f"expected={manifest['source_geometry_sha256']} actual={fingerprint}"
        )
    if mesh_count != manifest["source_mesh_count"]:
        failures.append(
            f"source meshes expected={manifest['source_mesh_count']} actual={mesh_count}"
        )
    if point_count != manifest["source_point_count"]:
        failures.append(
            f"source points expected={manifest['source_point_count']} actual={point_count}"
        )
    if failures:
        raise RuntimeError("A02 源资产契约变化，需重新审计：" + "; ".join(failures))


def _write_output(
    output_path: Path,
    source_url: str,
    source_stage: Any,
    deformed: list[DeformedMesh],
    manifest: dict[str, Any],
    usd: Any,
    usd_geom: Any,
) -> None:
    with tempfile.TemporaryDirectory(prefix="a02_narrow_") as tmp_dir:
        temporary_path = Path(tmp_dir) / output_path.name
        stage = usd.Stage.CreateNew(str(temporary_path))
        if not stage:
            raise RuntimeError(f"无法创建临时 USD：{temporary_path}")
        usd_geom.SetStageMetersPerUnit(stage, usd_geom.GetStageMetersPerUnit(source_stage))
        usd_geom.SetStageUpAxis(stage, usd_geom.GetStageUpAxis(source_stage))

        root = usd_geom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(root)
        root.GetReferences().AddReference(source_url, "/World")
        root.SetDocumentation(
            "A02 visual mesh narrowed from 0.90 m to 0.60 m around its unchanged L-path centerline."
        )
        stage.GetRootLayer().customLayerData = {
            "generator": manifest["generator"],
            "sourceGeometrySha256": manifest["source_geometry_sha256"],
            "sourceLaneWidth": float(manifest["source_lane_width_source_units"]),
            "targetLaneWidth": float(manifest["target_lane_width_source_units"]),
        }

        # 引用已组合后，只在根层为每个 Mesh 作者 points/extent；拓扑、材质、
        # UV、法线和所有非几何属性仍直接来自源资产。
        for record in deformed:
            prim = stage.GetPrimAtPath(record.path)
            if not prim or not prim.IsA(usd_geom.Mesh):
                raise RuntimeError(f"输出引用中缺少源 Mesh：{record.path}")
            mesh = usd_geom.Mesh(prim)
            mesh.GetPointsAttr().Set(record.points)
            mesh.GetExtentAttr().Set(record.extent)

        stage.GetRootLayer().Save()
        del stage
        temporary_path.replace(output_path)


def _max_point_error(actual: Any, expected: Any) -> float:
    if len(actual) != len(expected):
        return math.inf
    error = 0.0
    for actual_point, expected_point in zip(actual, expected, strict=True):
        error = max(
            error,
            abs(float(actual_point[0]) - float(expected_point[0])),
            abs(float(actual_point[1]) - float(expected_point[1])),
            abs(float(actual_point[2]) - float(expected_point[2])),
        )
    return error


def _validate_output(
    output_path: Path,
    source_stage: Any,
    expected: list[DeformedMesh],
    manifest: dict[str, Any],
    usd: Any,
    usd_geom: Any,
) -> None:
    if not output_path.is_file():
        raise RuntimeError(f"缺少已生成的 A02 窄弯道资产：{output_path}")
    stage = usd.Stage.Open(str(output_path), usd.Stage.LoadAll)
    if not stage:
        raise RuntimeError(f"无法打开 A02 窄弯道资产：{output_path}")
    root = stage.GetDefaultPrim()
    if not root or str(root.GetPath()) != "/World":
        raise RuntimeError(f"A02 窄弯道 defaultPrim 错误：{root.GetPath() if root else None}")

    root_layer = stage.GetRootLayer()
    custom = root_layer.customLayerData
    if custom.get("sourceGeometrySha256") != manifest["source_geometry_sha256"]:
        raise RuntimeError("A02 窄弯道层记录的源几何哈希已过期")

    expected_by_path = {record.path: record for record in expected}
    actual_meshes = _source_meshes(stage, usd_geom)
    if {str(prim.GetPath()) for prim in actual_meshes} != set(expected_by_path):
        raise RuntimeError("A02 窄弯道组合后的 Mesh 路径集合与源资产不一致")

    max_error = 0.0
    for prim in actual_meshes:
        path = str(prim.GetPath())
        record = expected_by_path[path]
        mesh = usd_geom.Mesh(prim)
        max_error = max(max_error, _max_point_error(mesh.GetPointsAttr().Get(), record.points))
        max_error = max(max_error, _max_point_error(mesh.GetExtentAttr().Get(), record.extent))

        spec = root_layer.GetPrimAtPath(path)
        if spec is None or "points" not in spec.properties or "extent" not in spec.properties:
            raise RuntimeError(f"A02 窄弯道根层缺少稀疏 points/extent 覆写：{path}")
        # 输出层只允许覆写这两个几何数组，避免意外复制拓扑、材质或
        # 物理语义。
        authored = {property_spec.name for property_spec in spec.properties}
        if authored != {"points", "extent"}:
            raise RuntimeError(
                f"A02 窄弯道 Mesh 出现额外根层属性：{path}: {sorted(authored)}"
            )

    if max_error > 2.0e-5:
        raise RuntimeError(f"A02 窄弯道顶点与生成算法不一致：max_error={max_error}")

    source_signature = sorted(
        (
            str(prim.GetPath()),
            tuple(usd_geom.Mesh(prim).GetFaceVertexCountsAttr().Get() or ()),
            tuple(usd_geom.Mesh(prim).GetFaceVertexIndicesAttr().Get() or ()),
        )
        for prim in _source_meshes(source_stage, usd_geom)
    )
    output_signature = sorted(
        (
            str(prim.GetPath()),
            tuple(usd_geom.Mesh(prim).GetFaceVertexCountsAttr().Get() or ()),
            tuple(usd_geom.Mesh(prim).GetFaceVertexIndicesAttr().Get() or ()),
        )
        for prim in actual_meshes
    )
    if output_signature != source_signature:
        raise RuntimeError("A02 窄弯道意外改变了源 Mesh 拓扑")

    bbox_cache = usd_geom.BBoxCache(
        usd.TimeCode.Default(),
        [usd_geom.Tokens.default_, usd_geom.Tokens.render, usd_geom.Tokens.proxy],
    )
    bbox = bbox_cache.ComputeWorldBound(root).ComputeAlignedRange()
    bbox_min = tuple(round(float(value), 4) for value in bbox.GetMin())
    bbox_max = tuple(round(float(value), 4) for value in bbox.GetMax())
    print(
        "[a02_narrow] PASS "
        f"meshes={len(actual_meshes)} points={manifest['source_point_count']} "
        f"max_error={max_error:.3g} bbox={bbox_min}->{bbox_max} "
        f"size={output_path.stat().st_size / (1024 * 1024):.2f}MiB",
        flush=True,
    )


def main() -> int:
    args = _parse_args()
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    output_path = ASSET_DIR / manifest["output_asset"]
    spec = _curve_spec(manifest)

    # USD/Kit import 必须晚于 AppLauncher。
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True, device="cpu")
    simulation_app = launcher.app
    try:
        from pxr import Gf, Usd, UsdGeom, Vt

        source_stage = Usd.Stage.Open(manifest["source_asset"], Usd.Stage.LoadAll)
        if not source_stage:
            raise RuntimeError(f"无法打开 A02 源资产：{manifest['source_asset']}")
        _validate_source_contract(source_stage, manifest, UsdGeom)
        _validate_centerline_contract(spec, Gf)
        deformed = _deformed_meshes(source_stage, spec, Gf, Vt, UsdGeom)

        if not args.check:
            _write_output(
                output_path,
                manifest["source_asset"],
                source_stage,
                deformed,
                manifest,
                Usd,
                UsdGeom,
            )
            print(f"[a02_narrow] wrote {output_path}", flush=True)

        _validate_output(output_path, source_stage, deformed, manifest, Usd, UsdGeom)
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
