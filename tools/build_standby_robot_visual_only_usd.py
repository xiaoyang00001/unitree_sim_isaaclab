#!/usr/bin/env python3
"""Generate the full-fidelity, physics-free G1 standby asset.

The output references ``g1_43dof_peer.usd`` so its meshes and materials are
identical to the two working SONIC robots.  All three source variant sets are
selected as ``None`` to remove physics, joints, and sensors.  The SONIC default
joint pose is then baked into the 44 sibling body Xforms so the static robots
also have the same initial stance as the working pair.

This generator runs in ordinary Python.  Use ``--check`` to verify that the
tracked USDA still matches both the pure-Python FK manifest and source hash.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
FK_MODULE_PATH = (
    REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/peer_visual_lod.py"
)
ASSET_DIR = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot"
)
SOURCE_PATH = ASSET_DIR / "g1_43dof_peer.usd"
BASE_SOURCE_PATH = ASSET_DIR / "configuration/g1_43dof_peer_base.usd"
OUTPUT_PATH = ASSET_DIR / "g1_43dof_standby_visual_only.usda"


def _load_fk_module():
    spec = importlib.util.spec_from_file_location("_standby_visual_fk", FK_MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load FK manifest: {FK_MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _visual_dependency_sha256() -> str:
    """Hash the base layer and every referenced per-link visual crate."""

    paths = (BASE_SOURCE_PATH, *sorted(BASE_SOURCE_PATH.parent.glob("*.tmp.usd")))
    if len(paths) != 50:
        raise RuntimeError(
            f"expected base layer + 49 visual crates, received {len(paths)} files"
        )
    digest = hashlib.sha256()
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(f"standby visual dependency is missing: {path}")
        digest.update(path.relative_to(ASSET_DIR).as_posix().encode("utf-8"))
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


def sonic_default_joint_positions(joint_names: tuple[str, ...]) -> tuple[float, ...]:
    """Expand ``make_sonic_robot_cfg`` regex defaults into the 43-name wire order."""

    result: list[float] = []
    for name in joint_names:
        value = 0.0
        if name.endswith("_hip_pitch_joint"):
            value = -0.312
        elif name.endswith("_knee_joint"):
            value = 0.669
        elif name.endswith("_ankle_pitch_joint"):
            value = -0.363
        elif name.endswith("_elbow_joint"):
            value = 0.6
        elif name in {"left_shoulder_pitch_joint", "right_shoulder_pitch_joint"}:
            value = 0.2
        elif name == "left_shoulder_roll_joint":
            value = 0.2
        elif name == "right_shoulder_roll_joint":
            value = -0.2
        result.append(value)
    return tuple(result)


def _format_number(value: float) -> str:
    if abs(value) < 5.0e-13:
        value = 0.0
    return f"{value:.12g}"


def _matrix_rows_for_usd(matrix) -> tuple[tuple[float, ...], ...]:
    """Transpose column-vector FK output into USD/Gf row-vector convention."""

    return tuple(tuple(matrix[column][row] for column in range(4)) for row in range(4))


def _matrix_lines(matrix, indent: str) -> list[str]:
    rows = _matrix_rows_for_usd(matrix)
    return [
        f"{indent}({', '.join(_format_number(value) for value in row)})"
        + ("," if index < 3 else "")
        for index, row in enumerate(rows)
    ]


def build_text(source_path: Path = SOURCE_PATH) -> str:
    if not source_path.is_file():
        raise FileNotFoundError(f"standby visual source is missing: {source_path}")

    fk = _load_fk_module()
    joint_positions = sonic_default_joint_positions(fk.JOINT_NAMES)
    transforms = fk.forward_kinematics(
        (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        joint_positions,
        fk.JOINT_NAMES,
    )
    body_links = ("pelvis", *(spec.child for spec in fk.JOINT_SPECS))
    if len(body_links) != 44 or len(set(body_links)) != 44:
        raise RuntimeError(f"expected 44 unique body links, received {len(body_links)}")

    lines = [
        "#usda 1.0",
        "(",
        '    defaultPrim = "StandbyG1Visual"',
        '    doc = "Full SONIC G1 visuals in the baked default stance; no physics"',
        "    metersPerUnit = 1",
        '    upAxis = "Z"',
        ")",
        "",
        'def Xform "StandbyG1Visual"',
        "(",
        "    customData = {",
        '        string generatedBy = "tools/build_standby_robot_visual_only_usd.py"',
        '        string sourceAsset = "g1_43dof_peer.usd"',
        f'        string sourceSha256 = "{_sha256(source_path)}"',
        f'        string visualDependencySha256 = "{_visual_dependency_sha256()}"',
        '        string standbyVisualRole = "full_fidelity_baked_pose_no_physics"',
        "    }",
        '    prepend references = @./g1_43dof_peer.usd@',
        "    variants = {",
        '        string Physics = "None"',
        '        string Robot = "None"',
        '        string Sensor = "None"',
        "    }",
        ")",
        "{",
    ]

    for index, link_name in enumerate(body_links):
        lines.extend(
            [
                f'    over "{link_name}"',
                "    {",
                "        matrix4d xformOp:transform = (",
                *_matrix_lines(transforms[link_name], "            "),
                "        )",
                '        uniform token[] xformOpOrder = ["xformOp:transform"]',
                "    }",
            ]
        )
        if index != len(body_links) - 1:
            lines.append("")
    lines.extend(["}", ""])
    return "\n".join(lines)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify without writing")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    expected = build_text()
    if args.check:
        if not OUTPUT_PATH.is_file():
            print(f"missing generated asset: {OUTPUT_PATH}", file=sys.stderr)
            return 1
        if OUTPUT_PATH.read_text(encoding="utf-8") != expected:
            print(
                "standby G1 asset is stale; rerun "
                "python3 tools/build_standby_robot_visual_only_usd.py",
                file=sys.stderr,
            )
            return 1
        print(f"OK: {OUTPUT_PATH}")
        return 0

    OUTPUT_PATH.write_text(expected, encoding="utf-8", newline="\n")
    print(f"wrote {OUTPUT_PATH} ({len(expected.encode('utf-8'))} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
