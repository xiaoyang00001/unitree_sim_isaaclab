#!/usr/bin/env python3
"""Generate the tracked pure-visual G1 peer LOD USDA.

This generator runs in ordinary Python.  It loads the standalone FK manifest
without importing the eager ``tasks`` package, then authors only Xform,
UsdGeom analytical primitives, and three shared PreviewSurface materials.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib.util
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
FK_MODULE_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/peer_visual_lod.py"
)
OUTPUT_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot/g1_43dof_visual_lod.usda"
)


def _load_fk_module():
    spec = importlib.util.spec_from_file_location("_peer_visual_lod_manifest", FK_MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load FK manifest: {FK_MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class Shape:
    kind: str
    dimensions: tuple[float, ...]
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: str = "Z"
    material: str = "White"


def _shape_for(link: str) -> Shape:
    side_sign = -1.0 if link.startswith("left_hand_thumb") else 1.0
    if link == "pelvis":
        return Shape("Cube", (0.24, 0.20, 0.16))
    if link == "waist_yaw_link":
        return Shape("Sphere", (0.075,), material="Accent")
    if link == "waist_roll_link":
        return Shape("Sphere", (0.075,))
    if link == "torso_link":
        return Shape("Cube", (0.25, 0.21, 0.34), (0.0, 0.0, 0.15))
    if link == "head_link":
        return Shape("Sphere", (0.09,), (0.0, 0.0, 0.43), material="Dark")
    if "hip_pitch_link" in link or "hip_roll_link" in link:
        return Shape("Sphere", (0.055,), material="Dark" if "pitch" in link else "White")
    if "hip_yaw_link" in link:
        return Shape("Capsule", (0.055, 0.18), (-0.035, 0.0, -0.09))
    if "knee_link" in link:
        return Shape("Capsule", (0.048, 0.25), (0.0, 0.0, -0.145))
    if "ankle_pitch_link" in link:
        return Shape("Sphere", (0.045,))
    if "ankle_roll_link" in link:
        return Shape("Cube", (0.22, 0.09, 0.05), (0.08, 0.0, -0.025), material="Dark")
    if "shoulder_pitch_link" in link or "shoulder_roll_link" in link:
        return Shape("Sphere", (0.047,))
    if "shoulder_yaw_link" in link:
        return Shape("Capsule", (0.042, 0.105), (0.008, 0.0, -0.052))
    if "elbow_link" in link:
        return Shape("Capsule", (0.038, 0.10), (0.052, 0.0, 0.0), axis="X")
    if "wrist_" in link:
        return Shape("Sphere", (0.030,))
    if "hand_palm_link" in link:
        return Shape("Cube", (0.095, 0.072, 0.038), (0.046, 0.0, 0.0), material="Dark")
    if "hand_thumb_0_link" in link:
        return Shape("Capsule", (0.010, 0.025), (0.013, 0.0, 0.0), axis="X", material="Dark")
    if "hand_thumb_" in link:
        return Shape("Capsule", (0.009, 0.038), (0.0, side_sign * 0.020, 0.0), axis="Y", material="Dark")
    if "hand_middle_" in link or "hand_index_" in link:
        return Shape("Capsule", (0.009, 0.040), (0.021, 0.0, 0.0), axis="X", material="Dark")
    return Shape("Sphere", (0.025,))


def _fmt_vector(values: tuple[float, ...]) -> str:
    return "(" + ", ".join(f"{value:.9g}" for value in values) + ")"


def _shape_lines(shape: Shape, indent: str) -> list[str]:
    lines = [f'{indent}def {shape.kind} "visual"', f"{indent}{{"]
    child_indent = indent + "    "
    if shape.kind == "Cube":
        lines.append(f"{child_indent}double size = 1")
        scale = tuple(value for value in shape.dimensions)
    elif shape.kind == "Sphere":
        lines.append(f"{child_indent}double radius = {shape.dimensions[0]:.9g}")
        scale = (1.0, 1.0, 1.0)
    else:
        radius, height = shape.dimensions
        lines.extend(
            [
                f'{child_indent}uniform token axis = "{shape.axis}"',
                f"{child_indent}double radius = {radius:.9g}",
                f"{child_indent}double height = {height:.9g}",
            ]
        )
        scale = (1.0, 1.0, 1.0)
    lines.extend(
        [
            f'{child_indent}uniform token purpose = "render"',
            f"{child_indent}double3 xformOp:scale = {_fmt_vector(scale)}",
            f"{child_indent}double3 xformOp:translate = {_fmt_vector(shape.translate)}",
            f'{child_indent}uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
            f"{child_indent}rel material:binding = </PeerVisualLod/Looks/{shape.material}>",
            f"{indent}}}",
        ]
    )
    return lines


def _material_lines(name: str, color: tuple[float, float, float]) -> list[str]:
    return [
        f'        def Material "{name}"',
        "        {",
        '            token outputs:surface.connect = </PeerVisualLod/Looks/' + name + "/Shader.outputs:surface>",
        '            def Shader "Shader"',
        "            {",
        '                uniform token info:id = "UsdPreviewSurface"',
        f"                color3f inputs:diffuseColor = {_fmt_vector(color)}",
        "                float inputs:metallic = 0.15",
        "                float inputs:roughness = 0.48",
        "                token outputs:surface",
        "            }",
        "        }",
    ]


def build_text() -> str:
    fk = _load_fk_module()
    transforms = {"pelvis": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}
    children: dict[str, list[str]] = {"pelvis": []}
    for spec in (*fk.JOINT_SPECS, *fk.FIXED_LINK_SPECS):
        transforms[spec.child] = (spec.xyz, spec.rpy)
        children.setdefault(spec.parent, []).append(spec.child)
        children.setdefault(spec.child, [])

    lines = [
        "#usda 1.0",
        "(",
        '    defaultPrim = "PeerVisualLod"',
        '    doc = "Pure-visual 43-DoF G1 mirror LOD; generated by tools/build_peer_visual_lod_usd.py"',
        "    metersPerUnit = 1",
        '    upAxis = "Z"',
        ")",
        "",
        'def Xform "PeerVisualLod"',
        "(",
        "    customData = {",
        '        string peerVisualLodRole = "pure_visual_fk"',
        '        string peerVisualLodVersion = "1"',
        "    }",
        ")",
        "{",
        "    double3 xformOp:translate = (0, 0, 0)",
        "    quatf xformOp:orient = (1, 0, 0, 0)",
        '    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]',
        "",
        '    def Scope "Looks"',
        "    {",
    ]
    lines.extend(_material_lines("White", (0.68, 0.72, 0.76)))
    lines.extend(_material_lines("Dark", (0.045, 0.055, 0.065)))
    lines.extend(_material_lines("Accent", (0.12, 0.42, 0.82)))
    lines.extend(["    }", ""])

    def emit_link(link: str, depth: int) -> None:
        indent = "    " * depth
        xyz, _rpy = transforms[link]
        lines.extend(
            [
                f'{indent}def Xform "{link}"',
                f"{indent}{{",
                f"{indent}    double3 xformOp:translate = {_fmt_vector(xyz)}",
                f"{indent}    quatf xformOp:orient = (1, 0, 0, 0)",
                f'{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]',
            ]
        )
        lines.extend(_shape_lines(_shape_for(link), indent + "    "))
        for child in children[link]:
            lines.append("")
            emit_link(child, depth + 1)
        lines.append(f"{indent}}}")

    emit_link("pelvis", 1)
    lines.extend(["}", ""])
    return "\n".join(lines)


def main() -> int:
    text = build_text()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {OUTPUT_PATH} ({len(text.encode('utf-8'))} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
