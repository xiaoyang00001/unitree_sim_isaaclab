"""Apply the reference Unitree G1 appearance to the SONIC articulation.

The active SONIC robot is generated from the 43-DoF URDF because its joints,
inertias, collision geometry, and PhysX properties are part of the control
validation.  Isaac Sim's STL/URDF conversion currently replaces most of the
per-link URDF colors with a white material, however.  This module restores the
appearance from the repository's known-good G1-Dex3 USD without replacing any
part of the active articulation.

Only each link's ``visuals`` prim receives a USD visual-material binding.  The
parallel ``collisions`` prim, physics materials, rigid bodies, masses, inertias,
and joint drives are intentionally untouched.

This module must be imported after ``AppLauncher``/``SimulationApp`` has been
created because it imports Omniverse USD bindings.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdShade


_REFERENCE_ASSET = (
    Path(__file__).resolve().parents[1]
    / "assets/robots/g1-29dof_wholebody_dex3/g1_29dof_with_dex3_rev_1_0.usd"
)
_REFERENCE_ROOT = Sdf.Path("/g1_29dof_with_hand_rev_1_0/Looks")

_TARGET_LOOKS_ROOT = Sdf.Path("/World/Looks/G1Sonic")
_TARGET_MATERIAL_PATHS = {
    "white": _TARGET_LOOKS_ROOT.AppendChild("WhiteMetal"),
    "dark": _TARGET_LOOKS_ROOT.AppendChild("DarkMetal"),
    "logo": _TARGET_LOOKS_ROOT.AppendChild("LogoGray"),
}
_REFERENCE_MATERIAL_PATHS = {
    "white": _REFERENCE_ROOT.AppendChild("DefaultMaterial"),
    "dark": _REFERENCE_ROOT.AppendChild("material_CAD1EE"),
    "logo": _REFERENCE_ROOT.AppendChild("DefaultMaterial_0"),
}


# This is the material assignment authored in the reference USD.  In
# particular, its pelvis is white even though the older source URDF marks the
# pelvis as dark.  Following the USD is what reproduces the supplied image.
_DARK_LINK_NAMES = frozenset(
    {
        "left_hip_pitch_link",
        "right_hip_pitch_link",
        "left_ankle_roll_link",
        "right_ankle_roll_link",
        "waist_yaw_link",
        "head_link",
        "left_hand_palm_link",
        "left_hand_thumb_0_link",
        "left_hand_thumb_1_link",
        "left_hand_thumb_2_link",
        "left_hand_middle_0_link",
        "left_hand_middle_1_link",
        "left_hand_index_0_link",
        "left_hand_index_1_link",
        "right_hand_palm_link",
        "right_hand_thumb_0_link",
        "right_hand_thumb_1_link",
        "right_hand_thumb_2_link",
        "right_hand_middle_0_link",
        "right_hand_middle_1_link",
        "right_hand_index_0_link",
        "right_hand_index_1_link",
    }
)

_WHITE_LINK_NAMES = frozenset(
    {
        "pelvis",
        "pelvis_contour_link",
        "left_hip_roll_link",
        "left_hip_yaw_link",
        "left_knee_link",
        "left_ankle_pitch_link",
        "right_hip_roll_link",
        "right_hip_yaw_link",
        "right_knee_link",
        "right_ankle_pitch_link",
        "waist_roll_link",
        "torso_link",
        "left_shoulder_pitch_link",
        "left_shoulder_roll_link",
        "left_shoulder_yaw_link",
        "left_elbow_link",
        "left_wrist_roll_link",
        "left_wrist_pitch_link",
        "left_wrist_yaw_link",
        "right_shoulder_pitch_link",
        "right_shoulder_roll_link",
        "right_shoulder_yaw_link",
        "right_elbow_link",
        "right_wrist_roll_link",
        "right_wrist_pitch_link",
        "right_wrist_yaw_link",
        # These links exist in the reference USD but are optional in the
        # current URDF carrier.  Keeping them mapped makes this helper safe for
        # the camera-equipped G1-Dex3 variant as well.
        "left_hand_camera_base_link",
        "right_hand_camera_base_link",
    }
)

_LOGO_LINK_NAMES = frozenset({"logo_link"})
_LINK_MATERIAL_KIND = {
    **{name: "white" for name in _WHITE_LINK_NAMES},
    **{name: "dark" for name in _DARK_LINK_NAMES},
    **{name: "logo" for name in _LOGO_LINK_NAMES},
}

# The URDF converter folds fixed-child visuals into their nearest moving
# parent's visual instance. These three roots contain more than one material
# kind and therefore require visual-only uninstancing before their individual
# meshes can be rebound. All other visual roots remain instanceable.
_MIXED_VISUAL_ROOT_LINK_NAMES = frozenset(
    {"torso_link", "left_wrist_yaw_link", "right_wrist_yaw_link"}
)
_HOMOGENEOUS_FOLDED_LINK_NAMES = {
    # pelvis_contour_link is fixed to pelvis and folded into the same visual
    # instance. Both use the reference white material, so no uninstancing is
    # needed, but both logical links are included in the application report.
    "pelvis": ("pelvis_contour_link",),
}


@dataclass(frozen=True)
class G1SonicVisualMaterialReport:
    """Summary of one visual-material application pass."""

    white_links: int
    dark_links: int
    logo_links: int
    unmapped_visual_links: tuple[str, ...]

    @property
    def bound_links(self) -> int:
        return self.white_links + self.dark_links + self.logo_links


def _get_current_stage() -> Usd.Stage:
    # Import lazily so this file remains explicit about its post-AppLauncher
    # requirement and can still be used with a caller-provided stage in tests.
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("the current Omniverse USD stage is not initialized")
    return stage


def _reference_material(
    stage: Usd.Stage,
    *,
    target_path: Sdf.Path,
    source_path: Sdf.Path,
) -> UsdShade.Material:
    """Compose exactly one material prim from the local reference G1 USD."""

    material_prim = stage.DefinePrim(target_path, "Material")
    references = material_prim.GetReferences()
    references.ClearReferences()
    if not references.AddReference(str(_REFERENCE_ASSET), source_path):
        raise RuntimeError(
            f"failed to reference G1 material {source_path} from {_REFERENCE_ASSET}"
        )

    material = UsdShade.Material(material_prim)
    if not material:
        raise RuntimeError(f"referenced prim is not a material: {target_path}")
    return material


def _create_reference_materials(stage: Usd.Stage) -> dict[str, UsdShade.Material]:
    if not _REFERENCE_ASSET.is_file():
        raise FileNotFoundError(
            f"reference G1 visual USD is missing: {_REFERENCE_ASSET}"
        )

    stage.DefinePrim(_TARGET_LOOKS_ROOT.GetParentPath(), "Scope")
    stage.DefinePrim(_TARGET_LOOKS_ROOT, "Scope")
    return {
        kind: _reference_material(
            stage,
            target_path=_TARGET_MATERIAL_PATHS[kind],
            source_path=source_path,
        )
        for kind, source_path in _REFERENCE_MATERIAL_PATHS.items()
    }


def _bind_visual_prim(
    prim: Usd.Prim,
    material: UsdShade.Material,
) -> None:
    """Bind a visual material with enough strength to replace STL defaults."""

    binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
    if not binding_api.Bind(material, UsdShade.Tokens.strongerThanDescendants):
        raise RuntimeError(
            f"failed to bind {material.GetPath()} to {prim.GetPath()}"
        )


def _collect_visual_link_prims(robot_prim: Usd.Prim) -> list[tuple[Usd.Prim, Usd.Prim]]:
    """Collect link/visual pairs without entering mesh or collision instances."""

    result: list[tuple[Usd.Prim, Usd.Prim]] = []
    pending = [robot_prim]
    while pending:
        prim = pending.pop()
        visuals_prim = prim.GetChild("visuals")
        if visuals_prim.IsValid():
            result.append((prim, visuals_prim))

        for child in prim.GetChildren():
            # Visual and collision scopes are instance roots produced by the
            # URDF converter. Fixed child links, on the other hand, remain
            # ordinary Xforms and must still be traversed (head, logo, palms,
            # and pelvis contour).
            if child.GetName() in {"visuals", "collisions", "joints", "Looks"}:
                continue
            if child.IsInstance() or child.IsInstanceProxy():
                continue
            pending.append(child)
    return result


def _collect_visual_geometry_prims(visuals_prim: Usd.Prim) -> list[Usd.Prim]:
    """Return render geometry below one already-uninstanced visual root."""

    result: list[Usd.Prim] = []
    pending = list(visuals_prim.GetChildren())
    while pending:
        prim = pending.pop()
        if prim.IsA(UsdGeom.Gprim):
            result.append(prim)
        pending.extend(prim.GetChildren())
    return result


def _logical_link_name_for_geometry(
    geometry_prim: Usd.Prim,
    *,
    visuals_prim: Usd.Prim,
    owner_link_name: str,
) -> str:
    """Resolve a folded fixed-child visual back to its URDF link name."""

    current = geometry_prim.GetParent()
    while current.IsValid() and current != visuals_prim:
        if current.GetName() in _LINK_MATERIAL_KIND:
            return current.GetName()
        current = current.GetParent()
    return owner_link_name


def apply_g1_sonic_visual_materials(
    robot_prim_path: str = "/World/envs/env_0/Robot",
    *,
    stage: Usd.Stage | None = None,
) -> G1SonicVisualMaterialReport:
    """Apply the reference G1 materials to one spawned SONIC robot.

    Homogeneous visual roots keep USD instancing and receive one strong root
    binding. The torso and two wrist roots contain fixed children with another
    material kind, so only those three visual-only instances are expanded and
    rebound per render mesh. The sibling ``collisions`` hierarchy is never
    traversed or modified.
    """

    stage = _get_current_stage() if stage is None else stage
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        raise ValueError(f"G1 robot prim does not exist: {robot_prim_path}")

    materials = _create_reference_materials(stage)
    bound_link_names = {"white": set(), "dark": set(), "logo": set()}
    unmapped_visual_links: set[str] = set()

    # Snapshot before authoring. Applying a MaterialBindingAPI changes the
    # stage and can invalidate a live traversal iterator.
    visual_link_prims = _collect_visual_link_prims(robot_prim)

    for prim, visuals_prim in visual_link_prims:
        link_name = prim.GetName()
        material_kind = _LINK_MATERIAL_KIND.get(link_name)
        if material_kind is None:
            unmapped_visual_links.add(link_name)
            continue

        if link_name not in _MIXED_VISUAL_ROOT_LINK_NAMES:
            # The complete instance uses one material kind, so a binding at
            # its editable instance root preserves instancing and overrides
            # all nested STL DefaultMaterial bindings.
            _bind_visual_prim(visuals_prim, materials[material_kind])
            bound_link_names[material_kind].add(link_name)
            for folded_link_name in _HOMOGENEOUS_FOLDED_LINK_NAMES.get(
                link_name, ()
            ):
                if _LINK_MATERIAL_KIND.get(folded_link_name) == material_kind:
                    bound_link_names[material_kind].add(folded_link_name)
            continue

        # Head/logo and the two palms are fixed links folded into the torso or
        # wrist visual instance. Uninstance these three visual roots only; no
        # physics or collision prim lives below them.
        visuals_path = visuals_prim.GetPath()
        if visuals_prim.IsInstance():
            visuals_prim.SetInstanceable(False)
        visuals_prim = stage.GetPrimAtPath(visuals_path)

        geometry_prims = _collect_visual_geometry_prims(visuals_prim)
        if not geometry_prims:
            raise RuntimeError(
                f"no render geometry found after uninstancing {visuals_path}"
            )
        for geometry_prim in geometry_prims:
            logical_link_name = _logical_link_name_for_geometry(
                geometry_prim,
                visuals_prim=visuals_prim,
                owner_link_name=link_name,
            )
            geometry_material_kind = _LINK_MATERIAL_KIND.get(logical_link_name)
            if geometry_material_kind is None:
                unmapped_visual_links.add(logical_link_name)
                continue
            _bind_visual_prim(geometry_prim, materials[geometry_material_kind])
            bound_link_names[geometry_material_kind].add(logical_link_name)

    report = G1SonicVisualMaterialReport(
        white_links=len(bound_link_names["white"]),
        dark_links=len(bound_link_names["dark"]),
        logo_links=len(bound_link_names["logo"]),
        unmapped_visual_links=tuple(sorted(unmapped_visual_links)),
    )
    if report.bound_links == 0:
        raise RuntimeError(
            f"no mapped G1 visual links were found below robot prim {robot_prim_path}"
        )
    return report
