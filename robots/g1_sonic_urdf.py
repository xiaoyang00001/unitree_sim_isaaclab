"""Build the Isaac Lab G1-43DoF asset used by the SONIC bridge.

The source assets under ``GR00T-WholeBodyControl/gear_sonic/data`` are runtime
data and are intentionally not edited here.  This module creates a deterministic
temporary URDF with the parts that matter for the current validation phase:

* kinematics, inertias, visuals, and the 14 articulated Dex3 joints come from
  ``g1_29dof_with_hand_rev_1_0.urdf``;
* visual meshes are resolved from the MuJoCo/Pinocchio ``model_data`` mesh
  directory;
* body collision geometry and body effort/velocity limits come unchanged from
  the exact SONIC training URDF;
* body inertias remain identical to the articulated source/training model;
* Dex3 visuals keep the requested STL meshes, while dynamic collision uses
  low-complexity palm boxes and finger capsules suitable for PhysX contact.

Only the generated file is written.  The two GR00T source URDFs and their mesh
directories remain untouched.
"""

from __future__ import annotations

import copy
import getpass
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable

from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER


DEX3_HAND_JOINT_NAMES = (
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
)

_HAND_LINK_PREFIXES = ("left_hand_", "right_hand_")
_OUTPUT_FILE_NAME = "g1_29dof_with_hand_rev_1_0_sonic_isaaclab.urdf"


def _user_scoped_suffix() -> str:
    """Per-user suffix for the temp output directory.

    POSIX keeps using the uid so already generated paths stay valid; Windows
    has no ``os.getuid`` and falls back to the login name.
    """

    getuid = getattr(os, "getuid", None)
    return str(getuid()) if getuid is not None else getpass.getuser()


def default_sonic_g1_43dof_output_path() -> Path:
    """Return the stable per-user path used for the generated URDF."""

    return (
        Path(tempfile.gettempdir())
        / f"unitree_sim_isaaclab_sonic_{_user_scoped_suffix()}"
        / _OUTPUT_FILE_NAME
    )


def _require_file(path: Path, description: str) -> Path:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"{description} is missing: {path}")
    return path


def _require_directory(path: Path, description: str) -> Path:
    path = path.expanduser().resolve()
    if not path.is_dir():
        raise FileNotFoundError(f"{description} is missing: {path}")
    return path


def _joint_map(root: ET.Element) -> dict[str, ET.Element]:
    return {joint.get("name", ""): joint for joint in root.findall("joint")}


def _link_map(root: ET.Element) -> dict[str, ET.Element]:
    return {link.get("name", ""): link for link in root.findall("link")}


def _float_tuple(value: str | None, size: int) -> tuple[float, ...]:
    if value is None:
        return (0.0,) * size
    result = tuple(float(component) for component in value.split())
    if len(result) != size:
        raise ValueError(f"expected {size} values, got {value!r}")
    return result


def _inertial_signature(link: ET.Element) -> tuple[float, ...] | None:
    inertial = link.find("inertial")
    if inertial is None:
        return None

    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        raise ValueError(f"incomplete inertial element on link {link.get('name')!r}")

    xyz = _float_tuple(origin.get("xyz") if origin is not None else None, 3)
    rpy = _float_tuple(origin.get("rpy") if origin is not None else None, 3)
    tensor = tuple(
        float(inertia.get(attribute, "0"))
        for attribute in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    )
    return (*xyz, *rpy, float(mass.get("value", "0")), *tensor)


def _joint_kinematic_signature(
    joint: ET.Element,
) -> tuple[tuple[str | None, str | None, str | None], tuple[float, ...]]:
    parent = joint.find("parent")
    child = joint.find("child")
    origin = joint.find("origin")
    axis = joint.find("axis")
    topology = (
        joint.get("type"),
        parent.get("link") if parent is not None else None,
        child.get("link") if child is not None else None,
    )
    coordinates = (
        *_float_tuple(origin.get("xyz") if origin is not None else None, 3),
        *_float_tuple(origin.get("rpy") if origin is not None else None, 3),
        *_float_tuple(axis.get("xyz") if axis is not None else None, 3),
    )
    return topology, coordinates


def _assert_close_sequence(
    first: Iterable[float],
    second: Iterable[float],
    *,
    tolerance: float,
    description: str,
) -> None:
    first_tuple = tuple(first)
    second_tuple = tuple(second)
    if len(first_tuple) != len(second_tuple):
        raise ValueError(f"{description} has different lengths")
    max_difference = max(
        (abs(first_value - second_value) for first_value, second_value in zip(first_tuple, second_tuple)),
        default=0.0,
    )
    if max_difference > tolerance:
        raise ValueError(
            f"{description} differs by {max_difference:.6g}, exceeding {tolerance:.6g}"
        )


def _validate_source_models(
    articulated_root: ET.Element,
    training_root: ET.Element,
) -> None:
    articulated_joints = _joint_map(articulated_root)
    training_joints = _joint_map(training_root)

    movable_joint_names = {
        name
        for name, joint in articulated_joints.items()
        if joint.get("type") not in (None, "fixed")
    }
    expected_joint_names = set(G1_29DOF_DDS_JOINT_ORDER) | set(DEX3_HAND_JOINT_NAMES)
    if movable_joint_names != expected_joint_names:
        missing = sorted(expected_joint_names - movable_joint_names)
        unexpected = sorted(movable_joint_names - expected_joint_names)
        raise ValueError(
            "the articulated G1 source is not the expected 29+14 DoF model: "
            f"missing={missing}, unexpected={unexpected}"
        )

    missing_training_joints = [
        name for name in G1_29DOF_DDS_JOINT_ORDER if name not in training_joints
    ]
    if missing_training_joints:
        raise ValueError(
            f"the SONIC training URDF is missing body joints: {missing_training_joints}"
        )

    for name in G1_29DOF_DDS_JOINT_ORDER:
        articulated_topology, articulated_coordinates = _joint_kinematic_signature(
            articulated_joints[name]
        )
        training_topology, training_coordinates = _joint_kinematic_signature(
            training_joints[name]
        )
        if articulated_topology != training_topology:
            raise ValueError(
                f"body-joint topology for {name!r} differs from SONIC training: "
                f"articulated={articulated_topology}, training={training_topology}"
            )
        _assert_close_sequence(
            articulated_coordinates,
            training_coordinates,
            tolerance=1.0e-12,
            description=f"body-joint kinematics for {name}",
        )

    articulated_links = _link_map(articulated_root)
    training_links = _link_map(training_root)
    for name, articulated_link in articulated_links.items():
        articulated_inertial = _inertial_signature(articulated_link)
        if articulated_inertial is None:
            continue
        training_link = training_links.get(name)
        if training_link is None:
            raise ValueError(f"training URDF has no matching inertial link {name!r}")
        training_inertial = _inertial_signature(training_link)
        if training_inertial is None:
            raise ValueError(f"training URDF link {name!r} has no inertial element")
        _assert_close_sequence(
            articulated_inertial,
            training_inertial,
            tolerance=1.0e-12,
            description=f"inertial data for {name}",
        )


def _copy_training_body_collisions(
    articulated_root: ET.Element,
    training_root: ET.Element,
) -> None:
    training_links = _link_map(training_root)
    for articulated_link in articulated_root.findall("link"):
        link_name = articulated_link.get("name", "")
        for collision in list(articulated_link.findall("collision")):
            articulated_link.remove(collision)

        if link_name.startswith(_HAND_LINK_PREFIXES):
            _append_dex3_collision_proxy(articulated_link)
            continue

        training_link = training_links.get(link_name)
        if training_link is None:
            continue
        for collision in training_link.findall("collision"):
            articulated_link.append(copy.deepcopy(collision))


def _append_collision(
    link: ET.Element,
    *,
    xyz: str,
    rpy: str,
    geometry_tag: str,
    geometry_attributes: dict[str, str],
) -> None:
    collision = ET.SubElement(
        link,
        "collision",
        {"name": f"{link.get('name', 'dex3')}_physics_proxy"},
    )
    ET.SubElement(collision, "origin", {"xyz": xyz, "rpy": rpy})
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, geometry_tag, geometry_attributes)


def _append_dex3_collision_proxy(link: ET.Element) -> None:
    """Add one stable primitive collider matching the hand-link STL bounds.

    The URDF importer is configured with ``replace_cylinders_with_capsules``;
    therefore finger cylinders become rounded PhysX capsules in the generated
    USD.  The proxies are deliberately inset from the visual STL envelope to
    avoid adjacent-link contact chatter while retaining useful object contact.
    """

    link_name = link.get("name", "")
    is_left = link_name.startswith("left_hand_")
    is_right = link_name.startswith("right_hand_")
    if not (is_left or is_right):
        return

    if link_name.endswith("palm_link"):
        _append_collision(
            link,
            xyz="0.043 0 0",
            rpy="0 0 0",
            geometry_tag="box",
            geometry_attributes={"size": "0.080 0.034 0.075"},
        )
        return

    thumb_y_sign = -1.0 if is_left else 1.0
    if link_name.endswith("thumb_0_link"):
        _append_collision(
            link,
            xyz=f"0 {thumb_y_sign * 0.014:.3f} 0",
            rpy="0 0 0",
            geometry_tag="box",
            geometry_attributes={"size": "0.020 0.026 0.018"},
        )
        return

    if link_name.endswith("thumb_1_link"):
        _append_collision(
            link,
            xyz=f"0 {thumb_y_sign * 0.024:.3f} 0",
            rpy="1.57079632679 0 0",
            geometry_tag="cylinder",
            geometry_attributes={"radius": "0.012", "length": "0.036"},
        )
        return

    if link_name.endswith("thumb_2_link"):
        _append_collision(
            link,
            xyz=f"0 {thumb_y_sign * 0.0225:.4f} 0",
            rpy="1.57079632679 0 0",
            geometry_tag="cylinder",
            geometry_attributes={"radius": "0.0115", "length": "0.036"},
        )
        return

    if link_name.endswith(("middle_0_link", "index_0_link")):
        _append_collision(
            link,
            xyz="0.024 0 0",
            rpy="0 1.57079632679 0",
            geometry_tag="cylinder",
            geometry_attributes={"radius": "0.012", "length": "0.036"},
        )
        return

    if link_name.endswith(("middle_1_link", "index_1_link")):
        _append_collision(
            link,
            xyz="0.0225 0 0",
            rpy="0 1.57079632679 0",
            geometry_tag="cylinder",
            geometry_attributes={"radius": "0.0115", "length": "0.036"},
        )
        return

    raise ValueError(f"no Dex3 collision proxy specification for link {link_name!r}")


def _copy_training_body_limits(
    articulated_root: ET.Element,
    training_root: ET.Element,
) -> None:
    articulated_joints = _joint_map(articulated_root)
    training_joints = _joint_map(training_root)
    for joint_name in G1_29DOF_DDS_JOINT_ORDER:
        articulated_limit = articulated_joints[joint_name].find("limit")
        training_limit = training_joints[joint_name].find("limit")
        if articulated_limit is None or training_limit is None:
            raise ValueError(f"joint {joint_name!r} has no limit element")
        articulated_limit.attrib.clear()
        articulated_limit.attrib.update(training_limit.attrib)


def _rewrite_mesh_paths(root: ET.Element, mesh_directory: Path) -> None:
    missing_meshes: list[Path] = []
    for mesh in root.findall(".//mesh"):
        source_name = mesh.get("filename")
        if not source_name:
            raise ValueError("URDF mesh element has no filename")
        target_path = mesh_directory / Path(source_name).name
        if not target_path.is_file():
            missing_meshes.append(target_path)
            continue
        mesh.set("filename", str(target_path.resolve()))
    if missing_meshes:
        formatted = ", ".join(str(path) for path in sorted(set(missing_meshes)))
        raise FileNotFoundError(f"adapted URDF meshes are missing: {formatted}")


def build_sonic_g1_43dof_urdf(
    *,
    articulated_urdf: Path,
    training_urdf: Path,
    mesh_directory: Path,
    output_directory: Path | None = None,
) -> Path:
    """Create and return the deterministic phase-one G1-43DoF URDF path."""

    articulated_urdf = _require_file(articulated_urdf, "articulated G1 URDF")
    training_urdf = _require_file(training_urdf, "SONIC training G1 URDF")
    mesh_directory = _require_directory(mesh_directory, "G1 visual mesh directory")

    if output_directory is None:
        output_directory = default_sonic_g1_43dof_output_path().parent
    output_directory = output_directory.expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / _OUTPUT_FILE_NAME

    articulated_tree = ET.parse(articulated_urdf)
    training_tree = ET.parse(training_urdf)
    articulated_root = articulated_tree.getroot()
    training_root = training_tree.getroot()

    _validate_source_models(articulated_root, training_root)
    _copy_training_body_collisions(articulated_root, training_root)
    _copy_training_body_limits(articulated_root, training_root)
    _rewrite_mesh_paths(articulated_root, mesh_directory)
    articulated_root.set("name", "g1_29dof_with_hand_rev_1_0_sonic_isaaclab")

    ET.indent(articulated_tree, space="  ")
    payload = ET.tostring(
        articulated_root,
        encoding="utf-8",
        xml_declaration=True,
        short_empty_elements=True,
    )
    if output_path.is_file() and output_path.read_bytes() == payload:
        return output_path

    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=output_directory,
            prefix=f".{output_path.stem}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_file.write(payload)
            temporary_path = Path(temporary_file.name)
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()

    return output_path


def build_default_sonic_g1_43dof_urdf(groot_root: Path) -> Path:
    """Build the adapted URDF from the standard GR00T checkout layout."""

    groot_root = groot_root.expanduser().resolve()
    return build_sonic_g1_43dof_urdf(
        articulated_urdf=(
            groot_root
            / "gear_sonic/data/robots/g1/g1_29dof_with_hand_rev_1_0.urdf"
        ),
        training_urdf=(
            groot_root
            / "gear_sonic/data/assets/robot_description/urdf/g1/main.urdf"
        ),
        mesh_directory=(
            groot_root / "gear_sonic/data/robot_model/model_data/g1/meshes"
        ),
    )
