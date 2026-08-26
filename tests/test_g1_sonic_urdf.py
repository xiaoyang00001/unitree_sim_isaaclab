from __future__ import annotations

import hashlib
import os
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER
from robots.g1_sonic_urdf import (
    DEX3_HAND_JOINT_NAMES,
    build_sonic_g1_43dof_urdf,
)


GR00T_ROOT = Path(
    os.environ.get("GR00T_WBC_ROOT", "/home/nolovr/GR00T-WholeBodyControl")
).expanduser()
ARTICULATED_URDF = (
    GR00T_ROOT / "gear_sonic/data/robots/g1/g1_29dof_with_hand_rev_1_0.urdf"
)
TRAINING_URDF = (
    GR00T_ROOT / "gear_sonic/data/assets/robot_description/urdf/g1/main.urdf"
)
MESH_DIRECTORY = (
    GR00T_ROOT / "gear_sonic/data/robot_model/model_data/g1/meshes"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@unittest.skipUnless(
    ARTICULATED_URDF.is_file()
    and TRAINING_URDF.is_file()
    and MESH_DIRECTORY.is_dir(),
    "GR00T G1 source assets are not installed",
)
class SonicG143DoFUrdfTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.output_directory = Path(self.temporary_directory.name)
        self.source_hashes = {
            ARTICULATED_URDF: _sha256(ARTICULATED_URDF),
            TRAINING_URDF: _sha256(TRAINING_URDF),
        }

        self.output_path = build_sonic_g1_43dof_urdf(
            articulated_urdf=ARTICULATED_URDF,
            training_urdf=TRAINING_URDF,
            mesh_directory=MESH_DIRECTORY,
            output_directory=self.output_directory,
        )
        self.root = ET.parse(self.output_path).getroot()
        self.articulated_root = ET.parse(ARTICULATED_URDF).getroot()
        self.training_root = ET.parse(TRAINING_URDF).getroot()

    def tearDown(self) -> None:
        for source_path, expected_hash in self.source_hashes.items():
            self.assertEqual(_sha256(source_path), expected_hash)

    def _link(self, name: str) -> ET.Element:
        return next(link for link in self.root.findall("link") if link.get("name") == name)

    def _joint(self, name: str) -> ET.Element:
        return next(joint for joint in self.root.findall("joint") if joint.get("name") == name)

    def test_joint_set_is_exactly_29_body_plus_14_dex3(self) -> None:
        movable_joint_names = {
            joint.get("name")
            for joint in self.root.findall("joint")
            if joint.get("type") not in (None, "fixed")
        }
        self.assertEqual(len(movable_joint_names), 43)
        self.assertEqual(
            movable_joint_names,
            set(G1_29DOF_DDS_JOINT_ORDER) | set(DEX3_HAND_JOINT_NAMES),
        )

    def test_training_sole_collisions_and_flat_distal_hand_pads(self) -> None:
        left_sole_collisions = self._link("left_ankle_roll_link").findall("collision")
        right_sole_collisions = self._link("right_ankle_roll_link").findall("collision")
        self.assertEqual(len(left_sole_collisions), 7)
        self.assertEqual(len(right_sole_collisions), 7)
        for collision in (*left_sole_collisions, *right_sole_collisions):
            self.assertIsNotNone(collision.find("geometry/cylinder"))
            self.assertIsNone(collision.find("geometry/box"))

        training_links = {
            link.get("name"): link for link in self.training_root.findall("link")
        }
        for side in ("left", "right"):
            link_name = f"{side}_ankle_roll_link"
            expected_origins = [
                collision.find("origin").attrib
                for collision in training_links[link_name].findall("collision")
            ]
            generated_origins = [
                collision.find("origin").attrib
                for collision in self._link(link_name).findall("collision")
            ]
            self.assertEqual(generated_origins, expected_origins)

        terminal_link_suffixes = ("thumb_2_link", "middle_1_link", "index_1_link")
        hand_links = [
            link
            for link in self.root.findall("link")
            if link.get("name", "").startswith(("left_hand_", "right_hand_"))
        ]
        self.assertEqual(len(hand_links), 16)
        collision_count = 0
        for link in hand_links:
            link_name = link.get("name", "")
            collisions = link.findall("collision")
            collision_count += len(collisions)
            expected_collision_count = (
                2 if link_name.endswith(terminal_link_suffixes) else 1
            )
            self.assertEqual(len(collisions), expected_collision_count, link_name)
            expected_names = {f"{link_name}_physics_proxy"}
            if link_name.endswith(terminal_link_suffixes):
                expected_names.add(f"{link_name}_grip_pad")
            self.assertEqual(
                {collision.get("name") for collision in collisions},
                expected_names,
                link_name,
            )

            for collision in collisions:
                geometry = collision.find("geometry")
                self.assertIsNotNone(geometry, link_name)
                self.assertIsNone(geometry.find("mesh"), link_name)

            physics_proxy = next(
                collision
                for collision in collisions
                if collision.get("name") == f"{link_name}_physics_proxy"
            )
            proxy_geometry = physics_proxy.find("geometry")
            if link_name.endswith(("palm_link", "thumb_0_link")):
                self.assertIsNotNone(proxy_geometry.find("box"), link_name)
                self.assertIsNone(proxy_geometry.find("cylinder"), link_name)
            else:
                self.assertIsNotNone(proxy_geometry.find("cylinder"), link_name)
                self.assertIsNone(proxy_geometry.find("box"), link_name)

            if link_name.endswith(terminal_link_suffixes):
                grip_pad = next(
                    collision
                    for collision in collisions
                    if collision.get("name") == f"{link_name}_grip_pad"
                )
                self.assertIsNotNone(grip_pad.find("geometry/box"), link_name)
                self.assertIsNone(grip_pad.find("geometry/cylinder"), link_name)

        self.assertEqual(collision_count, 22)

        left_thumb_origin = self._link("left_hand_thumb_1_link").find(
            "collision/origin"
        )
        right_thumb_origin = self._link("right_hand_thumb_1_link").find(
            "collision/origin"
        )
        self.assertEqual(left_thumb_origin.get("xyz"), "0 -0.024 0")
        self.assertEqual(right_thumb_origin.get("xyz"), "0 0.024 0")

        for side, thumb_proxy_y, thumb_pad_y, finger_pad_y in (
            ("left", "-0.0225", "-0.029", "-0.010"),
            ("right", "0.0225", "0.029", "0.010"),
        ):
            thumb_link = self._link(f"{side}_hand_thumb_2_link")
            thumb_proxy = next(
                collision
                for collision in thumb_link.findall("collision")
                if collision.get("name") == f"{side}_hand_thumb_2_link_physics_proxy"
            )
            self.assertEqual(
                thumb_proxy.find("origin").attrib,
                {"xyz": f"0 {thumb_proxy_y} 0", "rpy": "1.57079632679 0 0"},
            )
            self.assertEqual(
                thumb_proxy.find("geometry/cylinder").attrib,
                {"radius": "0.0115", "length": "0.036"},
            )

            thumb_pad = next(
                collision
                for collision in thumb_link.findall("collision")
                if collision.get("name") == f"{side}_hand_thumb_2_link_grip_pad"
            )
            self.assertEqual(
                thumb_pad.find("origin").get("xyz"),
                f"0.010 {thumb_pad_y} 0",
            )
            self.assertEqual(thumb_pad.find("origin").get("rpy"), "0 0 0")
            self.assertEqual(
                thumb_pad.find("geometry/box").get("size"),
                "0.004 0.030 0.018",
            )

            for finger in ("middle", "index"):
                distal_link = self._link(f"{side}_hand_{finger}_1_link")
                distal_proxy = next(
                    collision
                    for collision in distal_link.findall("collision")
                    if collision.get("name")
                    == f"{side}_hand_{finger}_1_link_physics_proxy"
                )
                self.assertEqual(
                    distal_proxy.find("origin").attrib,
                    {"xyz": "0.0225 0 0", "rpy": "0 1.57079632679 0"},
                )
                self.assertEqual(
                    distal_proxy.find("geometry/cylinder").attrib,
                    {"radius": "0.0115", "length": "0.036"},
                )

                distal_pad = next(
                    collision
                    for collision in distal_link.findall("collision")
                    if collision.get("name")
                    == f"{side}_hand_{finger}_1_link_grip_pad"
                )
                self.assertEqual(
                    distal_pad.find("origin").get("xyz"),
                    f"0.029 {finger_pad_y} 0",
                )
                self.assertEqual(distal_pad.find("origin").get("rpy"), "0 0 0")
                self.assertEqual(
                    distal_pad.find("geometry/box").get("size"),
                    "0.030 0.004 0.018",
                )

    def test_body_limits_are_copied_and_hand_limits_are_preserved(self) -> None:
        for side in ("left", "right"):
            expected_hip_effort = {"pitch": "88", "roll": "139", "yaw": "88"}
            for axis, expected_effort in expected_hip_effort.items():
                limit = self._joint(f"{side}_hip_{axis}_joint").find("limit")
                self.assertIsNotNone(limit)
                self.assertEqual(limit.get("effort"), expected_effort)

            knee_limit = self._joint(f"{side}_knee_joint").find("limit")
            self.assertIsNotNone(knee_limit)
            self.assertEqual(knee_limit.get("effort"), "139")

        for joint_name in (
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
        ):
            limit = self._joint(joint_name).find("limit")
            self.assertIsNotNone(limit)
            self.assertEqual(limit.get("effort"), "50")
            self.assertEqual(limit.get("velocity"), "37")

        thumb_limit = self._joint("left_hand_thumb_0_joint").find("limit")
        index_limit = self._joint("right_hand_index_1_joint").find("limit")
        self.assertEqual(thumb_limit.get("effort"), "2.45")
        self.assertEqual(thumb_limit.get("velocity"), "3.14")
        self.assertEqual(index_limit.get("effort"), "1.4")
        self.assertEqual(index_limit.get("velocity"), "12")

    def test_body_inertias_are_preserved_from_the_articulated_source(self) -> None:
        def inertial_signature(link: ET.Element) -> tuple:
            inertial = link.find("inertial")
            self.assertIsNotNone(inertial)
            origin = inertial.find("origin")
            mass = inertial.find("mass")
            inertia = inertial.find("inertia")
            self.assertIsNotNone(mass)
            self.assertIsNotNone(inertia)
            return (
                origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
                origin.get("rpy", "0 0 0") if origin is not None else "0 0 0",
                mass.get("value"),
                *(inertia.get(name, "0") for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")),
            )

        source_links = {
            link.get("name"): link for link in self.articulated_root.findall("link")
        }
        compared = 0
        for generated_link in self.root.findall("link"):
            if generated_link.find("inertial") is None:
                continue
            source_link = source_links[generated_link.get("name")]
            self.assertEqual(
                inertial_signature(generated_link),
                inertial_signature(source_link),
                generated_link.get("name"),
            )
            compared += 1
        self.assertGreater(compared, 0)

    def test_all_meshes_use_the_requested_model_data_directory(self) -> None:
        mesh_paths = [Path(mesh.get("filename", "")) for mesh in self.root.findall(".//mesh")]
        self.assertGreater(len(mesh_paths), 0)
        expected_directory = MESH_DIRECTORY.resolve()
        for mesh_path in mesh_paths:
            self.assertTrue(mesh_path.is_absolute())
            self.assertEqual(mesh_path.parent, expected_directory)
            self.assertTrue(mesh_path.is_file())

    def test_generation_is_deterministic(self) -> None:
        first_payload = self.output_path.read_bytes()
        second_path = build_sonic_g1_43dof_urdf(
            articulated_urdf=ARTICULATED_URDF,
            training_urdf=TRAINING_URDF,
            mesh_directory=MESH_DIRECTORY,
            output_directory=self.output_directory,
        )
        self.assertEqual(second_path, self.output_path)
        self.assertEqual(second_path.read_bytes(), first_payload)


if __name__ == "__main__":
    unittest.main()
