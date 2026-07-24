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

    def test_training_sole_collisions_and_phase_one_hand_collision_policy(self) -> None:
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

        for link in self.root.findall("link"):
            link_name = link.get("name", "")
            if link_name.startswith(("left_hand_", "right_hand_")):
                self.assertEqual(link.findall("collision"), [])

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
