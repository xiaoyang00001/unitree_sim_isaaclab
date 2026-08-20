"""Standalone tests for the pure-visual peer mirror.

These tests intentionally do not import Isaac Lab, Omniverse, pxr, torch, or
the eager tasks package.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import re
import sys
import unittest

from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER
from robots.g1_sonic_urdf import DEX3_HAND_JOINT_NAMES


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/peer_visual_lod.py"
)
ASSET_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot/g1_43dof_visual_lod.usda"
)
GENERATOR_PATH = REPO_ROOT / "tools/build_peer_visual_lod_usd.py"
CONFIG_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
)
SYNC_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/zmq_scene_sync.py"
)
SIM_MAIN_PATH = REPO_ROOT / "sim_main.py"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


visual_lod = _load_module("_test_peer_visual_lod", MODULE_PATH)
generator = _load_module("_test_peer_visual_lod_generator", GENERATOR_PATH)


def _assert_matrix_close(
    test: unittest.TestCase, first, second, *, places: int = 10
) -> None:
    for first_row, second_row in zip(first, second):
        for first_value, second_value in zip(first_row, second_row):
            test.assertAlmostEqual(first_value, second_value, places=places)


class PeerVisualLodKinematicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.zero_root = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.zero_joints = (0.0,) * len(visual_lod.JOINT_NAMES)

    def test_manifest_covers_exact_sonic_body_and_dex3_joint_set(self) -> None:
        expected = set(G1_29DOF_DDS_JOINT_ORDER) | set(DEX3_HAND_JOINT_NAMES)
        self.assertEqual(len(visual_lod.JOINT_NAMES), 43)
        self.assertEqual(len(set(visual_lod.JOINT_NAMES)), 43)
        self.assertEqual(set(visual_lod.JOINT_NAMES), expected)

    def test_wire_mapping_is_name_based_not_position_based(self) -> None:
        values = tuple(index * 0.01 - 0.2 for index in range(43))
        normal = visual_lod.forward_kinematics(
            self.zero_root, values, visual_lod.JOINT_NAMES
        )
        reversed_names = tuple(reversed(visual_lod.JOINT_NAMES))
        by_name = dict(zip(visual_lod.JOINT_NAMES, values))
        reversed_values = tuple(by_name[name] for name in reversed_names)
        reordered = visual_lod.forward_kinematics(
            self.zero_root, reversed_values, reversed_names
        )
        self.assertEqual(normal.keys(), reordered.keys())
        for link_name in normal:
            _assert_matrix_close(self, normal[link_name], reordered[link_name])

    def test_leg_joint_changes_descendant_without_moving_other_leg(self) -> None:
        baseline = visual_lod.forward_kinematics(
            self.zero_root, self.zero_joints, visual_lod.JOINT_NAMES
        )
        moved_values = list(self.zero_joints)
        moved_values[visual_lod.JOINT_NAMES.index("left_hip_pitch_joint")] = math.pi / 2.0
        moved = visual_lod.forward_kinematics(
            self.zero_root, moved_values, visual_lod.JOINT_NAMES
        )
        self.assertNotEqual(
            baseline["left_ankle_roll_link"], moved["left_ankle_roll_link"]
        )
        _assert_matrix_close(
            self,
            baseline["right_ankle_roll_link"],
            moved["right_ankle_roll_link"],
        )

    def test_root_pose_moves_complete_fk_tree(self) -> None:
        half_yaw = math.pi / 4.0
        root = (
            2.0,
            -3.0,
            1.25,
            math.cos(half_yaw),
            0.0,
            0.0,
            math.sin(half_yaw),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        transforms = visual_lod.forward_kinematics(
            root, self.zero_joints, visual_lod.JOINT_NAMES
        )
        pelvis = transforms["pelvis"]
        self.assertAlmostEqual(pelvis[0][3], 2.0)
        self.assertAlmostEqual(pelvis[1][3], -3.0)
        self.assertAlmostEqual(pelvis[2][3], 1.25)
        self.assertAlmostEqual(pelvis[0][0], 0.0, places=12)
        self.assertAlmostEqual(pelvis[1][0], 1.0, places=12)
        # The source URDF's waist +0.044 m and fixed head -0.044 m cancel at
        # the head-link origin; a shoulder still proves descendants inherit
        # the translated/rotated base frame.
        self.assertNotEqual(transforms["left_shoulder_pitch_link"], pelvis)

    def test_wire_hash_rejects_tamper_and_unknown_joint(self) -> None:
        expected_hash = visual_lod.joint_order_hash(visual_lod.JOINT_NAMES)
        self.assertEqual(
            visual_lod.validate_wire_joint_order(
                visual_lod.JOINT_NAMES, expected_hash
            ),
            visual_lod.JOINT_NAMES,
        )
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            visual_lod.validate_wire_joint_order(visual_lod.JOINT_NAMES, "0" * 16)
        invalid = list(visual_lod.JOINT_NAMES)
        invalid[-1] = "unknown_joint"
        with self.assertRaisesRegex(ValueError, "关节集合不匹配"):
            visual_lod.validate_joint_order(invalid)

    def test_mode_is_explicit_opt_in_with_articulation_default(self) -> None:
        self.assertEqual(visual_lod.resolve_peer_robot_mode(None), "articulation")
        self.assertEqual(visual_lod.resolve_peer_robot_mode(" VISUAL_LOD "), "visual_lod")
        with self.assertRaisesRegex(ValueError, "可选值"):
            visual_lod.resolve_peer_robot_mode("visual-only")

    def test_xr_compatible_link_paths_exist_in_manifest(self) -> None:
        paths = visual_lod.link_paths("/World/envs/env_0/PeerRobot")
        self.assertEqual(paths["pelvis"], "/World/envs/env_0/PeerRobot/pelvis")
        self.assertTrue(paths["head_link"].endswith("/torso_link/head_link"))
        self.assertEqual(len(paths), 47)


class PeerVisualLodAssetTests(unittest.TestCase):
    def test_tracked_asset_is_reproducible(self) -> None:
        self.assertEqual(ASSET_PATH.read_text(encoding="utf-8"), generator.build_text())

    def test_asset_is_analytic_render_geometry_without_physics_schemas(self) -> None:
        text = ASSET_PATH.read_text(encoding="utf-8")
        self.assertIn('defaultPrim = "PeerVisualLod"', text)
        self.assertIn('peerVisualLodRole = "pure_visual_fk"', text)
        self.assertNotIn('def Mesh "', text)
        primitives = re.findall(r'def (?:Cube|Sphere|Capsule) "visual"', text)
        self.assertEqual(len(primitives), 47)
        for forbidden in (
            "PhysicsCollisionAPI",
            "PhysicsRigidBodyAPI",
            "PhysxSchema",
            "ArticulationRootAPI",
            "ContactReportAPI",
            "CollisionAPI",
        ):
            self.assertNotIn(forbidden, text)

    def test_every_fk_link_has_an_xform_prim(self) -> None:
        text = ASSET_PATH.read_text(encoding="utf-8")
        for link_name in visual_lod.link_paths("/PeerVisualLod"):
            self.assertIn(f'def Xform "{link_name}"', text)


class PeerVisualLodWiringTests(unittest.TestCase):
    def test_scene_factory_uses_asset_base_and_wires_three_viewer_mirrors(self) -> None:
        source = CONFIG_PATH.read_text(encoding="utf-8")
        visual_factory = source.split("def _make_peer_visual_lod_cfg(", 1)[1].split(
            "def _make_peer_scene_cfg", 1
        )[0]
        self.assertIn("return AssetBaseCfg(", visual_factory)
        self.assertIn("activate_contact_sensors=False", visual_factory)
        self.assertNotIn("ArticulationCfg(", visual_factory)
        self.assertNotIn("RigidBodyPropertiesCfg", visual_factory)
        self.assertIn('"robot_1": "/World/envs/env_0/PeerRobot"', source)
        self.assertIn('"robot_2": "/World/envs/env_0/PeerRobot2"', source)
        self.assertIn('"robot_3": "/World/envs/env_0/PeerRobot3"', source)
        self.assertIn("_make_second_peer_scene_cfg() if VIEWER_MODE", source)
        self.assertIn("_make_third_peer_scene_cfg() if VIEWER_MODE", source)

    def test_conveyor_standby_robots_use_full_meshes_without_physics(self) -> None:
        source = CONFIG_PATH.read_text(encoding="utf-8")
        factory = source.split("def _make_standby_robot_cfg(", 1)[1].split(
            "# ==================================================================\n# 场景", 1
        )[0]

        self.assertIn("if index >= len(STANDBY_ROBOT_POSES):", factory)
        self.assertIn("pose = STANDBY_ROBOT_POSES[index]", factory)
        self.assertIn("return AssetBaseCfg(", factory)
        self.assertIn("pos=pose.pos, rot=pose.rot", factory)
        self.assertIn("usd_path=str(_STANDBY_ROBOT_USD)", factory)
        self.assertIn(
            'variants={"Physics": "None", "Robot": "None", "Sensor": "None"}',
            factory,
        )
        self.assertIn("activate_contact_sensors=False", factory)
        self.assertNotIn("_make_peer_visual_lod_cfg(", factory)
        self.assertNotIn("ArticulationCfg(", factory)
        self.assertNotIn("RigidBodyPropertiesCfg", factory)
        self.assertEqual(source.count("= _make_standby_robot_cfg("), 2)
        for name in ("StandbyRobot1", "StandbyRobot2", "StandbyRobot3"):
            self.assertIn(f'"{name}"', source)
        self.assertIn("standby_robot_1: AssetBaseCfg | None = _make_standby_robot_cfg(0)", source)
        self.assertIn("None if (HOST_MODE or VIEWER_MODE) else _make_standby_robot_cfg(1)", source)
        self.assertIn("standby_robot_3: AssetBaseCfg | None = _make_standby_robot_cfg(2)", source)

        # 新增展示体不能改动原有两台的朝向契约。
        self.assertIn(
            "_ROBOT_1_ROT = (1.0, 0.0, 0.0, 0.0) if _ROBOT_YAW_IDENTITY "
            "else (0.0, 0.0, 0.0, 1.0)",
            source,
        )
        self.assertIn("_ROBOT_2_ROT = (1.0, 0.0, 0.0, 0.0)", source)

        # 涂装跟随实际 scene.extras，不重复读取开关、不硬编码三条 Prim 路径。
        sim_source = SIM_MAIN_PATH.read_text(encoding="utf-8")
        material_block = sim_source.split("standby_prim_paths = [", 1)[1].split(
            "except Exception as e:", 1
        )[0]
        self.assertIn("for asset_name, view in env.scene.extras.items()", material_block)
        self.assertIn('asset_name.startswith("standby_robot_")', material_block)
        self.assertIn("for prim_path in view.prim_paths", material_block)
        self.assertIn("apply_g1_sonic_visual_materials(standby_prim_path)", material_block)
        self.assertNotIn("os.environ", material_block)
        for name in ("StandbyRobot1", "StandbyRobot2", "StandbyRobot3"):
            self.assertNotIn(name, material_block)

    def test_sync_uses_visual_writer_before_articulation_methods(self) -> None:
        source = SYNC_PATH.read_text(encoding="utf-8")
        self.assertIn('"joint_names": list(self._joint_names or ())', source)
        apply_loop = source.split(
            "for name, (root_state, joint_pos, joint_vel) in robot_states.items():", 1
        )[1].split("for name, root_state in object_states.items():", 1)[0]
        visual_call = apply_loop.index("visual_robot.apply_state(")
        early_continue = apply_loop.index("continue", visual_call)
        articulation_lookup = apply_loop.index("robot = self._apply_robots[name]")
        self.assertLess(visual_call, early_continue)
        self.assertLess(early_continue, articulation_lookup)


if __name__ == "__main__":
    unittest.main()
