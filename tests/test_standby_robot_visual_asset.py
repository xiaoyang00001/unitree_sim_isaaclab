"""Checks for the full-fidelity, physics-free standby G1 asset."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot"
)
ASSET_PATH = ASSET_DIR / "g1_43dof_standby_visual_only.usda"
SOURCE_PATH = ASSET_DIR / "g1_43dof_peer.usd"
GENERATOR_PATH = REPO_ROOT / "tools/build_standby_robot_visual_only_usd.py"


def _load_generator():
    spec = importlib.util.spec_from_file_location(
        "_test_standby_robot_visual_generator", GENERATOR_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {GENERATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


generator = _load_generator()

try:
    from pxr import Usd, UsdGeom, UsdShade
except ImportError:  # Ordinary unit-test Python intentionally has no USD runtime.
    Usd = UsdGeom = UsdShade = None


class StandbyRobotVisualGeneratorTests(unittest.TestCase):
    def test_tracked_asset_is_reproducible(self) -> None:
        self.assertEqual(ASSET_PATH.read_text(encoding="utf-8"), generator.build_text())

    def test_asset_selects_no_physics_and_bakes_all_body_transforms(self) -> None:
        text = ASSET_PATH.read_text(encoding="utf-8")
        self.assertIn('prepend references = @./g1_43dof_peer.usd@', text)
        for name in ("Physics", "Robot", "Sensor"):
            self.assertIn(f'string {name} = "None"', text)
        self.assertEqual(text.count("matrix4d xformOp:transform = ("), 44)
        self.assertEqual(
            text.count('uniform token[] xformOpOrder = ["xformOp:transform"]'),
            44,
        )
        self.assertNotIn('def Mesh "', text)

    def test_baked_joint_values_match_sonic_default_stance(self) -> None:
        fk = generator._load_fk_module()
        values = dict(
            zip(
                fk.JOINT_NAMES,
                generator.sonic_default_joint_positions(fk.JOINT_NAMES),
            )
        )
        for side in ("left", "right"):
            self.assertEqual(values[f"{side}_hip_pitch_joint"], -0.312)
            self.assertEqual(values[f"{side}_knee_joint"], 0.669)
            self.assertEqual(values[f"{side}_ankle_pitch_joint"], -0.363)
            self.assertEqual(values[f"{side}_elbow_joint"], 0.6)
            self.assertEqual(values[f"{side}_shoulder_pitch_joint"], 0.2)
        self.assertEqual(values["left_shoulder_roll_joint"], 0.2)
        self.assertEqual(values["right_shoulder_roll_joint"], -0.2)
        self.assertTrue(
            all(value == 0.0 for name, value in values.items() if "hand" in name)
        )


@unittest.skipIf(Usd is None, "pxr is only available in an Isaac Sim Python environment")
class StandbyRobotVisualCompositionTests(unittest.TestCase):
    @staticmethod
    def _relative_paths(prims, root_path, schema_type) -> set[str]:
        prefix_length = len(str(root_path))
        return {
            str(prim.GetPath())[prefix_length:]
            for prim in prims
            if prim.IsA(schema_type)
        }

    def test_full_source_visuals_remain_but_active_physics_is_empty(self) -> None:
        stage = Usd.Stage.Open(str(ASSET_PATH), Usd.Stage.LoadAll)
        self.assertIsNotNone(stage)
        root = stage.GetDefaultPrim()
        self.assertTrue(root.IsValid())
        self.assertEqual(
            root.GetCustomDataByKey("standbyVisualRole"),
            "full_fidelity_baked_pose_no_physics",
        )
        for name in ("Physics", "Robot", "Sensor"):
            self.assertEqual(
                root.GetVariantSets().GetVariantSet(name).GetVariantSelection(),
                "None",
            )

        active_prims = list(stage.Traverse())
        self.assertNotIn("joints", {prim.GetName() for prim in active_prims})
        self.assertNotIn("collisions", {prim.GetName() for prim in active_prims})
        for prim in active_prims:
            self.assertNotIn("Joint", str(prim.GetTypeName()))
            for schema in prim.GetAppliedSchemas():
                self.assertFalse(
                    any(
                        token in str(schema)
                        for token in (
                            "Physics",
                            "Physx",
                            "RigidBody",
                            "Articulation",
                            "Collision",
                            "Contact",
                        )
                    ),
                    f"active physics schema remains on {prim.GetPath()}: {schema}",
                )

        visual_prims = list(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
        self.assertEqual(sum(prim.IsA(UsdGeom.Mesh) for prim in visual_prims), 49)
        self.assertEqual(sum(prim.IsA(UsdShade.Material) for prim in visual_prims), 49)
        for prim in visual_prims:
            for property_name in prim.GetPropertyNames():
                lowered = str(property_name).lower()
                self.assertFalse(
                    lowered.startswith(("physics:", "physx", "drive:"))
                    or "contactreport" in lowered,
                    f"active physics property remains on {prim.GetPath()}: {property_name}",
                )

        # The standby layer references the same source.  Relative render-Mesh
        # and Material paths must therefore be identical, not merely similar in count.
        source_stage = Usd.Stage.Open(str(SOURCE_PATH), Usd.Stage.LoadAll)
        source_root = source_stage.GetDefaultPrim()
        for name in ("Physics", "Robot", "Sensor"):
            source_root.GetVariantSets().GetVariantSet(name).SetVariantSelection("None")
        source_prims = list(
            Usd.PrimRange(source_root, Usd.TraverseInstanceProxies())
        )
        self.assertEqual(
            self._relative_paths(visual_prims, root.GetPath(), UsdGeom.Mesh),
            self._relative_paths(source_prims, source_root.GetPath(), UsdGeom.Mesh),
        )
        self.assertEqual(
            self._relative_paths(visual_prims, root.GetPath(), UsdShade.Material),
            self._relative_paths(
                source_prims, source_root.GetPath(), UsdShade.Material
            ),
        )

        bbox = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        ).ComputeWorldBound(root).ComputeAlignedRange()
        self.assertAlmostEqual(bbox.GetMin()[2], -0.7575015817, places=9)
        self.assertAlmostEqual(bbox.GetMax()[2], 0.5305725935, places=9)


if __name__ == "__main__":
    unittest.main()
