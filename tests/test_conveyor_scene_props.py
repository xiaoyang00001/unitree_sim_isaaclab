from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_props.py"
)
_SPEC = importlib.util.spec_from_file_location("conveyor_scene_props_for_test", _MODULE_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _MODULE
_SPEC.loader.exec_module(_MODULE)


class ConveyorScenePropsTest(unittest.TestCase):
    def test_conveyor_layout_only_spawns_task_totes(self) -> None:
        props = _MODULE.resolve_scene_props({}, totes_on_conveyor=True)

        self.assertEqual(props.mode, "layout")
        self.assertEqual(props.spawned_names, ("cart2_tote1", "cart2_tote2"))
        self.assertFalse(props.inherited_packing_table)
        for unused in (
            "pushcart",
            "cart_box1",
            "cart_box2",
            "test_box",
            "pushcart_2",
            "cube_1",
            "cube_2",
            "cube_3",
        ):
            self.assertFalse(props.spawns(unused), unused)

    def test_pushcart_layout_keeps_only_second_cart_and_totes(self) -> None:
        props = _MODULE.resolve_scene_props({}, totes_on_conveyor=False)

        self.assertEqual(
            props.spawned_names,
            ("pushcart_2", "cart2_tote1", "cart2_tote2"),
        )
        self.assertFalse(props.spawns("pushcart"))
        self.assertFalse(props.spawns("cart_box1"))
        self.assertFalse(props.spawns("test_box"))
        self.assertFalse(props.inherited_packing_table)

    def test_legacy_props_restores_previous_inventory_for_both_layouts(self) -> None:
        expected = (
            "pushcart",
            "cart_box1",
            "cart_box2",
            "test_box",
            "pushcart_2",
            "cart2_tote1",
            "cart2_tote2",
            "cube_1",
            "cube_2",
            "cube_3",
        )
        environ = {_MODULE.PROP_MODE_ENV: " legacy_props "}

        for totes_on_conveyor in (True, False):
            with self.subTest(totes_on_conveyor=totes_on_conveyor):
                props = _MODULE.resolve_scene_props(
                    environ,
                    totes_on_conveyor=totes_on_conveyor,
                )
                self.assertEqual(props.spawned_names, expected)
                self.assertTrue(props.inherited_packing_table)
                self.assertTrue(props.legacy_props)

    def test_unknown_prop_mode_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "可选值"):
            _MODULE.resolve_scene_props(
                {_MODULE.PROP_MODE_ENV: "all_maybe"},
                totes_on_conveyor=True,
            )


if __name__ == "__main__":
    unittest.main()
