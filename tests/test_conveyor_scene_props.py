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


_BELT_BOXES = ("belt_box_1", "belt_box_2", "belt_box_3", "belt_box_4", "belt_box_5")


class ConveyorScenePropsTest(unittest.TestCase):
    def test_conveyor_layout_only_spawns_belt_boxes(self) -> None:
        """流水线布局的作业对象是纸箱队列，两塑料筐已经彻底退场。"""

        props = _MODULE.resolve_scene_props(
            {}, totes_on_conveyor=True, belt_box_names=_BELT_BOXES
        )

        self.assertEqual(props.mode, "layout")
        self.assertEqual(props.spawned_names, _BELT_BOXES)
        self.assertFalse(props.inherited_packing_table)
        for unused in (
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
        ):
            self.assertFalse(props.spawns(unused), unused)

    def test_pushcart_layout_keeps_only_second_cart_and_totes(self) -> None:
        """推车布局的作业闭环仍是两塑料筐，不生成纸箱。"""

        props = _MODULE.resolve_scene_props(
            {}, totes_on_conveyor=False, belt_box_names=_BELT_BOXES
        )

        self.assertEqual(
            props.spawned_names,
            ("pushcart_2", "cart2_tote1", "cart2_tote2"),
        )
        self.assertFalse(props.spawns("pushcart"))
        self.assertFalse(props.spawns("cart_box1"))
        self.assertFalse(props.spawns("test_box"))
        for name in _BELT_BOXES:
            self.assertFalse(props.spawns(name), name)
        self.assertFalse(props.inherited_packing_table)

    def test_legacy_props_is_a_pure_rollback_without_belt_boxes(self) -> None:
        """legacy_props 是纯回退：恢复旧的 10 个道具，且**不**叠加纸箱队列。

        叠加会撞车：两塑料筐的流水线出生位 (-5.35, 17.4) / (-5.89, 18.0) 与纸箱队列的
        第 4、5 个同 y 且同在带上，筐半宽 0.15 + 纸箱半宽 0.125 = 0.275 > 车道间距 0.27
        ⇒ 各三轴互穿 5 mm，开局即非法初始状态。
        """

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
                    belt_box_names=_BELT_BOXES,
                )
                self.assertEqual(props.spawned_names, expected)
                self.assertTrue(props.inherited_packing_table)
                self.assertTrue(props.legacy_props)
                for name in _BELT_BOXES:
                    self.assertFalse(props.spawns(name), name)

    def test_belt_boxes_never_share_the_belt_with_the_totes(self) -> None:
        """任何道具模式下，纸箱和塑料筐都不得同时出现——它们的出生位会互穿。"""

        for mode in (_MODULE.DEFAULT_PROP_MODE, "legacy_props"):
            for totes_on_conveyor in (True, False):
                with self.subTest(mode=mode, totes_on_conveyor=totes_on_conveyor):
                    props = _MODULE.resolve_scene_props(
                        {_MODULE.PROP_MODE_ENV: mode},
                        totes_on_conveyor=totes_on_conveyor,
                        belt_box_names=_BELT_BOXES,
                    )
                    has_tote = props.spawns("cart2_tote1") or props.spawns("cart2_tote2")
                    has_box = any(props.spawns(name) for name in _BELT_BOXES)
                    self.assertFalse(has_tote and has_box, props.spawned_names)

    def test_belt_box_names_default_to_empty(self) -> None:
        """不传纸箱清单时流水线布局什么都不生成——调用方必须显式给出清单。"""

        props = _MODULE.resolve_scene_props({}, totes_on_conveyor=True)

        self.assertEqual(props.spawned_names, ())

    def test_unknown_prop_mode_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "可选值"):
            _MODULE.resolve_scene_props(
                {_MODULE.PROP_MODE_ENV: "all_maybe"},
                totes_on_conveyor=True,
            )


if __name__ == "__main__":
    unittest.main()
