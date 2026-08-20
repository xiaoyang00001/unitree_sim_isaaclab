"""Dependency-free contracts for the pure scene-sync viewer DDS boundary."""

from __future__ import annotations

import ast
import importlib.util
import os
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SIM_MAIN_PATH = REPO_ROOT / "sim_main.py"
DDS_MASTER_PATH = REPO_ROOT / "dds/dds_master.py"
CONVEYOR_CFG_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
)
G1_STATE_PATH = REPO_ROOT / "tasks/common_observations/g1_29dof_state.py"
H12_STATE_PATH = REPO_ROOT / "tasks/common_observations/h12_27dof_state.py"


def _dotted_name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        prefix = _dotted_name(node.value)
        return f"{prefix}.{node.attr}" if prefix else node.attr
    return ""


def _calls_named(tree: ast.AST, name: str) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and _dotted_name(node.func) == name
    ]


def _target_name(node: ast.AST) -> str:
    if isinstance(node, (ast.Name, ast.Attribute)):
        return _dotted_name(node)
    return ""


def _assignment_value(node: ast.AST) -> ast.AST | None:
    if isinstance(node, (ast.Assign, ast.AnnAssign)):
        return node.value
    return None


def _assignment_targets(node: ast.AST) -> tuple[str, ...]:
    if isinstance(node, ast.Assign):
        return tuple(_target_name(target) for target in node.targets)
    if isinstance(node, ast.AnnAssign):
        return (_target_name(node.target),)
    return ()


class _SourceTree:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.source = path.read_text(encoding="utf-8")
        self.tree = ast.parse(self.source, filename=str(path))
        self.parents: dict[ast.AST, ast.AST] = {}
        for parent in ast.walk(self.tree):
            for child in ast.iter_child_nodes(parent):
                self.parents[child] = parent

    def segment(self, node: ast.AST) -> str:
        return ast.get_source_segment(self.source, node) or ""

    def ancestors(self, node: ast.AST):
        current = node
        while current in self.parents:
            current = self.parents[current]
            yield current

    @staticmethod
    def _contains(container: list[ast.stmt], target: ast.AST) -> bool:
        return any(
            descendant is target
            for statement in container
            for descendant in ast.walk(statement)
        )

    def has_positive_if_guard(self, node: ast.AST, required_text: str) -> bool:
        """Return whether node is in the body of an If containing the guard."""

        for ancestor in self.ancestors(node):
            if not isinstance(ancestor, ast.If):
                continue
            if required_text not in self.segment(ancestor.test):
                continue
            if self._contains(ancestor.body, node):
                return True
        return False


class SimMainViewerDDSIsolationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = _SourceTree(SIM_MAIN_PATH)

    def test_viewer_forces_hold_before_any_dds_factory_can_run(self) -> None:
        hold_assignments = []
        for node in ast.walk(self.module.tree):
            if "args_cli.action_source" not in _assignment_targets(node):
                continue
            value = _assignment_value(node)
            if isinstance(value, ast.Constant) and value.value == "hold":
                hold_assignments.append(node)

        early_hold = min(hold_assignments, key=lambda node: node.lineno)
        self.assertTrue(
            self.module.has_positive_if_guard(early_hold, "is_scene_sync_viewer")
        )

        factory_calls = _calls_named(self.module.tree, "create_dds_objects")
        factory_calls += _calls_named(self.module.tree, "create_dds_objects_replay")
        self.assertTrue(factory_calls)
        self.assertLess(early_hold.lineno, min(call.lineno for call in factory_calls))

    def test_live_dds_creation_and_import_are_runtime_gated(self) -> None:
        live_calls = _calls_named(self.module.tree, "create_dds_objects")
        self.assertEqual(len(live_calls), 1)
        self.assertTrue(
            self.module.has_positive_if_guard(
                live_calls[0], "runtime_dds_enabled"
            )
        )

        dds_imports = [
            node
            for node in ast.walk(self.module.tree)
            if isinstance(node, ast.ImportFrom) and node.module == "dds.dds_create"
        ]
        self.assertEqual(len(dds_imports), 1)
        self.assertTrue(
            self.module.has_positive_if_guard(
                dds_imports[0], "runtime_dds_enabled"
            )
        )

    def test_viewer_rejects_replay_before_replay_dds_creation(self) -> None:
        replay_calls = _calls_named(self.module.tree, "create_dds_objects_replay")
        self.assertEqual(len(replay_calls), 1)

        viewer_blocks = [
            node
            for node in self.module.tree.body
            if isinstance(node, ast.If)
            and self.module.segment(node.test) == "is_scene_sync_viewer"
        ]
        viewer_block = next(
            node
            for node in viewer_blocks
            if any(
                isinstance(descendant, ast.If)
                and self.module.segment(descendant.test) == "args_cli.replay_data"
                for descendant in ast.walk(node)
            )
        )
        replay_rejection = next(
            node
            for node in ast.walk(viewer_block)
            if isinstance(node, ast.If)
            and self.module.segment(node.test) == "args_cli.replay_data"
        )
        error_calls = [
            call
            for call in ast.walk(replay_rejection)
            if isinstance(call, ast.Call) and _dotted_name(call.func) == "parser.error"
        ]
        self.assertEqual(len(error_calls), 1)
        self.assertLess(error_calls[0].lineno, replay_calls[0].lineno)

    def test_sim_state_export_is_disabled_without_runtime_dds(self) -> None:
        disable_gate = next(
            node
            for node in ast.walk(self.module.tree)
            if isinstance(node, ast.If)
            and "not runtime_dds_enabled" in self.module.segment(node.test)
            and "sim_state_dds is None" in self.module.segment(node.test)
        )
        disabled_assignments = [
            node
            for statement in disable_gate.body
            for node in ast.walk(statement)
            if "sim_state_export_enabled" in _assignment_targets(node)
            and isinstance(_assignment_value(node), ast.Constant)
            and _assignment_value(node).value is False
        ]
        self.assertEqual(len(disabled_assignments), 1)

        writes = _calls_named(
            self.module.tree, "sim_state_dds.write_sim_state_data"
        )
        self.assertEqual(len(writes), 1)
        self.assertTrue(self.module.has_positive_if_guard(writes[0], "export_due"))

    def test_reset_pose_reads_and_writes_require_the_runtime_object(self) -> None:
        reads = _calls_named(
            self.module.tree, "reset_pose_dds.get_reset_pose_command"
        )
        self.assertEqual(len(reads), 1)
        self.assertTrue(
            self.module.has_positive_if_guard(reads[0], "runtime_dds_enabled")
        )
        self.assertTrue(
            self.module.has_positive_if_guard(reads[0], "reset_pose_dds is not None")
        )

        writes = _calls_named(
            self.module.tree, "reset_pose_dds.write_reset_pose_command"
        )
        self.assertEqual(len(writes), 2)
        for write in writes:
            self.assertTrue(
                self.module.has_positive_if_guard(write, "reset_pose_cmd is not None")
            )

    def test_publish_tuning_and_initial_seeding_are_runtime_gated(self) -> None:
        tuning_calls = _calls_named(self.module.tree, "dds_manager.set_publish_rate")
        tuning_calls += _calls_named(
            self.module.tree, "dds_manager.enable_immediate_publish"
        )
        self.assertEqual(len(tuning_calls), 4)
        for call in tuning_calls:
            self.assertTrue(
                self.module.has_positive_if_guard(call, "runtime_dds_enabled"),
                self.module.segment(call),
            )

        seed_calls = _calls_named(self.module.tree, "get_robot_boy_joint_states")
        seed_calls += _calls_named(self.module.tree, "get_robot_dex3_joint_states")
        self.assertEqual(len(seed_calls), 2)
        for call in seed_calls:
            self.assertTrue(
                self.module.has_positive_if_guard(call, "runtime_dds_enabled"),
                self.module.segment(call),
            )


def _fake_module(name: str, **attributes) -> ModuleType:
    module = ModuleType(name)
    for attribute_name, value in attributes.items():
        setattr(module, attribute_name, value)
    return module


class DDSManagerDisableSwitchTest(unittest.TestCase):
    _module_counter = 0

    def _load_dds_master(self, disabled_value: str):
        channel_init_calls: list[tuple] = []
        unitree_package = _fake_module("unitree_sdk2py")
        unitree_package.__path__ = []
        unitree_core_package = _fake_module("unitree_sdk2py.core")
        unitree_core_package.__path__ = []
        fake_modules = {
            "unitree_sdk2py": unitree_package,
            "unitree_sdk2py.core": unitree_core_package,
            "unitree_sdk2py.core.channel": _fake_module(
                "unitree_sdk2py.core.channel",
                ChannelFactoryInitialize=lambda *args: channel_init_calls.append(args),
            ),
            "dds.dds_base": _fake_module("dds.dds_base", DDSObject=object),
        }
        type(self)._module_counter += 1
        module_name = f"_test_viewer_dds_master_{self._module_counter}"
        spec = importlib.util.spec_from_file_location(module_name, DDS_MASTER_PATH)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        test_environment = {
            "UNITREE_SIM_DISABLE_DDS": disabled_value,
            "UNITREE_DDS_DOMAIN": "177",
            "UNITREE_DDS_INTERFACE": "loopback-test",
        }
        with mock.patch.dict(os.environ, test_environment, clear=True):
            with mock.patch.dict(sys.modules, fake_modules):
                spec.loader.exec_module(module)
        return module, channel_init_calls

    def test_disable_switch_never_initializes_the_unitree_channel_factory(self) -> None:
        for value in ("1", "true", "yes", "on"):
            with self.subTest(value=value):
                module, calls = self._load_dds_master(value)

                self.assertEqual(calls, [])
                self.assertTrue(module.dds_manager.dds_disabled)
                self.assertFalse(module.dds_manager.dds_initialized)

    def test_enabled_manager_keeps_the_existing_initialization_path(self) -> None:
        module, calls = self._load_dds_master("0")

        self.assertEqual(calls, [(177, "loopback-test")])
        self.assertFalse(module.dds_manager.dds_disabled)
        self.assertTrue(module.dds_manager.dds_initialized)


class ViewerObservationDDSIsolationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = _SourceTree(CONVEYOR_CFG_PATH)

    def test_viewer_body_and_hand_observations_disable_dds(self) -> None:
        viewer_cfg = next(
            node
            for node in self.cfg.tree.body
            if isinstance(node, ast.ClassDef) and node.name == "ViewerObservationsCfg"
        )
        policy_cfg = next(
            node
            for node in viewer_cfg.body
            if isinstance(node, ast.ClassDef) and node.name == "PolicyCfg"
        )

        terms = {}
        for node in policy_cfg.body:
            targets = _assignment_targets(node)
            value = _assignment_value(node)
            if not targets or not isinstance(value, ast.Call):
                continue
            if _dotted_name(value.func) != "ObsTerm":
                continue
            terms[targets[0]] = value

        self.assertEqual(
            set(terms), {"robot_body_state", "robot_dex3_state"}
        )
        for name, call in terms.items():
            params_keyword = next(
                keyword for keyword in call.keywords if keyword.arg == "params"
            )
            self.assertIsInstance(params_keyword.value, ast.Dict)
            params = {
                key.value: value
                for key, value in zip(
                    params_keyword.value.keys, params_keyword.value.values
                )
                if isinstance(key, ast.Constant)
            }
            self.assertIn("enable_dds", params, name)
            self.assertIsInstance(params["enable_dds"], ast.Constant)
            self.assertIs(params["enable_dds"].value, False, name)

    def test_viewer_mode_selects_the_dds_free_observation_group(self) -> None:
        env_cfg = next(
            node
            for node in self.cfg.tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "G129SonicConveyorEnvCfg"
        )
        observations = next(
            node
            for node in env_cfg.body
            if "observations" in _assignment_targets(node)
        )
        value = _assignment_value(observations)
        self.assertIsInstance(value, ast.IfExp)
        viewer_choice = value.orelse
        self.assertIsInstance(viewer_choice, ast.IfExp)
        self.assertEqual(self.cfg.segment(viewer_choice.test), "VIEWER_MODE")
        self.assertEqual(_dotted_name(viewer_choice.body.func), "ViewerObservationsCfg")
        self.assertEqual(_dotted_name(viewer_choice.orelse.func), "SonicObservationsCfg")

    def test_robot_state_modules_do_not_import_dds_master_at_module_scope(self) -> None:
        for path in (G1_STATE_PATH, H12_STATE_PATH):
            with self.subTest(path=path.name):
                module = _SourceTree(path)
                top_level_dds_imports = [
                    node
                    for node in module.tree.body
                    if isinstance(node, ast.ImportFrom)
                    and node.module == "dds.dds_master"
                ]
                self.assertEqual(top_level_dds_imports, [])

                getter = next(
                    node
                    for node in module.tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "_get_g1_robot_dds_instance"
                )
                lazy_imports = [
                    node
                    for node in ast.walk(getter)
                    if isinstance(node, ast.ImportFrom)
                    and node.module == "dds.dds_master"
                ]
                self.assertEqual(len(lazy_imports), 1)


if __name__ == "__main__":
    unittest.main()
