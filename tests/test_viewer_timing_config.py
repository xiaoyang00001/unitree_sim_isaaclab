"""Dependency-free contracts for conveyor Viewer physics timing."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
)
SONIC_CONFIG_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_dex3_sonic/g1_29dof_dex3_sonic_env_cfg.py"
)
SCENE_SYNC_ENV_PATH = REPO_ROOT / "configs/scene_sync.env"


def _dotted_name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        prefix = _dotted_name(node.value)
        return f"{prefix}.{node.attr}" if prefix else node.attr
    return ""


def _assignment_target(node: ast.AST) -> str:
    if not isinstance(node, ast.Assign) or len(node.targets) != 1:
        return ""
    return _dotted_name(node.targets[0])


class ViewerTimingConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = CONFIG_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source, filename=str(CONFIG_PATH))
        cls.env_source = SCENE_SYNC_ENV_PATH.read_text(encoding="utf-8")

        cls.config_class = next(
            node
            for node in cls.tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "G129SonicConveyorEnvCfg"
        )
        cls.post_init = next(
            node
            for node in cls.config_class.body
            if isinstance(node, ast.FunctionDef) and node.name == "__post_init__"
        )
        cls.viewer_guard = next(
            node
            for node in cls.post_init.body
            if isinstance(node, ast.If)
            and "VIEWER_MODE" in (ast.get_source_segment(cls.source, node.test) or "")
            and {
                _assignment_target(descendant)
                for statement in node.body
                for descendant in ast.walk(statement)
            }
            >= {"self.sim.dt", "self.decimation", "self.sim.render_interval"}
        )
        cls.assignments = {
            _assignment_target(descendant): descendant
            for statement in cls.viewer_guard.body
            for descendant in ast.walk(statement)
            if _assignment_target(descendant)
            in {"self.sim.dt", "self.decimation", "self.sim.render_interval"}
        }

    def test_override_runs_after_parent_sonic_defaults(self) -> None:
        super_call = next(
            node
            for node in ast.walk(self.post_init)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "__post_init__"
            and isinstance(node.func.value, ast.Call)
            and isinstance(node.func.value.func, ast.Name)
            and node.func.value.func.id == "super"
        )
        self.assertLess(
            super_call.lineno,
            min(node.lineno for node in self.assignments.values()),
        )

    def test_override_is_viewer_only_and_has_an_explicit_rollback(self) -> None:
        guard = ast.get_source_segment(self.source, self.viewer_guard.test) or ""
        self.assertEqual(guard, "VIEWER_MODE and VIEWER_SINGLE_STEP_PHYSICS")
        all_timing_assignments = [
            node
            for node in ast.walk(self.post_init)
            if _assignment_target(node)
            in {"self.sim.dt", "self.decimation", "self.sim.render_interval"}
        ]
        self.assertEqual(len(all_timing_assignments), 3)
        self.assertEqual(
            {id(node) for node in all_timing_assignments},
            {id(node) for node in self.assignments.values()},
        )
        self.assertIn(
            'VIEWER_SINGLE_STEP_PHYSICS = _env_bool(\n'
            '    "ISAACLAB_VIEWER_SINGLE_STEP_PHYSICS", True\n'
            ")",
            self.source,
        )
        self.assertIn("ISAACLAB_VIEWER_SINGLE_STEP_PHYSICS=1", self.env_source)

    def test_viewer_uses_one_twenty_millisecond_physics_step(self) -> None:
        values = {
            target: ast.literal_eval(node.value)
            for target, node in self.assignments.items()
        }
        self.assertEqual(values["self.sim.dt"], 0.02)
        self.assertEqual(values["self.decimation"], 1)
        self.assertEqual(values["self.sim.render_interval"], 1)
        self.assertAlmostEqual(
            values["self.sim.dt"] * values["self.decimation"],
            0.02,
        )

    def test_authoritative_sonic_timing_remains_five_milliseconds_times_four(self) -> None:
        source = SONIC_CONFIG_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(SONIC_CONFIG_PATH))
        config_class = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "G129Dex3SonicEnvCfg"
        )
        post_init = next(
            node
            for node in config_class.body
            if isinstance(node, ast.FunctionDef) and node.name == "__post_init__"
        )
        assignments = {
            _assignment_target(node): node.value
            for node in ast.walk(post_init)
            if _assignment_target(node)
            in {"self.sim.dt", "self.decimation", "self.sim.render_interval"}
        }

        self.assertEqual(ast.literal_eval(assignments["self.sim.dt"]), 0.005)
        self.assertEqual(ast.literal_eval(assignments["self.decimation"]), 4)
        self.assertEqual(
            _dotted_name(assignments["self.sim.render_interval"]),
            "self.decimation",
        )


if __name__ == "__main__":
    unittest.main()
