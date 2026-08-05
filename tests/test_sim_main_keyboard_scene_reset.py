from __future__ import annotations

import ast
from pathlib import Path
import unittest


SIM_MAIN = Path(__file__).resolve().parents[1] / "sim_main.py"


class SimMainKeyboardSceneResetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SIM_MAIN.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source, filename=str(SIM_MAIN))

    def test_f12_callback_only_queues_the_request(self) -> None:
        callback = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.FunctionDef) and node.name == "_on_kb_scene_reset"
        )
        called_names = {
            node.func.id
            for node in ast.walk(callback)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        self.assertIn("request_scene_reset", called_names)
        self.assertNotIn("trigger_robot_reset", called_names)
        self.assertIn("KeyboardInput.F12", ast.get_source_segment(self.source, callback))

    def test_f12_binding_does_not_require_openxr(self) -> None:
        callback = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.FunctionDef) and node.name == "_on_kb_scene_reset"
        )
        xr_gate = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.If)
            and ast.get_source_segment(self.source, node.test)
            == 'args_cli.teleop_device != "none"'
        )
        self.assertNotIn(callback, ast.walk(xr_gate))

    def test_f12_is_limited_to_a_local_ubuntu_authority(self) -> None:
        self.assertIn('args_cli.no_render or getattr(args_cli, "headless", False)', self.source)
        self.assertIn('elif args_cli.replay_data:', self.source)
        self.assertIn('not sys.platform.startswith("linux") or is_scene_sync_viewer', self.source)

    def test_main_loop_uses_the_existing_safe_reset_path(self) -> None:
        self.assertIn('request_scene_reset("Ubuntu keyboard F12")', self.source)
        self.assertIn('if scene_reset_request["pending"]:', self.source)
        self.assertIn('trigger_robot_reset("reset_all_self", reset_source)', self.source)
        self.assertIn("unsubscribe_to_keyboard_events", self.source)


if __name__ == "__main__":
    unittest.main()
