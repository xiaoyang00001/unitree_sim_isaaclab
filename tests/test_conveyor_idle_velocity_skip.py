"""CPU-only no-op velocity-write skip contracts for conveyor belt boxes.

The production event imports Isaac Sim, so these tests compile the two small
real helpers and the real manager-term class from their AST with lightweight
dependencies.  This keeps the tests runnable without starting Kit while still
exercising the production function bodies rather than copied logic.
"""

from __future__ import annotations

import ast
import contextlib
import io
from pathlib import Path
from types import SimpleNamespace
import unittest

import torch


_REPO_ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_EVENTS_PATH = _TASK_DIR / "conveyor_events.py"
_CFG_PATH = _TASK_DIR / "conveyor_env_cfg.py"


def _top_level_node(tree: ast.Module, node_type, name: str):
    return next(
        node
        for node in tree.body
        if isinstance(node, node_type) and node.name == name
    )


def _compile_velocity_helpers():
    tree = ast.parse(_EVENTS_PATH.read_text(encoding="utf-8"))
    nodes = [
        _top_level_node(tree, ast.FunctionDef, "_should_skip_idle_velocity_write"),
        _top_level_node(tree, ast.FunctionDef, "_write_belt_box_velocities"),
    ]
    module = ast.Module(body=nodes, type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {"torch": torch}
    exec(compile(module, str(_EVENTS_PATH), "exec"), namespace)
    return (
        namespace["_should_skip_idle_velocity_write"],
        namespace["_write_belt_box_velocities"],
    )


_SHOULD_SKIP, _WRITE_VELOCITIES = _compile_velocity_helpers()


class _VelocityData:
    def __init__(self, root_vel_w: torch.Tensor) -> None:
        self._root_vel_w = root_vel_w.clone()
        self.reads = 0

    @property
    def root_vel_w(self) -> torch.Tensor:
        self.reads += 1
        return self._root_vel_w


class _FakeRigidObject:
    def __init__(self, root_vel_w: torch.Tensor) -> None:
        self.data = _VelocityData(root_vel_w)
        self.writes: list[tuple[torch.Tensor, torch.Tensor]] = []

    def write_root_velocity_to_sim(
        self, velocity: torch.Tensor, *, env_ids: torch.Tensor
    ) -> None:
        self.writes.append((velocity.clone(), env_ids.clone()))


def _objects(count: int, num_envs: int = 1) -> list[_FakeRigidObject]:
    result = []
    for object_index in range(count):
        values = torch.arange(num_envs * 6, dtype=torch.float32).reshape(num_envs, 6)
        values = values + object_index * 100.0 + 0.25
        result.append(_FakeRigidObject(values))
    return result


def _write(
    objects: list[_FakeRigidObject],
    drive: torch.Tensor,
    *,
    enabled: bool,
    path_enabled: bool = False,
    hx: torch.Tensor | None = None,
    hy: torch.Tensor | None = None,
) -> tuple[int, int]:
    return _WRITE_VELOCITIES(
        objects,
        torch.arange(drive.shape[1]),
        drive,
        velocity_y=-0.3,
        path_enabled=path_enabled,
        hx=hx,
        hy=hy,
        skip_idle_velocity_writes=enabled,
    )


class CpuIdleVelocityWriteTest(unittest.TestCase):
    def test_all_seventeen_false_rows_skip_every_read_and_write(self) -> None:
        objects = _objects(17)

        self.assertEqual(
            _write(objects, torch.zeros((17, 1), dtype=torch.bool), enabled=True),
            (0, 17),
        )
        self.assertTrue(all(obj.data.reads == 0 for obj in objects))
        self.assertTrue(all(not obj.writes for obj in objects))

    def test_true_and_mixed_rows_write_without_dropping_any_driven_env(self) -> None:
        objects = _objects(3, num_envs=2)
        before = [obj.data._root_vel_w.clone() for obj in objects]
        drive = torch.tensor(
            [[False, False], [False, True], [True, False]], dtype=torch.bool
        )

        self.assertEqual(_write(objects, drive, enabled=True), (2, 1))
        self.assertEqual([obj.data.reads for obj in objects], [0, 1, 1])
        self.assertEqual([len(obj.writes) for obj in objects], [0, 1, 1])

        for object_index in (1, 2):
            actual = objects[object_index].writes[0][0]
            expected = before[object_index].clone()
            expected[drive[object_index], 0] = 0.0
            expected[drive[object_index], 1] = -0.3
            torch.testing.assert_close(actual, expected)
            # Z and angular velocity stay entirely under physics ownership.
            torch.testing.assert_close(actual[:, 2:], before[object_index][:, 2:])

    def test_curved_path_driving_still_uses_per_box_heading(self) -> None:
        objects = _objects(2)
        before = objects[1].data._root_vel_w.clone()
        hx = torch.tensor([[0.0], [0.6]], dtype=torch.float32)
        hy = torch.tensor([[-1.0], [-0.8]], dtype=torch.float32)

        self.assertEqual(
            _write(
                objects,
                torch.tensor([[False], [True]]),
                enabled=True,
                path_enabled=True,
                hx=hx,
                hy=hy,
            ),
            (1, 1),
        )
        actual = objects[1].writes[0][0]
        self.assertAlmostEqual(float(actual[0, 0]), 0.18, places=6)
        self.assertAlmostEqual(float(actual[0, 1]), -0.24, places=6)
        torch.testing.assert_close(actual[:, 2:], before[:, 2:])

    def test_flag_off_is_the_historical_read_and_write_path(self) -> None:
        objects = _objects(4, num_envs=2)
        before = [obj.data._root_vel_w.clone() for obj in objects]

        self.assertEqual(
            _write(objects, torch.zeros((4, 2), dtype=torch.bool), enabled=False),
            (4, 0),
        )
        self.assertEqual([obj.data.reads for obj in objects], [1, 1, 1, 1])
        self.assertEqual([len(obj.writes) for obj in objects], [1, 1, 1, 1])
        for obj, expected in zip(objects, before):
            torch.testing.assert_close(obj.writes[0][0], expected)

    def test_non_cpu_and_disabled_checks_fail_closed_without_any_sync(self) -> None:
        class _MaskThatMustNotBeRead:
            def __init__(self, device_type: str) -> None:
                self.device = SimpleNamespace(type=device_type)

            def any(self):
                raise AssertionError("any() must remain behind the CPU opt-in gate")

        self.assertFalse(_SHOULD_SKIP(_MaskThatMustNotBeRead("cuda"), enabled=True))
        self.assertFalse(_SHOULD_SKIP(_MaskThatMustNotBeRead("cpu"), enabled=False))

    def test_cuda_tagged_table_uses_the_full_historical_write_path(self) -> None:
        class _CudaTaggedDriveTable:
            """CPU-backed torch rows with a synthetic non-CPU table device."""

            def __init__(self, values: torch.Tensor) -> None:
                self._values = values
                self.shape = values.shape
                self.device = SimpleNamespace(type="cuda")

            def __getitem__(self, index: int) -> torch.Tensor:
                return self._values[index]

        objects = _objects(2)
        drive = _CudaTaggedDriveTable(torch.zeros((2, 1), dtype=torch.bool))
        result = _WRITE_VELOCITIES(
            objects,
            torch.arange(1),
            drive,
            velocity_y=-0.3,
            path_enabled=False,
            hx=None,
            hy=None,
            skip_idle_velocity_writes=True,
        )
        self.assertEqual(result, (2, 0))
        self.assertEqual([obj.data.reads for obj in objects], [1, 1])
        self.assertEqual([len(obj.writes) for obj in objects], [1, 1])

    def test_idle_row_resumes_writing_as_soon_as_drive_becomes_true(self) -> None:
        objects = _objects(1)
        self.assertEqual(
            _write(objects, torch.tensor([[False]]), enabled=True), (0, 1)
        )
        self.assertEqual(
            _write(objects, torch.tensor([[True]]), enabled=True), (1, 0)
        )
        self.assertEqual(objects[0].data.reads, 1)
        self.assertEqual(len(objects[0].writes), 1)
        self.assertAlmostEqual(float(objects[0].writes[0][0][0, 1]), -0.3)


class _FakeManagerTermBase:
    def __init__(self, cfg, env) -> None:
        self.cfg = cfg
        self.env = env


def _compile_manager_term(report_interval: int = 3):
    tree = ast.parse(_EVENTS_PATH.read_text(encoding="utf-8"))
    class_node = _top_level_node(tree, ast.ClassDef, "DriveBeltBoxesOnConveyor")
    module = ast.Module(body=[class_node], type_ignores=[])
    ast.fix_missing_locations(module)
    calls = []

    def _fake_drive(*args, **kwargs):
        calls.append((args, kwargs))
        return len(kwargs["object_names"]), 0

    namespace = {
        "torch": torch,
        "ManagerTermBase": _FakeManagerTermBase,
        "ManagerBasedEnv": object,
        "BELT_TOP_Z": 0.772,
        "BELT_Y_MIN": 10.19,
        "BELT_Y_MAX": 18.22,
        "_IDLE_WRITE_DIAGNOSTIC_INTERVAL_CALLS": report_interval,
        "drive_belt_boxes_on_conveyor": _fake_drive,
    }
    exec(compile(module, str(_EVENTS_PATH), "exec"), namespace)
    return namespace["DriveBeltBoxesOnConveyor"], calls


class ManagerTermContractTest(unittest.TestCase):
    def _term(
        self,
        *,
        skip_requested: bool = True,
        drive_enabled: bool = True,
        report_interval: int = 3,
        object_names: tuple[str, ...] = ("box_1", "box_2"),
    ):
        term_class, calls = _compile_manager_term(report_interval)
        cfg = SimpleNamespace(
            params={
                "object_names": object_names,
                "skip_idle_velocity_writes": skip_requested,
                "enabled": drive_enabled,
            }
        )
        env = SimpleNamespace(num_envs=2, device="cpu")
        with contextlib.redirect_stdout(io.StringIO()):
            term = term_class(cfg, env)
        return term, env, calls

    def test_reset_clears_latches_and_next_call_uses_cleared_state(self) -> None:
        term, env, calls = self._term()
        term._departure_completed[:] = True
        term._arrived[:] = True
        term._lifted[:] = True
        term._belt_dwell[:] = 9
        term._idle_write_diag_calls = 2

        term.reset()
        self.assertFalse(bool(term._departure_completed.any()))
        self.assertFalse(bool(term._arrived.any()))
        self.assertFalse(bool(term._lifted.any()))
        self.assertFalse(bool(term._belt_dwell.any()))
        self.assertEqual(term._idle_write_diag_calls, 0)

        term(
            env,
            None,
            object_names=("box_1", "box_2"),
            half_lengths=(0.19, 0.19),
            skip_idle_velocity_writes=True,
        )
        kwargs = calls[-1][1]
        self.assertIs(kwargs["departure_completed"], term._departure_completed)
        self.assertIs(kwargs["arrived"], term._arrived)
        self.assertIs(kwargs["lifted"], term._lifted)
        self.assertIs(kwargs["belt_dwell"], term._belt_dwell)
        self.assertTrue(kwargs["skip_idle_velocity_writes"])

    def test_diagnostics_are_opt_in_batched_and_show_seventeen_of_seventeen(self) -> None:
        term, _, _ = self._term(report_interval=3)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            term._record_idle_write_diagnostics(0, 17)
            term._record_idle_write_diagnostics(0, 17)
        self.assertEqual(output.getvalue(), "")

        disabled_backend, _, _ = self._term(
            skip_requested=True,
            drive_enabled=False,
            report_interval=1,
        )
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            disabled_backend._record_idle_write_diagnostics(0, 17)
        self.assertEqual(output.getvalue(), "")

        with contextlib.redirect_stdout(output):
            term._record_idle_write_diagnostics(0, 17)
        report = output.getvalue()
        self.assertIn("calls=3", report)
        self.assertIn("skipped=51", report)
        self.assertIn("writes=0", report)
        self.assertIn("skipped_per_call=17.00/17.00", report)
        self.assertEqual(term._idle_write_diag_calls, 0)

        disabled_term, _, _ = self._term(skip_requested=False, report_interval=1)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            disabled_term._record_idle_write_diagnostics(0, 17)
        self.assertEqual(output.getvalue(), "")

    def test_empty_layout_does_not_advertise_or_accumulate_fast_path(self) -> None:
        term_class, _ = _compile_manager_term(report_interval=1)
        cfg = SimpleNamespace(
            params={
                "object_names": (),
                "skip_idle_velocity_writes": True,
                "enabled": True,
            }
        )
        env = SimpleNamespace(num_envs=2, device="cpu")
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            term = term_class(cfg, env)
            term._record_idle_write_diagnostics(0, 0)

        self.assertFalse(term._idle_write_skip_requested)
        self.assertEqual(term._idle_write_diag_calls, 0)
        self.assertEqual(output.getvalue(), "")


class StaticConfigurationContractTest(unittest.TestCase):
    def test_env_flag_defaults_false_and_is_passed_to_event_term(self) -> None:
        tree = ast.parse(_CFG_PATH.read_text(encoding="utf-8"))
        assignment = next(
            node
            for node in tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "CONVEYOR_SKIP_IDLE_VELOCITY_WRITES"
                for target in node.targets
            )
        )
        self.assertIsInstance(assignment.value, ast.Call)
        call = assignment.value
        assert isinstance(call, ast.Call)
        self.assertEqual(ast.unparse(call.func), "_env_bool")
        self.assertEqual(
            [ast.literal_eval(argument) for argument in call.args],
            ["ISAACLAB_CONVEYOR_SKIP_IDLE_VELOCITY_WRITES", False],
        )

        events_class = _top_level_node(tree, ast.ClassDef, "ConveyorEventsCfg")
        drive_assignment = next(
            node
            for node in events_class.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "drive_belt_boxes"
                for target in node.targets
            )
        )
        event_call = drive_assignment.value
        assert isinstance(event_call, ast.Call)
        params = next(
            keyword.value for keyword in event_call.keywords if keyword.arg == "params"
        )
        assert isinstance(params, ast.Dict)
        mapping = {
            ast.literal_eval(key): value for key, value in zip(params.keys, params.values)
        }
        self.assertEqual(
            ast.unparse(mapping["skip_idle_velocity_writes"]),
            "CONVEYOR_SKIP_IDLE_VELOCITY_WRITES",
        )

    def test_belt_box_paths_have_no_acceleration_cache_consumer(self) -> None:
        # RigidObject velocity writes also clear body_com_acc_w.  Skipping a
        # no-op write therefore leaves only that cache untouched.  These are
        # the complete belt-box state/queue/sync consumers; lock the premise
        # that none of them observes acceleration, while sync still exports
        # the six-component velocity as part of root_state.
        paths = (
            _EVENTS_PATH,
            _TASK_DIR / "conveyor_queue.py",
            _TASK_DIR / "zmq_scene_sync.py",
        )
        for path in paths:
            source = path.read_text(encoding="utf-8")
            self.assertNotIn("body_com_acc_w", source, path.name)
            self.assertNotIn("body_acc_w", source, path.name)
        sync_source = paths[-1].read_text(encoding="utf-8")
        self.assertIn("rigid_object.data.root_state_w[0].tolist()", sync_source)


if __name__ == "__main__":
    unittest.main()
