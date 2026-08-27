from __future__ import annotations

import importlib.util
from pathlib import Path
import re
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/timing_stats.py"
)
SYNC_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/zmq_scene_sync.py"
)
CONFIG_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
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


timing = _load_module("_test_scene_sync_timing", MODULE_PATH)


class FakeClock:
    def __init__(self) -> None:
        self.value = 100.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


class SceneSyncTimingWindowTests(unittest.TestCase):
    def test_percentiles_use_linear_interpolation_in_milliseconds(self) -> None:
        clock = FakeClock()
        window = timing.SceneSyncTimingWindow(enabled=True, clock=clock)
        for milliseconds in (1.0, 2.0, 3.0, 4.0, 5.0):
            window.record_seconds("apply", milliseconds / 1000.0)
        clock.advance(5.0)

        snapshot = window.pop()

        self.assertIsNotNone(snapshot)
        metric = snapshot.metrics["apply"]
        self.assertEqual(metric.total_count, 5)
        self.assertEqual(metric.retained_count, 5)
        self.assertAlmostEqual(metric.mean_ms, 3.0)
        self.assertAlmostEqual(metric.p50_ms, 3.0)
        self.assertAlmostEqual(metric.p95_ms, 4.8)
        self.assertAlmostEqual(metric.p99_ms, 4.96)
        self.assertAlmostEqual(metric.max_ms, 5.0)
        self.assertAlmostEqual(snapshot.window_s, 5.0)

    def test_retention_is_bounded_but_full_window_mean_and_max_are_preserved(self) -> None:
        window = timing.SceneSyncTimingWindow(enabled=True, max_samples=3)
        for milliseconds in (1.0, 2.0, 3.0, 4.0, 5.0):
            window.record_seconds("pump", milliseconds / 1000.0)

        metric = window.pop().metrics["pump"]

        self.assertEqual(metric.total_count, 5)
        self.assertEqual(metric.retained_count, 3)
        self.assertAlmostEqual(metric.mean_ms, 3.0)
        self.assertAlmostEqual(metric.p50_ms, 4.0)
        self.assertAlmostEqual(metric.max_ms, 5.0)

    def test_retention_does_not_drop_an_early_full_window_maximum(self) -> None:
        window = timing.SceneSyncTimingWindow(enabled=True, max_samples=2)
        for milliseconds in (100.0, 1.0, 2.0):
            window.record_seconds("receive", milliseconds / 1000.0)

        metric = window.pop().metrics["receive"]

        self.assertAlmostEqual(metric.mean_ms, 103.0 / 3.0)
        self.assertAlmostEqual(metric.max_ms, 100.0)
        self.assertAlmostEqual(metric.p50_ms, 1.5)

    def test_disabled_window_has_no_samples_and_reconfigure_clears_old_data(self) -> None:
        window = timing.SceneSyncTimingWindow(enabled=False)
        window.record_seconds("pump", 1.0)
        window.increment("pump_calls")
        self.assertIsNone(window.pop())

        window.configure(True, max_samples=8)
        window.record_seconds("pump", 0.001)
        window.increment("pump_calls")
        window.configure(False)
        window.configure(True)

        snapshot = window.pop()
        self.assertEqual(snapshot.metrics, {})
        self.assertEqual(snapshot.counters, {})
        self.assertEqual(window.max_samples, 8)

    def test_format_is_stable_and_reports_rate_and_tail_percentiles(self) -> None:
        clock = FakeClock()
        window = timing.SceneSyncTimingWindow(enabled=True, clock=clock)
        for _ in range(10):
            window.record_seconds("apply_objects", 0.002)
            window.increment("accepted_frames")
            window.increment("send_enqueued_frames")
            window.increment("object_writes", 2)
        window.increment("apply_errors")
        clock.advance(2.0)

        lines = window.pop().format_lines()

        self.assertIn("accepted_hz=5.00", lines[0])
        self.assertIn("send_enqueued_hz=5.00", lines[0])
        self.assertIn("apply_errors=1", lines[0])
        self.assertIn("object_writes=20", lines[0])
        self.assertIn("apply_objects n=10", lines[1])
        self.assertIn("p95=2.000ms", lines[1])
        self.assertIn("p99=2.000ms", lines[1])


class SceneSyncProfilingWiringTests(unittest.TestCase):
    def test_term_profiles_all_required_phases(self) -> None:
        source = SYNC_PATH.read_text(encoding="utf-8")
        for phase in (
            "pump",
            "publish",
            "publish_snapshot",
            "publish_encode",
            "publish_send",
            "receive",
            "decode",
            "apply",
            "apply_robots",
            "apply_objects",
        ):
            self.assertRegex(
                source,
                rf'''record_seconds\(\s*["']{re.escape(phase)}["']''',
            )
        self.assertIn("def pop_profile_stats", source)
        self.assertIn("profile_max_samples: int = 4096", source)
        self.assertIn('increment("apply_errors")', source)
        self.assertIn("Failed to apply valid scene frame", source)

    def test_environment_and_cli_controls_are_wired(self) -> None:
        config_source = CONFIG_PATH.read_text(encoding="utf-8")
        sim_source = SIM_MAIN_PATH.read_text(encoding="utf-8")
        self.assertIn("ISAACLAB_SCENE_SYNC_PROFILE", config_source)
        self.assertIn("ISAACLAB_SCENE_SYNC_PROFILE_MAX_SAMPLES", config_source)
        self.assertIn('"--scene-sync-profile"', sim_source)
        self.assertIn('"--no-scene-sync-profile"', sim_source)
        self.assertIn("scene_sync_term.pop_profile_stats()", sim_source)
        self.assertIn("recent loop pacing", sim_source)
        self.assertIn("recent_loop_percentile_ms(0.99)", sim_source)

    def test_publish_decimation_log_uses_the_resolved_mainloop_default(self) -> None:
        config_source = CONFIG_PATH.read_text(encoding="utf-8")
        self.assertIn(
            "SCENE_SYNC_PUBLISH_DECIMATION = max(", config_source
        )
        self.assertIn('"ISAACLAB_SCENE_SYNC_PUBLISH_DECIMATION"', config_source)
        self.assertIn("1 if SCENE_SYNC_MAINLOOP else 4", config_source)
        self.assertIn(
            "publish_decimation=SCENE_SYNC_PUBLISH_DECIMATION", config_source
        )
        self.assertIn(
            'publish_clock = "控制主循环" if SCENE_SYNC_MAINLOOP else "物理步"',
            config_source,
        )
        self.assertIn(
            'f"发布节流=每{SCENE_SYNC_PUBLISH_DECIMATION}个{publish_clock}一帧"',
            config_source,
        )
        self.assertIn(
            'effective_connect = ("" if HOST_MODE else SCENE_SYNC_CONNECT_ENDPOINT)',
            config_source,
        )
        self.assertIn('publish_detail = "发布=关（只收）"', config_source)

    def test_publishing_enabled_requires_both_intent_and_a_live_socket(self) -> None:
        source = SYNC_PATH.read_text(encoding="utf-8")
        self.assertIn(
            "return bool(self._publish_enabled and self._pub_socket is not None)",
            source,
        )
        initialization_failure = source[source.index("except Exception as exc:") :]
        self.assertIn("self._pub_socket = None", initialization_failure)
        self.assertIn("self._publish_enabled = False", initialization_failure)


if __name__ == "__main__":
    unittest.main()
