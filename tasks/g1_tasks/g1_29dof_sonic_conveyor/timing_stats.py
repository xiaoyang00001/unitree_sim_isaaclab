"""Low-overhead rolling timing statistics for conveyor scene synchronization.

This module is deliberately dependency-free so percentile behavior can be
tested without importing Isaac Lab or starting Kit.  Samples are retained in
bounded deques; ``total_count``, mean and max still cover the complete window
when a long reporting interval exceeds the percentile retention limit.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
import math
import time
from typing import Callable


PROFILE_PHASES = (
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
)


def _percentile(sorted_values: list[float], percentile: float) -> float:
    """Return a linearly interpolated percentile from sorted values."""

    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    position = (len(sorted_values) - 1) * min(1.0, max(0.0, percentile))
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(sorted_values[lower])
    fraction = position - lower
    return float(
        sorted_values[lower]
        + (sorted_values[upper] - sorted_values[lower]) * fraction
    )


@dataclass(frozen=True)
class TimingMetric:
    """One phase summary in ms; percentiles use the retained sample suffix."""

    total_count: int
    retained_count: int
    mean_ms: float
    p50_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float


@dataclass(frozen=True)
class TimingSnapshot:
    """A completed reporting window."""

    window_s: float
    metrics: dict[str, TimingMetric]
    counters: dict[str, int]

    def format_lines(self, prefix: str = "[Scene Sync Profile]") -> list[str]:
        """Format stable, grep-friendly lines for runtime logs."""

        accepted = self.counters.get("accepted_frames", 0)
        send_enqueued = self.counters.get("send_enqueued_frames", 0)
        accepted_hz = accepted / self.window_s if self.window_s > 0.0 else 0.0
        send_enqueued_hz = (
            send_enqueued / self.window_s if self.window_s > 0.0 else 0.0
        )
        header_keys = (
            "pump_calls",
            "publish_attempts",
            "send_enqueued_frames",
            "receive_polls",
            "accepted_frames",
            "empty_polls",
            "rejected_frames",
            "invalid_frames",
            "apply_errors",
            "drained_messages",
            "coalesced_messages",
            "robot_writes",
            "object_writes",
        )
        counter_text = " ".join(
            f"{key}={self.counters.get(key, 0)}" for key in header_keys
        )
        lines = [
            f"{prefix} window={self.window_s:.3f}s "
            f"send_enqueued_hz={send_enqueued_hz:.2f} "
            f"accepted_hz={accepted_hz:.2f} "
            f"{counter_text}"
        ]
        for phase in PROFILE_PHASES:
            metric = self.metrics.get(phase)
            if metric is None or metric.total_count <= 0:
                continue
            count_text = str(metric.total_count)
            if metric.retained_count != metric.total_count:
                count_text = f"{metric.retained_count}/{metric.total_count}"
            lines.append(
                f"{prefix} {phase} n={count_text} mean={metric.mean_ms:.3f}ms "
                f"p50={metric.p50_ms:.3f}ms p95={metric.p95_ms:.3f}ms "
                f"p99={metric.p99_ms:.3f}ms max={metric.max_ms:.3f}ms"
            )
        return lines


class SceneSyncTimingWindow:
    """Bounded rolling phase timings and counters for one sync term."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        max_samples: int = 4096,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        self._clock = clock
        self._enabled = bool(enabled)
        self._max_samples = max(1, int(max_samples))
        self._samples: dict[str, deque[float]] = {}
        self._total_counts: Counter[str] = Counter()
        self._total_ms: Counter[str] = Counter()
        self._max_ms: dict[str, float] = {}
        self._counters: Counter[str] = Counter()
        self._window_start = self._clock()

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def max_samples(self) -> int:
        return self._max_samples

    def configure(self, enabled: bool, max_samples: int | None = None) -> None:
        """Change profiling state and start a clean reporting window."""

        if max_samples is not None:
            self._max_samples = max(1, int(max_samples))
        self._enabled = bool(enabled)
        self.reset()

    def reset(self) -> None:
        self._samples.clear()
        self._total_counts.clear()
        self._total_ms.clear()
        self._max_ms.clear()
        self._counters.clear()
        self._window_start = self._clock()

    def record_seconds(self, phase: str, elapsed_s: float) -> None:
        if not self._enabled:
            return
        samples = self._samples.get(phase)
        if samples is None:
            samples = deque(maxlen=self._max_samples)
            self._samples[phase] = samples
        value_ms = max(0.0, float(elapsed_s)) * 1000.0
        samples.append(value_ms)
        self._total_counts[phase] += 1
        self._total_ms[phase] += value_ms
        self._max_ms[phase] = max(value_ms, self._max_ms.get(phase, 0.0))

    def increment(self, counter: str, amount: int = 1) -> None:
        if self._enabled:
            self._counters[counter] += int(amount)

    def pop(self) -> TimingSnapshot | None:
        """Return and clear the current window, or ``None`` when disabled."""

        if not self._enabled:
            return None
        now = self._clock()
        metrics: dict[str, TimingMetric] = {}
        for phase, raw_values in self._samples.items():
            values = sorted(raw_values)
            if not values:
                continue
            metrics[phase] = TimingMetric(
                total_count=self._total_counts[phase],
                retained_count=len(values),
                mean_ms=self._total_ms[phase] / self._total_counts[phase],
                p50_ms=_percentile(values, 0.50),
                p95_ms=_percentile(values, 0.95),
                p99_ms=_percentile(values, 0.99),
                max_ms=self._max_ms[phase],
            )
        snapshot = TimingSnapshot(
            window_s=max(0.0, now - self._window_start),
            metrics=metrics,
            counters=dict(self._counters),
        )
        self._samples.clear()
        self._total_counts.clear()
        self._total_ms.clear()
        self._max_ms.clear()
        self._counters.clear()
        self._window_start = now
        return snapshot
