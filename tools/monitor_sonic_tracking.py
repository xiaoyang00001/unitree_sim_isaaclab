#!/usr/bin/env python3
"""Measure SONIC reference-motion tracking from the C++ ZMQ debug stream.

The C++ deploy process publishes both ``body_q_target`` (the active reference
motion) and ``body_q_measured`` (the joint state returned through Isaac/PhysX)
on the deploy process's configured debug topic.  This tool compares those two
signals.  It is deliberately separate from the Isaac action-provider metrics: the latter
measure displacement from the policy's virtual PD equilibrium and must not be
interpreted as reference-motion tracking error.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field

import msgpack
import numpy as np
import zmq


JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

JOINT_GROUPS = {
    "left_leg": slice(0, 6),
    "right_leg": slice(6, 12),
    "waist": slice(12, 15),
    "left_arm": slice(15, 22),
    "right_arm": slice(22, 29),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure 29-DoF reference tracking from SONIC g1_debug ZMQ output."
    )
    parser.add_argument("--host", default="localhost", help="C++ ZMQ publisher host")
    parser.add_argument("--port", type=int, default=5557, help="C++ ZMQ publisher port")
    parser.add_argument(
        "--topic",
        default="g1_1_debug",
        help="ZMQ topic prefix; use the exact ZMQ Output topic printed by deploy.sh",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="measurement duration after warmup in seconds; 0 runs until Ctrl-C",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0,
        help="discard this many seconds after the first valid message",
    )
    parser.add_argument(
        "--report-interval",
        type=float,
        default=5.0,
        help="periodic report interval in seconds; 0 disables periodic reports",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="fail if no valid state message arrives for this many seconds",
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=5,
        help="number of highest-error joints shown in reports",
    )
    parser.add_argument(
        "--max-lag-frames",
        type=int,
        default=10,
        help="search measured-vs-reference delay from 0 through this many frames",
    )
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if args.duration < 0.0:
        parser.error("--duration must be non-negative")
    if args.warmup < 0.0:
        parser.error("--warmup must be non-negative")
    if args.report_interval < 0.0:
        parser.error("--report-interval must be non-negative")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if args.top_k <= 0:
        parser.error("--top-k must be positive")
    if args.max_lag_frames < 0:
        parser.error("--max-lag-frames must be non-negative")
    if not args.topic:
        parser.error("--topic must not be empty")
    return args


def as_finite_vector(message: dict, key: str, size: int) -> np.ndarray:
    value = np.asarray(message.get(key), dtype=np.float64)
    if value.shape != (size,):
        raise ValueError(f"{key} has shape {value.shape}; expected ({size},)")
    if not np.isfinite(value).all():
        raise ValueError(f"{key} contains NaN or Inf")
    return value


def quaternion_error_deg(measured: np.ndarray, target: np.ndarray) -> float:
    measured_norm = float(np.linalg.norm(measured))
    target_norm = float(np.linalg.norm(target))
    if measured_norm < 1.0e-9 or target_norm < 1.0e-9:
        return math.nan
    dot = abs(float(np.dot(measured / measured_norm, target / target_norm)))
    return math.degrees(2.0 * math.acos(float(np.clip(dot, 0.0, 1.0))))


@dataclass
class SampleWindow:
    timestamps: list[float] = field(default_factory=list)
    indices: list[int] = field(default_factory=list)
    targets: list[np.ndarray] = field(default_factory=list)
    measured: list[np.ndarray] = field(default_factory=list)
    base_orientation_errors_deg: list[float] = field(default_factory=list)

    def append(
        self,
        timestamp: float,
        index: int,
        target: np.ndarray,
        measured: np.ndarray,
        base_orientation_error_deg: float,
    ) -> None:
        self.timestamps.append(timestamp)
        self.indices.append(index)
        self.targets.append(target.copy())
        self.measured.append(measured.copy())
        if math.isfinite(base_orientation_error_deg):
            self.base_orientation_errors_deg.append(base_orientation_error_deg)

    def clear(self) -> None:
        self.timestamps.clear()
        self.indices.clear()
        self.targets.clear()
        self.measured.clear()
        self.base_orientation_errors_deg.clear()

    def __len__(self) -> int:
        return len(self.targets)


def format_summary(window: SampleWindow, top_k: int, label: str) -> str:
    if len(window) == 0:
        return f"[{label}] no samples"

    targets = np.stack(window.targets)
    measured = np.stack(window.measured)
    errors = measured - targets
    abs_errors = np.abs(errors)
    per_joint_mae = abs_errors.mean(axis=0)
    per_joint_p95 = np.percentile(abs_errors, 95.0, axis=0)
    per_joint_max = abs_errors.max(axis=0)

    elapsed = max(window.timestamps[-1] - window.timestamps[0], 0.0)
    receive_hz = (len(window) - 1) / elapsed if elapsed > 0.0 and len(window) > 1 else 0.0
    index_span = window.indices[-1] - window.indices[0] if len(window.indices) > 1 else 0
    index_missing = max(0, index_span - (len(window.indices) - 1))

    if len(window) > 1:
        dt = np.diff(np.asarray(window.timestamps))
        target_step = np.diff(targets, axis=0)
        valid_dt = dt > 1.0e-6
        if valid_dt.any():
            target_speed = target_step[valid_dt] / dt[valid_dt, None]
            target_speed_mean = float(np.abs(target_speed).mean())
            target_speed_max = float(np.abs(target_speed).max())
        else:
            target_speed_mean = 0.0
            target_speed_max = 0.0
    else:
        target_speed_mean = 0.0
        target_speed_max = 0.0

    target_range = np.ptp(targets, axis=0)
    top_indices = np.argsort(per_joint_mae)[::-1][: min(top_k, len(JOINT_NAMES))]
    top_text = ", ".join(
        f"{JOINT_NAMES[index]}={per_joint_mae[index]:.4f}/"
        f"{per_joint_p95[index]:.4f}/{per_joint_max[index]:.4f}"
        for index in top_indices
    )
    group_text = ", ".join(
        f"{name}={abs_errors[:, group_slice].mean():.4f}"
        for name, group_slice in JOINT_GROUPS.items()
    )

    orientation_text = "n/a"
    if window.base_orientation_errors_deg:
        orientation = np.asarray(window.base_orientation_errors_deg)
        orientation_text = f"{orientation.mean():.2f}/{orientation.max():.2f}deg(mean/max)"

    return (
        f"[{label}] samples={len(window)}, elapsed={elapsed:.2f}s, receive={receive_hz:.2f}Hz, "
        f"missing_index_steps={index_missing}\n"
        f"  reference_joint_mae={abs_errors.mean():.4f}rad, "
        f"rmse={np.sqrt(np.mean(errors * errors)):.4f}rad, "
        f"p95={np.percentile(abs_errors, 95.0):.4f}rad, max={abs_errors.max():.4f}rad\n"
        f"  group_mae(rad): {group_text}\n"
        f"  target_activity: mean_abs_dq={target_speed_mean:.3f}rad/s, "
        f"max_abs_dq={target_speed_max:.3f}rad/s, max_joint_range={target_range.max():.3f}rad\n"
        f"  base_orientation_error={orientation_text}\n"
        f"  top_joint_mae/p95/max(rad): {top_text}"
    )


def lag_report(window: SampleWindow, max_lag_frames: int) -> str:
    if len(window) < 3 or max_lag_frames <= 0:
        return "[lag] insufficient samples or lag search disabled"

    targets = np.stack(window.targets)
    measured = np.stack(window.measured)
    max_lag = min(max_lag_frames, len(window) - 2)
    lag_mae = []
    for lag in range(max_lag + 1):
        if lag == 0:
            error = measured - targets
        else:
            # Positive lag means the measured robot state is compared with an
            # earlier reference frame, i.e. the physical response trails the
            # reference by ``lag`` control frames.
            error = measured[lag:] - targets[:-lag]
        lag_mae.append(float(np.abs(error).mean()))

    minimum_mae = min(lag_mae)
    # Prefer the smallest lag when values are numerically indistinguishable;
    # a static reference otherwise has no meaningful unique delay.
    best_lag = next(
        index for index, value in enumerate(lag_mae) if value <= minimum_mae + 1.0e-12
    )
    if len(window.timestamps) > 1:
        median_dt = float(np.median(np.diff(np.asarray(window.timestamps))))
    else:
        median_dt = 0.0
    lag_values = ", ".join(f"{index}:{value:.4f}" for index, value in enumerate(lag_mae))
    return (
        f"[lag] best={best_lag} frames (~{best_lag * median_dt * 1000.0:.1f}ms), "
        f"mae={lag_mae[best_lag]:.4f}rad; frame:mae={lag_values}"
    )


def decode_message(raw: bytes, topic: bytes) -> dict:
    if not raw.startswith(topic):
        raise ValueError("received message does not start with the configured topic")
    message = msgpack.unpackb(raw[len(topic) :], raw=False)
    if not isinstance(message, dict):
        raise ValueError(f"decoded payload is {type(message).__name__}, expected dict")
    return message


def main() -> int:
    args = parse_args()
    endpoint = f"tcp://{args.host}:{args.port}"
    topic = args.topic.encode("utf-8")

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, topic)
    socket.setsockopt(zmq.RCVHWM, 100)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(endpoint)
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    print(f"[monitor] connected to {endpoint}, topic={args.topic!r}")
    print(
        "[monitor] waiting for body_q_target/body_q_measured; "
        "start CONTROL and play a reference motion for dynamic validation"
    )

    all_samples = SampleWindow()
    report_samples = SampleWindow()
    first_valid_time: float | None = None
    measurement_start: float | None = None
    last_valid_time = time.monotonic()
    last_report_time: float | None = None
    bad_message_count = 0

    try:
        while True:
            now = time.monotonic()
            if first_valid_time is None and now - last_valid_time > args.timeout:
                raise TimeoutError(f"no valid {args.topic!r} message received within {args.timeout:.1f}s")
            if first_valid_time is not None and now - last_valid_time > args.timeout:
                raise TimeoutError(f"state stream has been silent for {args.timeout:.1f}s")

            if measurement_start is not None and args.duration > 0.0:
                if now - measurement_start >= args.duration:
                    break

            events = dict(poller.poll(timeout=100))
            if socket not in events:
                continue

            raw = socket.recv()
            now = time.monotonic()
            try:
                message = decode_message(raw, topic)
                target = as_finite_vector(message, "body_q_target", 29)
                measured = as_finite_vector(message, "body_q_measured", 29)
                base_target = as_finite_vector(message, "base_quat_target", 4)
                base_measured = as_finite_vector(message, "base_quat_measured", 4)
                message_index = int(message.get("index", -1))
            except (KeyError, TypeError, ValueError, msgpack.exceptions.UnpackException) as exc:
                bad_message_count += 1
                if bad_message_count <= 5:
                    print(f"[monitor] ignored invalid message: {exc}", file=sys.stderr)
                continue

            last_valid_time = now
            if first_valid_time is None:
                first_valid_time = now
                measurement_start = first_valid_time + args.warmup
                last_report_time = measurement_start
                if args.warmup > 0.0:
                    print(f"[monitor] first valid message received; warming up for {args.warmup:.1f}s")

            assert measurement_start is not None
            if now < measurement_start:
                continue

            orientation_error = quaternion_error_deg(base_measured, base_target)
            all_samples.append(now, message_index, target, measured, orientation_error)
            report_samples.append(now, message_index, target, measured, orientation_error)

            if (
                args.report_interval > 0.0
                and last_report_time is not None
                and now - last_report_time >= args.report_interval
            ):
                print(format_summary(report_samples, args.top_k, "window"), flush=True)
                report_samples.clear()
                last_report_time = now
    except KeyboardInterrupt:
        print("\n[monitor] interrupted; printing collected summary")
    except TimeoutError as exc:
        print(f"[monitor] ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        poller.unregister(socket)
        socket.close()
        context.term()

    if len(all_samples) == 0:
        print("[monitor] ERROR: no samples collected after warmup", file=sys.stderr)
        return 3

    print(format_summary(all_samples, args.top_k, "total"))
    print(lag_report(all_samples, args.max_lag_frames))
    if bad_message_count:
        print(f"[monitor] invalid_messages={bad_message_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
