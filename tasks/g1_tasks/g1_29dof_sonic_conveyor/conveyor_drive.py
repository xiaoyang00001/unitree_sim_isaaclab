# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Dependency-free conveyor drive-mode and collision-segment resolver.

This module deliberately has no Isaac Sim imports.  The environment config and
ordinary Python unit tests therefore share one source of truth for the
``legacy`` / ``surface_velocity`` A/B switch, authority gating, and the split
between the moving feed surface and the static grasp/stop surface.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


DRIVE_MODE_LEGACY = "legacy"
DRIVE_MODE_SURFACE_VELOCITY = "surface_velocity"
DRIVE_MODES = (DRIVE_MODE_LEGACY, DRIVE_MODE_SURFACE_VELOCITY)

# Measured usable top surface of the three ConveyorBelt_A08 visual sections.
BELT_X_CENTER = -5.62
BELT_WIDTH = 0.90
BELT_Y_MIN = 10.19
BELT_Y_MAX = 18.22
BELT_TOP_Z = 0.772
BELT_COLLIDER_THICKNESS = 0.04


def _env_bool(environ: Mapping[str, str], name: str, default: bool) -> bool:
    value = environ.get(name)
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _env_float(environ: Mapping[str, str], name: str, default: float) -> float:
    value = environ.get(name)
    if value is None or not str(value).strip():
        return default
    try:
        return float(str(value).strip())
    except ValueError:
        return default


def _require_finite(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数")
    return value


@dataclass(frozen=True)
class ConveyorCollisionSegment:
    """One contiguous Y interval of the invisible conveyor collider."""

    y_min: float
    y_max: float

    @property
    def length(self) -> float:
        return self.y_max - self.y_min

    @property
    def center_y(self) -> float:
        return (self.y_min + self.y_max) * 0.5


@dataclass(frozen=True)
class ConveyorDriveConfig:
    """Fully resolved conveyor backend, authority, and segment geometry."""

    mode: str
    requested_enabled: bool
    speed: float
    velocity_y: float
    y_stop: float | None
    y_recycle: float
    y_respawn: float
    handoff_offset: float
    legacy_enabled: bool
    surface_velocity_enabled: bool
    surface_recycle_enabled: bool
    drive_segment: ConveyorCollisionSegment
    stop_segment: ConveyorCollisionSegment | None


def resolve_conveyor_drive(
    environ: Mapping[str, str],
    *,
    object_authority: bool,
    mirror_objects: bool,
    default_y_stop: float,
    default_handoff_offset: float = 0.10,
) -> ConveyorDriveConfig:
    """Resolve the conveyor A/B backend and its two collision regions.

    ``legacy`` preserves the historical per-tote root-velocity overwrite.  It
    remains available on a standalone ID=2 process when scene sync is disabled,
    matching the old single-process diagnostic behavior.

    ``surface_velocity`` is stricter: it can only activate on the fixed object
    authority (ID=1), even when scene sync is disabled.  This prevents a second
    process from independently applying PhysX contact drive to mirrored totes.

    For a stopped workflow, the moving surface ends ``handoff_offset`` upstream
    of the desired tote-center stop coordinate.  The default 0.10 m is the
    half-length of the half-scale 60x40 tote along the belt; callers use 0.20 m
    for the full-size pushcart layout.  When the trailing edge leaves the moving
    segment, its center is approximately at ``y_stop``.  Everything downstream
    is a static high-friction stop/grasp segment.
    """

    mode = str(environ.get("ISAACLAB_CONVEYOR_DRIVE_MODE", DRIVE_MODE_LEGACY)).strip().lower()
    if mode not in DRIVE_MODES:
        valid = "|".join(DRIVE_MODES)
        raise ValueError(
            f"ISAACLAB_CONVEYOR_DRIVE_MODE={mode!r} 无效；只支持 {valid}"
        )

    requested_enabled = _env_bool(environ, "ISAACLAB_CONVEYOR_ENABLED", True)
    speed = _require_finite(
        _env_float(environ, "ISAACLAB_CONVEYOR_SPEED", 0.3),
        "ISAACLAB_CONVEYOR_SPEED",
    )
    if speed < 0.0:
        raise ValueError("ISAACLAB_CONVEYOR_SPEED 必须是非负速率；方向固定为 -Y")

    raw_y_stop = _require_finite(
        _env_float(environ, "ISAACLAB_CONVEYOR_Y_STOP", default_y_stop),
        "ISAACLAB_CONVEYOR_Y_STOP",
    )
    y_stop = raw_y_stop if raw_y_stop > 0.0 else None
    handoff_offset = _require_finite(
        _env_float(
            environ,
            "ISAACLAB_CONVEYOR_SURFACE_HANDOFF_OFFSET",
            default_handoff_offset,
        ),
        "ISAACLAB_CONVEYOR_SURFACE_HANDOFF_OFFSET",
    )
    if handoff_offset < 0.0:
        raise ValueError("ISAACLAB_CONVEYOR_SURFACE_HANDOFF_OFFSET 不能为负数")

    if y_stop is None:
        drive_segment = ConveyorCollisionSegment(BELT_Y_MIN, BELT_Y_MAX)
        stop_segment = None
    else:
        split_y = y_stop + handoff_offset
        if not (BELT_Y_MIN < split_y < BELT_Y_MAX):
            raise ValueError(
                "流水线停止分区超出可用带面：要求 "
                f"{BELT_Y_MIN:.3f} < y_stop + handoff_offset < {BELT_Y_MAX:.3f}，"
                f"当前为 {split_y:.3f}"
            )
        drive_segment = ConveyorCollisionSegment(split_y, BELT_Y_MAX)
        stop_segment = ConveyorCollisionSegment(BELT_Y_MIN, split_y)

    legacy_enabled = (
        requested_enabled and mode == DRIVE_MODE_LEGACY and not mirror_objects
    )
    surface_velocity_enabled = (
        requested_enabled
        and mode == DRIVE_MODE_SURFACE_VELOCITY
        and object_authority
    )

    y_recycle = _require_finite(
        _env_float(environ, "ISAACLAB_CONVEYOR_Y_RECYCLE", 10.6),
        "ISAACLAB_CONVEYOR_Y_RECYCLE",
    )
    y_respawn = _require_finite(
        _env_float(environ, "ISAACLAB_CONVEYOR_Y_RESPAWN", 18.0),
        "ISAACLAB_CONVEYOR_Y_RESPAWN",
    )

    return ConveyorDriveConfig(
        mode=mode,
        requested_enabled=requested_enabled,
        speed=speed,
        velocity_y=-speed,
        y_stop=y_stop,
        y_recycle=y_recycle,
        y_respawn=y_respawn,
        handoff_offset=handoff_offset,
        legacy_enabled=legacy_enabled,
        surface_velocity_enabled=surface_velocity_enabled,
        surface_recycle_enabled=surface_velocity_enabled and y_stop is None,
        drive_segment=drive_segment,
        stop_segment=stop_segment,
    )
