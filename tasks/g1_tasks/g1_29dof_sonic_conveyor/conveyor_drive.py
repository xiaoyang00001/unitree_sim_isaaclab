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

# ------------------------------------------------------------------
# 流水线整体北移量（世界 +Y，单位 m）——**本工程唯一真源**。
#
# 由来：安检机改为靠 +Y 落地墙 SM_WallA_6M16（离线实测南面 y=23.6055，取
# 23.606）摆放，机身放大到 SHROUD_INFEED_DESIGN_SCALE=1.40 后半长 1.783936，留
# 0.05 m 墙缝 ⇒ 机身远端面 23.556、机身中心 y_c=21.772064。让流水线入料端
# （离线实测带体到 y=18.2223）正好推进到机身中心：
#     Δ = 21.772064 - 18.2223 = 3.54966  →  取 3.55
# 此时带端越过近端条帘 0.4147 m、离远端条帘还差 0.4142 m，端头断面正好藏在
# 不透明机柜 y[21.2816, 22.2626] 里，从南北两侧都不穿帮。
#
# ⚠️ Δ 有硬上限约 3.58：流水线每 2 m 一组的支腿站外偏 0.576 m 大于隧道洞口
#    半宽，再往北就会扎穿机柜侧板。改 Δ 前先看 shroud_config 的贯穿断言。
# ⚠️ 几何位移写在背景 clean wrapper warehouse-simple6_v61_visual_only.usda 的
#    over "ConveyorBelt"（世界 y +Δ ≡ 背景局部 x +Δ，因为背景挂载时绕 Z 转了
#    +90°），四种背景模式经 reference / subLayer 自动继承。唯一例外是
#    ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61 的裸 v61，它不含位移，带体会与本
#    常量差 Δ 米——那条历史回退路径已在 README 里显式声明失效。
# ⚠️ shroud_config.py / scene_layout.py 为保持零相对 import 各抄了一份 Δ，
#    tests/test_conveyor_drive.py 与 test_conveyor_shroud_config.py 交叉断言三处
#    一致，别只改一边。
CONVEYOR_NORTH_SHIFT_Y = 3.55

# Measured usable top surface of the three ConveyorBelt_A08 visual sections.
# BELT_Y_*_BASE 是北移前的实测值，运行值 = 基准 + Δ。
BELT_X_CENTER = -5.62
BELT_WIDTH = 0.90
BELT_Y_MIN_BASE = 10.19
BELT_Y_MAX_BASE = 18.22
BELT_Y_MIN = round(BELT_Y_MIN_BASE + CONVEYOR_NORTH_SHIFT_Y, 6)  # 13.74
BELT_Y_MAX = round(BELT_Y_MAX_BASE + CONVEYOR_NORTH_SHIFT_Y, 6)  # 21.77
BELT_TOP_Z = 0.772
BELT_COLLIDER_THICKNESS = 0.04

# 循环模式（y_stop<=0）的回收线与回生落点，同样随北移整体平移。
DEFAULT_Y_RECYCLE = round(10.6 + CONVEYOR_NORTH_SHIFT_Y, 6)  # 14.15
DEFAULT_Y_RESPAWN = round(18.0 + CONVEYOR_NORTH_SHIFT_Y, 6)  # 21.55


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
        _env_float(environ, "ISAACLAB_CONVEYOR_Y_RECYCLE", DEFAULT_Y_RECYCLE),
        "ISAACLAB_CONVEYOR_Y_RECYCLE",
    )
    y_respawn = _require_finite(
        _env_float(environ, "ISAACLAB_CONVEYOR_Y_RESPAWN", DEFAULT_Y_RESPAWN),
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
