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
# 当前 Δ=0.25：**X 支线越过货架排 B 北侧所需的最小北移**（取 50 mm 整倍数）。
# 西拐方案要把出生/回生点真正藏到货架（和叉车）后面，支线必须向西延长越过
# 排 B（护板东缘 x=-13.386 曾是三段硬限）；整条线（主线+弯道+支线+工位）北移
# 后，支线机身南缘 19.4779 从排 B 端护板北缘 19.3946 上方擦过（净距 83.3 mm，
# 最小可行 Δ=0.2167 ⇒ 取 0.25），支线得以延到 5 段、把回生点压到 (-16.66,
# 20.0534)。约束绑定项是端护板；叉车货叉尖 19.321（净距 156.9 mm）不绑定。
# ⚠️ 再加大 Δ 只会加深遮挡阴影需求（眼位同移），勿上调。
# 历史：安检机时代的 Δ=3.55 已回退（挡边仅高出带面 29mm，箱子露顶，"藏进
# 机柜"不成立）；本轮 Δ=0.25 与那次的目的、量级都不同。
#
# ⚠️ 出生点/回收线/判据/布局经 _shifted 管线自动跟随本常量。几何位移写在背景
#    clean wrapper warehouse-simple6_v61_visual_only.usda 的 over "ConveyorBelt"
#    组变换 + 15 件带面装饰（世界 y +Δ ≡ 背景局部 x +Δ，因为背景挂载时绕 Z
#    转了 +90°）；地面标识已全部失活，不再需要位移 override。改本常量必须同步
#    重写 wrapper 的 override 并重钉 conveyor_workcell_lite.manifest.json 的 source sha。
# ⚠️ legacy_v61 裸 v61 回退档不含位移（Δ=0 几何），Δ≠0 期间它与代码常量错位
#    0.25 m，只可作视觉参考、不可作功能 A/B 基线。
# ⚠️ scene_layout.py 为保持零相对 import 抄了一份 Δ（抄本共两处），
#    tests/test_conveyor_scene_layout.py 交叉断言两处一致，别只改一边。
#    endless_intake.py 刻意**不抄 Δ**（绝对坐标 +Δ 重钉），关系锁在
#    tests/test_conveyor_endless_intake.py 的交叉断言里。
CONVEYOR_NORTH_SHIFT_Y = 0.25

# 三段 ConveyorBelt_A08 原始可用滚轮面宽 0.90 m。本任务把三段视觉沿横向等比
# 收窄到 0.60 m：生产默认最宽的 parcel_a02 为 0.4526 m，居中后两侧各留
# 73.7 mm；0.50 m 的 C01/C02 只供显式 PATTERN 回退。同时缩短双机手到箱心的
# 无效跨距。BELT_Y_*_BASE 是北移前的实测值，运行值 = 基准 + Δ。
BELT_X_CENTER = -5.62
BELT_SOURCE_WIDTH = 0.90
BELT_WIDTH = 0.60
BELT_VISUAL_WIDTH_SCALE = BELT_WIDTH / BELT_SOURCE_WIDTH
BELT_Y_MIN_BASE = 10.19
BELT_Y_MAX_BASE = 18.22
BELT_Y_MIN = round(BELT_Y_MIN_BASE + CONVEYOR_NORTH_SHIFT_Y, 6)  # 10.44
BELT_Y_MAX = round(BELT_Y_MAX_BASE + CONVEYOR_NORTH_SHIFT_Y, 6)  # 18.47
BELT_TOP_Z = 0.772
BELT_COLLIDER_THICKNESS = 0.04

# 循环模式（y_stop<=0）的回收线与回生落点，同样随北移整体平移。
DEFAULT_Y_RECYCLE = round(10.6 + CONVEYOR_NORTH_SHIFT_Y, 6)  # 10.85
DEFAULT_Y_RESPAWN = round(18.0 + CONVEYOR_NORTH_SHIFT_Y, 6)  # 18.25


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
