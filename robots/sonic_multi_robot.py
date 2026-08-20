# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Dependency-free naming contract for multi-robot SONIC scenes.

The conveyor host runs several independent Gear SONIC deploy processes in one
Isaac process.  Every process needs a unique scene asset, DDS namespace and
shared-memory suffix.  Keeping those names in one small module prevents the
scene, DDS factory, action provider and reset path from silently drifting apart.
"""

from __future__ import annotations

from dataclasses import dataclass


SONIC_ROBOT_COUNT_MIN = 2
SONIC_ROBOT_COUNT_MAX = 5
SONIC_ROBOT_COUNT_DEFAULT = 2


@dataclass(frozen=True)
class SonicRobotChannelSpec:
    """Names owned by one SONIC-controlled articulation."""

    robot_id: int
    asset_name: str
    prim_name: str
    global_name: str
    robot_dds_name: str
    dex3_dds_name: str
    foot_contact_name: str
    topic_prefix: str
    shm_suffix: str
    log_suffix: str

    @property
    def action_term_suffix(self) -> str:
        return "" if self.robot_id == 1 else f"_{self.robot_id}"


def sonic_robot_channel_spec(robot_id: int) -> SonicRobotChannelSpec:
    """Return the canonical names for robot ``robot_id`` (1 through 5)."""

    robot_id = int(robot_id)
    if not 1 <= robot_id <= SONIC_ROBOT_COUNT_MAX:
        raise ValueError(
            f"SONIC robot id must be in [1, {SONIC_ROBOT_COUNT_MAX}], got {robot_id}"
        )
    if robot_id == 1:
        return SonicRobotChannelSpec(
            robot_id=1,
            asset_name="robot",
            prim_name="Robot",
            global_name="robot_1",
            robot_dds_name="g129",
            dex3_dds_name="dex3",
            foot_contact_name="foot_contact",
            topic_prefix="rt",
            shm_suffix="",
            log_suffix="",
        )
    suffix = f"_r{robot_id}"
    return SonicRobotChannelSpec(
        robot_id=robot_id,
        asset_name=f"robot_{robot_id}",
        prim_name=f"Robot{robot_id}",
        global_name=f"robot_{robot_id}",
        robot_dds_name=f"g129_r{robot_id}",
        dex3_dds_name=f"dex3_r{robot_id}",
        foot_contact_name=f"foot_contact_{robot_id}",
        topic_prefix=f"rt/r{robot_id}",
        shm_suffix=suffix,
        log_suffix=f":r{robot_id}",
    )


def sonic_robot_channel_specs(robot_count: int) -> tuple[SonicRobotChannelSpec, ...]:
    """Return ordered robot-major channel specs for ``robot_count`` robots."""

    robot_count = int(robot_count)
    if not 1 <= robot_count <= SONIC_ROBOT_COUNT_MAX:
        raise ValueError(
            f"SONIC robot count must be in [1, {SONIC_ROBOT_COUNT_MAX}], got {robot_count}"
        )
    return tuple(sonic_robot_channel_spec(robot_id) for robot_id in range(1, robot_count + 1))


def sonic_host_action_terms(robot_count: int) -> tuple[str, ...]:
    """Return q/dq/tau ActionTerm names in provider concatenation order."""

    terms: list[str] = []
    for spec in sonic_robot_channel_specs(robot_count):
        suffix = spec.action_term_suffix
        terms.extend(
            (
                f"joint_pos{suffix}",
                f"joint_vel{suffix}",
                f"joint_effort{suffix}",
            )
        )
    return tuple(terms)
