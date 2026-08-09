"""Dependency-free resolver for the two conveyor scene layouts.

Keeping the layout arithmetic outside ``conveyor_env_cfg`` lets ordinary Python
tests verify the environment-variable switch without importing Isaac Lab.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


def _env_float(environ: Mapping[str, str], name: str, default: float) -> float:
    value = environ.get(name)
    if value is None or not str(value).strip():
        return default
    try:
        return float(str(value).strip())
    except ValueError:
        return default


def _env_bool(environ: Mapping[str, str], name: str, default: bool) -> bool:
    value = environ.get(name)
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _env_int(environ: Mapping[str, str], name: str, default: int) -> int:
    value = environ.get(name)
    if value is None or not str(value).strip():
        return default
    try:
        return int(str(value).strip())
    except ValueError:
        return default


# 流水线纸箱（SM_CardBoxD_01，与 v61 背景 ``ConveyorBelt_Box_XX`` 同一视觉资产）
# 缩放后的世界尺寸。箱子绕 Z 转 90° 摆放，长边沿输送方向 Y。
BELT_BOX_LENGTH_Y = 0.38
BELT_BOX_WIDTH_X = 0.25
BELT_BOX_HEIGHT_Z = 0.1487
# 带面几何与 conveyor_drive 的常量保持一致（那边是驱动/分段的真源，这里只用来
# 定位出生点；两处数值若要改必须同时改）。
BELT_BOX_LANE_X = -5.62
BELT_BOX_SPAWN_Z = 0.775
BELT_BOX_BELT_Y_MAX = 18.22
# SceneCfg 里的 belt_box_N 字段是显式声明的（configclass 需要类属性），所以数量
# 有硬上限。调大必须同时在 conveyor_env_cfg.G129SonicConveyorSceneCfg 里补字段。
BELT_BOX_MAX_COUNT = 5


@dataclass(frozen=True)
class ConveyorSceneLayout:
    """Resolved positions and scale shared by all conveyor scene assets."""

    totes_on_conveyor: bool
    cart_group_x: float
    cart_group_y: float
    robot_side_offset: float
    robot_workstation_y: float
    robot_1_x: float
    robot_2_x: float
    pushcart_2_pos: tuple[float, float, float]
    cart2_tote1_pos: tuple[float, float, float]
    cart2_tote2_pos: tuple[float, float, float]
    tote_scale: tuple[float, float, float]
    conveyor_y_stop: float
    belt_box_count: int
    belt_box_positions: tuple[tuple[float, float, float], ...]
    belt_box_queue_pitch: float

    @property
    def belt_box_names(self) -> tuple[str, ...]:
        return tuple(f"belt_box_{index + 1}" for index in range(self.belt_box_count))


def resolve_belt_box_positions(
    environ: Mapping[str, str],
    *,
    y_stop: float,
) -> tuple[int, tuple[tuple[float, float, float], ...], float]:
    """把 N 个纸箱排在工位**上游**那一段带面上（"流水线前段"）。

    机器人工位 ``y_stop`` 把带面切成两段：上游 (y_stop, 18.22] 是来料段，下游
    [10.19, y_stop) 是出料段。箱子只出生在上游，这样一次整场景复位 = 重新完整
    流一遍（与两塑料筐时代的落点约定一致，见 conveyor_env_cfg 的出生 y 注释）。

    ``belt_box_1`` 是队首（y 最小、最先到工位），编号递增向上游排。出生间距
    ``spawn_pitch`` 只决定初始队形；停下来后的实际队距由 ``queue_pitch`` 决定。

    非法配置一律 fail-fast：箱子排到带面外或队距小于箱长，都会在启动时就暴露，
    而不是等到运行时看见箱子悬空/互相穿模再回头猜。
    """

    count = _env_int(environ, "ISAACLAB_BELT_BOX_COUNT", BELT_BOX_MAX_COUNT)
    if count < 0:
        raise ValueError(f"ISAACLAB_BELT_BOX_COUNT 必须非负，当前 {count}")
    if count > BELT_BOX_MAX_COUNT:
        raise ValueError(
            f"ISAACLAB_BELT_BOX_COUNT={count} 超过上限 {BELT_BOX_MAX_COUNT}；"
            "SceneCfg 的 belt_box_N 字段是显式声明的，要加箱子请同时在 "
            "conveyor_env_cfg.G129SonicConveyorSceneCfg 里补字段并调大 BELT_BOX_MAX_COUNT"
        )

    lane_x = _env_float(environ, "ISAACLAB_BELT_BOX_LANE_X", BELT_BOX_LANE_X)
    spawn_z = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_Z", BELT_BOX_SPAWN_Z)
    y_lead = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD", 15.6)
    spawn_pitch = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_PITCH", 0.6)
    queue_pitch = _env_float(environ, "ISAACLAB_BELT_BOX_QUEUE_PITCH", 0.45)

    if count == 0:
        return 0, (), queue_pitch

    if queue_pitch < BELT_BOX_LENGTH_Y:
        raise ValueError(
            "ISAACLAB_BELT_BOX_QUEUE_PITCH 不能小于箱长 "
            f"{BELT_BOX_LENGTH_Y}，当前 {queue_pitch}"
        )
    if spawn_pitch < queue_pitch:
        raise ValueError(
            "ISAACLAB_BELT_BOX_SPAWN_PITCH 不能小于排队间距 "
            f"{queue_pitch}，当前 {spawn_pitch}"
        )

    half_len = BELT_BOX_LENGTH_Y * 0.5
    # 队首必须整体落在工位上游，队尾不能悬出带尾。
    if y_lead - half_len <= y_stop:
        raise ValueError(
            f"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD={y_lead} 让队首压在工位 y_stop={y_stop} 上；"
            f"至少要 {y_stop + half_len:.3f}"
        )
    y_tail = y_lead + spawn_pitch * (count - 1)
    if y_tail + half_len > BELT_BOX_BELT_Y_MAX:
        raise ValueError(
            f"{count} 个纸箱按 pitch={spawn_pitch} 从 y={y_lead} 排到 y={y_tail}，"
            f"队尾悬出带面 y_max={BELT_BOX_BELT_Y_MAX}"
        )

    positions = tuple(
        (lane_x, y_lead + spawn_pitch * index, spawn_z) for index in range(count)
    )
    return count, positions, queue_pitch


def resolve_scene_layout(environ: Mapping[str, str]) -> ConveyorSceneLayout:
    """Resolve the conveyor or pushcart layout from an environment mapping."""

    totes_on_conveyor = _env_bool(environ, "ISAACLAB_TOTES_ON_CONVEYOR", True)
    cart_group_x = _env_float(environ, "ISAACLAB_CART_GROUP_X", -5.62)
    cart_group_y = _env_float(environ, "ISAACLAB_CART_GROUP_Y", 18.75)
    robot_side_offset = _env_float(environ, "ISAACLAB_ROBOT_SIDE_OFFSET", 0.80)

    robot_workstation_y = _env_float(
        environ,
        "ISAACLAB_ROBOT_WORKSTATION_Y",
        14.148 if totes_on_conveyor else cart_group_y,
    )
    robot_1_x = _env_float(
        environ,
        "ISAACLAB_ROBOT_1_X",
        -4.75 if totes_on_conveyor else cart_group_x + robot_side_offset,
    )
    robot_2_x = _env_float(
        environ,
        "ISAACLAB_ROBOT_2_X",
        -6.7 if totes_on_conveyor else cart_group_x - robot_side_offset,
    )

    tote_spawn_y_lead = _env_float(environ, "ISAACLAB_TOTE_SPAWN_Y_LEAD", 17.4)
    tote_spawn_y_trail = _env_float(environ, "ISAACLAB_TOTE_SPAWN_Y_TRAIL", 18.0)

    if totes_on_conveyor:
        pushcart_2_pos = (-5.4, 19.39363, 0.0)
        cart2_tote1_pos = (-5.35, tote_spawn_y_lead, 0.775)
        cart2_tote2_pos = (-5.89, tote_spawn_y_trail, 0.775)
        tote_scale = (0.005, 0.005, 0.005)
        default_y_stop = robot_workstation_y
    else:
        pushcart_2_pos = (cart_group_x, cart_group_y, 0.0)
        cart2_tote1_pos = (cart_group_x, cart_group_y, 0.3794)
        cart2_tote2_pos = (cart_group_x, cart_group_y, 0.6814)
        tote_scale = (0.01, 0.01, 0.01)
        default_y_stop = 11.5

    conveyor_y_stop = _env_float(environ, "ISAACLAB_CONVEYOR_Y_STOP", default_y_stop)
    # 纸箱只在流水线布局下存在：推车布局的作业闭环仍是两塑料筐。
    if totes_on_conveyor:
        belt_box_count, belt_box_positions, belt_box_queue_pitch = (
            resolve_belt_box_positions(environ, y_stop=conveyor_y_stop)
        )
    else:
        belt_box_count, belt_box_positions, belt_box_queue_pitch = 0, (), 0.45

    return ConveyorSceneLayout(
        totes_on_conveyor=totes_on_conveyor,
        cart_group_x=cart_group_x,
        cart_group_y=cart_group_y,
        robot_side_offset=robot_side_offset,
        robot_workstation_y=robot_workstation_y,
        robot_1_x=robot_1_x,
        robot_2_x=robot_2_x,
        pushcart_2_pos=pushcart_2_pos,
        cart2_tote1_pos=cart2_tote1_pos,
        cart2_tote2_pos=cart2_tote2_pos,
        tote_scale=tote_scale,
        conveyor_y_stop=conveyor_y_stop,
        belt_box_count=belt_box_count,
        belt_box_positions=belt_box_positions,
        belt_box_queue_pitch=belt_box_queue_pitch,
    )
