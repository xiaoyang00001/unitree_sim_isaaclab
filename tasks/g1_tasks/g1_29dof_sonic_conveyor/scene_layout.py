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


def resolve_scene_layout(environ: Mapping[str, str]) -> ConveyorSceneLayout:
    """Resolve the conveyor or pushcart layout from an environment mapping."""

    totes_on_conveyor = _env_bool(environ, "ISAACLAB_TOTES_ON_CONVEYOR", True)
    cart_group_x = _env_float(environ, "ISAACLAB_CART_GROUP_X", -5.62)
    cart_group_y = _env_float(environ, "ISAACLAB_CART_GROUP_Y", 18.75)
    robot_side_offset = _env_float(environ, "ISAACLAB_ROBOT_SIDE_OFFSET", 0.80)

    # 13.548 = 14.148 - 0.600：v61 换版把工位家具整体往 -Y 挪了（离线 AABB 实测
    # SM_HeavyDutyPackingTable_C02_03 是纯平移 -0.600，blue_sorting_bin_01/02 中心
    # 分别 -0.642/-0.553），而工位 y 当初没跟着动，两台机器人开局就插进分拣料箱
    # （robot_1 -0.208、robot_2 -0.340）。按纯平移量补偿后，机器人相对家具的间隙
    # 精确回到 v48 的 +0.212 / +0.091。⚠️ 这个值同时是 =1 布局的 conveyor_y_stop
    # 默认值（料筐停在机器人面前），改它会一并改变料筐停位。
    robot_workstation_y = _env_float(
        environ,
        "ISAACLAB_ROBOT_WORKSTATION_Y",
        13.548 if totes_on_conveyor else cart_group_y,
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
        conveyor_y_stop=_env_float(
            environ, "ISAACLAB_CONVEYOR_Y_STOP", default_y_stop
        ),
    )
