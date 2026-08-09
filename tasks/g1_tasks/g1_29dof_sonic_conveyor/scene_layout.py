"""Dependency-free resolver for the two conveyor scene layouts.

Keeping the layout arithmetic outside ``conveyor_env_cfg`` lets ordinary Python
tests verify the environment-variable switch without importing Isaac Lab.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


# 流水线整体北移量（世界 +Y，m）。真源是 conveyor_drive.CONVEYOR_NORTH_SHIFT_Y，
# 那里写着完整推导：安检机靠 +Y 落地墙（南面 23.606）、放大 SHROUD_INFEED_DESIGN_SCALE=1.40
# 后机身中心落在 21.772064，让流水线入料端（实测 18.2223）正好推进机身中心 ⇒ 3.55。
# 本模块刻意零相对 import（测试用单文件加载），所以抄一份；
# tests/test_conveyor_scene_layout.py 交叉断言两边一致，别只改一边。
CONVEYOR_NORTH_SHIFT_Y = 3.55


def _shifted(base_y: float) -> float:
    """把北移前的世界 y 基准值平移到当前布局；round 掉二进制浮点尾巴。"""

    return round(base_y + CONVEYOR_NORTH_SHIFT_Y, 6)


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
    # =0 布局的作业组（拖车 + 叠筐 + 两台机器人）站在入料端之北，与流水线同轴。
    # 整条线北移 Δ 后它跟着 +Δ 即可：拖车实测占位 y 跨度 0.824（spawn scale 0.5），
    # 北移后 y[21.888, 22.712]，离带端 21.772 仍是原来的 0.116 m，离 +Y 墙面
    # 23.606 还有 0.894 m —— 墙给的上限（Δ0<=4.24）远大于 3.55，=0 无需另选位。
    cart_group_y = _env_float(environ, "ISAACLAB_CART_GROUP_Y", _shifted(18.75))
    robot_side_offset = _env_float(environ, "ISAACLAB_ROBOT_SIDE_OFFSET", 0.80)

    robot_workstation_y = _env_float(
        environ,
        "ISAACLAB_ROBOT_WORKSTATION_Y",
        _shifted(14.148) if totes_on_conveyor else cart_group_y,
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

    # 北移后两个出生点落在 20.95 / 21.55，两者都进了安检机机身 y[19.988, 23.556]，
    # 其中 tote2 已落进不透明机柜 y[21.2816, 22.2626]（出生瞬间真正被遮住），
    # tote1 还差 0.33 m 未进机柜——README「未真正遮住出生瞬间」那条限制由此收窄一半。
    tote_spawn_y_lead = _env_float(
        environ, "ISAACLAB_TOTE_SPAWN_Y_LEAD", _shifted(17.4)
    )
    tote_spawn_y_trail = _env_float(
        environ, "ISAACLAB_TOTE_SPAWN_Y_TRAIL", _shifted(18.0)
    )

    if totes_on_conveyor:
        pushcart_2_pos = (-5.4, _shifted(19.39363), 0.0)
        cart2_tote1_pos = (-5.35, tote_spawn_y_lead, 0.775)
        cart2_tote2_pos = (-5.89, tote_spawn_y_trail, 0.775)
        tote_scale = (0.005, 0.005, 0.005)
        default_y_stop = robot_workstation_y
    else:
        pushcart_2_pos = (cart_group_x, cart_group_y, 0.0)
        cart2_tote1_pos = (cart_group_x, cart_group_y, 0.3794)
        cart2_tote2_pos = (cart_group_x, cart_group_y, 0.6814)
        tote_scale = (0.01, 0.01, 0.01)
        default_y_stop = _shifted(11.5)

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
