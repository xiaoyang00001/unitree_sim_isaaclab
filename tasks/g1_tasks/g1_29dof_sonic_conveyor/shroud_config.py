# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Dependency-free resolver for the conveyor port shroud (方案 A：X 光安检机隧道).

流水线两端原本是断头，料筐凭空出现/消失。这里在生效端口的外侧贴一台纯视觉的
隧道式遮挡罩（DigitalTwin 行李安检机，自带隧道洞口 + 铅胶条帘），让带面看起来
是从设备里穿出来 / 钻进去的。

本模块刻意不 import Isaac Sim，也不写相对 import：普通 unittest 用
``importlib.util.spec_from_file_location`` 单文件加载即可验证开关与坐标算术。

**本期只做视觉观感**：罩子贴在端口外侧，不覆盖料筐出生点。真正"从罩子后面滑
出来"的两条路线记在任务 README 的『已知限制 / 待实测』里。
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass


SHROUD_MODE_ENV = "ISAACLAB_CONVEYOR_SHROUD"
SHROUD_FACE_Y_ENV = "ISAACLAB_CONVEYOR_SHROUD_FACE_Y"
DEFAULT_SHROUD_MODE = "auto"
SHROUD_MODES = ("auto", "on", "off")

SHROUD_ASSET_FILENAME = "infeed_scanner_visual.usda"
SHROUD_PRIM_NAME = "ConveyorShroud"

PORT_INFEED = "infeed"
PORT_OUTFEED = "outfeed"

# ------------------------------------------------------------------
# 带面几何：与 conveyor_drive 的同名常量同源。这里重复一份是为了保持本模块
# 零相对 import（测试用单文件加载）；tests/test_conveyor_shroud_config.py 会
# 交叉断言两边一致，别只改一边。
# ------------------------------------------------------------------
BELT_X_CENTER = -5.62
BELT_TOP_Z = 0.772
BELT_Y_MIN = 10.19  # 末尾（出料端），实测带体到 10.188
BELT_Y_MAX = 18.22  # 入料端，实测带体到 18.222

# ------------------------------------------------------------------
# 资产局部几何：pxr 离线量自
# NVIDIA/Assets/DigitalTwin/.../SecurityBaggageScanner_A01_01.usd，
# 数值已按 SHROUD_UNIT_SCALE 从原生 cm 换算成 m。
# 局部原点 = 机身中心，且落在地面（z=0 即机脚底面）。
# ------------------------------------------------------------------
SHROUD_UNIT_SCALE = 0.01
# 设计放大系数。保持 1.0：机身宽 0.843 < 流水线视觉宽 1.151，放大到 1.2 确实
# 更贴宽度，但会把机身整体再下沉到 -0.242（机脚埋进地面 0.15 m），并把机身拉长
# 到 3.06 m —— 出料端本来就没那么多净空（见下方 OUTFEED_FACE_Y 的说明），
# 放大反而把隧道机柜推进背景料箱堆里。宽度差留给后续方案权衡。
SHROUD_DESIGN_SCALE = 1.0

ASSET_HALF_LENGTH_Y = 1.27424
ASSET_HALF_WIDTH_X = 0.42161
ASSET_HEIGHT_Z = 1.50206
# 机身自带的滚筒/皮带顶面，用来和流水线带面 BELT_TOP_Z 对齐（整体下沉 0.0794）。
ASSET_ROLLER_TOP_Z = 0.851355
ASSET_CURTAIN_BOTTOM_Z = 0.855965
ASSET_CURTAIN_TOP_Z = 1.340204
ASSET_CURTAIN_HALF_LENGTH_Y = 0.296010
ASSET_CURTAIN_OPENING_WIDTH_X = 0.631573
# 隧道机柜（cover01/cover02/decal）是落地的，不是架空的：它从 z=0.0776 一直
# 长到 1.4977，且占满 y +/-0.350363。出料端定位受它约束。
ASSET_CABINET_HALF_LENGTH_Y = 0.350363

# ------------------------------------------------------------------
# 放置：迎风面 = 罩子朝流水线那一侧的端面世界 y。
# ------------------------------------------------------------------
# 入料端：带体到 18.222，留 0.078 缝。离线扫描确认 x[-7.6,-3.6] × y[18.25,21.5]
# × z<3.5 的背景里 0 个 Gprim，罩子占位 y[18.300,20.848] 完全干净。
INFEED_FACE_Y = 18.30
# 出料端：带尾 10.188。任务书假设"末尾外侧完全是空的"，实测**不成立**——
# 背景里 blue_sorting_bin_03 蓝料箱堆就在 x[-6.221,-4.555] y[8.757,10.092]
# z[0.003,0.420]，紧贴带尾 0.096 m。落地的隧道机柜（半长 0.350363，距迎风面
# 0.924）必须整体避开它，于是迎风面上限 = 8.757 + 0.924 ≈ 9.681，取 9.66 留
# 0.021 余量。代价是罩子离带尾 0.53 m（这段空隙正好被那堆料箱填住）。
OUTFEED_FACE_Y = 9.66


@dataclass(frozen=True)
class ConveyorShroudPlacement:
    """解析后的遮挡罩生效模式与放置参数（世界系，单位 m）。"""

    mode: str
    enabled: bool
    port: str
    asset_filename: str
    pos: tuple[float, float, float]
    rot: tuple[float, float, float, float]
    scale: tuple[float, float, float]
    face_y: float
    y_span: tuple[float, float]
    x_span: tuple[float, float]
    top_z: float
    opening_z: tuple[float, float]
    opening_width_x: float

    @property
    def yaw_degrees(self) -> float:
        """0 或 180——两种布局都让机身同一个端面对着流水线。"""

        return 0.0 if self.rot[0] != 0.0 else 180.0


def _resolve_mode(values: Mapping[str, str]) -> str:
    raw_mode = values.get(SHROUD_MODE_ENV, DEFAULT_SHROUD_MODE)
    mode = str(raw_mode).strip().lower() or DEFAULT_SHROUD_MODE
    if mode not in SHROUD_MODES:
        choices = ", ".join(SHROUD_MODES)
        raise ValueError(f"{SHROUD_MODE_ENV}={raw_mode!r} 无效，可选值: {choices}")
    return mode


def _resolve_face_y(values: Mapping[str, str], default: float) -> float:
    raw_value = values.get(SHROUD_FACE_Y_ENV)
    if raw_value is None or not str(raw_value).strip():
        return default
    try:
        return float(str(raw_value).strip())
    except ValueError:
        raise ValueError(
            f"{SHROUD_FACE_Y_ENV}={raw_value!r} 不是浮点数；"
            "该变量覆盖罩子迎风面的世界 y，留空即用布局默认值"
        ) from None


def resolve_conveyor_shroud(
    environ: Mapping[str, str] | None = None,
    *,
    totes_on_conveyor: bool,
) -> ConveyorShroudPlacement:
    """按开关与场景布局解析遮挡罩的生效模式和放置参数。

    ``auto``（默认）与 ``on`` 都生成，``off`` 只是不生成——放置参数照样算出来，
    这样启动日志在关掉时也能打印它本该在哪，排障不用改代码。
    非法值直接 ``ValueError``，不静默回退。

    布局决定端口：``totes_on_conveyor=True`` 时料筐出生在带面上游，罩子贴入料端；
    ``False`` 时料筐由机器人搬上带面并流向下游，罩子贴末尾并绕 Z 转 180°，
    使机身同一个端面始终朝着流水线。
    """

    values = os.environ if environ is None else environ
    mode = _resolve_mode(values)

    design_scale = SHROUD_DESIGN_SCALE
    half_length = ASSET_HALF_LENGTH_Y * design_scale
    half_width = ASSET_HALF_WIDTH_X * design_scale
    # 整体下沉，让机身自带滚筒面与流水线带面同高。
    pos_z = BELT_TOP_Z - ASSET_ROLLER_TOP_Z * design_scale

    if totes_on_conveyor:
        port = PORT_INFEED
        face_y = _resolve_face_y(values, INFEED_FACE_Y)
        center_y = face_y + half_length
        rot = (1.0, 0.0, 0.0, 0.0)
        y_span = (face_y, center_y + half_length)
    else:
        port = PORT_OUTFEED
        face_y = _resolve_face_y(values, OUTFEED_FACE_Y)
        center_y = face_y - half_length
        rot = (0.0, 0.0, 0.0, 1.0)
        y_span = (center_y - half_length, face_y)

    return ConveyorShroudPlacement(
        mode=mode,
        enabled=mode != "off",
        port=port,
        asset_filename=SHROUD_ASSET_FILENAME,
        pos=(BELT_X_CENTER, center_y, pos_z),
        rot=rot,
        scale=(
            SHROUD_UNIT_SCALE * design_scale,
            SHROUD_UNIT_SCALE * design_scale,
            SHROUD_UNIT_SCALE * design_scale,
        ),
        face_y=face_y,
        y_span=y_span,
        x_span=(BELT_X_CENTER - half_width, BELT_X_CENTER + half_width),
        top_z=pos_z + ASSET_HEIGHT_Z * design_scale,
        opening_z=(
            pos_z + ASSET_CURTAIN_BOTTOM_Z * design_scale,
            pos_z + ASSET_CURTAIN_TOP_Z * design_scale,
        ),
        opening_width_x=ASSET_CURTAIN_OPENING_WIDTH_X * design_scale,
    )
