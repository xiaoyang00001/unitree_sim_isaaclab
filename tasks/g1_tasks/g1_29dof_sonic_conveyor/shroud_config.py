# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Dependency-free resolver for the conveyor port shroud (方案 A：X 光安检机隧道).

流水线两端原本是断头，料筐凭空出现/消失。这里放一台纯视觉的隧道式遮挡罩
（DigitalTwin 行李安检机，自带隧道洞口 + 铅胶条帘），让带面看起来是从设备里
穿出来 / 钻进去的。

**入料端（``ISAACLAB_TOTES_ON_CONVEYOR=1``，默认）本轮改成"靠墙 + 贯穿"**：

* 机身放大到 ``SHROUD_INFEED_DESIGN_SCALE`` = 1.40，宽度 1.1805 略宽于流水线实测最宽
  处 1.1511，条帘净开口也从 0.632 撑到 0.884，终于容得下两条筐道横跨的 0.84；
* 机身**远端面贴 +Y 落地墙**（``WALL_FACE_Y`` = 23.606，留 ``WALL_CLEARANCE``
  0.05 m 缝），不再由带体端口反推位置；
* 流水线整体北移 ``CONVEYOR_NORTH_SHIFT_Y`` = 3.55 m 之后，带体**插进隧道**、
  端头断面藏在不透明机柜里。旧的"贴在端口外侧、首尾相接"那套 ``INFEED_FACE_Y``
  偏移逻辑已删除；
* 被带体吞掉的那半段外露滚筒床（``HIDDEN_SUBMESH_NAMES``，近端 roller/rollercover
  10..18 共 18 个 Mesh）在 ``infeed_scanner_visual.usda`` 里直接置
  ``visibility = "invisible"``，不靠"刚好被挡住"这种数值巧合。

出料端（``=0`` 布局）维持原样只随北移平移：仍是 scale 1.0 的整机贴在带尾外侧，
用没有隐藏清单的 ``outfeed_scanner_visual.usda``。放大 + 靠墙那套不能套到 =0——
放大后半长 1.784，贴带尾放会整块插进背景的 ``blue_sorting_bin_03`` 料箱堆。

本模块刻意不 import Isaac Sim，也不写相对 import：普通 unittest 用
``importlib.util.spec_from_file_location`` 单文件加载即可验证开关与坐标算术。
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass


SHROUD_MODE_ENV = "ISAACLAB_CONVEYOR_SHROUD"
SHROUD_FACE_Y_ENV = "ISAACLAB_CONVEYOR_SHROUD_FACE_Y"
DEFAULT_SHROUD_MODE = "auto"
SHROUD_MODES = ("auto", "on", "off")

# 入料端（=1）：带体贯穿机身，近端外露滚筒床按 HIDDEN_SUBMESH_NAMES 隐藏。
SHROUD_INFEED_ASSET_FILENAME = "infeed_scanner_visual.usda"
# 出料端（=0）：整机贴在带尾外侧，滚筒床要留着当"带体的去处"，因此是另一份
# 不带隐藏清单的薄层。两份薄层引用同一个 S3 源资产。
SHROUD_OUTFEED_ASSET_FILENAME = "outfeed_scanner_visual.usda"
SHROUD_PRIM_NAME = "ConveyorShroud"

PORT_INFEED = "infeed"
PORT_OUTFEED = "outfeed"

# ------------------------------------------------------------------
# 流水线整体北移量与带面几何：与 conveyor_drive 的同名常量同源。这里重复一份
# 是为了保持本模块零相对 import（测试用单文件加载）；
# tests/test_conveyor_shroud_config.py 会交叉断言两边一致，别只改一边。
# ------------------------------------------------------------------
CONVEYOR_NORTH_SHIFT_Y = 3.55
BELT_X_CENTER = -5.62
BELT_TOP_Z = 0.772
# 北移前实测带体 y[10.188, 18.222]；下面是取整后的运行常量 + Δ。
BELT_Y_MIN = round(10.19 + CONVEYOR_NORTH_SHIFT_Y, 6)  # 13.74，末尾（出料端）
BELT_Y_MAX = round(18.22 + CONVEYOR_NORTH_SHIFT_Y, 6)  # 21.77，入料端

# 两条筐道（x=-5.35 / -5.89，半尺寸筐 0.30(x)×0.20(y)×0.15(z)）的横向总跨度。
# 离线实测 0.84（README 旧稿写的 0.69 是把筐宽当成了 0.15，已更正）。
# 这是"必须放大机身"的硬理由：原尺寸条帘净开口只有 0.632，装不下。
TOTE_LANE_SPAN_X = 0.84

# ------------------------------------------------------------------
# 资产局部几何：pxr 离线量自
# NVIDIA/Assets/DigitalTwin/.../SecurityBaggageScanner_A01_01.usd，
# 数值已按 SHROUD_UNIT_SCALE 从原生 cm 换算成 m。
# 局部原点 = 机身中心，且落在地面（z=0 即机脚底面）。
# ------------------------------------------------------------------
SHROUD_UNIT_SCALE = 0.01
# 入料端设计放大系数。1.40 的取法：
#   * 等宽解是 1.151/0.84322 = 1.365，但流水线最宽处是每 2 m 一组的支腿站
#     （实测整线 x 跨度 1.15114），1.365 会让机身反而窄 0.9 mm；1.40 给单边
#     +7.4 mm 余量，且是个整数档，微调 Δ 时不会立刻翻负；
#   * 条帘净开口 0.631573 → 0.884，终于容得下两条筐道的 TOTE_LANE_SPAN_X=0.84
#     （单边余量 22 mm）——这是放大的**硬理由**，比"宽度好看"结实；
#   * 代价是整机下沉 0.4299 m（机脚/脚轮/底座全埋进地面，机柜直接落地，观感反而
#     更干净），可见高度 1.673 < 墙高 3.10。
# 旧注释里"放大会加深下沉 + 出料端净空不够"的否决理由只对**贴带尾**的摆法成立；
# 靠墙贯穿之后入料端外侧有 5.4 m 走廊（离线实测 0 个地面级 Gprim），前提已变。
SHROUD_INFEED_DESIGN_SCALE = 1.40
# 出料端保持 1.0。放大到 1.40 后半长 1.784，贴带尾放会整块插进背景的
# blue_sorting_bin_03（实测 x[-6.221,-4.555] y[8.757,10.092] z[0.003,0.420]，
# 北移后 y[12.307,13.642]），且 =0 布局根本吃不到"靠墙贯穿"的观感收益。
SHROUD_OUTFEED_DESIGN_SCALE = 1.0
# ⚠️ 刻意不留 SHROUD_DESIGN_SCALE 统一别名：放大系数已经按端口分叉，留个"当前那台"
#    的模块级别名会变成第二处真源。外部一律读 placement.design_scale。

# 额外下沉，专治共面 z-fighting。资产的 cartframe 顶面 native z 恰好等于
# ASSET_ROLLER_TOP_Z，只按滚筒面对齐会与流水线滚筒顶 0.7723 只差 0.3 mm，
# 俯视必然闪烁。压 10 mm 之后机身内部车架顶落到 0.762（低于带面 10 mm，被侧裙和
# 滚筒盖住），条帘底边 0.76845 比带面低 3.5 mm，帘子轻扎进带面，正是想要的观感。
SHROUD_ZFIGHT_SINK = 0.010

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
# 放置锚点：``face_y`` 统一表示**机身 +Y 端面**的世界 y（两种布局同一语义，
# ISAACLAB_CONVEYOR_SHROUD_FACE_Y 覆盖的也是它）。
# ------------------------------------------------------------------
# 入料端由墙反推，不再由带体端口反推。SM_WallA_6M16 横跨走廊
# （x[-7.680,-1.680] z[0,3.100]），离线实测南面 y=23.6055，取 23.606。
WALL_FACE_Y = 23.606
# 机身与墙面留的缝。给的是"贴着但不穿模"的观感，别设成 0：墙面本身有 0.2 m 厚度
# 与法线偏差，0 会让远端滚筒的 AABB 与墙皮共面。
WALL_CLEARANCE = 0.05
# 出料端：带尾北移后到 13.738。背景里 blue_sorting_bin_03 蓝料箱堆紧贴带尾
# 0.096 m（北移后 y[12.307,13.642]），而落地的隧道机柜（半长 0.350363，距端面
# 0.924）必须整体避开它 ⇒ 端面上限 = 12.307 + 0.924 ≈ 13.231，取 13.21 留
# 0.021 余量。整条约束随北移刚体平移，相对关系与北移前逐毫米一致。
OUTFEED_FACE_Y = round(9.66 + CONVEYOR_NORTH_SHIFT_Y, 6)  # 13.21

# ------------------------------------------------------------------
# 入料端要隐藏的外露滚筒床（``infeed_scanner_visual.usda`` 里置
# visibility="invisible"）。资产 68 个 Gprim、**无 instanceable**（已 pxr 核实），
# 所以 over 生效。局部 y[-1.2135,-0.6428] 是朝流水线那一端的 9 组滚筒 + 滚筒盖。
#
# ⚠️ cartframe_01 / joints_01 / feets_01 / feetbase_01 / screws_01 / belt_01 /
#    caps_01 是**贯穿全长的单一 Mesh**，没法只藏近端一半。好在按 scale=1.40 +
#    pos_z=-0.4299 换算后它们在近端全部落进流水线外轮廓内：最大外偏 screws
#    0.5116 < 流水线半宽 0.5756，最高 cartframe 顶 0.762 < 带面 0.772，
#    feets/feetbase 在 z<0 已埋进地面。所以是"能隐藏的隐藏，隐藏不了的正好被
#    流水线本体吞掉"，不是二选一。
# ------------------------------------------------------------------
HIDDEN_SUBMESH_NAMES = tuple(
    f"sm_securitybaggagescanner_a01_{kind}{index:02d}_01"
    for kind in ("roller", "rollercover")
    for index in range(10, 19)
)


@dataclass(frozen=True)
class ConveyorShroudPlacement:
    """解析后的遮挡罩生效模式与放置参数（世界系，单位 m）。"""

    mode: str
    enabled: bool
    port: str
    asset_filename: str
    design_scale: float
    pos: tuple[float, float, float]
    rot: tuple[float, float, float, float]
    scale: tuple[float, float, float]
    face_y: float
    y_span: tuple[float, float]
    x_span: tuple[float, float]
    top_z: float
    opening_z: tuple[float, float]
    opening_width_x: float
    curtain_y_span: tuple[float, float]
    cabinet_y_span: tuple[float, float]
    belt_penetrates: bool

    @property
    def yaw_degrees(self) -> float:
        """入料端 0°（完整滚筒床朝墙），出料端 180°（完整滚筒床朝流水线）。"""

        return 0.0 if self.rot[0] != 0.0 else 180.0

    @property
    def hidden_submesh_names(self) -> tuple[str, ...]:
        """被带体吞掉、需要在薄层里隐藏的子网格；出料端为空。"""

        return HIDDEN_SUBMESH_NAMES if self.port == PORT_INFEED else ()


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
            "该变量覆盖机身 +Y 端面的世界 y（入料端默认贴 +Y 墙），"
            "留空即用布局默认值"
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

    布局决定端口与摆法：

    * ``totes_on_conveyor=True``（入料端）：机身放大 1.40 后**远端面贴 +Y 墙**，
      流水线北移 3.55 m 反过来插进隧道，端头断面藏在不透明机柜里；近端外露的
      滚筒床由薄层隐藏。yaw 0°，让完整的那半段滚筒床朝墙（读起来是"出料段顶到
      墙上"），被隐藏的那半段朝流水线。
    * ``totes_on_conveyor=False``（出料端）：维持"贴在带尾外侧"的老摆法，只随
      北移整体平移，scale 保持 1.0，绕 Z 转 180° 让完整滚筒床朝着流水线。

    两种布局的 ``face_y`` 语义统一为**机身 +Y 端面**的世界 y。
    """

    values = os.environ if environ is None else environ
    mode = _resolve_mode(values)

    if totes_on_conveyor:
        port = PORT_INFEED
        design_scale = SHROUD_INFEED_DESIGN_SCALE
        default_face_y = round(WALL_FACE_Y - WALL_CLEARANCE, 6)
        rot = (1.0, 0.0, 0.0, 0.0)
        asset_filename = SHROUD_INFEED_ASSET_FILENAME
    else:
        port = PORT_OUTFEED
        design_scale = SHROUD_OUTFEED_DESIGN_SCALE
        default_face_y = OUTFEED_FACE_Y
        rot = (0.0, 0.0, 0.0, 1.0)
        asset_filename = SHROUD_OUTFEED_ASSET_FILENAME

    half_length = ASSET_HALF_LENGTH_Y * design_scale
    half_width = ASSET_HALF_WIDTH_X * design_scale
    # 整体下沉：先让机身自带滚筒面与带面同高，再多压 SHROUD_ZFIGHT_SINK 消共面。
    pos_z = BELT_TOP_Z - ASSET_ROLLER_TOP_Z * design_scale - SHROUD_ZFIGHT_SINK

    # face_y 一律是机身 +Y 端面，于是两种布局共用同一个中心公式。
    face_y = _resolve_face_y(values, default_face_y)
    center_y = face_y - half_length
    y_span = (center_y - half_length, face_y)
    curtain_half = ASSET_CURTAIN_HALF_LENGTH_Y * design_scale
    cabinet_half = ASSET_CABINET_HALF_LENGTH_Y * design_scale

    return ConveyorShroudPlacement(
        mode=mode,
        enabled=mode != "off",
        port=port,
        asset_filename=asset_filename,
        design_scale=design_scale,
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
        curtain_y_span=(center_y - curtain_half, center_y + curtain_half),
        cabinet_y_span=(center_y - cabinet_half, center_y + cabinet_half),
        # 入料端要求带体末端越过近端条帘并停在机柜内部；出料端则要求整机在带体
        # 之外。两条判据都在 tests/test_conveyor_shroud_config.py 里锁死。
        belt_penetrates=(
            port == PORT_INFEED
            and y_span[0] < BELT_Y_MAX < y_span[1]
        ),
    )
