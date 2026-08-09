# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""看不到头的流水线入料端（90° 西拐弯道 + X 支线，复用已有货架）——零依赖解析器。

安检机遮罩方案（shroud_config，已拆除）被否决后，入料端改为：

* 主线带头（Δ=0 实测公头面 y=18.2223）接一段 DigitalTwin **ConveyorBelt_A02**
  低位滚筒弯道向西（-X）拐 90°——A02 是全资产包唯一与 A08 同高同构造的弯道
  （滚筒顶 76.93 cm、连续侧挡边 79.82 cm、门柱 116.63 cm 全部同族）；
* 弯道西口（公头）插进 **ConveyorBelt_A05** 2 m 短直段的母口，共三段 A05 接成
  X 支线一路向西，端头尽量贴近仓库背景 v61 自带的西侧满载货架排 B
  （y[18.269,19.346]，x 连排到 -29.58）；
* **不新增任何货架/遮挡件**（用户硬要求）：遮挡只依赖背景既有结构。离线射线
  核算的结论是"零新增即全遮"在几何上**无解**（见下方「遮挡窗（洞）」），
  洞如实记录在 README 已知限制里，留用户拍板。

开关 ``ISAACLAB_CONVEYOR_ENDLESS``：``auto``（默认，=开）| ``on`` | ``off``；
非法值直接 ``ValueError``。只在流水线布局（``ISAACLAB_TOTES_ON_CONVEYOR=1``）
且道具策略为 ``layout`` 时真正生成：

* =0 推车布局的拖车组 (-5.62, 18.75)（AABB x[-6.09,-5.15] y[17.93,19.57]）约
  83% 落在弯道占位 x[-7.113,-5.042] y[18.207,20.308] 内，冲突不可微调回避
  （挪拖车会破坏 =0 "离带头 0.118 m" 的作业约定）；
* ``legacy_props`` 回退在 =1 下把空车 pushcart_2 摆在 (-5.4, 19.39363)，
  约 99% 包含于弯道占位——所以生成门是 "totes_on_conveyor 且 props==layout"，
  不能只看 =1/=0。

本模块刻意零 Isaac import、零相对 import（普通 unittest 用
``importlib.util.spec_from_file_location`` 单文件加载即可验证开关与几何算术）。
资产路径只给相对段，由 conveyor_env_cfg 拼 isaaclab.utils.assets 的
``NVIDIA_NUCLEUS_DIR``（本机 resolver 重定向到本地资产包）。

坐标全部是世界系、米。数值来自离线 pxr 实测链（A02/A05 局部 bbox 逐件量过，
弯道摆位由主线带头公头实测中心 x=-5617.0 mm、带头面 y=18222.3 mm 推出），
不是名义值；与 conveyor_drive 常量的关系由 tests/test_conveyor_endless_intake.py
交叉断言。

遮挡窗（洞）——离线射线核算（眼位 E1=(-4.54,14.448,1.6)/E2=(-6.7,14.448,1.6)，
箱顶最高 c01=1.0223）：已有货架排 B 的全遮阴影区要求箱心 x≤-14.36（E1）/
-14.22（E2），而支线物理极限是三段端头 s_min 处 x≈-13.07（排 B 端护板
x=-13.386 硬限，第四段必与货架排干涉 0.19-0.24 m）——差 1.15~1.36 m 不可达。
逐 slot：主线/弧上箱子裸露~半露（箱底沿被 0.801 挡边遮），支线段箱顶可见、
下半 (z<0.80) 被支线自身挡边遮；回生点 (-12.61,19.803) 距眼 8.0~9.6 m，
pop-in 几何上可见，观感靠远距 + 排 B 满货/叉车背景弱化。
"""

from __future__ import annotations

import math
import os
from collections.abc import Mapping
from dataclasses import dataclass


ENDLESS_MODE_ENV = "ISAACLAB_CONVEYOR_ENDLESS"
DEFAULT_ENDLESS_MODE = "auto"
ENDLESS_MODES = ("auto", "on", "off")

# 主线带头（Δ=0）：公头面世界 y 与公头中心世界 x 的离线实测值。
# ⚠️ 与 conveyor_drive.BELT_Y_MAX（碰撞板常量 18.22）差 2.3 mm——那是板的收口
#    取整，这里插接必须用实测面。
BELT_HEAD_FACE_Y = 18.2223
BELT_HEAD_MALE_CENTER_X = -5.617
PLUG_DEPTH = 0.015  # 公插母 15 mm（族内先例最多 119 mm；对顶才只能留 3 mm 缝）

# ------------------------------------------------------------------
# 弯道 ConveyorBelt_A02（2.1007×2.0714×1.1663 m，裸厘米资产 ⇒ UsdFileCfg.scale=0.01）
#
# 摆位推导（mm，世界系；yaw=-90° 即绕 Z 顺时针 90°，局部 (x,y) → 世界 (y, -x)）：
# * A02 默认（yaw=0）连通 南(公 y=0 面) + 东(母 x=2097.5 面)；转 -90° 后变
#   西(公) + 南(母)——南向母口正好**插接**主线带头公头（母套公 15 mm 正常插接，
#   比东拐方案的公对公对顶 3 mm 缝更顺，无断口转接框问题）；
# * y：南向母口面（局部 x=2097.5）落在 18222.3-15.0=18207.3 ⇒ t.y=20304.8；
# * x：南口车道中线（局部 y=1495.9）对齐主线公头实测中心 -5617.0
#   ⇒ t.x=-5617.0-1495.9=-7112.9（与驱动常量 -5620 偏 3 mm，与主线自身的
#   既有偏差同源，远小于母口孔宽 1151 对公头 1009 的单侧 71 mm 余量）；
# * z：场景统一偏移 +3.0 mm ⇒ 滚筒顶 772.3、门柱顶 1169.3 与主线严格共面。
# ------------------------------------------------------------------
CURVE_ASSET_NVIDIA_RELPATH = (
    "Assets/DigitalTwin/Assets/Warehouse/Equipment/Conveyors/"
    "ConveyorBelt_A/ConveyorBelt_A02_PR_NVD_01.usd"
)
CONVEYOR_UNIT_SCALE = 0.01  # DigitalTwin 资产是裸厘米（mpu=0.01 语义）
CURVE_POS = (-7.1129, 20.3048, 0.003)
CURVE_YAW_DEG = -90.0
# 离线实测世界 AABB（z 上限 1.1693 是南口旁两根门柱；连续挡边只到 0.8012）。
CURVE_AABB = ((-7.1130, -5.0416), (18.2073, 20.3080), (0.003, 1.1693))
# 西向公头端面 x 与西口车道中线 y（=t.y-501.4mm）。
CURVE_WEST_MALE_TIP_X = -7.1129
CURVE_WEST_LANE_Y = 19.8034

# ------------------------------------------------------------------
# X 支线 ConveyorBelt_A05 × 3（2000.1 mm 短直段，与 A08 同宽同高同滚筒构造）
#
# yaw=-90° 后母口朝东（受上游公头插入）、公头朝西（伸向下一段/自由端），传动
# 胶带条（局部 x<0 侧悬出 74.05 mm）甩到北侧背对工位。段 1 母口东端面
# （局部 y=0 ⇒ 世界 x=t.x）盖过弯道西公头尖 15.0 mm ⇒ t.x=-7112.9+15.0=-7097.9；
# 段间同样公插母 15.0 mm，每段净延伸 2000.1-15.0=1985.1 mm。
# 车道中线全部与弯道西口一致：y = t.y-501.4 = 19803.4。
# ------------------------------------------------------------------
XLEG_ASSET_NVIDIA_RELPATH = (
    "Assets/DigitalTwin/Assets/Warehouse/Equipment/Conveyors/"
    "ConveyorBelt_A/ConveyorBelt_A05_PR_NVD_01.usd"
)
XLEG_COUNT = 3
XLEG_YAW_DEG = -90.0
XLEG_POSITIONS = (
    (-7.0979, 20.3048, 0.003),
    (-9.0830, 20.3048, 0.003),
    (-11.0681, 20.3048, 0.003),
)
# 离线实测世界 AABB（南缘 19.2279 = 车道侧挡边；北缘 20.3789 = 传动胶带条悬出）。
XLEG_AABBS = (
    ((-9.0979, -7.0978), (19.2279, 20.3789), (0.003, 1.1693)),
    ((-11.0830, -9.0829), (19.2279, 20.3789), (0.003, 1.1693)),
    ((-13.0681, -11.0680), (19.2279, 20.3789), (0.003, 1.1693)),
)
# 每段滚筒可用带（局部实测反推）；段间断口 ≈41 mm 由 kinematic 托面补齐。
XLEG_ROLLER_X_RANGES = (
    (-9.0938, -7.1066),
    (-11.0789, -9.0917),
    (-13.0640, -11.0768),
)
BRANCH_ROLLER_WEST_END_X = XLEG_ROLLER_X_RANGES[-1][0]  # -13.0640

# 周边既有结构（世界系、离线实测；用于间隙断言与"最多三段"的硬限说明）：
# * 背景 v61 西侧货架排 B（前左）：y[18.269,19.346]、货物最东 -13.768、
#   钢架最东立柱东缘 -13.520、端护板东缘 -13.386（支线不可越过 ⇒ 三段是上限，
#   第四段西端 -15.053 会与货架排 y 重叠区干涉）；
# * +Y 落地墙 SM_WallA_6M16/17 南面 23.606：弯道北缘 20.308 留 3.298 m；
# * 交通锥 Cone_4 x[-11.07,-10.74] y[21.23,21.56]：与段 3 x 几乎相切但 y 净距
#   0.851 m；Cone_3 在弯道南 1.6 m，均无碰撞。
RACK_B_GUARD_EAST_X = -13.386
RACK_B_STEEL_EAST_X = -13.520
RACK_B_NORTH_Y = 19.346
RACK_B_Y_RANGE = (18.269, 19.346)
WALL_FACE_Y = 23.606
CONE_4_AABB = ((-11.07, -10.74), (21.23, 21.56))

# ------------------------------------------------------------------
# 驱动路径（沿路径距离 s 参数化；conveyor_queue 有等价的 torch 版）
#
#   s=0 锚点取支线 x=-12.76（车道中线 y=19.8034）；s 向下游递增：
#   ① 支线段 s∈[S_MIN, S_ARC_START]：pos=(S_ORIGIN_X+s, 19.8034)，航向 +X
#      ——⚠️ 与东拐存档镜像：这里 +s 对应 +X；
#   ② 圆角弧 s∈[S_ARC_START, S_ARC_END]：圆心 (-7.0200, 18.4034)、R=1.40，
#      θ=(s-S_ARC_START)/R∈[0,π/2]，pos=(cx+R·sinθ, cy+R·cosθ)，
#      航向 (cosθ,-sinθ)；直角尖拐点在滚筒外缘以外（东拐已验证同族事实），
#      必须切 R=1.4 圆角（与 A02 资产真实弧线 R≈1.40-1.45 吻合）；
#   ③ 主线段 s>S_ARC_END：pos=(-5.62, 18.4034-(s-S_ARC_END))，航向 -Y。
#
# 圆心由车道几何导出：cx = 主车道中线 − R（东拐是 +R），cy = 支线车道中线 − R。
# ------------------------------------------------------------------
MAIN_LANE_X = -5.62          # = conveyor_drive.BELT_X_CENTER（交叉断言）
LANE_HALF_WIDTH = 0.45       # = conveyor_drive.BELT_WIDTH / 2（交叉断言）
BRANCH_LANE_Y = 19.8034      # 弯道西口车道中线（离线实测；= CURVE_WEST_LANE_Y）
CORNER_RADIUS = 1.40
CORNER_CENTER = (MAIN_LANE_X - CORNER_RADIUS, BRANCH_LANE_Y - CORNER_RADIUS)
S_ORIGIN_X = -12.76
S_ARC_START = CORNER_CENTER[0] - S_ORIGIN_X              # 5.7400
S_ARC_END = S_ARC_START + CORNER_RADIUS * math.pi / 2.0  # ≈7.93911
# 支线滚筒可用端（段 3 西端 -13.0640 ⇒ s=-0.3040，取 -0.30）：出生/排队不得越过。
PATH_S_MIN = -0.30

# 循环模式（y_stop<=0）的回生落点：支线最深处 s=0.15 ⇒ (-12.61, 19.8034)。
# ⚠️ 没有全遮窗（见模块 docstring）：回生 pop-in 几何上可见，距双机眼位
# 8.0~9.6 m、背景是排 B 满货+叉车，观感弱化但不为零——README 已知限制。
RESPAWN_S = 0.15
RESPAWN_XY = (S_ORIGIN_X + RESPAWN_S, BRANCH_LANE_Y)


def path_point(s: float) -> tuple[float, float]:
    """沿路径距离 s → 世界 (x, y)（车道中线上）。"""

    cx, cy = CORNER_CENTER
    if s <= S_ARC_START:
        return (S_ORIGIN_X + s, BRANCH_LANE_Y)
    if s >= S_ARC_END:
        return (MAIN_LANE_X, cy - (s - S_ARC_END))
    theta = (s - S_ARC_START) / CORNER_RADIUS
    return (cx + CORNER_RADIUS * math.sin(theta), cy + CORNER_RADIUS * math.cos(theta))


def path_heading(s: float) -> tuple[float, float]:
    """沿路径距离 s 处的下游航向单位向量。"""

    if s <= S_ARC_START:
        return (1.0, 0.0)
    if s >= S_ARC_END:
        return (0.0, -1.0)
    theta = (s - S_ARC_START) / CORNER_RADIUS
    return (math.cos(theta), -math.sin(theta))


def path_s_of_main_y(y: float) -> float:
    """主车道上的世界 y → s（工位/停止线都在主车道上用它换算）。"""

    return S_ARC_END + (CORNER_CENTER[1] - y)


def path_s_of_point(x: float, y: float) -> float:
    """任意 (x, y) → s：与 conveyor_queue.path_progress 同一分段判据的标量版。"""

    cx, cy = CORNER_CENTER
    if y <= cy:
        return S_ARC_END + (cy - y)
    if x <= cx:
        return x - S_ORIGIN_X
    theta = math.atan2(max(x - cx, 0.0), max(y - cy, 0.0))
    return S_ARC_START + CORNER_RADIUS * theta


# ------------------------------------------------------------------
# kinematic 碰撞板延伸（与主线 conveyor_collider 同 z 同做法，纯不可见托面）：
# * 拐角补块：覆盖弧段车道摆幅（车道 ±0.45 ⇒ 半径 [0.95,1.85]）的包围矩形，
#   南缘接主线碰撞板末端 18.22，整块落在弯道足迹内；
# * 支线条带：车道中线 ±0.45，东接拐角补块、西到段 3 滚筒可用端（段间 ≈41 mm
#   滚筒断口一并补齐）。
# 板厚/顶面与主线一致（顶 z=0.772，厚 0.04）。
# ⚠️ 条带南缘 19.3534 与货架排 B 北缘 19.346 只差 7 mm，但 x 区间不重叠
#   （条带止于 -13.064，护板在 -13.386）——离线定量核过，无碰撞。
# ------------------------------------------------------------------
BELT_TOP_Z = 0.772           # = conveyor_drive.BELT_TOP_Z（交叉断言）
COLLIDER_THICKNESS = 0.04    # = conveyor_drive.BELT_COLLIDER_THICKNESS（交叉断言）
CORNER_PLATE_X_RANGE = (CORNER_CENTER[0], MAIN_LANE_X + LANE_HALF_WIDTH)   # (-7.02, -5.17)
CORNER_PLATE_Y_RANGE = (18.22, BRANCH_LANE_Y + LANE_HALF_WIDTH)            # (18.22, 20.2534)
BRANCH_PLATE_X_RANGE = (BRANCH_ROLLER_WEST_END_X, CORNER_CENTER[0])        # (-13.064, -7.02)
BRANCH_PLATE_Y_RANGE = (BRANCH_LANE_Y - LANE_HALF_WIDTH, BRANCH_LANE_Y + LANE_HALF_WIDTH)

# 驱动事件的带面判据（on_belt）在主线矩形之外增加的两块并集矩形
# （比碰撞板各向外放宽 0.10，与主线 x_range 的放宽口径一致）：(x0, x1, y0, y1)。
ON_BELT_EXTRA_RECTS = (
    (
        CORNER_PLATE_X_RANGE[0] - 0.10,
        CORNER_PLATE_X_RANGE[1] + 0.10,
        CORNER_PLATE_Y_RANGE[0] - 0.10,
        CORNER_PLATE_Y_RANGE[1] + 0.10,
    ),
    (
        BRANCH_PLATE_X_RANGE[0] - 0.10,
        BRANCH_PLATE_X_RANGE[1] + 0.10,
        BRANCH_PLATE_Y_RANGE[0] - 0.10,
        BRANCH_PLATE_Y_RANGE[1] + 0.10,
    ),
)


def yaw_quat(yaw_deg: float) -> tuple[float, float, float, float]:
    """绕 Z 的 yaw → (w, x, y, z) 四元数。"""

    half = math.radians(yaw_deg) * 0.5
    return (math.cos(half), 0.0, 0.0, math.sin(half))


@dataclass(frozen=True)
class EndlessIntakeConfig:
    """解析后的生效模式；几何常量直接读模块级（都是不可变元组/标量）。"""

    mode: str
    requested: bool
    enabled: bool
    disabled_reason: str

    @property
    def respawn_xy(self) -> tuple[float, float]:
        return RESPAWN_XY


def resolve_endless_mode(values: Mapping[str, str]) -> str:
    raw_mode = values.get(ENDLESS_MODE_ENV, DEFAULT_ENDLESS_MODE)
    mode = str(raw_mode).strip().lower() or DEFAULT_ENDLESS_MODE
    if mode not in ENDLESS_MODES:
        choices = ", ".join(ENDLESS_MODES)
        raise ValueError(f"{ENDLESS_MODE_ENV}={raw_mode!r} 无效，可选值: {choices}")
    return mode


def resolve_endless_intake(
    environ: Mapping[str, str] | None = None,
    *,
    totes_on_conveyor: bool,
    props_mode: str = "layout",
) -> EndlessIntakeConfig:
    """按开关与布局解析弯道/支线是否生成。

    ``auto``（默认）与 ``on`` 都请求生成，``off`` 关闭；非法值 ``ValueError``。
    请求生成还要过布局门（模块 docstring 里的两处占位冲突）：

    * ``totes_on_conveyor=False``（=0 推车布局）→ 不生成；
    * ``props_mode="legacy_props"`` → 不生成（空车 pushcart_2 在弯道足迹里）。

    被门拦下时 ``disabled_reason`` 说明原因，启动日志照打，排障不用猜。
    """

    values = os.environ if environ is None else environ
    mode = resolve_endless_mode(values)
    requested = mode != "off"

    if not requested:
        reason = f"{ENDLESS_MODE_ENV}=off"
        return EndlessIntakeConfig(mode=mode, requested=False, enabled=False, disabled_reason=reason)
    if not totes_on_conveyor:
        reason = "=0 推车布局：作业组 (-5.62, 18.75) 约 83% 落在弯道占位 x[-7.113,-5.042] y[18.207,20.308] 内"
        return EndlessIntakeConfig(mode=mode, requested=True, enabled=False, disabled_reason=reason)
    if str(props_mode).strip().lower() == "legacy_props":
        reason = "legacy_props 回退：空车 pushcart_2 (-5.4, 19.39363) 约 99% 落在弯道占位内"
        return EndlessIntakeConfig(mode=mode, requested=True, enabled=False, disabled_reason=reason)
    return EndlessIntakeConfig(mode=mode, requested=True, enabled=True, disabled_reason="")
