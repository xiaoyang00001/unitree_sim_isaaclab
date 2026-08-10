# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""看不到头的流水线入料端（90° 西拐弯道 + X 支线，复用已有货架）——零依赖解析器。

安检机遮罩方案（shroud_config，已拆除）被否决后，入料端改为：

* 主线带头（Δ=0.25 实测公头面 y=18.4723）接一段 DigitalTwin **ConveyorBelt_A02**
  低位滚筒弯道向西（-X）拐 90°——A02 是全资产包唯一与 A08 同高同构造的弯道
  （滚筒顶 76.93 cm、连续侧挡边 79.82 cm、门柱 116.63 cm 全部同族）；
* 弯道西口（公头）插进 **ConveyorBelt_A05** 2 m 短直段的母口，共**五段** A05
  接成 X 支线一路向西，机身从仓库背景 v61 西侧满载货架排 B（y[18.269,19.346]，
  x 连排到 -29.58）**北侧擦过去**（Δ=0.25 整体北移换来的通道：支线机身南缘
  19.4779 对排 B 端护板北缘 19.3946 净距 83.3 mm），端头伸到排 B/叉车后面；
* **不新增任何货架/遮挡件**（用户硬要求）：遮挡只依赖背景既有结构。出生/回生
  点深藏在 s=-3.90（箱心 (-16.66, 20.0534)），遮挡结论见下方「遮挡核算」。

⚠️ 本模块的坐标常量是**绝对世界坐标**（Δ=0.25 已计入），刻意不 import
conveyor_drive 的 Δ（零依赖单文件可测是设计目标，本地再抄 Δ 又违反"Δ 抄本只有
两处"的承诺）；Δ 关系锁死在 tests/test_conveyor_endless_intake.py 的交叉断言
里——整体平移改漏任何一处，测试即红。

开关 ``ISAACLAB_CONVEYOR_ENDLESS``：``auto``（默认，=开）| ``on`` | ``off``；
非法值直接 ``ValueError``。只在流水线布局（``ISAACLAB_TOTES_ON_CONVEYOR=1``）
且道具策略为 ``layout`` 时真正生成：

* =0 推车布局的拖车组 (-5.62, 19.0)（AABB x[-6.09,-5.15] y[18.18,19.82]，随 Δ
  同移）约 83% 落在弯道占位 x[-7.113,-5.042] y[18.457,20.558] 内，冲突不可微调
  回避（挪拖车会破坏 =0 "离带头 0.118 m" 的作业约定）；
* ``legacy_props`` 回退在 =1 下把空车 pushcart_2 摆在 (-5.4, 19.64363)，
  约 99% 包含于弯道占位——所以生成门是 "totes_on_conveyor 且 props==layout"，
  不能只看 =1/=0。

本模块刻意零 Isaac import、零相对 import（普通 unittest 用
``importlib.util.spec_from_file_location`` 单文件加载即可验证开关与几何算术）。
资产路径只给相对段，由 conveyor_env_cfg 拼 isaaclab.utils.assets 的
``NVIDIA_NUCLEUS_DIR``（本机 resolver 重定向到本地资产包）。

坐标全部是世界系、米。数值来自离线 pxr 实测链（A02/A05 局部 bbox 逐件量过，
弯道摆位由主线带头公头实测中心 x=-5617.0 mm、带头面 y=18472.3 mm 推出），
不是名义值；与 conveyor_drive 常量的关系由 tests/test_conveyor_endless_intake.py
交叉断言。

遮挡核算（Δ=0.25，mesh 级射线；眼位 E1=(-4.54,14.698,1.6)/E2=(-6.7,14.698,1.6)，
箱顶最高 c01=1.0223）：

* **E2（robot_2 侧）**：回生点按"东上角"判据**全遮**（顶面 25 点 + 四顶角 +
  顶心全部被排 B 首层架板板面实体挡住，遮挡源稳健）；全遮边界 s≤-3.81，
  RESPAWN_S=-3.90 留 90 mm 裕量。残余=东立面中段 z≈0.86-0.94 有 4/20 点经
  架板下缝可见。
* **E1（robot_1 侧）几何无解**：mesh 级复测发现排 B 首层**不是实心遮挡体**——
  地面货堆顶 z=0.960 与架板板面底 z=1.176 之间有 216 mm 的**通视水平缝**贯穿
  排 B 全长（下弦梁只在架板南北沿），E1 视线在排 B 北面出口高度 z≈1.099 恰
  落在缝内：加段数、更深、换矮箱 d01 都无效。E1 残余=箱顶条带经缝可见（顶面
  68%、距 13.25 m、缝的角高约 1.2°），底角 4/4 全遮。
* 四段方案已核算否决：E2 全遮要求箱心 x≤-16.57，第 4 段极限 -14.79 差 1.78 m；
  且 4 段回生点从 E1 完全裸露——**必须五段**。
* 主线/弧上 slot 维持刻意揭示（箱底沿被 0.801 挡边遮，"绕弯而来"）。
相比旧三段方案（回生点两眼全裸露、距 8-9.6 m）是大幅弱化；"双眼全遮"严格目标
只达成 E2，E1 缝隙残余记录在 README 已知限制。
"""

from __future__ import annotations

import math
import os
from collections.abc import Mapping
from dataclasses import dataclass


ENDLESS_MODE_ENV = "ISAACLAB_CONVEYOR_ENDLESS"
DEFAULT_ENDLESS_MODE = "auto"
ENDLESS_MODES = ("auto", "on", "off")

# 主线带头（Δ=0.25）：公头面世界 y（=实测 18.2223+Δ）与公头中心世界 x。
# ⚠️ 与 conveyor_drive.BELT_Y_MAX（碰撞板常量 18.47）差 2.3 mm——那是板的收口
#    取整，这里插接必须用实测面。
BELT_HEAD_FACE_Y = 18.4723
BELT_HEAD_MALE_CENTER_X = -5.617
PLUG_DEPTH = 0.015  # 公插母 15 mm（族内先例最多 119 mm；对顶才只能留 3 mm 缝）

# ------------------------------------------------------------------
# 弯道 ConveyorBelt_A02（2.1007×2.0714×1.1663 m，裸厘米资产 ⇒ UsdFileCfg.scale=0.01）
#
# 摆位推导（mm，世界系；yaw=-90° 即绕 Z 顺时针 90°，局部 (x,y) → 世界 (y, -x)）：
# * A02 默认（yaw=0）连通 南(公 y=0 面) + 东(母 x=2097.5 面)；转 -90° 后变
#   西(公) + 南(母)——南向母口正好**插接**主线带头公头（母套公 15 mm 正常插接，
#   比东拐方案的公对公对顶 3 mm 缝更顺，无断口转接框问题）；
# * y：南向母口面（局部 x=2097.5）落在 18472.3-15.0=18457.3 ⇒ t.y=20554.8；
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
CURVE_POS = (-7.1129, 20.5548, 0.003)
CURVE_YAW_DEG = -90.0
# 离线实测世界 AABB（z 上限 1.1693 是南口旁两根门柱；连续挡边只到 0.8012）。
CURVE_AABB = ((-7.1130, -5.0416), (18.4573, 20.5580), (0.003, 1.1693))
# 西向公头端面 x 与西口车道中线 y（=t.y-501.4mm）。
CURVE_WEST_MALE_TIP_X = -7.1129
CURVE_WEST_LANE_Y = 20.0534

# ------------------------------------------------------------------
# X 支线 ConveyorBelt_A05 × 5（2000.1 mm 短直段，与 A08 同宽同高同滚筒构造）
#
# yaw=-90° 后母口朝东（受上游公头插入）、公头朝西（伸向下一段/自由端），传动
# 胶带条（局部 x<0 侧悬出 74.05 mm）甩到北侧背对工位。段 1 母口东端面
# （局部 y=0 ⇒ 世界 x=t.x）盖过弯道西公头尖 15.0 mm ⇒ t.x=-7112.9+15.0=-7097.9；
# 段间同样公插母 15.0 mm，每段净延伸 2000.1-15.0=1985.1 mm。
# 车道中线全部与弯道西口一致：y = t.y-501.4 = 20053.4。
# 五段=遮挡最小段数（E2 全遮要求箱心 x≤-16.57，第 4 段极限 -14.79 差 1.78 m
# 不可达——见模块 docstring「遮挡核算」），Δ=0.25 北移后段 4/5 从排 B 北侧
# 擦过（机身南缘对端护板北缘净距 83.3 mm），不再受端护板东缘硬限。
# ------------------------------------------------------------------
XLEG_ASSET_NVIDIA_RELPATH = (
    "Assets/DigitalTwin/Assets/Warehouse/Equipment/Conveyors/"
    "ConveyorBelt_A/ConveyorBelt_A05_PR_NVD_01.usd"
)
XLEG_COUNT = 5
XLEG_YAW_DEG = -90.0
XLEG_POSITIONS = (
    (-7.0979, 20.5548, 0.003),
    (-9.0830, 20.5548, 0.003),
    (-11.0681, 20.5548, 0.003),
    (-13.0532, 20.5548, 0.003),
    (-15.0383, 20.5548, 0.003),
)
# 离线实测世界 AABB（南缘 19.4779 = 车道侧挡边；北缘 20.6289 = 传动胶带条悬出）。
XLEG_AABBS = (
    ((-9.0979, -7.0978), (19.4779, 20.6289), (0.003, 1.1693)),
    ((-11.0830, -9.0829), (19.4779, 20.6289), (0.003, 1.1693)),
    ((-13.0681, -11.0680), (19.4779, 20.6289), (0.003, 1.1693)),
    ((-15.0532, -13.0531), (19.4779, 20.6289), (0.003, 1.1693)),
    ((-17.0383, -15.0382), (19.4779, 20.6289), (0.003, 1.1693)),
)
# 每段滚筒可用带（局部实测反推）；段间断口 ≈41 mm 由 kinematic 托面补齐。
XLEG_ROLLER_X_RANGES = (
    (-9.0938, -7.1066),
    (-11.0789, -9.0917),
    (-13.0640, -11.0768),
    (-15.0491, -13.0619),
    (-17.0342, -15.0470),
)
BRANCH_ROLLER_WEST_END_X = XLEG_ROLLER_X_RANGES[-1][0]  # -17.0342

# 周边既有结构（世界系、离线实测，**不随 Δ 平移**——货架/墙/锥/叉车都不动）：
# * 背景 v61 西侧货架排 B（前左）：y[18.269,19.346]、货物最东 -13.768、
#   钢架最东立柱东缘 -13.520、端护板东缘 -13.386；mesh 级复测（本轮）：
#   端护板 Rackshield_59 北缘 y=19.3946（x[-13.734,-13.386] z[0,0.5]）是支线
#   南侧最近物，架板北缘 19.3457、钢架柱北缘 19.3073 更靠南；
#   支线机身南缘 19.4779 从其北侧擦过（净距 83.3/132.2/170.6 mm）。
# * 排 B 首层 mesh 级**不实心**：地面货堆顶 z=0.960 与架板板面底 z=1.176 之间
#   有 216 mm 通视水平缝贯穿全长（E1 遮挡无解的根因，见模块 docstring）。
# * 叉车 /Root/forklift：整体 AABB x[-16.087,-14.873] y[15.826,19.321]；北缘
#   19.321 是**货叉尖**（z≤1.049），车体北缘只到 18.140。与段 5 x 重叠段的
#   y 净距 156.9 mm（对货叉尖）/1338 mm（对车体）。
# * +Y 落地墙 SM_WallA_6M16/17 南面 23.606：支线北缘 20.629 留 2.977 m、弯道
#   北缘 20.558 留 3.048 m；
# * 交通锥 Cone_4 x[-11.07,-10.74] y[21.23,21.56]：与段 3 x 几乎相切但 y 净距
#   0.597 m；Cone_3 在弯道南侧，均无碰撞。
# * 柱 SM_PillarPartA_9M10_1473（视觉上像挡在 x≈-13.7 的 9m 大件）：落地柱身
#   只在 y[21.996,23.906]（墙边），y<20.9 部分全在 z≥7.958（屋面斜梁），支线
#   走廊无碰撞——wrapper AABB 审计工具按包围盒判会误报，注意。
RACK_B_GUARD_EAST_X = -13.386
RACK_B_STEEL_EAST_X = -13.520
RACK_B_NORTH_Y = 19.346
RACK_B_GUARD_NORTH_Y = 19.3946
RACK_B_Y_RANGE = (18.269, 19.346)
RACK_B_FIRST_TIER_SEAM_Z = (0.960, 1.176)  # 216 mm 通视缝（货堆顶↔架板底）
FORKLIFT_X_RANGE = (-16.087, -14.873)
FORKLIFT_FORK_NORTH_Y = 19.321  # 货叉尖（矮件 z≤1.049）；车体北缘 18.140
WALL_FACE_Y = 23.606
CONE_4_AABB = ((-11.07, -10.74), (21.23, 21.56))

# ------------------------------------------------------------------
# 驱动路径（沿路径距离 s 参数化；conveyor_queue 有等价的 torch 版）
#
#   s=0 锚点取支线 x=-12.76（车道中线 y=20.0534）；s 向下游递增：
#   ① 支线段 s∈[S_MIN, S_ARC_START]：pos=(S_ORIGIN_X+s, 20.0534)，航向 +X
#      ——⚠️ 与东拐存档镜像：这里 +s 对应 +X；
#   ② 圆角弧 s∈[S_ARC_START, S_ARC_END]：圆心 (-7.0200, 18.6534)、R=1.40，
#      θ=(s-S_ARC_START)/R∈[0,π/2]，pos=(cx+R·sinθ, cy+R·cosθ)，
#      航向 (cosθ,-sinθ)；直角尖拐点在滚筒外缘以外（东拐已验证同族事实），
#      必须切 R=1.4 圆角（与 A02 资产真实弧线 R≈1.40-1.45 吻合）；
#   ③ 主线段 s>S_ARC_END：pos=(-5.62, 18.6534-(s-S_ARC_END))，航向 -Y。
#
# 圆心由车道几何导出：cx = 主车道中线 − R（东拐是 +R），cy = 支线车道中线 − R。
# ⚠️ s 参数化对 Δ 整体平移**不变**（cy 与主线上任意目标 y 同加抵消）：工位 s、
#    停止线 s、S_LEAD 在 Δ 变更时全部零改动。
# ------------------------------------------------------------------
MAIN_LANE_X = -5.62          # = conveyor_drive.BELT_X_CENTER（交叉断言）
LANE_HALF_WIDTH = 0.45       # = conveyor_drive.BELT_WIDTH / 2（交叉断言）
BRANCH_LANE_Y = 20.0534      # 弯道西口车道中线（离线实测；= CURVE_WEST_LANE_Y）
CORNER_RADIUS = 1.40
CORNER_CENTER = (MAIN_LANE_X - CORNER_RADIUS, BRANCH_LANE_Y - CORNER_RADIUS)
S_ORIGIN_X = -12.76
S_ARC_START = CORNER_CENTER[0] - S_ORIGIN_X              # 5.7400
S_ARC_END = S_ARC_START + CORNER_RADIUS * math.pi / 2.0  # ≈7.93911
# 支线滚筒可用端（段 5 西端 -17.0342 ⇒ s=-4.2742，取 -4.27）：出生/排队不得越过。
PATH_S_MIN = -4.27

# 循环模式（y_stop<=0）的回生落点=队尾深藏槽位：s=-3.90 ⇒ (-16.66, 20.0534)，
# 藏在排 B（近端还有叉车）后面。E2 眼位"东上角"判据全遮（边界 s≤-3.81，留 90 mm
# 裕量；遮挡源=排 B 首层架板板面实体，稳健）；E1 眼位受排 B 首层 216 mm 通视缝
# 影响**无全遮解**（见模块 docstring「遮挡核算」）——README 已知限制。
# 回生箱西缘 -16.91 距段 5 滚筒西端 -17.0342 余 124 mm（c01 半长 0.25 口径）。
RESPAWN_S = -3.90
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
#   南缘接主线碰撞板末端 18.47，整块落在弯道足迹内；
# * 支线条带：车道中线 ±0.45，东接拐角补块、西到段 5 滚筒可用端（西端延长
#   3970.2 mm，段间 ≈41 mm 滚筒断口照旧一并补齐）。
# 板厚/顶面与主线一致（顶 z=0.772，厚 0.04）。
# ⚠️ 条带南缘 19.6034 与排 B 端护板北缘 19.3946 现在 x 区间**有重叠**（条带西伸
#   到 -17.034，护板在 -13.734..-13.386），但 y 净距 208.8 mm 充裕——旧版
#   "7 mm 错缝险胜"的问题随 Δ=0.25 北移消失，离线定量核过，无碰撞。
# ------------------------------------------------------------------
BELT_TOP_Z = 0.772           # = conveyor_drive.BELT_TOP_Z（交叉断言）
COLLIDER_THICKNESS = 0.04    # = conveyor_drive.BELT_COLLIDER_THICKNESS（交叉断言）
CORNER_PLATE_X_RANGE = (CORNER_CENTER[0], MAIN_LANE_X + LANE_HALF_WIDTH)   # (-7.02, -5.17)
CORNER_PLATE_Y_RANGE = (18.47, BRANCH_LANE_Y + LANE_HALF_WIDTH)            # (18.47, 20.5034)
BRANCH_PLATE_X_RANGE = (BRANCH_ROLLER_WEST_END_X, CORNER_CENTER[0])        # (-17.0342, -7.02)
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
        reason = "=0 推车布局：作业组 (-5.62, 19.0) 约 83% 落在弯道占位 x[-7.113,-5.042] y[18.457,20.558] 内"
        return EndlessIntakeConfig(mode=mode, requested=True, enabled=False, disabled_reason=reason)
    if str(props_mode).strip().lower() == "legacy_props":
        reason = "legacy_props 回退：空车 pushcart_2 (-5.4, 19.64363) 约 99% 落在弯道占位内"
        return EndlessIntakeConfig(mode=mode, requested=True, enabled=False, disabled_reason=reason)
    return EndlessIntakeConfig(mode=mode, requested=True, enabled=True, disabled_reason="")
