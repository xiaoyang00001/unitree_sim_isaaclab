"""Dependency-free resolver for the two conveyor scene layouts.

Keeping the layout arithmetic outside ``conveyor_env_cfg`` lets ordinary Python
tests verify the environment-variable switch without importing Isaac Lab.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from typing import Mapping


# 流水线整体北移量（世界 +Y，m）。真源是 conveyor_drive.CONVEYOR_NORTH_SHIFT_Y，
# 当前 Δ=0.25：X 支线越过货架排 B 北侧所需的最小北移（推导见真源注释——支线
# 机身南缘对排 B 端护板北缘留 83.3 mm，支线因此可延到 5 段、把出生/回生点藏进
# 货架排 B 后面）。本模块刻意零相对 import（测试用单文件加载），所以抄一份；
# tests/test_conveyor_scene_layout.py 交叉断言两边一致，别只改一边。
CONVEYOR_NORTH_SHIFT_Y = 0.25

# 纯显示站位机器人使用与现有两台相同的 SONIC 默认姿态。该姿态烘焙后的完整
# G1 网格包围盒 min z=-0.757501582；沿用工作机器人 root z=0.76 时脚底离地
# 约 2.5 mm，视觉贴地且 pelvis 高度一致。
STANDBY_ROBOT_ROOT_Z = 0.76


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


def _env_int(environ: Mapping[str, str], name: str, default: int) -> int:
    value = environ.get(name)
    if value is None or not str(value).strip():
        return default
    try:
        return int(str(value).strip())
    except ValueError:
        return default


@dataclass(frozen=True)
class BeltBoxKind:
    """一种流水线箱型：视觉资产 + 缩放后的世界尺寸 + 质量。

    ``length_y`` 是**摆上带面后沿输送方向 Y 的长度**。箱子保持资产原始朝向，
    让较短的资产 Y 边沿输送方向，机器人从流水线侧面抱取时双臂无需跨过长边；
    因此 ``length_y`` 取视觉资产的 Y 尺寸、``width_x`` 取 X 尺寸。原点都在箱底面，
    出生 z 因此可以直接用带面高度。
    """

    key: str
    asset: str
    length_y: float
    width_x: float
    height_z: float
    mass: float

    @property
    def half_length_y(self) -> float:
        return self.length_y * 0.5

    @property
    def half_queue_extent(self) -> float:
        """固定世界朝向下用于 L 形路径排队的半长保守值。"""

        # 主线沿 Y、支线沿 X，而箱子拐弯时不随路径旋转；取两轴较大的半尺寸，
        # 避免为了缩短机器人抱取方向的 Y 跨距，反而低估支线上的占用长度。
        return max(self.length_y, self.width_x) * 0.5


# C01/C02 与 D01~D05 是 Simple Warehouse 里真实存在的 canonical 纸箱资产；场景
# 树中的 C03~C09、D06~D11 只是复用这些网格的别名，不重复登记。每个登记项都用
# 任务级平底碰撞 wrapper，避免官方视觉 mesh 自带的 triangle collision 被动态刚体
# 退化处理。软包裹三种取自 IsaacLab 分叉 feat/pickplace-parcel-assets 的
# 程序化快递袋资产（塑料袋装衣服：枕形鼓包+热封边+顶面白色面单）。⚠️ 软包裹是
# **刚体不是软体**——"软"只是视觉造型，物理上与纸箱同一套约定（根挂 RigidBody+Mass、
# convexHull 碰撞、原点在袋底），所以能直接进队列/同步/镜像分流，CPU pipeline 可用。
BELT_BOX_KINDS = {
    "d01": BeltBoxKind(
        key="d01",
        asset="cart_box_d01_physics.usda",
        length_y=0.25,
        width_x=0.38,
        height_z=0.1487,
        mass=0.1,
    ),
    "d02": BeltBoxKind(
        key="d02",
        asset="cart_box_d02_physics.usda",
        length_y=0.25,
        width_x=0.38,
        height_z=0.1663,
        mass=0.1,
    ),
    "c01": BeltBoxKind(
        key="c01",
        asset="cart_box_c01_physics.usda",
        length_y=0.50,
        width_x=0.50,
        height_z=0.25,
        mass=1.5,
    ),
    "c02": BeltBoxKind(
        key="c02",
        asset="cart_box_c02_physics.usda",
        length_y=0.50,
        width_x=0.50,
        height_z=0.311,
        mass=1.5,
    ),
    "d03": BeltBoxKind(
        key="d03",
        asset="cart_box_d03_physics.usda",
        length_y=0.2606,
        width_x=0.4126,
        height_z=0.2656,
        mass=1.5,
    ),
    "d04": BeltBoxKind(
        key="d04",
        asset="cart_box_d04_physics.usda",
        length_y=0.25,
        width_x=0.38,
        height_z=0.149,
        mass=1.5,
    ),
    "d05": BeltBoxKind(
        key="d05",
        asset="cart_box_d05_physics.usda",
        length_y=0.25,
        width_x=0.38,
        height_z=0.149,
        mass=1.5,
    ),
    # 尺寸取实测组合包围盒（生成器标称 40×30 / 45×35 / 32×24 cm，鼓包略溢出）。
    "parcel_a01": BeltBoxKind(
        key="parcel_a01",
        asset="parcel_soft_a01.usda",
        length_y=0.3013,
        width_x=0.4017,
        height_z=0.0801,
        mass=0.35,
    ),
    "parcel_a02": BeltBoxKind(
        key="parcel_a02",
        asset="parcel_soft_a02.usda",
        length_y=0.3520,
        width_x=0.4526,
        height_z=0.0968,
        mass=0.5,
    ),
    "parcel_a03": BeltBoxKind(
        key="parcel_a03",
        asset="parcel_soft_a03.usda",
        length_y=0.2414,
        width_x=0.3218,
        height_z=0.0603,
        mass=0.22,
    ),
}
# 默认队首永远保持 D01 + 压皱 D02；后 15 件从全部十种箱/包中做带 seed 的
# 平衡随机：每轮先让池中每种资产各出现一次，再进入下一轮，避免纯抽样漏掉某种
# 外观。排序只用 SHA-256，不依赖 Python hash / 全局 random / 系统熵，所以 Linux
# 权威端与 Windows viewer 在相同 seed 下会得到逐槽一致的 USD。
BELT_BOX_FIXED_FRONT = ("d01", "d02")
DEFAULT_BELT_BOX_RANDOM_POOL = (
    "d01",
    "d02",
    "c01",
    "c02",
    "d03",
    "d04",
    "d05",
    "parcel_a01",
    "parcel_a02",
    "parcel_a03",
)
DEFAULT_BELT_BOX_RANDOM_SEED = "20260811"
BELT_BOX_RANDOM_SEED_ENV = "ISAACLAB_BELT_BOX_RANDOM_SEED"


def _stable_shuffled_cycle(seed: str, cycle: int) -> list[str]:
    """跨进程/平台稳定地打乱一轮默认资产池。"""

    decorated = []
    for index, key in enumerate(DEFAULT_BELT_BOX_RANDOM_POOL):
        token = f"belt-box-v1\0{seed}\0{cycle}\0{index}\0{key}".encode("utf-8")
        decorated.append((hashlib.sha256(token).digest(), index, key))
    return [key for _digest, _index, key in sorted(decorated)]


def _default_belt_box_keys(seed: str, count: int = 17) -> tuple[str, ...]:
    """固定队首 + 平衡随机尾队列；先生成完整队列再截前缀。"""

    keys = list(BELT_BOX_FIXED_FRONT[:count])
    cycle = 0
    while len(keys) < count:
        shuffled = _stable_shuffled_cycle(seed, cycle)
        # 两轮边界或固定队首边界若刚好同型，稳定交换本轮前两个不同项，避免视觉连号。
        if keys and shuffled[0] == keys[-1]:
            swap_index = next(index for index, key in enumerate(shuffled[1:], 1) if key != keys[-1])
            shuffled[0], shuffled[swap_index] = shuffled[swap_index], shuffled[0]
        keys.extend(shuffled[: count - len(keys)])
        cycle += 1
    return tuple(keys)


DEFAULT_BELT_BOX_PATTERN = _default_belt_box_keys(DEFAULT_BELT_BOX_RANDOM_SEED)

# 带面几何与 conveyor_drive 的常量保持一致（那边是驱动/分段的真源，这里只用来
# 定位出生点；两处数值若要改必须同时改）。y 值必须过 _shifted（Δ 非零时自动
# 跟随）：当前 Δ=0.25，带面入料端在 18.47。
BELT_BOX_LANE_X = -5.62
BELT_BOX_SPAWN_Z = 0.775
BELT_BOX_BELT_Y_MAX = _shifted(18.22)
BELT_BOX_BELT_WIDTH = 0.90

# 队首两箱（belt_box_1/2）不摆在带面正中线：两台机器人沿 Y 错开后，第一箱偏东
# 贴近 robot_1，第二箱偏西贴近上游的 robot_2，一台一箱都能从近侧直接
# 抱（2026-08-10 用户反馈）。偏移量 0.20 m：默认队首固定 d01/d02，带半宽
# 0.45 − 箱半宽 0.19 − 偏移 0.20 = 0.06 m，仍留 60 mm 到带边；显式 PATTERN
# 若把更宽的 C 型箱放到队首，下面仍会按真实半宽校验（0.50 m C 型恰好贴边，
# 因此不作为默认队首）。带面驱动判据
# on_belt_mask 的 x_range 半宽 0.55、主线段 s 只按 y 取值（见 conveyor_queue.
# path_progress / endless_intake.path_s_of_point），偏移量在直线段上恒定，
# 不会被驱动逻辑纠偏回中线，也不影响排队/防撞判据（那些只看 s，s 只看 y）。
# 只对落在主线直段的槽位生效——弧段/支线段的 x 是路径方向不是横向偏移，
# 见 resolve_belt_box_positions 里按 slot 是否越过 s_arc_end 传的 guard。
BELT_BOX_FRONT_X_OFFSET = 0.20

# SceneCfg 里的 belt_box_N 字段是显式声明的（configclass 需要类属性），所以数量
# 有硬上限。调大必须同时在 conveyor_env_cfg.G129SonicConveyorSceneCfg 里补字段。
BELT_BOX_MAX_COUNT = 17

# ------------------------------------------------------------------
# 入口弯道（endless intake，**西拐**）出生路径：真源是 endless_intake.py 的路径
# 常量，本模块零相对 import（测试用单文件加载）所以抄一份最小集；
# tests/test_conveyor_scene_layout.py 交叉断言出生点 = endless_intake.path_point。
#
#   s=0 锚点 x=-12.76（支线车道中线 y=20.0534）；支线段 s∈[-4.27, 5.74] 沿 +X；
#   圆角弧 s∈[5.74, ~7.939]（圆心 (-5.62-1.4, 20.0534-1.4)、R=1.4）；
#   主线段 s>~7.939 沿 -Y（工位/停止线都在这段上）。
#   s 参数化对 Δ 整体平移不变（圆心 cy 与工位 y 同加抵消），所以工位 s、停止
#   线 s、S_LEAD 全部零改动——这是"整体北移"方案最省事的性质。
# ------------------------------------------------------------------
BELT_BOX_BRANCH_LANE_Y = 20.0534
BELT_BOX_CORNER_RADIUS = 1.40
BELT_BOX_PATH_S_ORIGIN_X = -12.76
# 支线滚筒可用端（段 5 西端 x=-17.0342 ⇒ s=-4.2742，取 -4.27）：队尾不得越过。
BELT_BOX_PATH_S_MIN = -4.27
# 默认队首 s=11.06：落在主线 y≈15.533，距工位停止点约 1.135 m。
BELT_BOX_DEFAULT_S_LEAD = 11.06
# 直线回退形态（endless off / legacy_props）的默认箱数：上游带面只有 ~4.07 m，
# 17 箱队列依赖弯道路径的额外容量，直排默认维持历史 5 箱（显式 COUNT 对两种
# 形态都生效，直排给大了会被"悬出带面"fail-fast 拦住）。
BELT_BOX_STRAIGHT_DEFAULT_COUNT = 5
# 默认出生**显式槽位序列**（沿路径距离 s，队首→上游）：以队首 s=11.06 为锚，
# pitch 0.75 排成主线 5 + 弧上 3 + X 支线 9，共 17 箱/包。
# 队尾槽位 s = 11.06 − 0.75×16 = -0.94，逐项核算：
# * 对支线滚筒可用端 PATH_S_MIN=-4.27 的净距（d01/d02 路径半长 0.19 口径）
#   = -0.94 − 0.19 − (-4.27) = 3.14 m；
# * endless_intake.RESPAWN_S=-3.90 是循环模式独立使用的回生点，比默认出生队尾
#   再向上游 2.96 m，不随这组出生槽位改变。
# 显式给出 S_LEAD / Y_LEAD / PITCH 任一环境变量时回退等距排布。
# COUNT<17 时取序列前缀（从队首往上游数；队尾不再有单独的深藏特例）。
BELT_BOX_DEFAULT_SLOT_S = tuple(
    round(BELT_BOX_DEFAULT_S_LEAD - 0.75 * index, 6) for index in range(BELT_BOX_MAX_COUNT)
)


def _endless_intake_active(environ: Mapping[str, str], totes_on_conveyor: bool) -> bool:
    """弯道出生路径是否生效——判据抄 endless_intake.resolve_endless_intake。

    合法值校验以 endless_intake 为准（env cfg 在 import 时先解析它，非法值先在
    那边 ValueError）；这里对未知值宽松按 auto 处理，避免两处各自报错口径分叉。
    """

    mode = str(environ.get("ISAACLAB_CONVEYOR_ENDLESS", "auto")).strip().lower() or "auto"
    if mode == "off" or not totes_on_conveyor:
        return False
    props = str(environ.get("ISAACLAB_CONVEYOR_PROPS", "layout")).strip().lower() or "layout"
    return props != "legacy_props"


def _path_point(s: float) -> tuple[float, float]:
    """沿路径距离 s → 世界 (x, y)。与 endless_intake.path_point 同式（测试交叉断言）。"""

    corner_x = BELT_BOX_LANE_X - BELT_BOX_CORNER_RADIUS
    corner_y = BELT_BOX_BRANCH_LANE_Y - BELT_BOX_CORNER_RADIUS
    s_arc_start = corner_x - BELT_BOX_PATH_S_ORIGIN_X
    s_arc_end = s_arc_start + BELT_BOX_CORNER_RADIUS * math.pi / 2.0
    if s <= s_arc_start:
        return (BELT_BOX_PATH_S_ORIGIN_X + s, BELT_BOX_BRANCH_LANE_Y)
    if s >= s_arc_end:
        return (BELT_BOX_LANE_X, corner_y - (s - s_arc_end))
    theta = (s - s_arc_start) / BELT_BOX_CORNER_RADIUS
    return (
        corner_x + BELT_BOX_CORNER_RADIUS * math.sin(theta),
        corner_y + BELT_BOX_CORNER_RADIUS * math.cos(theta),
    )


def _path_s_of_main_y(y: float) -> float:
    """主线段世界 y → s（工位换算用）。"""

    corner_x = BELT_BOX_LANE_X - BELT_BOX_CORNER_RADIUS
    corner_y = BELT_BOX_BRANCH_LANE_Y - BELT_BOX_CORNER_RADIUS
    s_arc_start = corner_x - BELT_BOX_PATH_S_ORIGIN_X
    return s_arc_start + BELT_BOX_CORNER_RADIUS * math.pi / 2.0 + (corner_y - y)


@dataclass(frozen=True)
class StandbyRobotPose:
    """One physics-free G1 pose in the conveyor workcell."""

    pos: tuple[float, float, float]
    # 当前项目的 Isaac Lab InitialStateCfg 约定为 (w, x, y, z)。
    rot: tuple[float, float, float, float]


@dataclass(frozen=True)
class ConveyorSceneLayout:
    """Resolved positions and scale shared by all conveyor scene assets."""

    totes_on_conveyor: bool
    cart_group_x: float
    cart_group_y: float
    robot_side_offset: float
    robot_workstation_y: float
    robot_2_workstation_y: float
    robot_1_x: float
    robot_2_x: float
    standby_robot_poses: tuple[StandbyRobotPose, ...]
    pushcart_2_pos: tuple[float, float, float]
    cart2_tote1_pos: tuple[float, float, float]
    cart2_tote2_pos: tuple[float, float, float]
    tote_scale: tuple[float, float, float]
    conveyor_y_stop: float
    belt_box_count: int
    belt_box_positions: tuple[tuple[float, float, float], ...]
    belt_box_kinds: tuple[BeltBoxKind, ...]
    belt_box_queue_gap: float

    @property
    def belt_box_names(self) -> tuple[str, ...]:
        return tuple(f"belt_box_{index + 1}" for index in range(self.belt_box_count))

    @property
    def belt_box_half_lengths(self) -> tuple[float, ...]:
        """每个箱子用于 L 形路径排队的保守半长——防撞判据按它算净间隙。"""

        return tuple(kind.half_queue_extent for kind in self.belt_box_kinds)


def resolve_belt_box_pattern(environ: Mapping[str, str], count: int) -> tuple[BeltBoxKind, ...]:
    """解析每个位置的箱型。

    默认前两位固定 d01/d02，后面按 ``ISAACLAB_BELT_BOX_RANDOM_SEED`` 做可复现的
    平衡随机；始终先生成完整 17 项再取前缀，因此 COUNT 不会重排已有槽位。显式
    ``ISAACLAB_BELT_BOX_PATTERN`` 保留历史完整覆盖语义并优先于 seed：按给定
    pattern 从第 1 件开始循环，写单个 key 就是全用一种。未知 key 一律 fail-fast。
    """

    raw = environ.get("ISAACLAB_BELT_BOX_PATTERN")
    if raw is None or not str(raw).strip():
        seed = str(environ.get(BELT_BOX_RANDOM_SEED_ENV, DEFAULT_BELT_BOX_RANDOM_SEED)).strip()
        if not seed:
            seed = DEFAULT_BELT_BOX_RANDOM_SEED
        pattern = _default_belt_box_keys(seed, BELT_BOX_MAX_COUNT)
    else:
        pattern = tuple(item.strip().lower() for item in str(raw).split(",") if item.strip())
    if not pattern:
        raise ValueError("ISAACLAB_BELT_BOX_PATTERN 不能为空")

    unknown = [key for key in pattern if key not in BELT_BOX_KINDS]
    if unknown:
        choices = ", ".join(sorted(BELT_BOX_KINDS))
        raise ValueError(f"ISAACLAB_BELT_BOX_PATTERN 含未知箱型 {unknown}；可选值: {choices}")

    return tuple(BELT_BOX_KINDS[pattern[index % len(pattern)]] for index in range(count))


def _apply_front_two_x_offset(
    positions: tuple[tuple[float, float, float], ...],
    offset: float,
    guard: tuple[bool, ...],
) -> tuple[tuple[float, float, float], ...]:
    """队首两箱（下标 0/1）分别向东/西偏移 ``offset``，其余箱子不动。

    ``guard`` 逐箱开关：弯道形态下箱子若落在弧段/支线（那里 x 是路径方向，不是
    横向偏移），调用方传 False 跳过，避免破坏路径几何。
    """

    if offset <= 0.0 or not positions:
        return positions
    out = list(positions)
    for index, sign in enumerate((1.0, -1.0)):
        if index >= len(out) or not guard[index]:
            continue
        x, y, z = out[index]
        out[index] = (x + sign * offset, y, z)
    return tuple(out)


def resolve_belt_box_positions(
    environ: Mapping[str, str],
    *,
    y_stop: float,
) -> tuple[int, tuple[tuple[float, float, float], ...], tuple[BeltBoxKind, ...], float]:
    """把 N 个纸箱排在工位**上游**（默认沿西拐入口弯道路径，退化态沿主线直排）。

    ``belt_box_1`` 是队首（最靠下游、最先到工位），编号递增向上游排。默认队首两箱
    固定为 D01/D02，其余 15 件从十种纸箱/软包中做带 seed 的平衡随机；显式
    ``ISAACLAB_BELT_BOX_PATTERN`` 则保留完整覆盖语义。出生间距 ``spawn_pitch``
    只决定初始队形；停下来后的实际队距由**每个箱子自己的半长**加净间隙
    ``queue_gap`` 决定，因此 C 型大箱和软包混排时也使用各自真实占位。

    **弯道形态（endless intake 生效，默认）**：出生点按显式槽位序列
    ``BELT_BOX_DEFAULT_SLOT_S``（以队首 s=11.06 为锚、pitch 0.75 排成主线 5 +
    弧上 3 + X 支线 9 共 17 箱/包；队尾 s=-0.94，对支线可用端仍留 3.14 m
    净距）。显式给出
    ``S_LEAD``/``Y_LEAD``/``PITCH`` 任一旋钮时回退等距
    排布 ``s_lead − k·pitch``，pitch 语义是"沿路径距离间距"。
    队首覆盖优先级：``ISAACLAB_BELT_BOX_SPAWN_S_LEAD``（路径距离）>
    ``ISAACLAB_BELT_BOX_SPAWN_Y_LEAD``（仅接受主线段 y，自动换算成 s）> 默认。
    ``ISAACLAB_BELT_BOX_LANE_X`` 在弯道形态下**不生效**（x 由路径决定）。

    **直线形态（``ISAACLAB_CONVEYOR_ENDLESS=off`` / legacy_props）**：维持历史
    行为——工位 ``y_stop`` 把带面切成两段，箱子沿主车道直排在上游来料段。

    两种形态下出生位同时是**复位落点**（整场景复位写回 init_state），一次复位 =
    重新完整流一遍。非法配置一律 fail-fast：箱子排到带面/支线外、相邻箱子出生
    就互穿、队首压在工位上，都会在启动时暴露，而不是等到运行时看见箱子悬空/
    穿模再回头猜。
    """

    endless = _endless_intake_active(environ, totes_on_conveyor=True)
    # 弯道形态默认使用 17 槽；直线回退的上游带面只有 ~4.07 m，默认维持历史 5 箱
    # （legacy_props 也走直线分支，默认 17 会启动即"悬出带面"）。
    count = _env_int(
        environ,
        "ISAACLAB_BELT_BOX_COUNT",
        BELT_BOX_MAX_COUNT if endless else BELT_BOX_STRAIGHT_DEFAULT_COUNT,
    )
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
    spawn_pitch = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_PITCH", 0.75)
    queue_gap = _env_float(environ, "ISAACLAB_BELT_BOX_QUEUE_GAP", 0.07)

    kinds = resolve_belt_box_pattern(environ, count)
    if count == 0:
        return 0, (), (), queue_gap

    if queue_gap < 0.0:
        raise ValueError(f"ISAACLAB_BELT_BOX_QUEUE_GAP 不能为负，当前 {queue_gap}")

    # 箱子保持世界朝向，主线沿 Y、支线沿 X；排队/边界校验不能只取 Y 向短边。
    halves = [kind.half_queue_extent for kind in kinds]

    def _check_adjacent_clearance(gaps: list[float], what: str) -> None:
        # 相邻箱子出生就不能互穿：中心距至少是两个半长之和（直线/弯道形态同判据，
        # 弯道形态下间距是沿路径距离，弧段上的弦距略小于弧距，仍然安全——最小
        # 间距余量 0.75-0.44=0.31 远大于 R=1.4、θ≤0.54rad 下的弦弧差 <0.01）。
        for index, gap in enumerate(gaps):
            need = halves[index] + halves[index + 1]
            if gap < need:
                raise ValueError(
                    f"{what}={gap:g} 放不下相邻的 "
                    f"{kinds[index].key}/{kinds[index + 1].key}：至少要 {need:.3f}"
                )

    # 最宽的箱型也要放得进带面宽度。
    half_width = BELT_BOX_BELT_WIDTH * 0.5
    for kind in kinds:
        if kind.width_x * 0.5 > half_width:
            raise ValueError(
                f"箱型 {kind.key} 宽 {kind.width_x} 放不进带面宽度 {BELT_BOX_BELT_WIDTH}"
            )

    # 队首两箱额外偏移，也要各自放得进带面（偏移 + 半宽 不能压出带边）。
    if BELT_BOX_FRONT_X_OFFSET > 0.0:
        for index in range(min(2, count)):
            half_w = kinds[index].width_x * 0.5
            if BELT_BOX_FRONT_X_OFFSET + half_w > half_width:
                raise ValueError(
                    f"BELT_BOX_FRONT_X_OFFSET={BELT_BOX_FRONT_X_OFFSET:g} 加队首箱型 "
                    f"{kinds[index].key} 半宽 {half_w:.3f} 超出带面半宽 {half_width:.3f}"
                )

    if endless:
        # —— 弯道形态：沿路径距离 s 排队 ——
        corner_x = BELT_BOX_LANE_X - BELT_BOX_CORNER_RADIUS
        corner_y = BELT_BOX_BRANCH_LANE_Y - BELT_BOX_CORNER_RADIUS
        s_arc_end = (corner_x - BELT_BOX_PATH_S_ORIGIN_X) + BELT_BOX_CORNER_RADIUS * math.pi / 2.0
        if y_stop >= corner_y:
            raise ValueError(
                f"弯道形态要求工位在主线段上：y_stop={y_stop} 必须 < 拐角切线 {corner_y:.4f}"
            )
        s_stop = _path_s_of_main_y(y_stop)
        raw_s_lead = str(environ.get("ISAACLAB_BELT_BOX_SPAWN_S_LEAD", "")).strip()
        raw_y_lead = str(environ.get("ISAACLAB_BELT_BOX_SPAWN_Y_LEAD", "")).strip()
        raw_pitch = str(environ.get("ISAACLAB_BELT_BOX_SPAWN_PITCH", "")).strip()
        if raw_s_lead:
            s_lead = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_S_LEAD", BELT_BOX_DEFAULT_S_LEAD)
        elif raw_y_lead:
            y_lead = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD", 0.0)
            if y_lead >= corner_y:
                raise ValueError(
                    f"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD={y_lead} 不在主线段上（须 < {corner_y:.4f}）；"
                    "要把队首排上弧段/支线请改用 ISAACLAB_BELT_BOX_SPAWN_S_LEAD（沿路径距离）"
                )
            s_lead = _path_s_of_main_y(y_lead)
        else:
            s_lead = None

        if s_lead is None and not raw_pitch:
            # 默认：显式槽位序列——pitch 0.75 从队首向上游排列（见常量注释）。
            slots = list(BELT_BOX_DEFAULT_SLOT_S[:count])
        else:
            # 显式覆盖任一旋钮 ⇒ 回退等距排布，语义与历史一致。
            if s_lead is None:
                s_lead = BELT_BOX_DEFAULT_S_LEAD
            slots = [s_lead - spawn_pitch * index for index in range(count)]

        # 队首必须整体落在工位上游（s 向下游递增），队尾不能越过支线滚筒可用端。
        if slots[0] + halves[0] >= s_stop:
            raise ValueError(
                f"队首出生 s={slots[0]:.3f} 压在工位 s_stop={s_stop:.3f} 上；"
                f"最多到 {s_stop - halves[0]:.3f}"
            )
        if slots[-1] - halves[-1] < BELT_BOX_PATH_S_MIN:
            raise ValueError(
                f"{count} 个纸箱从 s={slots[0]:.3f} 排到 s={slots[-1]:.3f}，"
                f"队尾（{kinds[-1].key}）悬出支线带面（s_min={BELT_BOX_PATH_S_MIN}）；"
                "调小 pitch/count，或把 ISAACLAB_BELT_BOX_SPAWN_S_LEAD 往工位方向调大"
            )
        _check_adjacent_clearance(
            [slots[index] - slots[index + 1] for index in range(count - 1)],
            "出生槽位间距（ISAACLAB_BELT_BOX_SPAWN_PITCH）",
        )
        # 停稳后的队列不需要单独校验：整带节拍保持出生间距整体向下游平移，
        # 停稳队尾（s_stop − (slots[0]−slots[-1])）一定比出生队尾更靠下游。
        positions = tuple((*_path_point(s), spawn_z) for s in slots)
        # 队首两箱偏西/偏东——仅当它们落在主线直段（s >= s_arc_end）时才生效，
        # 弧段/支线上 x 是路径方向，偏移会破坏路径几何。
        front_guard = tuple(
            index < len(slots) and slots[index] >= s_arc_end for index in range(2)
        )
        positions = _apply_front_two_x_offset(positions, BELT_BOX_FRONT_X_OFFSET, front_guard)
        return count, positions, kinds, queue_gap

    # —— 直线形态（endless off / legacy_props）：沿主车道 y 直排 ——
    _check_adjacent_clearance(
        [spawn_pitch] * (count - 1), "ISAACLAB_BELT_BOX_SPAWN_PITCH"
    )
    # ⚠️ 出生间距同时**就是**停稳后的队列间距（整带节拍：所有箱子同起同停，相对
    # 位置恒定）。工位上游只有 18.47-14.398 ≈ 4.07 m，因此
    #     队列长度 (count-1)*pitch + 端部半长  +  队首行程 (y_lead - y_stop)  ≤ 4.07
    # 间距、行程、数量三者此消彼长；调大 pitch 必须同时下调 y_lead 或 count，
    # 否则下面的两条 fail-fast 会拦住。y_lead 是世界 y，走 _shifted 管线（Δ 非零
    # 时自动跟随，忘了会让队首出生在工位下游、启动即触发 fail-fast）。
    y_lead = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD", _shifted(14.95))

    # 队首必须整体落在工位上游，队尾不能悬出带尾。
    if y_lead - halves[0] <= y_stop:
        raise ValueError(
            f"ISAACLAB_BELT_BOX_SPAWN_Y_LEAD={y_lead} 让队首压在工位 y_stop={y_stop} 上；"
            f"至少要 {y_stop + halves[0]:.3f}"
        )
    y_tail = y_lead + spawn_pitch * (count - 1)
    if y_tail + halves[-1] > BELT_BOX_BELT_Y_MAX:
        raise ValueError(
            f"{count} 个纸箱按 pitch={spawn_pitch} 从 y={y_lead} 排到 y={y_tail}，"
            f"队尾（{kinds[-1].key}）悬出带面 y_max={BELT_BOX_BELT_Y_MAX}；"
            "调小 pitch/count，或把 ISAACLAB_BELT_BOX_SPAWN_Y_LEAD 往工位方向下调"
        )
    # 停稳后的队列不需要单独校验：整带节拍保持出生间距整体**向下游**平移（队首从
    # y_lead 走到 y_stop < y_lead），所以停稳队尾一定比出生队尾更靠下游，上面这条
    # 出生位校验已经覆盖了它。

    positions = tuple(
        (lane_x, y_lead + spawn_pitch * index, spawn_z) for index in range(count)
    )
    # 直线形态里所有箱子都在同一车道 x 上，队首两箱同样偏西/偏东。
    positions = _apply_front_two_x_offset(positions, BELT_BOX_FRONT_X_OFFSET, (True, True))
    return count, positions, kinds, queue_gap


def resolve_scene_layout(environ: Mapping[str, str]) -> ConveyorSceneLayout:
    """Resolve the conveyor or pushcart layout from an environment mapping."""

    totes_on_conveyor = _env_bool(environ, "ISAACLAB_TOTES_ON_CONVEYOR", True)
    cart_group_x = _env_float(environ, "ISAACLAB_CART_GROUP_X", -5.62)
    # =0 布局的作业组（拖车 + 叠筐 + 两台机器人）站在入料端之北，与流水线同轴。
    # Δ=0.25 下拖车组随线到 (-5.62, 19.0)：拖车实测占位 y 跨度 0.824（spawn
    # scale 0.5）⇒ y[18.18, 19.82]，离带端 18.472 仍是 0.118 m（相对几何不变）、
    # 离 +Y 墙面 23.606 有 4.194 m。
    # ⚠️ 该占位与西拐弯道 footprint x[-7.113,-5.042] y[18.457,20.558] 大面积重叠
    #    （相对几何同 Δ=0，约 83% 被吞没），弯道组必须只在 =1 流水线布局生成（见
    #    endless_intake 的生成门与 conveyor_env_cfg 的门控断言）。
    cart_group_y = _env_float(environ, "ISAACLAB_CART_GROUP_Y", _shifted(18.75))
    robot_side_offset = _env_float(environ, "ISAACLAB_ROBOT_SIDE_OFFSET", 0.80)

    robot_workstation_y = _env_float(
        environ,
        "ISAACLAB_ROBOT_WORKSTATION_Y",
        _shifted(14.148) if totes_on_conveyor else cart_group_y,
    )
    # 流水线模式采用前后错位双工位：robot_1 对准队首停止线，robot_2 沿 +Y
    # 上游移动一个默认队列节距，对准第二件物体。推车回退布局仍保持两机同 Y。
    robot_2_workstation_y = _env_float(
        environ,
        "ISAACLAB_ROBOT_2_WORKSTATION_Y",
        robot_workstation_y + 0.75 if totes_on_conveyor else robot_workstation_y,
    )
    # 两台机器人对称分站带两侧：带中线 x=-5.62，各距中线 1.08 m（距带边 0.63 m）。
    # 历史值 robot_1_x=-4.75 距中线只有 0.87，比对面近 0.21——视觉上一台贴着流水线
    # 一台离得远（2026-08-09 用户反馈），对称化取 -5.62+1.08=-4.54。
    robot_1_x = _env_float(
        environ,
        "ISAACLAB_ROBOT_1_X",
        -4.54 if totes_on_conveyor else cart_group_x + robot_side_offset,
    )
    robot_2_x = _env_float(
        environ,
        "ISAACLAB_ROBOT_2_X",
        -6.7 if totes_on_conveyor else cart_group_x - robot_side_offset,
    )

    # 只在 =1 流水线布局生成三台纯显示 G1。它们分散在 X 支线队尾、主线上游和
    # X 支线中段，不组成面对面队列。第一台站在默认最后一个物体南侧偏东 1 m 并
    # 朝西北正对该物体；第二台沿用现有机器人到主带中线的横向站距；第三台在支线北侧。
    # 现有两台的坐标和朝向保持不变。
    standby_robot_poses = (
        (
            StandbyRobotPose(
                pos=(-12.70, 18.9534, STANDBY_ROBOT_ROOT_Z),
                rot=(0.40455358, 0.0, 0.0, 0.91451430),  # 朝西北正对队尾物体
            ),
            StandbyRobotPose(
                # 西侧桌箱随第二机器人北移 0.75 m 后，此待机位也等量后移，
                # 保持它与工位家具之间的既有净距。
                pos=(robot_2_x, _shifted(17.70), STANDBY_ROBOT_ROOT_Z),
                rot=(1.0, 0.0, 0.0, 0.0),  # yaw 0°，朝 +X
            ),
            StandbyRobotPose(
                pos=(-9.50, _shifted(20.90), STANDBY_ROBOT_ROOT_Z),
                rot=(0.70710678, 0.0, 0.0, -0.70710678),  # yaw -90°，朝 -Y
            ),
        )
        if totes_on_conveyor
        else ()
    )

    # Δ=0.25 下两个出生点在 17.65 / 18.25（贴着入料端）。出生瞬间可见是已知限制
    # （见 README「已知限制」；西拐弯道方案的遮挡评估属 endless_intake）。
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

    conveyor_y_stop = _env_float(environ, "ISAACLAB_CONVEYOR_Y_STOP", default_y_stop)
    # 纸箱只在流水线布局下存在：推车布局的作业闭环仍是两塑料筐。
    if totes_on_conveyor:
        (
            belt_box_count,
            belt_box_positions,
            belt_box_kinds,
            belt_box_queue_gap,
        ) = resolve_belt_box_positions(environ, y_stop=conveyor_y_stop)
    else:
        belt_box_count, belt_box_positions, belt_box_kinds, belt_box_queue_gap = 0, (), (), 0.07

    return ConveyorSceneLayout(
        totes_on_conveyor=totes_on_conveyor,
        cart_group_x=cart_group_x,
        cart_group_y=cart_group_y,
        robot_side_offset=robot_side_offset,
        robot_workstation_y=robot_workstation_y,
        robot_2_workstation_y=robot_2_workstation_y,
        robot_1_x=robot_1_x,
        robot_2_x=robot_2_x,
        standby_robot_poses=standby_robot_poses,
        pushcart_2_pos=pushcart_2_pos,
        cart2_tote1_pos=cart2_tote1_pos,
        cart2_tote2_pos=cart2_tote2_pos,
        tote_scale=tote_scale,
        conveyor_y_stop=conveyor_y_stop,
        belt_box_count=belt_box_count,
        belt_box_positions=belt_box_positions,
        belt_box_kinds=belt_box_kinds,
        belt_box_queue_gap=belt_box_queue_gap,
    )
