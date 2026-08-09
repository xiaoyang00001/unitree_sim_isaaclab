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


@dataclass(frozen=True)
class BeltBoxKind:
    """一种流水线箱型：视觉资产 + 缩放后的世界尺寸 + 质量。

    ``length_y`` 是**摆上带面后沿输送方向 Y 的长度**。两种箱型都绕 Z 转 90° 摆放
    （与 v61 背景装饰箱朝向一致），所以 ``length_y`` 取的是视觉资产的 X 尺寸、
    ``width_x`` 取 Y 尺寸。原点都在箱底面，出生 z 因此可以直接用带面高度。
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


# 纸箱两种取自 v61 背景（`ConveyorBelt_Box_XX` 引 SM_CardBoxD_01、`KLT_Bin_XX` 引
# SM_CardBoxC_01）；软包裹三种取自 IsaacLab 分叉 feat/pickplace-parcel-assets 的
# 程序化快递袋资产（塑料袋装衣服：枕形鼓包+热封边+顶面白色面单）。⚠️ 软包裹是
# **刚体不是软体**——"软"只是视觉造型，物理上与纸箱同一套约定（根挂 RigidBody+Mass、
# convexHull 碰撞、原点在袋底），所以能直接进队列/同步/镜像分流，CPU pipeline 可用。
BELT_BOX_KINDS = {
    "d01": BeltBoxKind(
        key="d01",
        asset="cart_box_d01_physics.usda",
        length_y=0.38,
        width_x=0.25,
        height_z=0.1487,
        mass=1.0,
    ),
    "c01": BeltBoxKind(
        key="c01",
        asset="cart_box_c01_physics.usda",
        length_y=0.50,
        width_x=0.50,
        height_z=0.25,
        mass=1.5,
    ),
    # 尺寸取实测组合包围盒（生成器标称 40×30 / 45×35 / 32×24 cm，鼓包略溢出）。
    "parcel_a01": BeltBoxKind(
        key="parcel_a01",
        asset="parcel_soft_a01.usda",
        length_y=0.4017,
        width_x=0.3013,
        height_z=0.0801,
        mass=0.35,
    ),
    "parcel_a02": BeltBoxKind(
        key="parcel_a02",
        asset="parcel_soft_a02.usda",
        length_y=0.4526,
        width_x=0.3520,
        height_z=0.0968,
        mass=0.5,
    ),
    "parcel_a03": BeltBoxKind(
        key="parcel_a03",
        asset="parcel_soft_a03.usda",
        length_y=0.3218,
        width_x=0.2414,
        height_z=0.0603,
        mass=0.22,
    ),
}
# 默认交错：小纸箱 + 白色软包裹（软包裹替换掉原来的大纸箱 c01；
# ISAACLAB_BELT_BOX_PATTERN=d01,c01 可随时切回两种纸箱）。
DEFAULT_BELT_BOX_PATTERN = ("d01", "parcel_a02")

# 带面几何与 conveyor_drive 的常量保持一致（那边是驱动/分段的真源，这里只用来
# 定位出生点；两处数值若要改必须同时改）。
BELT_BOX_LANE_X = -5.62
BELT_BOX_SPAWN_Z = 0.775
BELT_BOX_BELT_Y_MAX = 18.22
BELT_BOX_BELT_WIDTH = 0.90
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
    belt_box_kinds: tuple[BeltBoxKind, ...]
    belt_box_queue_gap: float

    @property
    def belt_box_names(self) -> tuple[str, ...]:
        return tuple(f"belt_box_{index + 1}" for index in range(self.belt_box_count))

    @property
    def belt_box_half_lengths(self) -> tuple[float, ...]:
        """每个箱子沿输送方向的半长——排队判据按它算净间隙。"""

        return tuple(kind.half_length_y for kind in self.belt_box_kinds)


def resolve_belt_box_pattern(environ: Mapping[str, str], count: int) -> tuple[BeltBoxKind, ...]:
    """按 ``ISAACLAB_BELT_BOX_PATTERN`` 循环出每个位置的箱型。

    默认 ``d01,c01`` ⇒ 交错排布 d01/c01/d01/c01/d01。写单个 key（如 ``d01``）就是
    全用一种。未知 key 一律 fail-fast，不静默回退——否则场景里会悄悄少一种箱型。
    """

    raw = environ.get("ISAACLAB_BELT_BOX_PATTERN")
    if raw is None or not str(raw).strip():
        pattern = DEFAULT_BELT_BOX_PATTERN
    else:
        pattern = tuple(item.strip().lower() for item in str(raw).split(",") if item.strip())
    if not pattern:
        raise ValueError("ISAACLAB_BELT_BOX_PATTERN 不能为空")

    unknown = [key for key in pattern if key not in BELT_BOX_KINDS]
    if unknown:
        choices = ", ".join(sorted(BELT_BOX_KINDS))
        raise ValueError(f"ISAACLAB_BELT_BOX_PATTERN 含未知箱型 {unknown}；可选值: {choices}")

    return tuple(BELT_BOX_KINDS[pattern[index % len(pattern)]] for index in range(count))


def resolve_belt_box_positions(
    environ: Mapping[str, str],
    *,
    y_stop: float,
) -> tuple[int, tuple[tuple[float, float, float], ...], tuple[BeltBoxKind, ...], float]:
    """把 N 个纸箱排在工位**上游**那一段带面上（"流水线前段"）。

    机器人工位 ``y_stop`` 把带面切成两段：上游 (y_stop, 18.22] 是来料段，下游
    [10.19, y_stop) 是出料段。箱子只出生在上游，这样一次整场景复位 = 重新完整
    流一遍（与两塑料筐时代的落点约定一致，见 conveyor_env_cfg 的出生 y 注释）。

    ``belt_box_1`` 是队首（y 最小、最先到工位），编号递增向上游排。箱型按
    ``ISAACLAB_BELT_BOX_PATTERN`` 循环（默认两种交错）。出生间距 ``spawn_pitch``
    只决定初始队形；停下来后的实际队距由**每个箱子自己的半长**加净间隙
    ``queue_gap`` 决定——两种箱型尺寸不同，统一的"中心距"要么让大箱穿模、要么让
    小箱之间留出突兀的空档。

    非法配置一律 fail-fast：箱子排到带面外、相邻箱子出生就互穿、停稳后的队列
    伸出带尾，都会在启动时暴露，而不是等到运行时看见箱子悬空/穿模再回头猜。
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
    # ⚠️ 出生间距同时**就是**停稳后的队列间距（整带节拍：所有箱子同起同停，相对
    # 位置恒定）。工位上游只有 18.22-14.148 ≈ 4.07 m，因此
    #     队列长度 (count-1)*pitch + 端部半长  +  队首行程 (y_lead - y_stop)  ≤ 4.07
    # 间距、行程、数量三者此消彼长；调大 pitch 必须同时下调 y_lead 或 count，
    # 否则下面的两条 fail-fast 会拦住。
    y_lead = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_Y_LEAD", 14.95)
    spawn_pitch = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_PITCH", 0.75)
    queue_gap = _env_float(environ, "ISAACLAB_BELT_BOX_QUEUE_GAP", 0.07)

    kinds = resolve_belt_box_pattern(environ, count)
    if count == 0:
        return 0, (), (), queue_gap

    if queue_gap < 0.0:
        raise ValueError(f"ISAACLAB_BELT_BOX_QUEUE_GAP 不能为负，当前 {queue_gap}")

    halves = [kind.half_length_y for kind in kinds]

    # 相邻箱子出生就不能互穿：中心距至少是两个半长之和。
    for index in range(count - 1):
        need = halves[index] + halves[index + 1]
        if spawn_pitch < need:
            raise ValueError(
                f"ISAACLAB_BELT_BOX_SPAWN_PITCH={spawn_pitch} 放不下相邻的 "
                f"{kinds[index].key}/{kinds[index + 1].key}：至少要 {need:.3f}"
            )

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

    # 最宽的箱型也要放得进带面宽度。
    half_width = BELT_BOX_BELT_WIDTH * 0.5
    for kind in kinds:
        if kind.width_x * 0.5 > half_width:
            raise ValueError(
                f"箱型 {kind.key} 宽 {kind.width_x} 放不进带面宽度 {BELT_BOX_BELT_WIDTH}"
            )

    positions = tuple(
        (lane_x, y_lead + spawn_pitch * index, spawn_z) for index in range(count)
    )
    return count, positions, kinds, queue_gap


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
        robot_1_x=robot_1_x,
        robot_2_x=robot_2_x,
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
