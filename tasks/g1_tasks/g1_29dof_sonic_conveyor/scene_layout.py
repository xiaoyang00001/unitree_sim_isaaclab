"""Dependency-free resolver for the two conveyor scene layouts.

Keeping the layout arithmetic outside ``conveyor_env_cfg`` lets ordinary Python
tests verify the environment-variable switch without importing Isaac Lab.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


# 流水线整体北移量（世界 +Y，m）。真源是 conveyor_drive.CONVEYOR_NORTH_SHIFT_Y，
# 当前 Δ=0（安检机时代的北移已回退，带体回世界 y[10.188, 18.222]）。
# 本模块刻意零相对 import（测试用单文件加载），所以抄一份；
# tests/test_conveyor_scene_layout.py 交叉断言两边一致，别只改一边。
CONVEYOR_NORTH_SHIFT_Y = 0.0


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
# 定位出生点；两处数值若要改必须同时改）。y 值必须过 _shifted（Δ 非零时自动
# 跟随）：当前 Δ=0，带面入料端在 18.22。
BELT_BOX_LANE_X = -5.62
BELT_BOX_SPAWN_Z = 0.775
BELT_BOX_BELT_Y_MAX = _shifted(18.22)
BELT_BOX_BELT_WIDTH = 0.90
# SceneCfg 里的 belt_box_N 字段是显式声明的（configclass 需要类属性），所以数量
# 有硬上限。调大必须同时在 conveyor_env_cfg.G129SonicConveyorSceneCfg 里补字段。
BELT_BOX_MAX_COUNT = 5

# ------------------------------------------------------------------
# 入口弯道（endless intake，**西拐**）出生路径：真源是 endless_intake.py 的路径
# 常量，本模块零相对 import（测试用单文件加载）所以抄一份最小集；
# tests/test_conveyor_scene_layout.py 交叉断言出生点 = endless_intake.path_point。
#
#   s=0 锚点 x=-12.76（支线车道中线 y=19.8034）；支线段 s∈[-0.30, 5.74] 沿 +X；
#   圆角弧 s∈[5.74, ~7.939]（圆心 (-5.62-1.4, 19.8034-1.4)、R=1.4）；
#   主线段 s>~7.939 沿 -Y（工位/停止线都在这段上）。
# ------------------------------------------------------------------
BELT_BOX_BRANCH_LANE_Y = 19.8034
BELT_BOX_CORNER_RADIUS = 1.40
BELT_BOX_PATH_S_ORIGIN_X = -12.76
# 支线滚筒可用端（段 3 西端 x=-13.0640 ⇒ s=-0.3040，取 -0.30）：队尾不得越过。
BELT_BOX_PATH_S_MIN = -0.30
# 默认队首 s=8.06：队首刚拐出弯 0.121 m、落在主线 y≈18.283（整箱上主线），
# 5 箱（pitch 0.75）分布 = 主线 1 + 弧上 3 + 支线 1，开局即"绕弯而来"。
# ⚠️ 西侧没有全遮窗（离线射线核算：排 B 货架阴影区 x≤-14.36/-14.22 不可达，
# 支线端头只到 -13.07）——出生/回生可见是记录在 README 的已知限制，不是漏遮。
BELT_BOX_DEFAULT_S_LEAD = 8.06


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
    """把 N 个纸箱排在工位**上游**（默认沿西拐入口弯道路径，退化态沿主线直排）。

    ``belt_box_1`` 是队首（最靠下游、最先到工位），编号递增向上游排。箱型按
    ``ISAACLAB_BELT_BOX_PATTERN`` 循环（默认两种交错）。出生间距 ``spawn_pitch``
    只决定初始队形；停下来后的实际队距由**每个箱子自己的半长**加净间隙
    ``queue_gap`` 决定——两种箱型尺寸不同，统一的"中心距"要么让大箱穿模、要么让
    小箱之间留出突兀的空档。

    **弯道形态（endless intake 生效，默认）**：出生点排在沿路径距离
    ``s_lead − k·pitch`` 上，队首默认刚拐出弯（s=8.06 → 主线 y≈18.283）、队尾
    落到 X 支线上，pitch 语义从"y 间距"变为"沿路径距离间距"。
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
    spawn_pitch = _env_float(environ, "ISAACLAB_BELT_BOX_SPAWN_PITCH", 0.75)
    queue_gap = _env_float(environ, "ISAACLAB_BELT_BOX_QUEUE_GAP", 0.07)
    endless = _endless_intake_active(environ, totes_on_conveyor=True)

    kinds = resolve_belt_box_pattern(environ, count)
    if count == 0:
        return 0, (), (), queue_gap

    if queue_gap < 0.0:
        raise ValueError(f"ISAACLAB_BELT_BOX_QUEUE_GAP 不能为负，当前 {queue_gap}")

    halves = [kind.half_length_y for kind in kinds]

    # 相邻箱子出生就不能互穿：中心距至少是两个半长之和（直线/弯道形态同判据，
    # 弯道形态下 pitch 是沿路径距离，弧段上的弦距略小于弧距，仍然安全——最小
    # 间距余量 0.75-0.4163=0.33 远大于 R=1.4、θ≤0.54rad 下的弦弧差 <0.01）。
    for index in range(count - 1):
        need = halves[index] + halves[index + 1]
        if spawn_pitch < need:
            raise ValueError(
                f"ISAACLAB_BELT_BOX_SPAWN_PITCH={spawn_pitch} 放不下相邻的 "
                f"{kinds[index].key}/{kinds[index + 1].key}：至少要 {need:.3f}"
            )

    # 最宽的箱型也要放得进带面宽度。
    half_width = BELT_BOX_BELT_WIDTH * 0.5
    for kind in kinds:
        if kind.width_x * 0.5 > half_width:
            raise ValueError(
                f"箱型 {kind.key} 宽 {kind.width_x} 放不进带面宽度 {BELT_BOX_BELT_WIDTH}"
            )

    if endless:
        # —— 弯道形态：沿路径距离 s 排队 ——
        corner_y = BELT_BOX_BRANCH_LANE_Y - BELT_BOX_CORNER_RADIUS
        if y_stop >= corner_y:
            raise ValueError(
                f"弯道形态要求工位在主线段上：y_stop={y_stop} 必须 < 拐角切线 {corner_y:.4f}"
            )
        s_stop = _path_s_of_main_y(y_stop)
        raw_s_lead = str(environ.get("ISAACLAB_BELT_BOX_SPAWN_S_LEAD", "")).strip()
        raw_y_lead = str(environ.get("ISAACLAB_BELT_BOX_SPAWN_Y_LEAD", "")).strip()
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
            s_lead = BELT_BOX_DEFAULT_S_LEAD

        # 队首必须整体落在工位上游（s 向下游递增），队尾不能越过支线滚筒可用端。
        if s_lead + halves[0] >= s_stop:
            raise ValueError(
                f"队首出生 s={s_lead:.3f} 压在工位 s_stop={s_stop:.3f} 上；"
                f"最多到 {s_stop - halves[0]:.3f}"
            )
        s_tail = s_lead - spawn_pitch * (count - 1)
        if s_tail - halves[-1] < BELT_BOX_PATH_S_MIN:
            raise ValueError(
                f"{count} 个纸箱按 pitch={spawn_pitch} 从 s={s_lead:.3f} 排到 s={s_tail:.3f}，"
                f"队尾（{kinds[-1].key}）悬出支线带面（s_min={BELT_BOX_PATH_S_MIN}）；"
                "调小 pitch/count，或把 ISAACLAB_BELT_BOX_SPAWN_S_LEAD 往工位方向调大"
            )
        # 停稳后的队列不需要单独校验：整带节拍保持出生间距整体向下游平移，
        # 停稳队尾（s_stop − (count-1)·pitch）一定比出生队尾更靠下游。
        positions = tuple(
            (*_path_point(s_lead - spawn_pitch * index), spawn_z)
            for index in range(count)
        )
        return count, positions, kinds, queue_gap

    # —— 直线形态（endless off / legacy_props）：沿主车道 y 直排 ——
    # ⚠️ 出生间距同时**就是**停稳后的队列间距（整带节拍：所有箱子同起同停，相对
    # 位置恒定）。工位上游只有 18.22-14.148 ≈ 4.07 m，因此
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
    return count, positions, kinds, queue_gap


def resolve_scene_layout(environ: Mapping[str, str]) -> ConveyorSceneLayout:
    """Resolve the conveyor or pushcart layout from an environment mapping."""

    totes_on_conveyor = _env_bool(environ, "ISAACLAB_TOTES_ON_CONVEYOR", True)
    cart_group_x = _env_float(environ, "ISAACLAB_CART_GROUP_X", -5.62)
    # =0 布局的作业组（拖车 + 叠筐 + 两台机器人）站在入料端之北，与流水线同轴。
    # Δ=0 下拖车组回 (-5.62, 18.75)：拖车实测占位 y 跨度 0.824（spawn scale 0.5）
    # ⇒ y[17.93, 19.57]，离带端 18.222 仍是 0.118 m、离 +Y 墙面 23.606 有 4.444 m。
    # ⚠️ 该占位与西拐弯道 footprint x[-7.113,-5.042] y[18.207,20.308] 大面积重叠
    #    （离线实测约 83% 被吞没），弯道组必须只在 =1 流水线布局生成（见
    #    endless_intake 的生成门与 conveyor_env_cfg 的门控断言）。
    cart_group_y = _env_float(environ, "ISAACLAB_CART_GROUP_Y", _shifted(18.75))
    robot_side_offset = _env_float(environ, "ISAACLAB_ROBOT_SIDE_OFFSET", 0.80)

    robot_workstation_y = _env_float(
        environ,
        "ISAACLAB_ROBOT_WORKSTATION_Y",
        _shifted(14.148) if totes_on_conveyor else cart_group_y,
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

    # Δ=0 下两个出生点回到 17.4 / 18.0（贴着入料端）。出生瞬间可见是已知限制
    # （见 README「已知限制」；西拐弯道方案的遮挡评估属提交②/endless_intake）。
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
