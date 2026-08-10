"""流水线整带节拍判据的单测（不启 Kit，只用 torch）。

这些用例锁的是核心语义：队首一到工位整条带一起停、后面的保持间距不再挤上来、
取走队首后整列一起前进一格，外加一层按各箱半长算的防撞保底。
``conveyor_queue`` 刻意不依赖 Isaac Sim 就是为了让这一层能被这样逐条验证。
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import torch


_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_queue.py"
)
_SPEC = importlib.util.spec_from_file_location("conveyor_queue_for_test", _MODULE_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _MODULE
_SPEC.loader.exec_module(_MODULE)


_Y_STOP = 14.148
# 两种箱型沿输送方向的半长：d01=0.38/2、c01=0.50/2。
_HALF_D = 0.19
_HALF_C = 0.25
_GAP = 0.07
_BELT_TOP_Z = 0.772
_X_RANGE = (-6.17, -5.07)
_Y_RANGE = (10.19, 18.22)


def _column(values: list[float]) -> torch.Tensor:
    """把 N 个箱子的标量列表变成 (N, E=1) 张量。"""

    return torch.tensor(values, dtype=torch.float32).unsqueeze(-1)


def _drive(
    ys: list[float],
    on_belt: list[bool],
    y_stop: float | None = _Y_STOP,
    halves: list[float] | None = None,
) -> list[bool]:
    """默认全用 d01 半长，需要混排时显式传 ``halves``。"""

    if halves is None:
        halves = [_HALF_D] * len(ys)
    mask = _MODULE.queue_drive_mask(
        _column(ys),
        torch.tensor(on_belt, dtype=torch.bool).unsqueeze(-1),
        _column(halves),
        y_stop=y_stop,
        queue_gap=_GAP,
    )
    return [bool(v) for v in mask.squeeze(-1).tolist()]


def _slot_after(lead_y: float, lead_half: float, follow_half: float) -> float:
    """前车停在 lead_y 时，后车该停的中心 y。"""

    return lead_y + lead_half + _GAP + follow_half


class OnBeltMaskTest(unittest.TestCase):
    def _mask(self, positions: list[tuple[float, float, float]]) -> list[bool]:
        pos = torch.tensor(positions, dtype=torch.float32).unsqueeze(1)  # (N, 1, 3)
        mask = _MODULE.on_belt_mask(
            pos,
            belt_top_z=_BELT_TOP_Z,
            z_tolerance=0.15,
            x_range=_X_RANGE,
            y_range=_Y_RANGE,
        )
        return [bool(v) for v in mask.squeeze(-1).tolist()]

    def test_box_resting_on_the_belt_is_on_belt(self) -> None:
        self.assertEqual(self._mask([(-5.62, 16.0, 0.775)]), [True])

    def test_lifted_or_dropped_box_leaves_the_window(self) -> None:
        lifted = (-5.62, 16.0, 1.10)  # 被机器人拎起
        dropped = (-5.62, 16.0, 0.10)  # 掉到地上
        self.assertEqual(self._mask([lifted, dropped]), [False, False])

    def test_box_carried_off_to_the_side_leaves_the_window(self) -> None:
        self.assertEqual(self._mask([(-4.20, 16.0, 0.775)]), [False])

    def test_box_past_either_belt_end_leaves_the_window(self) -> None:
        self.assertEqual(
            self._mask([(-5.62, 9.50, 0.775), (-5.62, 19.00, 0.775)]),
            [False, False],
        )


class QueueDriveMaskTest(unittest.TestCase):
    """整带节拍：队首一到工位，整条带一起停；取走队首，整列一起前进一格。"""

    def test_lead_box_runs_until_the_workstation_then_stops(self) -> None:
        self.assertEqual(_drive([16.0], [True]), [True])
        self.assertEqual(_drive([_Y_STOP + 0.001], [True]), [True])
        self.assertEqual(_drive([_Y_STOP], [True]), [False])
        self.assertEqual(_drive([_Y_STOP - 0.05], [True]), [False])

    def test_stop_line_is_a_strict_threshold(self) -> None:
        """停止线是严格不等号：正好压在线上就不再驱动，越过一点点才驱动。

        ⚠️ 判据在 ``y == stop_line`` 处是临界的，float32 下这一点的取值由舍入
        决定，所以其它用例一律留出 ≥1 cm 的余量而不是拿等号做断言。实际运行也
        永远不会精确停在线上：箱子是被摩擦拖停的，会越线一小段（现有塑料筐实测
        越过 y_stop 约 11 mm），越线后驱动即关闭，不会来回抖。
        """

        self.assertEqual(_drive([_Y_STOP + 0.01], [True]), [True])
        self.assertEqual(_drive([_Y_STOP - 0.01], [True]), [False])

    def test_whole_belt_stops_together_when_the_lead_arrives(self) -> None:
        """核心语义：队首压到工位，后面的**原地停住**，不再往前挤。"""

        # 队首还没到 → 整带都在走，不管后面隔多远。
        self.assertEqual(
            _drive([_Y_STOP + 0.5, 15.5, 16.5], [True] * 3), [True, True, True]
        )
        # 队首到位 → 三个一起停，即使后两个离前车还很远。
        self.assertEqual(
            _drive([_Y_STOP, 15.5, 16.5], [True] * 3), [False, False, False]
        )

    def test_followers_keep_their_spacing_instead_of_closing_up(self) -> None:
        """停下后队列保持原有间距，不会压缩到 queue_gap。

        这正是与"逐个积放"的区别：后车不会因为前面有空档就继续前进。
        """

        spacing = 0.6
        settled = [_Y_STOP + index * spacing for index in range(5)]
        self.assertEqual(_drive(settled, [True] * 5), [False] * 5)

        # 把某个后车往上游挪远一点，它依然不动——带停了就是停了。
        nudged = list(settled)
        nudged[3] += 0.4
        self.assertEqual(_drive(nudged, [True] * 5), [False] * 5)

    def test_picking_the_lead_restarts_the_whole_belt(self) -> None:
        """取走队首 → 整列一起重新起步（不是只放行下一个）。"""

        spacing = 0.6
        queued = [_Y_STOP + index * spacing for index in range(5)]

        # 取件前：整带停。
        self.assertEqual(_drive(queued, [True] * 5), [False] * 5)

        # 队首被拎起（离开带面窗口）→ 新队首在 y_stop+0.6，带重启，剩下四个同时走。
        released = _drive(queued, [False, True, True, True, True])
        self.assertEqual(released, [False, True, True, True, True])

    def test_belt_stops_again_once_the_new_lead_reaches_the_station(self) -> None:
        """整列前进一格后，新队首压到工位，再次整带停——节拍循环成立。"""

        spacing = 0.6
        advanced = [_Y_STOP + index * spacing for index in range(4)]
        self.assertEqual(_drive(advanced, [True] * 4), [False] * 4)

    def test_a_box_off_the_belt_neither_drives_nor_holds_the_belt(self) -> None:
        """离开带面的箱子既不被驱动，也不参与"队首是谁"的判定。"""

        # 下游那个已被搬走 → 它不再把带按停，上游那个继续走。
        self.assertEqual(_drive([13.0, 16.0], [False, True]), [False, True])
        # 它也不再充当前车挡住后面的箱子。
        self.assertEqual(_drive([15.0, 15.2], [False, True]), [False, True])

    def test_anti_collision_still_holds_a_follower_that_drifts_too_close(self) -> None:
        """防撞保底：带在跑，但后车已经贴到前车尾部时不再驱动它。

        整带同起同停时相对间距恒定，正常跑不到这一步；它只兜住摩擦/质量差异带来的
        缓慢漂移。构造法：让队首远在工位上游（带在跑），把后车塞到前车紧后面。
        """

        lead = 16.0  # 远大于 y_stop，带处于运行状态
        too_close = _slot_after(lead, _HALF_D, _HALF_D) - 0.05
        self.assertEqual(_drive([lead, too_close], [True, True]), [True, False])

        # 拉开到净间隙以外就恢复驱动。
        far_enough = _slot_after(lead, _HALF_D, _HALF_D) + 0.05
        self.assertEqual(_drive([lead, far_enough], [True, True]), [True, True])

    def test_anti_collision_uses_each_box_half_length(self) -> None:
        """两种箱型混排时，兜底净空隙按各自半长算，恒为 queue_gap。"""

        lead = 16.0
        for lead_half, follow_half in (
            (_HALF_D, _HALF_C),
            (_HALF_C, _HALF_D),
            (_HALF_C, _HALF_C),
        ):
            with self.subTest(lead=lead_half, follow=follow_half):
                slot = _slot_after(lead, lead_half, follow_half)
                halves = [lead_half, follow_half]
                self.assertEqual(
                    _drive([lead, slot - 0.05], [True, True], halves=halves), [True, False]
                )
                self.assertEqual(
                    _drive([lead, slot + 0.05], [True, True], halves=halves), [True, True]
                )
                clear = (slot - follow_half) - (lead + lead_half)
                self.assertAlmostEqual(clear, _GAP, places=6)

    def test_a_big_box_blocks_further_upstream_than_a_small_one(self) -> None:
        """同一位置上，大箱比小箱把后车顶得更远——半长确实进了防撞判据。

        取一个恰好落在两条兜底线**之间**的位置：小箱前车放行，大箱前车按住。
        """

        lead = 16.0
        slot_small = _slot_after(lead, _HALF_D, _HALF_D)
        slot_big = _slot_after(lead, _HALF_C, _HALF_D)
        self.assertLess(slot_small, slot_big)
        follow = (slot_small + slot_big) * 0.5

        # 前车是 d01 时后车已越过兜底线 → 跟着带走。
        self.assertEqual(
            _drive([lead, follow], [True, True], halves=[_HALF_D, _HALF_D]), [True, True]
        )
        # 同样位置，前车换成更大的 c01 → 兜底线后移，后车被按住。
        self.assertEqual(
            _drive([lead, follow], [True, True], halves=[_HALF_C, _HALF_D]), [True, False]
        )

    def test_loop_mode_never_stops_the_belt(self) -> None:
        """y_stop=None（循环模式）：没有工位停止线，整带长跑，只剩防撞保底。"""

        self.assertEqual(_drive([11.0], [True], y_stop=None), [True])
        self.assertEqual(_drive([_Y_STOP], [True], y_stop=None), [True])
        self.assertEqual(_drive([10.5, 12.0], [True, True], y_stop=None), [True, True])
        # 贴太近时仍被兜底按住。
        self.assertEqual(_drive([12.0, 12.2], [True, True], y_stop=None), [True, False])

    def test_order_of_object_names_does_not_matter(self) -> None:
        """判据只看坐标，不看清单顺序——两端进程排列不同也不会分叉。"""

        # 第 1 个才是队首（压在工位），于是整带停。
        self.assertEqual(_drive([15.0, _Y_STOP, 17.0], [True] * 3), [False, False, False])
        # 把队首挪到工位上游 → 整带一起走。
        self.assertEqual(
            _drive([15.0, _Y_STOP + 0.3, 17.0], [True] * 3), [True, True, True]
        )

    def test_multiple_envs_are_resolved_independently(self) -> None:
        """(N, E) 的 E 维必须逐 env 独立，一个 env 停带不能连累另一个。"""

        # env 0 的队首压在工位 → 该 env 整带停；env 1 都在上游 → 整带走。
        ys = torch.tensor([[_Y_STOP, 16.0], [_Y_STOP + 0.6, 16.6]], dtype=torch.float32)
        on_belt = torch.ones_like(ys, dtype=torch.bool)
        halves = torch.full((2, 1), _HALF_D)
        mask = _MODULE.queue_drive_mask(ys, on_belt, halves, y_stop=_Y_STOP, queue_gap=_GAP)

        self.assertEqual(mask.tolist(), [[False, True], [False, True]])

    def test_mask_is_computed_without_host_synchronisation(self) -> None:
        """判据必须全程留在张量里：返回 bool 张量而不是 Python 值。

        这是性能铁律的可执行版本——一旦有人在 queue_drive_mask 里加
        ``.item()``/``.any()``，形状/类型断言不会挂，但 CPU 同步会悄悄回来。
        这里至少钉住"输出是与输入同形的 bool 张量"这条契约。
        """

        ys = _column([16.0, 17.0, 18.0])
        on_belt = torch.ones_like(ys, dtype=torch.bool)
        halves = torch.full_like(ys, _HALF_D)
        mask = _MODULE.queue_drive_mask(ys, on_belt, halves, y_stop=_Y_STOP, queue_gap=_GAP)

        self.assertIsInstance(mask, torch.Tensor)
        self.assertEqual(mask.dtype, torch.bool)
        self.assertEqual(mask.shape, ys.shape)


# —— 西拐入口弯道（endless intake）常量：与 endless_intake.py 同源（Δ=0.25） ——
_CORNER_CENTER = (-7.0200, 18.6534)
_RADIUS = 1.40
_S_ORIGIN_X = -12.76
_BRANCH_Y = 20.0534
_S_ARC_START = _CORNER_CENTER[0] - _S_ORIGIN_X  # 5.74
_S_ARC_END = _S_ARC_START + _RADIUS * 3.141592653589793 / 2.0
# 与 conveyor_env_cfg 传给事件的 extra_rects 同口径（endless_intake 常量 ±0.10）。
_EXTRA_RECTS = (
    (-7.12, -5.07, 18.37, 20.6034),
    (-17.1342, -6.92, 19.5034, 20.6034),
)


class PathProgressTest(unittest.TestCase):
    """西拐 L 形路径的沿路径距离 s 与航向（与 endless_intake.path_point 同分段）。"""

    def _progress(self, positions: list[tuple[float, float, float]]):
        pos = torch.tensor(positions, dtype=torch.float32).unsqueeze(1)  # (N, 1, 3)
        s, hx, hy = _MODULE.path_progress(
            pos,
            corner_center=_CORNER_CENTER,
            radius=_RADIUS,
            s_origin_x=_S_ORIGIN_X,
        )
        return (
            [float(v) for v in s.squeeze(-1).tolist()],
            [float(v) for v in hx.squeeze(-1).tolist()],
            [float(v) for v in hy.squeeze(-1).tolist()],
        )

    def test_branch_segment_heads_plus_x(self) -> None:
        s, hx, hy = self._progress([(-12.61, _BRANCH_Y, 0.775)])
        self.assertAlmostEqual(s[0], 0.15, places=4)
        self.assertEqual((hx[0], hy[0]), (1.0, 0.0))

    def test_main_segment_heads_minus_y(self) -> None:
        s, hx, hy = self._progress([(-5.62, 14.398, 0.775)])
        self.assertAlmostEqual(s[0], _S_ARC_END + (_CORNER_CENTER[1] - 14.398), places=4)
        self.assertEqual((hx[0], hy[0]), (0.0, -1.0))

    def test_arc_midpoint_heads_diagonally(self) -> None:
        import math

        theta = math.pi / 4
        x = _CORNER_CENTER[0] + _RADIUS * math.sin(theta)
        y = _CORNER_CENTER[1] + _RADIUS * math.cos(theta)
        s, hx, hy = self._progress([(x, y, 0.775)])
        self.assertAlmostEqual(s[0], _S_ARC_START + _RADIUS * theta, places=4)
        self.assertAlmostEqual(hx[0], math.sqrt(0.5), places=4)
        self.assertAlmostEqual(hy[0], -math.sqrt(0.5), places=4)

    def test_s_is_monotonic_downstream_across_segments(self) -> None:
        """支线→弧段→主线依次取点，s 必须严格递增（排队判据的前提）。"""

        import math

        pts = [(-12.61, _BRANCH_Y, 0.775), (-7.70, _BRANCH_Y, 0.775)]
        for theta_deg in (10, 45, 80):
            theta = math.radians(theta_deg)
            pts.append(
                (
                    _CORNER_CENTER[0] + _RADIUS * math.sin(theta),
                    _CORNER_CENTER[1] + _RADIUS * math.cos(theta),
                    0.775,
                )
            )
        pts += [(-5.62, 17.0, 0.775), (-5.62, 14.148, 0.775)]
        s, _hx, _hy = self._progress(pts)
        self.assertEqual(s, sorted(s))


class QueueDriveMaskAlongPathTest(unittest.TestCase):
    """沿路径 s 的整带节拍：语义与 y 版逐项镜像（s 向下游递增）。"""

    _S_STOP = 12.1945  # = path_s_of_main_y(14.148)
    _UNSET = object()

    def _drive(
        self,
        ss: list[float],
        on_belt: list[bool],
        s_stop: object = _UNSET,
        halves: list[float] | None = None,
    ) -> list[bool]:
        if s_stop is self._UNSET:
            s_stop = self._S_STOP
        if halves is None:
            halves = [_HALF_D] * len(ss)
        mask = _MODULE.queue_drive_mask_along_path(
            _column(ss),
            torch.tensor(on_belt, dtype=torch.bool).unsqueeze(-1),
            _column(halves),
            s_stop=s_stop,
            queue_gap=_GAP,
        )
        return [bool(v) for v in mask.squeeze(-1).tolist()]

    def test_lead_runs_until_the_workstation_then_stops(self) -> None:
        self.assertEqual(self._drive([8.06], [True]), [True])
        self.assertEqual(self._drive([self._S_STOP - 0.01], [True]), [True])
        self.assertEqual(self._drive([self._S_STOP + 0.01], [True]), [False])

    def test_whole_belt_stops_together_when_the_lead_arrives(self) -> None:
        # 队首（s 最大）到位 → 整带停，包括还在支线/弧段上的箱子。
        self.assertEqual(
            self._drive([self._S_STOP, 6.5, 0.5], [True] * 3), [False, False, False]
        )
        self.assertEqual(
            self._drive([self._S_STOP - 0.5, 6.5, 0.5], [True] * 3), [True, True, True]
        )

    def test_picking_the_lead_restarts_the_whole_belt(self) -> None:
        queued = [self._S_STOP - 0.75 * index for index in range(5)]
        self.assertEqual(self._drive(queued, [True] * 5), [False] * 5)
        released = self._drive(queued, [False, True, True, True, True])
        self.assertEqual(released, [False, True, True, True, True])

    def test_anti_collision_uses_each_box_half_length(self) -> None:
        lead = 6.5  # 远在工位上游，带在跑
        slot = lead - (_HALF_C + _GAP + _HALF_D)  # 前车 c01、后车 d01 的兜底线
        self.assertEqual(
            self._drive([lead, slot + 0.05], [True, True], halves=[_HALF_C, _HALF_D]),
            [True, False],
        )
        self.assertEqual(
            self._drive([lead, slot - 0.05], [True, True], halves=[_HALF_C, _HALF_D]),
            [True, True],
        )

    def test_loop_mode_never_stops_the_belt(self) -> None:
        self.assertEqual(self._drive([13.0], [True], s_stop=None), [True])
        self.assertEqual(
            self._drive([13.0, 12.9], [True, True], s_stop=None), [True, False]
        )


class OnBeltExtraRectsTest(unittest.TestCase):
    """L 形带面 = 主线矩形 ∪ 拐角补块 ∪ 西拐 X 支线条带。"""

    def _mask(self, positions: list[tuple[float, float, float]]) -> list[bool]:
        pos = torch.tensor(positions, dtype=torch.float32).unsqueeze(1)
        mask = _MODULE.on_belt_mask(
            pos,
            belt_top_z=_BELT_TOP_Z,
            z_tolerance=0.15,
            x_range=_X_RANGE,
            y_range=_Y_RANGE,
            extra_rects=_EXTRA_RECTS,
        )
        return [bool(v) for v in mask.squeeze(-1).tolist()]

    def test_branch_and_arc_boxes_are_on_belt(self) -> None:
        self.assertEqual(
            self._mask(
                [
                    (-16.66, _BRANCH_Y, 0.775),  # 支线回生点/队尾深藏槽位（段 5）
                    (-12.61, _BRANCH_Y, 0.775),  # 支线中段（段 3）
                    (-7.70, _BRANCH_Y, 0.775),   # 支线东段
                    (-6.03, 19.643, 0.775),      # 弧段中点附近
                    (-5.62, 16.0, 0.775),        # 主线（原判据不受影响）
                ]
            ),
            [True, True, True, True, True],
        )

    def test_off_path_boxes_stay_out(self) -> None:
        self.assertEqual(
            self._mask(
                [
                    (-12.61, 20.70, 0.775),   # 支线条带以北（悬出）
                    (-17.30, _BRANCH_Y, 0.775),  # 越过支线西端（放宽后 -17.1342）
                    (-16.66, _BRANCH_Y, 1.10),   # 被拎起（z 出窗）
                    (-4.60, 19.0, 0.775),        # 弯道以东的空地
                ]
            ),
            [False, False, False, False],
        )

    def test_without_extra_rects_the_branch_is_not_belt(self) -> None:
        """extra_rects 缺省（直线形态）时支线/弧段不在带面判据内——回退语义。"""

        pos = torch.tensor([(-12.61, _BRANCH_Y, 0.775)], dtype=torch.float32).unsqueeze(1)
        mask = _MODULE.on_belt_mask(
            pos,
            belt_top_z=_BELT_TOP_Z,
            z_tolerance=0.15,
            x_range=_X_RANGE,
            y_range=_Y_RANGE,
        )
        self.assertEqual([bool(v) for v in mask.squeeze(-1).tolist()], [False])


if __name__ == "__main__":
    unittest.main()
