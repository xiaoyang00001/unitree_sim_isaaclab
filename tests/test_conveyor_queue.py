"""流水线整带节拍判据的单测（不启 Kit，只用 torch）。

这些用例锁的是核心语义：队首一到工位整条带一起停、后面的保持间距不再挤上来；
工位箱只抬高仍停线，根位置 XY 偏出流水线通道后整列才前进一格；另有一层按各箱
半长算的防撞保底。
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
    transfer_complete: bool | None = None,
) -> list[bool]:
    """默认全用 d01 半长，需要混排时显式传 ``halves``。"""

    if halves is None:
        halves = [_HALF_D] * len(ys)
    transfer_complete_tensor = (
        None
        if transfer_complete is None
        else torch.tensor([transfer_complete], dtype=torch.bool)
    )
    mask = _MODULE.queue_drive_mask(
        _column(ys),
        torch.tensor(on_belt, dtype=torch.bool).unsqueeze(-1),
        _column(halves),
        y_stop=y_stop,
        queue_gap=_GAP,
        transfer_complete=transfer_complete_tensor,
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


class ConveyorCorridorMaskTest(unittest.TestCase):
    """平面通道判据只看根位置 XY，与是否抬高/掉低无关。"""

    def _mask(self, positions: list[tuple[float, float, float]]) -> list[bool]:
        pos = torch.tensor(positions, dtype=torch.float32).unsqueeze(1)
        mask = _MODULE.in_conveyor_corridor_mask(
            pos,
            x_range=_X_RANGE,
            y_range=_Y_RANGE,
        )
        return [bool(v) for v in mask.squeeze(-1).tolist()]

    def test_height_is_ignored(self) -> None:
        self.assertEqual(
            self._mask(
                [
                    (-5.62, 16.0, 0.10),
                    (-5.62, 16.0, 0.775),
                    (-5.62, 16.0, 1.20),
                ]
            ),
            [True, True, True],
        )

    def test_sideways_or_endwise_departure_leaves_the_corridor(self) -> None:
        self.assertEqual(
            self._mask(
                [
                    (-4.20, 16.0, 1.20),
                    (-5.62, 9.50, 0.10),
                    (-5.62, 19.00, 0.775),
                ]
            ),
            [False, False, False],
        )

    def test_boundary_is_inclusive_until_the_root_crosses_it(self) -> None:
        self.assertEqual(
            self._mask(
                [
                    (_X_RANGE[0], 16.0, 1.20),
                    (_X_RANGE[1], 16.0, 1.20),
                    (_X_RANGE[0] - 0.01, 16.0, 1.20),
                    (_X_RANGE[1] + 0.01, 16.0, 1.20),
                ]
            ),
            [True, True, False, False],
        )


class DepartureCompletionTest(unittest.TestCase):
    """只抬高继续按住整线；XY 首次偏出后立即锁存完成。"""

    def test_lift_holds_then_planar_departure_releases_and_latches(self) -> None:
        completed = torch.zeros((2, 1), dtype=torch.bool)
        on_belt = torch.tensor([[False], [True]], dtype=torch.bool)

        completed = _MODULE.update_departure_completion_latch(
            completed,
            torch.tensor([[True], [True]], dtype=torch.bool),
        )
        self.assertEqual(completed.tolist(), [[False], [False]])
        self.assertEqual(
            _MODULE.transfer_complete_mask(on_belt, completed).tolist(),
            [False],
        )

        completed = _MODULE.update_departure_completion_latch(
            completed,
            torch.tensor([[False], [True]], dtype=torch.bool),
        )
        self.assertEqual(completed.tolist(), [[True], [False]])
        self.assertEqual(
            _MODULE.transfer_complete_mask(on_belt, completed).tolist(),
            [True],
        )

        # 抓取轨迹回摆到通道上方后不反悔。
        completed = _MODULE.update_departure_completion_latch(
            completed,
            torch.ones_like(completed),
        )
        self.assertEqual(completed.tolist(), [[True], [False]])

    def test_shape_mismatch_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "形状必须一致"):
            _MODULE.update_departure_completion_latch(
                torch.zeros((2, 1), dtype=torch.bool),
                torch.ones((1, 1), dtype=torch.bool),
            )
        with self.assertRaisesRegex(ValueError, "形状必须一致"):
            _MODULE.transfer_complete_mask(
                torch.ones((2, 1), dtype=torch.bool),
                torch.ones((1, 1), dtype=torch.bool),
            )


class QueueDriveMaskTest(unittest.TestCase):
    """整带节拍：队首到工位停线；箱根 XY 偏出通道后整列前进一格。"""

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

    def test_lifting_the_lead_holds_until_planar_departure(self) -> None:
        """队首只抬高仍停；根位置 XY 偏出流水线后整列才重新起步。"""

        spacing = 0.6
        queued = [_Y_STOP + index * spacing for index in range(5)]

        # 取件前：整带停。
        self.assertEqual(_drive(queued, [True] * 5), [False] * 5)

        # 队首被拎起但 XY 仍在线上方：即使新队首在 y_stop+0.6，整带也继续停。
        held = _drive(
            queued,
            [False, True, True, True, True],
            transfer_complete=False,
        )
        self.assertEqual(held, [False] * 5)

        # 队首根位置 XY 偏出流水线通道 → 门控闭合，剩下四个同时走。
        released = _drive(
            queued,
            [False, True, True, True, True],
            transfer_complete=True,
        )
        self.assertEqual(released, [False, True, True, True, True])

    def test_position_masks_hold_on_vertical_lift_and_release_on_xy_departure(self) -> None:
        """从根位置串起带面、平面通道、偏离锁存与放行。"""

        queued = [_Y_STOP + index * 0.6 for index in range(3)]
        followers = [(-5.62, queued[1], 0.775), (-5.62, queued[2], 0.775)]

        for lead_position, expected in (
            ((-5.62, queued[0], 1.20), [False, False, False]),  # 只抬高
            ((-5.62, queued[0], 0.10), [False, False, False]),  # 掉低但 XY 未偏出
            ((-4.0, queued[0], 1.20), [False, True, True]),  # 横向偏出
        ):
            positions = torch.tensor([lead_position, *followers], dtype=torch.float32).unsqueeze(1)
            on_belt = _MODULE.on_belt_mask(
                positions,
                belt_top_z=_BELT_TOP_Z,
                z_tolerance=0.15,
                x_range=_X_RANGE,
                y_range=_Y_RANGE,
            )
            in_corridor = _MODULE.in_conveyor_corridor_mask(
                positions,
                x_range=_X_RANGE,
                y_range=_Y_RANGE,
            )
            completed = _MODULE.update_departure_completion_latch(
                torch.zeros_like(in_corridor),
                in_corridor,
            )
            ready = _MODULE.transfer_complete_mask(on_belt, completed)
            drive = _MODULE.queue_drive_mask(
                _column(queued),
                on_belt,
                torch.full((3, 1), _HALF_D),
                y_stop=_Y_STOP,
                queue_gap=_GAP,
                transfer_complete=ready,
            )
            self.assertEqual(drive.squeeze(-1).tolist(), expected)

    def test_belt_stops_again_once_the_new_lead_reaches_the_station(self) -> None:
        """整列前进一格后，新队首压到工位，再次整带停——节拍循环成立。"""

        spacing = 0.6
        advanced = [_Y_STOP + index * spacing for index in range(4)]
        self.assertEqual(_drive(advanced, [True] * 4), [False] * 4)

    def test_an_off_belt_box_holds_until_its_xy_has_departed(self) -> None:
        """离开高度窗口不等于偏离流水线；XY 偏出后才退出节拍门控。"""

        self.assertEqual(
            _drive([13.0, 16.0], [False, True], transfer_complete=False),
            [False, False],
        )
        # XY 已偏出后，它既不驱动，也不再挡住上游箱子。
        self.assertEqual(
            _drive([13.0, 16.0], [False, True], transfer_complete=True),
            [False, True],
        )
        self.assertEqual(
            _drive([15.0, 15.2], [False, True], transfer_complete=True),
            [False, True],
        )

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
        # 循环模式没有取放工位，显式的未偏离门也不应改变长跑语义。
        self.assertEqual(
            _drive([11.0], [True], y_stop=None, transfer_complete=False), [True]
        )
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

    def test_departure_gate_is_applied_per_environment(self) -> None:
        """一个 env 仍在线上方不能按住另一个已完成 XY 偏离的 env。"""

        ys = torch.tensor([[13.0, 13.0], [16.0, 16.0]], dtype=torch.float32)
        on_belt = torch.tensor([[False, False], [True, True]], dtype=torch.bool)
        halves = torch.full((2, 1), _HALF_D)
        transfer_complete = torch.tensor([False, True], dtype=torch.bool)
        mask = _MODULE.queue_drive_mask(
            ys,
            on_belt,
            halves,
            y_stop=_Y_STOP,
            queue_gap=_GAP,
            transfer_complete=transfer_complete,
        )

        self.assertEqual(mask.tolist(), [[False, False], [False, True]])

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
        transfer_complete: bool | None = None,
    ) -> list[bool]:
        if s_stop is self._UNSET:
            s_stop = self._S_STOP
        if halves is None:
            halves = [_HALF_D] * len(ss)
        transfer_complete_tensor = (
            None
            if transfer_complete is None
            else torch.tensor([transfer_complete], dtype=torch.bool)
        )
        mask = _MODULE.queue_drive_mask_along_path(
            _column(ss),
            torch.tensor(on_belt, dtype=torch.bool).unsqueeze(-1),
            _column(halves),
            s_stop=s_stop,
            queue_gap=_GAP,
            transfer_complete=transfer_complete_tensor,
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

    def test_lifting_the_lead_waits_for_xy_departure_before_restarting(self) -> None:
        queued = [self._S_STOP - 0.75 * index for index in range(5)]
        self.assertEqual(self._drive(queued, [True] * 5), [False] * 5)
        held = self._drive(
            queued,
            [False, True, True, True, True],
            transfer_complete=False,
        )
        self.assertEqual(held, [False] * 5)
        released = self._drive(
            queued,
            [False, True, True, True, True],
            transfer_complete=True,
        )
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
            self._drive([13.0], [True], s_stop=None, transfer_complete=False), [True]
        )
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

    def _corridor_mask(self, positions: list[tuple[float, float, float]]) -> list[bool]:
        pos = torch.tensor(positions, dtype=torch.float32).unsqueeze(1)
        mask = _MODULE.in_conveyor_corridor_mask(
            pos,
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

    def test_lifted_branch_and_arc_boxes_still_occupy_the_xy_corridor(self) -> None:
        self.assertEqual(
            self._corridor_mask(
                [
                    (-16.66, _BRANCH_Y, 1.20),
                    (-6.03, 19.643, 1.20),
                    (-5.62, 16.0, 1.20),
                    (-4.60, 19.0, 1.20),
                ]
            ),
            [True, True, True, False],
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


class ArrivalLatchTest(unittest.TestCase):
    """到位锁存：首次越过停止线置位，被推回上游也不解除。"""

    def test_latch_sets_once_crossed_and_holds(self) -> None:
        arrived = torch.zeros((1, 1), dtype=torch.bool)
        arrived = _MODULE.update_arrival_latch(
            arrived, _column([_Y_STOP + 0.02]), y_stop=_Y_STOP
        )
        self.assertEqual(arrived.tolist(), [[False]])
        arrived = _MODULE.update_arrival_latch(
            arrived, _column([_Y_STOP - 0.011]), y_stop=_Y_STOP
        )
        self.assertEqual(arrived.tolist(), [[True]])
        # 抱取把箱子推挤回停止线上游 → 锁存保持。
        arrived = _MODULE.update_arrival_latch(
            arrived, _column([_Y_STOP + 0.02]), y_stop=_Y_STOP
        )
        self.assertEqual(arrived.tolist(), [[True]])

    def test_along_path_variant_latches_at_s_stop(self) -> None:
        s_stop = 12.1945
        arrived = torch.zeros((1, 1), dtype=torch.bool)
        arrived = _MODULE.update_arrival_latch_along_path(
            arrived, _column([s_stop - 0.01]), s_stop=s_stop
        )
        self.assertEqual(arrived.tolist(), [[False]])
        arrived = _MODULE.update_arrival_latch_along_path(
            arrived, _column([s_stop + 0.01]), s_stop=s_stop
        )
        self.assertEqual(arrived.tolist(), [[True]])
        arrived = _MODULE.update_arrival_latch_along_path(
            arrived, _column([s_stop - 0.05]), s_stop=s_stop
        )
        self.assertEqual(arrived.tolist(), [[True]])

    def test_shape_mismatch_fails_fast(self) -> None:
        with self.assertRaises(ValueError):
            _MODULE.update_arrival_latch(
                torch.zeros((2, 1), dtype=torch.bool),
                _column([14.0]),
                y_stop=_Y_STOP,
            )


class BeltReleaseGateTest(unittest.TestCase):
    """整带放行门：兼容循环取件，并支持双机器人单批次终止停线。"""

    def test_gate_blocks_while_an_arrived_box_is_still_on_the_line(self) -> None:
        arrived = torch.tensor([[True], [False]], dtype=torch.bool)
        completed = torch.zeros((2, 1), dtype=torch.bool)
        self.assertEqual(
            _MODULE.belt_release_gate(arrived, completed).tolist(), [False]
        )

    def test_completion_reopens_the_gate(self) -> None:
        arrived = torch.tensor([[True], [False]], dtype=torch.bool)
        completed = torch.tensor([[True], [False]], dtype=torch.bool)
        self.assertEqual(
            _MODULE.belt_release_gate(arrived, completed).tolist(), [True]
        )

    def test_single_batch_mode_stays_closed_after_completion(self) -> None:
        arrived = torch.tensor([[True], [True], [False]], dtype=torch.bool)
        completed = torch.tensor([[True], [True], [False]], dtype=torch.bool)
        self.assertEqual(
            _MODULE.belt_release_gate(
                arrived,
                completed,
                restart_after_departure=False,
            ).tolist(),
            [False],
        )

    def test_single_batch_mode_opens_again_only_after_reset(self) -> None:
        arrived = torch.zeros((3, 1), dtype=torch.bool)
        completed = torch.zeros((3, 1), dtype=torch.bool)
        self.assertEqual(
            _MODULE.belt_release_gate(
                arrived,
                completed,
                restart_after_departure=False,
            ).tolist(),
            [True],
        )

    def test_boxes_never_arrived_do_not_block(self) -> None:
        # 上游截抓的箱子从未到过工位：门不受它影响（悬空压停由 transfer_complete 管）。
        arrived = torch.zeros((3, 1), dtype=torch.bool)
        completed = torch.zeros((3, 1), dtype=torch.bool)
        self.assertEqual(
            _MODULE.belt_release_gate(arrived, completed).tolist(), [True]
        )

    def test_shape_mismatch_fails_fast(self) -> None:
        with self.assertRaises(ValueError):
            _MODULE.belt_release_gate(
                torch.zeros((2, 1), dtype=torch.bool),
                torch.zeros((3, 1), dtype=torch.bool),
            )


class GrabCycleStabilityTest(unittest.TestCase):
    """一次抱取的完整启停节拍：拿出流水线之前恒停、拿出之后恒跑。

    帧序列取自离线重放（运行时真参数按 14.148 基准换算）：推挤 → 回位 → 抬起 →
    横移出通道 → 手臂回摆 → 再移出。修复前该序列整带启停翻转 3 次、工位箱两次被
    反向写输送速度（与机器人拔河）；锁存化后翻转恰 1 次、全程不拖拽。
    """

    _BOX2 = (-5.82, 14.887, 0.775)
    _BOX3 = (-5.62, 15.637, 0.775)

    _RELEASE_STEPS = 25
    _FRONT_ARRIVAL_GROUP_SIZE = 1
    _RESTART_AFTER_DEPARTURE = True

    def setUp(self) -> None:
        self._completed = torch.zeros((3, 1), dtype=torch.bool)
        self._arrived = torch.zeros((3, 1), dtype=torch.bool)
        self._lifted = torch.zeros((3, 1), dtype=torch.bool)
        self._dwell = torch.zeros((3, 1), dtype=torch.int32)

    def _step(
        self,
        box1: tuple[float, float, float],
        box2: tuple[float, float, float] | None = None,
    ) -> list[bool]:
        """按事件层顺序推进一帧：三锁存更新 → 场内解除 → 双门 → drive mask。"""

        pos = torch.tensor(
            [list(box1), list(box2 or self._BOX2), list(self._BOX3)],
            dtype=torch.float32,
        ).unsqueeze(1)
        in_corridor = _MODULE.in_conveyor_corridor_mask(
            pos, x_range=_X_RANGE, y_range=_Y_RANGE
        )
        on_belt = _MODULE.on_belt_mask(
            pos,
            belt_top_z=_BELT_TOP_Z,
            z_tolerance=0.15,
            x_range=_X_RANGE,
            y_range=_Y_RANGE,
        )
        self._completed = _MODULE.update_departure_completion_latch(
            self._completed, in_corridor
        )
        self._arrived = _MODULE.update_arrival_latch(
            self._arrived, pos[..., 1], y_stop=_Y_STOP
        )
        self._arrived = _MODULE.latch_front_arrival_group(
            self._arrived,
            group_size=self._FRONT_ARRIVAL_GROUP_SIZE,
        )
        self._lifted, self._dwell, settled = _MODULE.update_lift_hold_latch(
            self._lifted,
            self._dwell,
            on_belt,
            in_corridor,
            self._completed,
            release_steps=self._RELEASE_STEPS,
        )
        if self._RESTART_AFTER_DEPARTURE:
            self._completed, self._arrived = _MODULE.release_resettled_boxes(
                self._completed, self._arrived, settled
            )
        gate = _MODULE.lift_hold_gate(self._lifted) & _MODULE.belt_release_gate(
            self._arrived,
            self._completed,
            restart_after_departure=self._RESTART_AFTER_DEPARTURE,
        )
        drive = _MODULE.queue_drive_mask(
            pos[..., 1],
            on_belt,
            _column([_HALF_D] * 3),
            y_stop=_Y_STOP,
            queue_gap=_GAP,
            transfer_complete=gate,
            completed=self._completed,
        )
        return [bool(v) for v in drive.squeeze(-1).tolist()]

    def test_shoving_the_stopped_lead_upstream_does_not_restart_the_belt(self) -> None:
        self.assertEqual(self._step((-5.42, _Y_STOP - 0.011, 0.775)), [False] * 3)
        # 抱取推挤：y 被推回停止线上游 2cm——修复前这里整带误启动且箱1 被拖拽。
        self.assertEqual(self._step((-5.42, _Y_STOP + 0.02, 0.775)), [False] * 3)

    def test_completed_box_swinging_back_respects_restart_policy(
        self,
    ) -> None:
        self._step((-5.42, _Y_STOP - 0.011, 0.775))  # 到位锁存
        self._step((-5.42, _Y_STOP - 0.011, 0.955))  # 抬起（z 出窗）
        expected = [False, self._RESTART_AFTER_DEPARTURE, self._RESTART_AFTER_DEPARTURE]
        # 横移出通道后，循环取件模式放行，单批次模式继续停线。
        self.assertEqual(self._step((-5.05, _Y_STOP - 0.01, 0.955)), expected)
        # 手臂回摆：箱1 短暂回到通道内 + 带面高度窗内。修复前它重新参与队首判定
        # 或重新满足 drive（被拖拽）；现在保持所配置的重启策略且箱1始终不被驱动。
        self.assertEqual(self._step((-5.12, _Y_STOP + 0.02, 0.885)), expected)
        self.assertEqual(self._step((-5.00, _Y_STOP + 0.02, 0.900)), expected)

    def test_full_grab_cycle_has_expected_belt_transitions(self) -> None:
        frames = [
            (-5.42, _Y_STOP - 0.011, 0.775),  # 停在工位
            (-5.42, _Y_STOP + 0.020, 0.775),  # 抱取推挤
            (-5.42, _Y_STOP - 0.011, 0.775),  # 回位
            (-5.42, _Y_STOP - 0.011, 0.895),  # 抬起 12cm（z 窗内）
            (-5.42, _Y_STOP - 0.011, 0.955),  # 抬起 18cm（z 出窗）
            (-5.05, _Y_STOP - 0.010, 0.955),  # 横移出通道
            (-5.12, _Y_STOP + 0.020, 0.885),  # 手臂回摆
            (-5.00, _Y_STOP + 0.020, 0.900),  # 再次移出
            (-4.60, _Y_STOP + 0.050, 0.900),  # 彻底搬离
        ]
        transitions = 0
        prev_running: bool | None = None
        for box1 in frames:
            drive = self._step(box1)
            self.assertFalse(drive[0], f"工位箱在 {box1} 被反向拖拽")
            running = drive[1]
            if prev_running is not None and running != prev_running:
                transitions += 1
            prev_running = running
        self.assertEqual(transitions, int(self._RESTART_AFTER_DEPARTURE))

    def test_completed_box_left_on_the_belt_still_blocks_a_follower(self) -> None:
        """防撞保底仍看物理 on_belt：已放行的箱体真实挡在带上时后车要刹住。"""

        completed = torch.tensor([[True], [False]], dtype=torch.bool)
        on_belt = torch.tensor([[True], [True]], dtype=torch.bool)
        # 前车尾 15.0+0.19，后车兜底线 15.19+0.19+0.07=15.45。
        drive = _MODULE.queue_drive_mask(
            _column([15.0, 15.35]),
            on_belt,
            _column([_HALF_D, _HALF_D]),
            y_stop=_Y_STOP,
            queue_gap=_GAP,
            transfer_complete=torch.tensor([True], dtype=torch.bool),
            completed=completed,
        )
        self.assertEqual([bool(v) for v in drive.squeeze(-1).tolist()], [False, False])
        # 间距拉开后后车恢复驱动；completed 前车自身仍不被驱动。
        drive = _MODULE.queue_drive_mask(
            _column([15.0, 15.55]),
            on_belt,
            _column([_HALF_D, _HALF_D]),
            y_stop=_Y_STOP,
            queue_gap=_GAP,
            transfer_complete=torch.tensor([True], dtype=torch.bool),
            completed=completed,
        )
        self.assertEqual([bool(v) for v in drive.squeeze(-1).tolist()], [False, True])

    def test_along_path_variant_is_isomorphic(self) -> None:
        """s 版关键帧同构：推挤(s 回落)不启动、completed 回摆不按停不拖拽。"""

        s_stop = 12.1945
        arrived = torch.zeros((2, 1), dtype=torch.bool)
        completed = torch.zeros((2, 1), dtype=torch.bool)
        lifted = torch.zeros((2, 1), dtype=torch.bool)
        dwell = torch.zeros((2, 1), dtype=torch.int32)

        def step(ss, on_belt, in_corridor):
            nonlocal arrived, completed, lifted, dwell
            ss_t = _column(ss)
            on_belt_t = torch.tensor(on_belt, dtype=torch.bool).unsqueeze(-1)
            corridor_t = torch.tensor(in_corridor, dtype=torch.bool).unsqueeze(-1)
            completed = _MODULE.update_departure_completion_latch(completed, corridor_t)
            arrived = _MODULE.update_arrival_latch_along_path(arrived, ss_t, s_stop=s_stop)
            lifted, dwell, settled = _MODULE.update_lift_hold_latch(
                lifted, dwell, on_belt_t, corridor_t, completed, release_steps=25
            )
            completed, arrived = _MODULE.release_resettled_boxes(
                completed, arrived, settled
            )
            gate = _MODULE.lift_hold_gate(lifted) & _MODULE.belt_release_gate(
                arrived, completed
            )
            drive = _MODULE.queue_drive_mask_along_path(
                ss_t,
                on_belt_t,
                _column([_HALF_D, _HALF_D]),
                s_stop=s_stop,
                queue_gap=_GAP,
                transfer_complete=gate,
                completed=completed,
            )
            return [bool(v) for v in drive.squeeze(-1).tolist()]

        # 队首到位 → 停；推挤让 s 回落到停止线上游 → 仍停。
        self.assertEqual(step([s_stop + 0.011, 11.4], [True, True], [True, True]), [False, False])
        self.assertEqual(step([s_stop - 0.020, 11.4], [True, True], [True, True]), [False, False])
        # 平面偏出（corridor False）→ 放行；回摆回到带上 → 不按停、不拖拽。
        self.assertEqual(step([s_stop + 0.01, 11.4], [False, True], [False, True]), [False, True])
        self.assertEqual(step([s_stop + 0.02, 11.5], [True, True], [True, True]), [False, True])


class LiftHoldLatchTest(unittest.TestCase):
    """悬空压停锁存：抬离带面即锁存，z 抖动不翻转，回带驻留满步数才解除。"""

    _K = 5  # 测试用小驻留步数

    def _roll(self, frames: list[tuple[bool, bool, bool]]) -> tuple[list[bool], list[bool]]:
        """frames 每项 (on_belt, in_corridor, completed)；返回逐帧 (lifted, settled)。"""

        lifted = torch.zeros((1, 1), dtype=torch.bool)
        dwell = torch.zeros((1, 1), dtype=torch.int32)
        lifted_seq, settled_seq = [], []
        for on_belt, in_corridor, completed in frames:
            lifted, dwell, settled = _MODULE.update_lift_hold_latch(
                lifted,
                dwell,
                torch.tensor([[on_belt]]),
                torch.tensor([[in_corridor]]),
                torch.tensor([[completed]]),
                release_steps=self._K,
            )
            lifted_seq.append(bool(lifted[0, 0]))
            settled_seq.append(bool(settled[0, 0]))
        return lifted_seq, settled_seq

    def test_lift_latches_and_z_flutter_does_not_unlatch(self) -> None:
        # 抬起 → 锁存；携行高度颠簸让 z 短暂回窗（on_belt=True 一两帧）→ 仍锁存。
        lifted, _ = self._roll(
            [
                (True, True, False),   # 躺在带上
                (False, True, False),  # 抬离带面 → 锁存
                (True, True, False),   # z 颠簸回窗一帧 → 保持
                (False, True, False),  # 又出窗 → 保持
                (True, True, False),
                (True, True, False),
            ]
        )
        self.assertEqual(lifted, [False, True, True, True, True, True])

    def test_settling_back_on_the_belt_releases_after_dwell(self) -> None:
        frames = [(False, True, False)] + [(True, True, False)] * self._K
        lifted, settled = self._roll(frames)
        # 回带的前 K-1 帧仍压停，第 K 帧驻留满 → 解除。
        self.assertEqual(lifted, [True, True, True, True, True, False])
        self.assertTrue(settled[-1])

    def test_planar_departure_immediately_clears_the_hold(self) -> None:
        lifted, _ = self._roll(
            [
                (False, True, False),  # 抬起 → 锁存
                (False, False, True),  # 横移出通道（completed）→ 即刻让位
            ]
        )
        self.assertEqual(lifted, [True, False])

    def test_gate_blocks_only_while_lifted(self) -> None:
        lifted = torch.tensor([[True], [False]], dtype=torch.bool)
        self.assertEqual(_MODULE.lift_hold_gate(lifted).tolist(), [False])
        self.assertEqual(
            _MODULE.lift_hold_gate(torch.zeros((2, 1), dtype=torch.bool)).tolist(),
            [True],
        )

    def test_shape_mismatch_fails_fast(self) -> None:
        with self.assertRaises(ValueError):
            _MODULE.update_lift_hold_latch(
                torch.zeros((2, 1), dtype=torch.bool),
                torch.zeros((2, 1), dtype=torch.int32),
                torch.zeros((3, 1), dtype=torch.bool),
                torch.zeros((2, 1), dtype=torch.bool),
                torch.zeros((2, 1), dtype=torch.bool),
                release_steps=5,
            )


class ReleaseResettledBoxesTest(unittest.TestCase):
    """completed 的场内解除：脱手掉回带面并稳定驻留后重新入队。"""

    def test_settled_completed_box_rejoins_the_queue(self) -> None:
        completed = torch.tensor([[True], [False]], dtype=torch.bool)
        arrived = torch.tensor([[True], [False]], dtype=torch.bool)
        settled = torch.tensor([[True], [True]], dtype=torch.bool)
        completed, arrived = _MODULE.release_resettled_boxes(completed, arrived, settled)
        self.assertEqual(completed.tolist(), [[False], [False]])
        self.assertEqual(arrived.tolist(), [[False], [False]])

    def test_brief_touchdown_does_not_release(self) -> None:
        completed = torch.tensor([[True]], dtype=torch.bool)
        arrived = torch.tensor([[True]], dtype=torch.bool)
        settled = torch.tensor([[False]], dtype=torch.bool)
        completed, arrived = _MODULE.release_resettled_boxes(completed, arrived, settled)
        self.assertEqual(completed.tolist(), [[True]])
        self.assertEqual(arrived.tolist(), [[True]])

    def test_shape_mismatch_fails_fast(self) -> None:
        with self.assertRaises(ValueError):
            _MODULE.release_resettled_boxes(
                torch.zeros((2, 1), dtype=torch.bool),
                torch.zeros((2, 1), dtype=torch.bool),
                torch.zeros((3, 1), dtype=torch.bool),
            )


class InterceptGrabStabilityTest(GrabCycleStabilityTest):
    """双机单批次流程：前两箱同时抱取，首批到位后不再启动流水线。

    复用 GrabCycleStabilityTest 的事件层链路 helper，并启用运行时相同的前两箱成组
    到位与禁止重启参数。核心断言：任一箱或两箱横向离线后，box3 都不补位。
    """

    _FRONT_ARRIVAL_GROUP_SIZE = 2
    _RESTART_AFTER_DEPARTURE = False

    def test_both_boxes_departing_never_restarts_the_belt(self) -> None:
        box2_lane = self._BOX2[0]
        # box1 在工位、box2 停排队位：整带停。
        self.assertEqual(self._step((-5.42, _Y_STOP - 0.011, 0.775)), [False] * 3)
        # robot1 把 box1 横移出通道，首批到位锁存仍保持整带停线。
        self.assertEqual(
            self._step((-5.05, _Y_STOP - 0.01, 0.955)), [False, False, False]
        )
        # robot2 把 box2 抬离带面，携行颠簸穿越 z 窗也不改变终止停线状态。
        self.assertEqual(
            self._step((-4.80, _Y_STOP, 0.955), (box2_lane, 14.85, 0.960)),
            [False, False, False],
        )
        self.assertEqual(
            self._step((-4.80, _Y_STOP, 0.955), (box2_lane, 14.85, 0.900)),
            [False, False, False],
        )
        self.assertEqual(
            self._step((-4.80, _Y_STOP, 0.955), (box2_lane, 14.85, 0.960)),
            [False, False, False],
        )
        # 两箱都横移出通道后仍不重启，box3 不补位。
        self.assertEqual(
            self._step((-4.80, _Y_STOP, 0.955), (-5.00, 14.85, 0.960)),
            [False, False, False],
        )

    def test_both_boxes_returning_to_belt_does_not_clear_terminal_stop(self) -> None:
        # 首批到位后两箱均离线，完成位已经锁存。
        self._step((-5.42, _Y_STOP - 0.011, 0.775))
        self._step((-5.00, _Y_STOP, 0.960), (-5.00, _Y_STOP + 0.75, 0.960))

        # 即使两箱随后落回带面并驻留超过旧自愈窗口，也不能重新入队或启动 box3。
        drive = None
        for _ in range(self._RELEASE_STEPS + 1):
            drive = self._step(
                (-5.42, _Y_STOP + 0.30, 0.775),
                (-5.82, _Y_STOP + 1.05, 0.775),
            )
        self.assertEqual(drive, [False, False, False])


class LoopModeCompletedExclusionTest(unittest.TestCase):
    """循环模式（y_stop=None）同样兑现 completed 剔除契约。"""

    def test_completed_box_is_not_driven_in_loop_mode(self) -> None:
        drive = _MODULE.queue_drive_mask(
            _column([15.0]),
            torch.tensor([[True]], dtype=torch.bool),
            _column([_HALF_D]),
            y_stop=None,
            queue_gap=_GAP,
            completed=torch.tensor([[True]], dtype=torch.bool),
        )
        self.assertEqual([bool(v) for v in drive.squeeze(-1).tolist()], [False])
        drive = _MODULE.queue_drive_mask_along_path(
            _column([11.0]),
            torch.tensor([[True]], dtype=torch.bool),
            _column([_HALF_D]),
            s_stop=None,
            queue_gap=_GAP,
            completed=torch.tensor([[True]], dtype=torch.bool),
        )
        self.assertEqual([bool(v) for v in drive.squeeze(-1).tolist()], [False])


if __name__ == "__main__":
    unittest.main()
