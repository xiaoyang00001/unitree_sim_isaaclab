"""挡停放行队列的判据单测（不启 Kit，只用 torch）。

这些用例锁的是纸箱流水线的核心语义：队首停工位、后车排队、抓走后自动补位。
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


# 两个 d01 相邻时的中心距（旧版统一 queue_pitch 恰好等于它，行为不变）。
_PITCH_DD = _HALF_D + _GAP + _HALF_D


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
    def test_lead_box_runs_until_the_workstation_then_stops(self) -> None:
        """队首只受工位约束：到 y_stop 之前一直走，到了就停。"""

        self.assertEqual(_drive([16.0], [True]), [True])
        self.assertEqual(_drive([_Y_STOP + 0.001], [True]), [True])
        self.assertEqual(_drive([_Y_STOP], [True]), [False])
        self.assertEqual(_drive([_Y_STOP - 0.05], [True]), [False])

    def test_follower_is_blocked_one_gap_behind_the_lead(self) -> None:
        """队首停在工位后，后车顶在"前车尾部 + 自己半长 + gap"处，不会撞上去。"""

        lead = _Y_STOP
        self.assertEqual(_drive([lead, lead + _PITCH_DD + 0.2], [True, True]), [False, True])
        self.assertEqual(_drive([lead, lead + _PITCH_DD - 0.01], [True, True]), [False, False])
        self.assertEqual(_drive([lead, lead + _PITCH_DD - 0.1], [True, True]), [False, False])

    def test_stop_line_is_a_strict_threshold(self) -> None:
        """停止线是严格不等号：正好压在线上就不再驱动，越过一点点才驱动。

        ⚠️ 判据在 ``y == stop_line`` 处是临界的，float32 下这一点的取值由舍入
        决定，所以其它用例一律留出 ≥1 cm 的余量而不是拿等号做断言。实际运行也
        永远不会精确停在线上：箱子是被摩擦拖停的，会越线一小段（现有塑料筐实测
        越过 y_stop 约 11 mm），越线后驱动即关闭，不会来回抖。
        """

        self.assertEqual(_drive([_Y_STOP + 0.01], [True]), [True])
        self.assertEqual(_drive([_Y_STOP - 0.01], [True]), [False])

    def test_full_queue_settles_into_evenly_spaced_slots(self) -> None:
        """5 个箱子全部停稳后占据 y_stop + k*pitch 这一串槽位。

        每个箱子取比自己停止线低 1 cm 的位置代表"已停稳"（见
        ``test_stop_line_is_a_strict_threshold`` 关于临界点的说明）。
        """

        settled = [_Y_STOP + index * (_PITCH_DD - 0.01) for index in range(5)]
        self.assertEqual(_drive(settled, [True] * 5), [False] * 5)

        # 任何一个还没到自己的槽位就应该继续走。
        for index in range(1, 5):
            nudged = list(settled)
            nudged[index] += 0.3
            expected = [False] * 5
            expected[index] = True
            self.assertEqual(_drive(nudged, [True] * 5), expected, f"box {index}")

    def test_picking_the_workstation_box_releases_the_next_one(self) -> None:
        """核心语义：拎走工位那个箱子，下一个立刻被放行——不需要任何状态机。"""

        queued = [_Y_STOP + index * (_PITCH_DD - 0.01) for index in range(3)]

        # 抓取前：三个都停着排队。
        self.assertEqual(_drive(queued, [True, True, True]), [False, False, False])

        # 队首被拎起（离开带面窗口）后，它不再挡住任何人：第二个恢复行进，
        # 而第三个仍被第二个挡着——一次只放行一个。
        self.assertEqual(_drive(queued, [False, True, True]), [False, True, False])

    def test_a_box_off_the_belt_never_blocks_and_never_drives(self) -> None:
        """离开带面的箱子既不被驱动，也不该继续充当"前车"。"""

        # 下游有个箱子，但它已被搬走 → 上游那个按工位停，不按它停。
        self.assertEqual(_drive([15.0, 16.0], [False, True]), [False, True])
        self.assertEqual(_drive([15.0, 15.2], [False, True]), [False, True])

    def test_upstream_box_is_unaffected_by_a_downstream_gap(self) -> None:
        """前车已经远远走开时，后车不受队列约束，只看工位。"""

        self.assertEqual(_drive([_Y_STOP, 17.0], [True, True]), [False, True])

    def test_loop_mode_drops_the_workstation_line_but_keeps_the_queue(self) -> None:
        """y_stop=None（循环模式）：没有工位停止线，但后车仍不许撞前车。"""

        self.assertEqual(_drive([11.0], [True], y_stop=None), [True])
        self.assertEqual(_drive([_Y_STOP], [True], y_stop=None), [True])
        # 前车挡着时后车照样停。
        self.assertEqual(_drive([12.0, 12.2], [True, True], y_stop=None), [True, False])

    def test_order_of_object_names_does_not_matter(self) -> None:
        """判据只看坐标，不看清单顺序——两端进程排列不同也不会分叉。"""

        # 清单顺序打乱：第 0 个其实是队列第二位（已停在槽位内侧 1 cm），
        # 第 1 个才是队首，第 2 个还在上游。
        ys = [_Y_STOP + _PITCH_DD - 0.01, _Y_STOP, 17.0]
        self.assertEqual(_drive(ys, [True, True, True]), [False, False, True])

    def test_multiple_envs_are_resolved_independently(self) -> None:
        """(N, E) 的 E 维必须逐 env 独立，不能串味。"""

        ys = torch.tensor(
            [[_Y_STOP, 16.0], [_Y_STOP + _PITCH_DD - 0.01, 17.0]], dtype=torch.float32
        )
        on_belt = torch.ones_like(ys, dtype=torch.bool)
        halves = torch.full((2, 1), _HALF_D)
        mask = _MODULE.queue_drive_mask(ys, on_belt, halves, y_stop=_Y_STOP, queue_gap=_GAP)

        # env 0：两个都停（队首在工位、后车在槽位）；env 1：两个都还在上游、都走。
        self.assertEqual(mask.tolist(), [[False, True], [False, True]])

    def test_mixed_box_sizes_keep_a_constant_clear_gap(self) -> None:
        """两种箱型混排时，恒定的是**净空隙**而不是中心距。

        这正是把判据从"统一 queue_pitch"改成"前车尾部 + 自己半长 + gap"的原因：
        d01(0.38) 和 c01(0.50) 交错时，统一中心距要么让大箱穿模、要么在小箱之间
        留出突兀的空档。
        """

        for lead_half, follow_half in (
            (_HALF_D, _HALF_C),  # 小箱在前、大箱在后
            (_HALF_C, _HALF_D),  # 大箱在前、小箱在后
            (_HALF_C, _HALF_C),
        ):
            with self.subTest(lead=lead_half, follow=follow_half):
                lead = _Y_STOP
                slot = _slot_after(lead, lead_half, follow_half)
                halves = [lead_half, follow_half]
                # 恰好停在槽位内侧 → 不驱动；离槽位还有距离 → 继续走。
                self.assertEqual(
                    _drive([lead, slot - 0.01], [True, True], halves=halves), [False, False]
                )
                self.assertEqual(
                    _drive([lead, slot + 0.2], [True, True], halves=halves), [False, True]
                )
                # 两箱之间的净空隙确实是 gap：后车尾侧边缘 - 前车头侧边缘。
                clear = (slot - follow_half) - (lead + lead_half)
                self.assertAlmostEqual(clear, _GAP, places=6)

    def test_default_interleaved_queue_settles_without_overlap(self) -> None:
        """默认 d01/c01 交错的 5 箱队列停稳后逐个相邻、互不穿模。"""

        halves = [_HALF_D, _HALF_C, _HALF_D, _HALF_C, _HALF_D]
        settled = [_Y_STOP]
        for index in range(1, 5):
            settled.append(_slot_after(settled[-1], halves[index - 1], halves[index]) - 0.01)

        self.assertEqual(_drive(settled, [True] * 5, halves=halves), [False] * 5)

        # 相邻箱子的实体边缘不得重叠。
        for index in range(4):
            lead_edge = settled[index] + halves[index]
            follow_edge = settled[index + 1] - halves[index + 1]
            self.assertGreater(follow_edge, lead_edge, f"box {index} 与 {index + 1} 穿模")

        # 拎走队首后，第二个（c01）放行、第三个仍被挡住。
        released = _drive(settled, [False, True, True, True, True], halves=halves)
        self.assertEqual(released, [False, True, False, False, False])

    def test_a_big_box_blocks_further_upstream_than_a_small_one(self) -> None:
        """同一位置上，大箱比小箱把后车顶得更远——半长确实进了判据。"""

        lead = _Y_STOP
        follow = _slot_after(lead, _HALF_D, _HALF_D) + 0.005

        # 前车是 d01 时后车已越过槽位 → 放行。
        self.assertEqual(
            _drive([lead, follow], [True, True], halves=[_HALF_D, _HALF_D]), [False, True]
        )
        # 同样位置，前车换成更大的 c01 → 槽位后移，后车被挡住。
        self.assertEqual(
            _drive([lead, follow], [True, True], halves=[_HALF_C, _HALF_D]), [False, False]
        )

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


if __name__ == "__main__":
    unittest.main()
