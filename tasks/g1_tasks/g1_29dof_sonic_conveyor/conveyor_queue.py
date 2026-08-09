# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Dependency-free belt-surface and queue arithmetic for the conveyor scene.

这里只依赖 torch，**刻意不 import Isaac Sim**——``conveyor_events`` 一旦被导入就要
拉起 carb/Kit，普通 unittest 跑不起来。把判据抽到这一层之后，"谁在带上""谁该被
驱动""挡停放行"全都能在不启 Kit 的情况下逐条验证，事件函数只剩读写场景状态。

⚠️ 两个函数都在每个物理步被调用，**不允许任何 GPU→CPU 同步**
（``.item()`` / ``.any()`` / ``if tensor``）。历史教训见 ``conveyor_events``
里 ``drive_totes_on_conveyor`` 的性能注释：早期版本用 ``if not on_belt.any()``
提前返回，两个筐每步最多 6 次同步，实测让 env.step 从 ~120 ms 涨到 ~160 ms。
"""

from __future__ import annotations

import torch


def on_belt_mask(
    pos_local: torch.Tensor,
    *,
    belt_top_z: float,
    z_tolerance: float,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
) -> torch.Tensor:
    """"确实还躺在滚轮面上"的判定；输出形状是 ``pos_local`` 去掉最后一维。

    * ``belt_top_z ± z_tolerance``：物体原点在底面，静置时 z≈0.775。被机器人拎起
      或掉到地上就落出窗口 → 立即停止驱动，不会把抓在手里的东西硬拖走。
    * ``x_range`` / ``y_range``：可用带面（略放宽于碰撞板）。
    """

    return (
        (pos_local[..., 2] >= belt_top_z - z_tolerance)
        & (pos_local[..., 2] <= belt_top_z + z_tolerance)
        & (pos_local[..., 0] >= x_range[0])
        & (pos_local[..., 0] <= x_range[1])
        & (pos_local[..., 1] >= y_range[0])
        & (pos_local[..., 1] <= y_range[1])
    )


def queue_drive_mask(
    ys: torch.Tensor,
    on_belt: torch.Tensor,
    half_lengths: torch.Tensor,
    *,
    y_stop: float | None,
    queue_gap: float,
) -> torch.Tensor:
    """整带节拍启停：算出这一帧哪些箱子该继续被送走。

    ``ys`` / ``on_belt`` 形状同为 ``(N, E)``——N 个箱子 × E 个 env。
    ``half_lengths`` 是 ``(N, 1)``（或可广播到 ``(N, E)``）的每个箱子沿输送方向的
    半长。返回与 ``ys`` 同形的 bool。

    语义是**整条带一起启停**，而不是逐个积放：

        belt_running = (最下游那个仍在带面的箱子的 y) > y_stop
        drive[i] = on_belt[i] & belt_running & 没顶到前车

    也就是说队首一到工位，整条带立刻停下，后面的箱子**原地保持当前间距**，不会继续
    往前挤到贴紧前车。队首被取走后，剩下的箱子里最下游那个成为新队首，整带重新启动，
    一起前进到它也压到工位为止——每取走一件，整列前进一格，就是真实节拍输送线的样子。

    第三个因子是**防撞保底**，正常情况下不会触发：整带同起同停时相对间距恒定，
    而出生间距（默认 0.6）远大于任何一对箱子的最小净距。它只在两箱因摩擦/质量差异
    缓慢漂移到快要追尾时才介入，约束是

        ys[i] > (前车 y + 前车半长) + 自己半长 + queue_gap

    即**前车尾部 + 自己半长 + 净间隙**，而不是一个统一的中心距——流水线上两种箱型
    尺寸不同（0.38 与 0.50），统一中心距要么让大箱互相穿模、要么在小箱之间留出突兀
    的空档；按各自半长算，无论怎么交错，兜底的净空隙都恒为 ``queue_gap``。

    整套判据不需要任何显式状态机，也不需要知道谁被抓走了：语义完全由当前帧的位置
    推出来，天然可复位、可断点续跑、双机一致（镜像端根本不跑驱动事件，位姿全部来自
    权威端的 scene_state 帧）。

    ``y_stop=None``（``ISAACLAB_CONVEYOR_Y_STOP<=0`` 的循环模式）下没有工位停止线，
    整带长跑不停，只剩防撞保底。
    """

    # —— 防撞保底 ——
    # 前车用尾部（上游边缘）参与比较，本车再加自己的半长，于是两侧尺寸各自计入，
    # 不需要按 argmax 去 gather 前车是谁。
    tails = ys + half_lengths  # (N, E)
    # ahead[i, j] = "j 在 i 的下游且 j 还在带上"，即 j 会挡住 i。
    # ys.unsqueeze(0) 是 j 轴 (1, N, E)，ys.unsqueeze(1) 是 i 轴 (N, 1, E)。
    ahead = (ys.unsqueeze(0) < ys.unsqueeze(1)) & on_belt.unsqueeze(0)  # (N, N, E)
    # 无前车的行整行取到哨兵值，加上半长和间隙后仍远小于任何真实坐标，于是该行的
    # 防撞约束恒真——省掉一次"判空"分支（分支就意味着同步）。
    no_blocker = ys.new_tensor(-1.0e9)
    blocker_tail = torch.where(ahead, tails.unsqueeze(0), no_blocker).amax(dim=1)  # (N, E)
    clear_of_leader = ys > blocker_tail + half_lengths + queue_gap  # (N, E)

    if y_stop is None:
        return on_belt & clear_of_leader

    # —— 整带节拍 ——
    # 只看仍在带面的箱子；被抓走的那个不参与，于是它一离开带面，整带就自动重启。
    # 全部离开带面时 lead_y 取到哨兵大值、belt_running 为真，但 on_belt 全假，
    # drive 仍是全假，无需额外分支。
    far_away = ys.new_tensor(1.0e9)
    lead_y = torch.where(on_belt, ys, far_away).amin(dim=0)  # (E,)
    belt_running = (lead_y > y_stop).unsqueeze(0)  # (1, E) → 广播到 (N, E)
    return on_belt & belt_running & clear_of_leader
