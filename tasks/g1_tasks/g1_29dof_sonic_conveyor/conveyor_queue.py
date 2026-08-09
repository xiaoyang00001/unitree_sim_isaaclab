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
    """挡停放行：算出这一帧哪些箱子该继续被送走。

    ``ys`` / ``on_belt`` 形状同为 ``(N, E)``——N 个箱子 × E 个 env。
    ``half_lengths`` 是 ``(N, 1)``（或可广播到 ``(N, E)``）的每个箱子沿输送方向的
    半长。返回与 ``ys`` 同形的 bool。

    每个箱子的停止线是::

        stop_line[i] = max(y_stop, (前车 y + 前车半长) + 自己半长 + queue_gap)

    也就是**前车尾部 + 自己半长 + 净间隙**，而不是一个统一的中心距。流水线上两种
    箱型尺寸不同（0.38 与 0.50），统一中心距要么让大箱互相穿模、要么在小箱之间留出
    突兀的空档；按各自半长算，无论怎么交错，箱子之间的**净空隙**恒为 ``queue_gap``。

    "前车"取**下游（-Y）方向上仍在带面的最近一个箱子**。于是：

    * 队首没有前车 → 停止线就是工位 ``y_stop``，流到工位停住等抓取；
    * 后车被前车顶住 → 在上游排队，与前车保持 ``queue_gap`` 的净空隙；
    * 工位那个箱子被拎走 → ``on_belt`` 转假 → 它不再是任何人的前车 → 下一个箱子
      的停止线塌回 ``y_stop``，自动补位。

    "抓走后放行下一个"因此不需要任何显式状态机，也不需要知道谁被抓走了：队列语义
    完全由当前帧的位置推出来，天然可复位、可断点续跑、双机一致（镜像端根本不跑
    驱动事件，位姿全部来自权威端的 scene_state 帧）。

    ``y_stop=None``（``ISAACLAB_CONVEYOR_Y_STOP<=0`` 的循环模式）下退化为只受前车
    约束、一路流到带尾。
    """

    # 前车用尾部（上游边缘）参与比较，本车再加自己的半长——于是两侧尺寸各自计入，
    # 不需要按 argmax 去 gather 前车是谁。
    tails = ys + half_lengths  # (N, E)
    # ahead[i, j] = "j 在 i 的下游且 j 还在带上"，即 j 会挡住 i。
    # ys.unsqueeze(0) 是 j 轴 (1, N, E)，ys.unsqueeze(1) 是 i 轴 (N, 1, E)。
    ahead = (ys.unsqueeze(0) < ys.unsqueeze(1)) & on_belt.unsqueeze(0)  # (N, N, E)
    # 无前车的行整行取到哨兵值，加上半长和间隙后仍远小于任何真实坐标，会被下面的
    # clamp_min 吃掉——省掉一次"判空"分支（分支就意味着同步）。
    no_blocker = ys.new_tensor(-1.0e9)
    blocker_tail = torch.where(ahead, tails.unsqueeze(0), no_blocker).amax(dim=1)  # (N, E)
    queue_line = blocker_tail + half_lengths + queue_gap
    stop_line = queue_line if y_stop is None else queue_line.clamp_min(y_stop)
    return on_belt & (ys > stop_line)
