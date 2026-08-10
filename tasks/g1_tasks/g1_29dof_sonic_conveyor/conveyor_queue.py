# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Dependency-free belt-surface and queue arithmetic for the conveyor scene.

这里只依赖 torch，**刻意不 import Isaac Sim**——``conveyor_events`` 一旦被导入就要
拉起 carb/Kit，普通 unittest 跑不起来。把判据抽到这一层之后，"谁在带上"
"谁已投框""谁该被驱动""挡停放行"全都能在不启 Kit 的情况下逐条验证，
事件函数只剩读写场景状态。

⚠️ 本模块的函数都在每个物理步被调用，**不允许任何 GPU→CPU 同步**
（``.item()`` / ``.any()`` / ``if tensor``）。历史教训见 ``conveyor_events``
里 ``drive_totes_on_conveyor`` 的性能注释：早期版本用 ``if not on_belt.any()``
提前返回，两个筐每步最多 6 次同步，实测让 env.step 从 ~120 ms 涨到 ~160 ms。
"""

from __future__ import annotations

import math

import torch


def on_belt_mask(
    pos_local: torch.Tensor,
    *,
    belt_top_z: float,
    z_tolerance: float,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    extra_rects: tuple[tuple[float, float, float, float], ...] = (),
) -> torch.Tensor:
    """"确实还躺在滚轮面上"的判定；输出形状是 ``pos_local`` 去掉最后一维。

    * ``belt_top_z ± z_tolerance``：物体原点在底面，静置时 z≈0.775。被机器人拎起
      或掉到地上就落出窗口 → 立即停止驱动，不会把抓在手里的东西硬拖走。
    * ``x_range`` / ``y_range``：可用带面（略放宽于碰撞板）。
    * ``extra_rects``：与主矩形取**并集**的附加矩形（每项 ``(x0, x1, y0, y1)``）。
      入口弯道方案的 L 形带面 = 主线矩形 ∪ 拐角补块 ∪ X 支线条带；矩形个数是
      Python 常量，循环展开不引入张量同步。
    """

    x = pos_local[..., 0]
    y = pos_local[..., 1]
    in_plane = (
        (x >= x_range[0])
        & (x <= x_range[1])
        & (y >= y_range[0])
        & (y <= y_range[1])
    )
    for x0, x1, y0, y1 in extra_rects:
        in_plane = in_plane | ((x >= x0) & (x <= x1) & (y >= y0) & (y <= y1))
    return (
        (pos_local[..., 2] >= belt_top_z - z_tolerance)
        & (pos_local[..., 2] <= belt_top_z + z_tolerance)
        & in_plane
    )


def inside_drop_zone_mask(
    pos_local: torch.Tensor,
    *,
    root_zones: tuple[tuple[float, float, float, float, float, float], ...],
) -> torch.Tensor:
    """判断箱底根位置是否进入任一目标分拣框的保守接收区。

    ``root_zones`` 每项是 ``(x_min, x_max, y_min, y_max, z_min, z_max)``；区域已经
    按最大箱型从分拣框外包围盒向内缩，所以这里只判断根位置，不在热路径里重算每个
    箱型的几何。多个区域取并集，对应两台机器人各自一只目标框。

    输出形状与 ``pos_local[..., 0]`` 相同。实现只用张量布尔运算，不产生
    GPU→CPU 同步。
    """

    inside = torch.zeros_like(pos_local[..., 0], dtype=torch.bool)
    x = pos_local[..., 0]
    y = pos_local[..., 1]
    z = pos_local[..., 2]
    for x_min, x_max, y_min, y_max, z_min, z_max in root_zones:
        inside = inside | (
            (x >= x_min)
            & (x <= x_max)
            & (y >= y_min)
            & (y <= y_max)
            & (z >= z_min)
            & (z <= z_max)
        )
    return inside


def update_drop_completion_latch(
    completed: torch.Tensor,
    dwell_counts: torch.Tensor,
    settled_in_drop_zone: torch.Tensor,
    *,
    dwell_steps: int,
) -> tuple[torch.Tensor, torch.Tensor]:
    """累计稳定入框帧数，并锁存每个箱子的投放完成状态。

    三个输入都采用 ``(N, E)``：N 个箱子 × E 个 env。未完成箱只有连续
    ``dwell_steps`` 次同时满足“位于接收区且速度已稳定”才会完成；中途离区或重新运动
    会把连续计数清零。完成后状态保持为真，不会因框内堆叠、碰撞或后续清框而反悔；
    场景复位由事件 term 的 ``reset`` 显式清除锁存。

    函数本身无原地修改，方便用纯 torch 单测锁住状态迁移，也避免 GPU→CPU 同步。
    """

    if completed.shape != dwell_counts.shape or completed.shape != settled_in_drop_zone.shape:
        raise ValueError(
            "completed、dwell_counts 与 settled_in_drop_zone 形状必须一致："
            f"{tuple(completed.shape)} / {tuple(dwell_counts.shape)} / "
            f"{tuple(settled_in_drop_zone.shape)}"
        )
    if dwell_steps < 1:
        raise ValueError(f"dwell_steps 必须 >= 1，收到 {dwell_steps}")

    next_counts = torch.where(
        settled_in_drop_zone,
        torch.clamp(dwell_counts + 1, max=dwell_steps),
        torch.zeros_like(dwell_counts),
    )
    next_completed = completed | (next_counts >= dwell_steps)
    # 已锁存项把计数固定在阈值，便于调试，也避免物体离框后计数表面回到 0。
    next_counts = torch.where(
        next_completed,
        torch.full_like(next_counts, dwell_steps),
        next_counts,
    )
    return next_completed, next_counts


def transfer_complete_mask(
    on_belt: torch.Tensor,
    drop_completed: torch.Tensor,
) -> torch.Tensor:
    """返回每个 env 是否已完成当前取放、可以让队列前进一步。

    每个箱子必须满足二选一：仍在流水线上，或它的“稳定入框”完成位已经锁存。于是
    队首被抱起后会形成一个“既不在带上、也未完成投框”的缺口，整带继续保持停止；
    箱子稳定落入框内后缺口闭合才放行。下一箱被抱起会再次形成新缺口，因此不会被
    首次投放永久解锁。
    """

    if on_belt.shape != drop_completed.shape:
        raise ValueError(
            "on_belt 与 drop_completed 形状必须一致："
            f"{tuple(on_belt.shape)} vs {tuple(drop_completed.shape)}"
        )
    return (on_belt | drop_completed).all(dim=0)


def path_progress(
    pos_local: torch.Tensor,
    *,
    corner_center: tuple[float, float],
    radius: float,
    s_origin_x: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """L 形车道的沿路径距离 s 与下游航向 (hx, hy)——**西拐版**。

    路径三段（世界系；与 endless_intake.path_point / path_s_of_point 同一分段，
    与东拐存档 f3995f1 镜像——x 分量与航向符号全部反转）：

    * 支线段（y > cy 且 x < cx）：s = x − s_origin_x，航向 (+1, 0)；
    * 圆角弧（y > cy 且 x ≥ cx）：θ = atan2(x−cx, y−cy)∈[0,π/2]，
      s = (cx−s_origin_x) + R·θ，航向 (cosθ, −sinθ)；
    * 主线段（y ≤ cy）：s = 弧段末端 + (cy − y)，航向 (0, -1)。

    返回三个与输入去掉最后一维同形的张量。全程无分支/无 GPU→CPU 同步；
    ``atan2`` 的两参数在弧段区域内都非负（clamp 兜住轻微出格的箱子），
    (0,0) 时 torch.atan2 返回 0，不产生 NaN。s 对箱子的**实际位置**取值：
    偏离车道中线的箱子按它自己的极角/坐标算进度，判据仍然连续。
    """

    x = pos_local[..., 0]
    y = pos_local[..., 1]
    cx, cy = corner_center
    s_arc_start = cx - s_origin_x
    s_arc_end = s_arc_start + radius * math.pi / 2.0

    main = y <= cy
    branch = (~main) & (x < cx)
    theta = torch.atan2((x - cx).clamp(min=0.0), (y - cy).clamp(min=0.0))

    s = torch.where(
        main,
        s_arc_end + (cy - y),
        torch.where(branch, x - s_origin_x, s_arc_start + radius * theta),
    )
    zeros = torch.zeros_like(x)
    ones = torch.ones_like(x)
    hx = torch.where(main, zeros, torch.where(branch, ones, torch.cos(theta)))
    hy = torch.where(main, -ones, torch.where(branch, zeros, -torch.sin(theta)))
    return s, hx, hy


def queue_drive_mask_along_path(
    ss: torch.Tensor,
    on_belt: torch.Tensor,
    half_lengths: torch.Tensor,
    *,
    s_stop: float | None,
    queue_gap: float,
    transfer_complete: torch.Tensor | None = None,
) -> torch.Tensor:
    """沿路径距离 s 的整带节拍判据；语义与 ``queue_drive_mask`` 逐项镜像。

    s 向下游**递增**（y 版里下游是 y 递减），直接复用 y 版实现：喂入 ``-s``
    之后"下游=更小"恰好翻转为"下游=更大"——队首判定、整带启停、按各箱半长
    算的防撞兜底全部逐项对应，无需第二份实现。
    """

    return queue_drive_mask(
        -ss,
        on_belt,
        half_lengths,
        y_stop=None if s_stop is None else -s_stop,
        queue_gap=queue_gap,
        transfer_complete=transfer_complete,
    )


def queue_drive_mask(
    ys: torch.Tensor,
    on_belt: torch.Tensor,
    half_lengths: torch.Tensor,
    *,
    y_stop: float | None,
    queue_gap: float,
    transfer_complete: torch.Tensor | None = None,
) -> torch.Tensor:
    """整带节拍启停：算出这一帧哪些箱子该继续被送走。

    ``ys`` / ``on_belt`` 形状同为 ``(N, E)``——N 个箱子 × E 个 env。
    ``half_lengths`` 是 ``(N, 1)``（或可广播到 ``(N, E)``）的每个箱子沿输送方向的
    半长。返回与 ``ys`` 同形的 bool。

    语义是**整条带一起启停**，而不是逐个积放：

        belt_running = ((最下游那个仍在带面的箱子的 y) > y_stop) & transfer_complete
        drive[i] = on_belt[i] & belt_running & 没顶到前车

    也就是说队首一到工位，整条带立刻停下，后面的箱子**原地保持当前间距**，不会继续
    往前挤到贴紧前车。队首被取走后 ``transfer_complete=False``，整带仍保持停止；
    只有它进入目标分拣框、门控恢复为真，剩余队列才一起前进到新队首压住工位。

    第三个因子是**防撞保底**，正常情况下不会触发：整带同起同停时相对间距恒定，
    而出生间距（默认 0.6）远大于任何一对箱子的最小净距。它只在两箱因摩擦/质量差异
    缓慢漂移到快要追尾时才介入，约束是

        ys[i] > (前车 y + 前车半长) + 自己半长 + queue_gap

    即**前车尾部 + 自己半长 + 净间隙**，而不是一个统一的中心距——流水线上两种箱型
    尺寸不同（0.38 与 0.50），统一中心距要么让大箱互相穿模、要么在小箱之间留出突兀
    的空档；按各自半长算，无论怎么交错，兜底的净空隙都恒为 ``queue_gap``。

    ``transfer_complete`` 是可选的 ``(E,)`` bool 张量；不传时保持通用队列函数的
    历史行为，由调用方决定是否启用投框门。投放锁存由权威端事件维护，并随 env reset
    清零；镜像端根本不跑驱动事件，箱子位姿仍全部来自权威端的 scene_state 帧。

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
    # 只看仍在带面的箱子来确定新队首；被抓走的箱子是否允许新队首启动，则由下方
    # transfer_complete 统一门控。全部离开带面时 lead_y 取到哨兵大值，但 on_belt
    # 全假，drive 仍是全假，无需额外分支。
    far_away = ys.new_tensor(1.0e9)
    lead_y = torch.where(on_belt, ys, far_away).amin(dim=0)  # (E,)
    belt_running = lead_y > y_stop  # (E,)
    if transfer_complete is not None:
        if transfer_complete.shape != belt_running.shape:
            raise ValueError(
                "transfer_complete 形状必须为 (E,)："
                f"{tuple(transfer_complete.shape)} vs {tuple(belt_running.shape)}"
            )
        belt_running = belt_running & transfer_complete
    belt_running = belt_running.unsqueeze(0)  # (1, E) → 广播到 (N, E)
    return on_belt & belt_running & clear_of_leader
