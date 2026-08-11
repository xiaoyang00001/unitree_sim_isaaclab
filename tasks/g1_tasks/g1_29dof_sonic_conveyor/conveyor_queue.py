# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Dependency-free belt-surface and queue arithmetic for the conveyor scene.

这里只依赖 torch，**刻意不 import Isaac Sim**——``conveyor_events`` 一旦被导入就要
拉起 carb/Kit，普通 unittest 跑不起来。把判据抽到这一层之后，"谁在带上"
"谁仍在流水线上方""谁该被驱动""挡停放行"全都能在不启 Kit 的情况下逐条验证，
事件函数只剩读写场景状态。

⚠️ 本模块的函数都在每个物理步被调用，**不允许任何 GPU→CPU 同步**
（``.item()`` / ``.any()`` / ``if tensor``）。历史教训见 ``conveyor_events``
里 ``drive_totes_on_conveyor`` 的性能注释：早期版本用 ``if not on_belt.any()``
提前返回，两个筐每步最多 6 次同步，实测让 env.step 从 ~120 ms 涨到 ~160 ms。
"""

from __future__ import annotations

import math

import torch


def in_conveyor_corridor_mask(
    pos_local: torch.Tensor,
    *,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    extra_rects: tuple[tuple[float, float, float, float], ...] = (),
) -> torch.Tensor:
    """根位置的 XY 投影是否仍在流水线通道内；完全忽略高度 Z。

    * ``x_range`` / ``y_range``：可用带面（略放宽于碰撞板）。
    * ``extra_rects``：与主矩形取**并集**的附加矩形（每项 ``(x0, x1, y0, y1)``）。
      入口弯道方案的 L 形带面 = 主线矩形 ∪ 拐角补块 ∪ X 支线条带；矩形个数是
      Python 常量，循环展开不引入张量同步。

    这张纯平面 mask 与 ``on_belt_mask`` 刻意分开：箱子只被抬高时已经不应继续受
    流水线驱动，但它的 XY 投影仍压在通道上，后续队列必须继续停住；只有根位置真正
    偏出通道后才放行下一格。
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
    return in_plane


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
    * XY 平面范围复用 ``in_conveyor_corridor_mask``，直线与 L 形入口共用一套边界。
    """

    in_plane = in_conveyor_corridor_mask(
        pos_local,
        x_range=x_range,
        y_range=y_range,
        extra_rects=extra_rects,
    )
    return (
        (pos_local[..., 2] >= belt_top_z - z_tolerance)
        & (pos_local[..., 2] <= belt_top_z + z_tolerance)
        & in_plane
    )


def transfer_complete_mask(
    on_belt: torch.Tensor,
    departure_completed: torch.Tensor,
) -> torch.Tensor:
    """返回每个 env 的工位箱是否已偏出流水线、可以让队列前进一步。

    每个箱子必须满足二选一：仍在带面高度窗口内，或根位置的 XY 投影已经偏出过流水线
    通道。于是队首只被竖直抬高时 ``on_belt=False`` 但仍在通道内，整带继续停住；
    横向搬出通道后立即放行，不再依赖蓝箱位置、速度阈值或驻留时间。完成位按箱锁存，
    避免抓取轨迹回摆或在边界附近抖动时把已经启动的流水线再次按停。
    """

    if on_belt.shape != departure_completed.shape:
        raise ValueError(
            "on_belt 与 departure_completed 形状必须一致："
            f"{tuple(on_belt.shape)} vs {tuple(departure_completed.shape)}"
        )
    return (on_belt | departure_completed).all(dim=0)


def update_departure_completion_latch(
    completed: torch.Tensor,
    in_conveyor_corridor: torch.Tensor,
) -> torch.Tensor:
    """箱根 XY 首次偏出流水线通道后锁存完成位。

    输入均为 ``(N, E)``。正好压在边界上仍属于通道内；只有跨出边界才置位。锁存项
    保持为真，直到事件 term 随 F12/DDS/env reset 清零。
    """

    if completed.shape != in_conveyor_corridor.shape:
        raise ValueError(
            "completed 与 in_conveyor_corridor 形状必须一致："
            f"{tuple(completed.shape)} vs {tuple(in_conveyor_corridor.shape)}"
        )
    return completed | ~in_conveyor_corridor


def update_arrival_latch(
    arrived: torch.Tensor,
    ys: torch.Tensor,
    *,
    y_stop: float,
) -> torch.Tensor:
    """箱子首次到达工位停止线（``y <= y_stop``）后锁存到位。

    ``lead_y > y_stop`` 的裸比较没有迟滞：机器人抱取时把工位箱往上游推 1-2 cm，
    队首 y 就重新越回停止线，整带被误判"队首未到位"而瞬间启动，还会把手里的
    箱子按输送速度往回拖（离线重放实测一次抱取内启停翻转 3 次）。锁存后箱子
    无论被推到哪，"工位被占用"这件事都保持为真，直到平面偏离完成位
    （``update_departure_completion_latch``）放行；两个锁存随同一个事件 term 在
    F12/DDS/env reset 时一起清零。
    """

    if arrived.shape != ys.shape:
        raise ValueError(
            "arrived 与 ys 形状必须一致："
            f"{tuple(arrived.shape)} vs {tuple(ys.shape)}"
        )
    return arrived | (ys <= y_stop)


def update_arrival_latch_along_path(
    arrived: torch.Tensor,
    ss: torch.Tensor,
    *,
    s_stop: float,
) -> torch.Tensor:
    """沿路径距离版到位锁存；s 向下游递增，到位判据是 ``s >= s_stop``。

    与 ``queue_drive_mask_along_path`` 同一手法：喂入 ``-s`` 复用 y 版实现。
    """

    return update_arrival_latch(arrived, -ss, y_stop=-s_stop)


def belt_release_gate(
    arrived: torch.Tensor,
    completed: torch.Tensor,
) -> torch.Tensor:
    """整带放行门：存在"已到位但尚未偏出流水线"的箱子时禁止整带运行。

    输入均为 ``(N, E)``，输出 ``(E,)``。与 ``transfer_complete_mask`` 互补：
    那张 mask 拦的是"被抬离带面但 XY 未偏出"的悬空箱，这里拦的是"仍躺在工位
    （含被抓取推挤回上游）"的到位箱——两者一起构成"箱子被拿到流水线之外才
    重新开带"的完整口径。``.any(dim=0)`` 是张量 reduce，不产生 GPU→CPU 同步。
    """

    if arrived.shape != completed.shape:
        raise ValueError(
            "arrived 与 completed 形状必须一致："
            f"{tuple(arrived.shape)} vs {tuple(completed.shape)}"
        )
    return ~((arrived & ~completed).any(dim=0))


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
    completed: torch.Tensor | None = None,
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
        completed=completed,
    )


def queue_drive_mask(
    ys: torch.Tensor,
    on_belt: torch.Tensor,
    half_lengths: torch.Tensor,
    *,
    y_stop: float | None,
    queue_gap: float,
    transfer_complete: torch.Tensor | None = None,
    completed: torch.Tensor | None = None,
) -> torch.Tensor:
    """整带节拍启停：算出这一帧哪些箱子该继续被送走。

    ``ys`` / ``on_belt`` 形状同为 ``(N, E)``——N 个箱子 × E 个 env。
    ``half_lengths`` 是 ``(N, 1)``（或可广播到 ``(N, E)``）的每个箱子沿输送方向的
    半长。返回与 ``ys`` 同形的 bool。

    语义是**整条带一起启停**，而不是逐个积放：

        belt_running = ((最下游那个仍在带面的箱子的 y) > y_stop) & transfer_complete
        drive[i] = on_belt[i] & belt_running & 没顶到前车

    也就是说队首一到工位，整条带立刻停下，后面的箱子**原地保持当前间距**，不会继续
    往前挤到贴紧前车。队首只被抬高、XY 仍在线上方时 ``transfer_complete=False``，
    整带继续停止；根位置横向偏出流水线通道后门控恢复为真，剩余队列一起前进到新
    队首压住工位。

    第三个因子是**防撞保底**，正常情况下不会触发：整带同起同停时相对间距恒定，
    而出生间距（默认 0.6）远大于任何一对箱子的最小净距。它只在两箱因摩擦/质量差异
    缓慢漂移到快要追尾时才介入，约束是

        ys[i] > (前车 y + 前车半长) + 自己半长 + queue_gap

    即**前车尾部 + 自己半长 + 净间隙**，而不是一个统一的中心距——流水线上两种箱型
    尺寸不同（0.38 与 0.50），统一中心距要么让大箱互相穿模、要么在小箱之间留出突兀
    的空档；按各自半长算，无论怎么交错，兜底的净空隙都恒为 ``queue_gap``。

    ``transfer_complete`` 是可选的 ``(E,)`` bool 张量；不传时保持通用队列函数的
    历史行为。停止式 legacy 事件用"仍在带上或已偏出通道"且"工位无到位未放行
    箱"（``belt_release_gate``）生成它；镜像端根本不跑驱动事件，箱子位姿仍全部
    来自权威端的 scene_state 帧。

    ``completed`` 是可选的 ``(N, E)`` 平面偏离锁存（同 ``transfer_complete`` 的
    per-box 来源）。已锁存的箱子在机器人手里，抓取轨迹回摆让它短暂回到带面
    高度窗/通道内时，绝不能再被当成"在带上的队首"——否则它会把 ``lead_y`` 拉回
    停止线以内按停整带，甚至自己重新满足 drive 被按输送速度拖走（跟机器人
    拔河）。传入后该箱从队首判定与驱动输出中彻底剔除；防撞保底仍用物理
    ``on_belt``（回摆的箱体真实挡在带上时后车照样刹住）。

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

    # 已放行的箱子不再属于队列：不参与队首判定，也不再被写输送速度。
    if completed is not None:
        if completed.shape != on_belt.shape:
            raise ValueError(
                "completed 形状必须与 on_belt 一致："
                f"{tuple(completed.shape)} vs {tuple(on_belt.shape)}"
            )
        active = on_belt & ~completed
    else:
        active = on_belt

    if y_stop is None:
        return on_belt & clear_of_leader

    # —— 整带节拍 ——
    # 只看仍在队列里的箱子来确定新队首；被抓走的箱子是否已经横向偏出流水线，则由
    # 下方 transfer_complete 统一门控。全部离开带面时 lead_y 取到哨兵大值，但
    # active 全假，drive 仍是全假，无需额外分支。
    far_away = ys.new_tensor(1.0e9)
    lead_y = torch.where(active, ys, far_away).amin(dim=0)  # (E,)
    belt_running = lead_y > y_stop  # (E,)
    if transfer_complete is not None:
        if transfer_complete.shape != belt_running.shape:
            raise ValueError(
                "transfer_complete 形状必须为 (E,)："
                f"{tuple(transfer_complete.shape)} vs {tuple(belt_running.shape)}"
            )
        belt_running = belt_running & transfer_complete
    belt_running = belt_running.unsqueeze(0)  # (1, E) → 广播到 (N, E)
    return active & belt_running & clear_of_leader
