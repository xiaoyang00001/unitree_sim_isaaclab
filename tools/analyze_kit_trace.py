#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""统计 kit CPU profiler 的 trace(.gz),按 zone 汇总耗时。

用途:把 `[Performance]` 里 R 那一格拆开——R 同时装着"我们的渲染 CPU"与
"xrWaitFrame 帧闸门阻塞",Python 侧计时分不开这两者。trace 里两者是不同的
zone,能直接读出各占几毫秒。

用法:
    python tools/analyze_kit_trace.py kit_trace_20260730_120000.gz
    python tools/analyze_kit_trace.py trace.gz --grep 'anchor|xr|wait' --top 40
    python tools/analyze_kit_trace.py a.gz b.gz          # 两次抓取逐 zone 对比

判读要点:
- **self 时间**(自身耗时,扣掉子 zone)才是"这段代码花的时间";total 含子调用,
  嵌套 zone 会重复计入,别拿 total 排名下结论。
- 关注每帧调用次数 calls/frame:每帧一次的 zone 才在帧预算里,偶发的不是。
- xrWaitFrame 类 zone 的时间是**阻塞等待**,不是 CPU 消耗;它变大不一定是坏事
  (可能只是我们提交得早),要结合主循环 Hz 一起看。
"""

from __future__ import annotations

import argparse
import gzip
import json
import re
import sys
from collections import defaultdict
from pathlib import Path


def load_events(path: Path) -> list[dict]:
    """读 Chrome Trace 格式(.gz 或裸 json)。容忍尾部截断的 JSON 数组。"""
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", errors="replace") as f:
        raw = f.read()
    raw = raw.strip()
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        # 抓取被中断时数组可能没闭合,补上再试
        cut = raw.rstrip().rstrip(",")
        for tail in ("]", "}]", '"}]'):
            try:
                data = json.loads(cut + tail)
                break
            except json.JSONDecodeError:
                continue
        else:
            raise
    if isinstance(data, dict):
        data = data.get("traceEvents", [])
    return [e for e in data if isinstance(e, dict)]


def summarize(events: list[dict]) -> tuple[dict, float, int]:
    """返回 (zone 统计, trace 时长 ms, 估计帧数)。

    同时支持 'X'(complete,带 dur) 与 'B'/'E'(begin/end 配对)两种编码。
    self 时间 = total - 直接子 zone 的 total,按 (pid,tid) 维护调用栈。
    """
    stats: dict[str, dict] = defaultdict(
        lambda: {"total": 0.0, "self": 0.0, "calls": 0, "max": 0.0}
    )
    stacks: dict[tuple, list] = defaultdict(list)
    t_min, t_max = float("inf"), float("-inf")

    def close(name: str, dur: float, key: tuple):
        s = stats[name]
        s["total"] += dur
        s["calls"] += 1
        s["max"] = max(s["max"], dur)
        s["self"] += dur
        # 从父 zone 的 self 里扣掉本 zone 的 total
        stack = stacks[key]
        if stack:
            stats[stack[-1][0]]["self"] -= dur

    for e in events:
        ph = e.get("ph")
        ts = e.get("ts")
        if ts is None:
            continue
        key = (e.get("pid"), e.get("tid"))
        t_min = min(t_min, ts)
        if ph == "X":
            dur = float(e.get("dur") or 0.0)
            t_max = max(t_max, ts + dur)
            # complete 事件本身不入栈,但要归到当前父 zone 名下
            close(str(e.get("name", "?")), dur, key)
        elif ph == "B":
            stacks[key].append((str(e.get("name", "?")), float(ts)))
        elif ph == "E":
            stack = stacks[key]
            if not stack:
                continue
            name, t0 = stack.pop()
            dur = float(ts) - t0
            t_max = max(t_max, ts)
            close(name, dur, key)
    span_ms = (t_max - t_min) / 1000.0 if t_max > t_min else 0.0

    # 帧数:优先用公认的每帧一次的 zone;找不到就用最高频的 zone 兜底
    frames = 0
    for cand in ("carb::framework::update", "Frame", "app update", "Kit update"):
        for name, s in stats.items():
            if name.lower() == cand.lower():
                frames = s["calls"]
                break
        if frames:
            break
    return stats, span_ms, frames


def report(path: Path, pattern: str | None, top: int) -> dict:
    events = load_events(path)
    stats, span_ms, frames = summarize(events)
    print(f"\n===== {path.name} =====")
    print(f"事件 {len(events)} 条, 跨度 {span_ms:.0f} ms, 检出帧数 {frames or 'n/a'}")
    rx = re.compile(pattern, re.I) if pattern else None
    rows = [(n, s) for n, s in stats.items() if not rx or rx.search(n)]
    rows.sort(key=lambda kv: kv[1]["self"], reverse=True)
    hdr = f"{'zone':<58} {'self ms':>9} {'self%':>6} {'total ms':>9} {'calls':>7} {'ms/call':>8}"
    print(hdr)
    print("-" * len(hdr))
    for name, s in rows[:top]:
        share = 100.0 * s["self"] / 1000.0 / span_ms if span_ms else 0.0
        per = s["self"] / s["calls"] / 1000.0 if s["calls"] else 0.0
        print(
            f"{name[:58]:<58} {s['self']/1000.0:>9.2f} {share:>5.1f}% "
            f"{s['total']/1000.0:>9.2f} {s['calls']:>7d} {per:>8.3f}"
        )
    if frames:
        print(f"\n(每帧口径:self ms / {frames} 帧 = 每帧毫秒)")
    return {n: s for n, s in rows}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("traces", nargs="+", type=Path)
    ap.add_argument("--grep", default=None, help="只看名字匹配该正则的 zone")
    ap.add_argument("--top", type=int, default=30)
    args = ap.parse_args()

    reports = []
    for p in args.traces:
        if not p.exists():
            print(f"✗ 不存在: {p}", file=sys.stderr)
            return 1
        reports.append((p, report(p, args.grep, args.top)))

    if len(reports) == 2:
        (pa, a), (pb, b) = reports
        print(f"\n===== 对比 {pa.name} → {pb.name} (按 self 差值排序) =====")
        names = set(a) | set(b)
        diff = []
        for n in names:
            sa = a.get(n, {}).get("self", 0.0) / 1000.0
            sb = b.get(n, {}).get("self", 0.0) / 1000.0
            diff.append((sb - sa, n, sa, sb))
        diff.sort(key=lambda t: abs(t[0]), reverse=True)
        print(f"{'zone':<58} {'A ms':>9} {'B ms':>9} {'Δ ms':>9}")
        for d, n, sa, sb in diff[: args.top]:
            print(f"{n[:58]:<58} {sa:>9.2f} {sb:>9.2f} {d:>+9.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
