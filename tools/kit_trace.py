# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""kit CPU profiler 抓取(诊断用,默认关闭)。

为什么需要它:`[Performance]` 的 R 一格里同时装着"我们的渲染 CPU"和
"`xrWaitFrame` 帧闸门阻塞",两者混在一起时无法判断某个优化到底有没有用
(实测踩到:dashboard 一开 R 就 +5-6ms,而 CPU 工作量毫无变化)。Python 侧
的计时只能测到我们自己的代码,测不到 kit C++ 每帧的 USD 工作,也测不到
WaitFrame 到底阻塞了几毫秒——只能当残差推断。

kit 自带的 carb CPU profiler 能给出**逐 zone 的耗时**,包括
`runUpdateAnchorSpaceAfterInputStage` / `updateAnchorPrim`(kit 每帧读写
anchor 的 USD 工作)以及 XR 帧闸门相关 zone,这才是把差值拆开的正解。

产物是 Chrome Trace 格式的 .gz,可直接拖进 chrome://tracing 或用
tools/analyze_kit_trace.py 统计。

⚠️ 抓取本身有开销(每个 zone 多一次记账),所以 trace 里的绝对帧率会略低于
不抓取时。它的用途是**相对归因**(时间花在哪),不是拿绝对值当帧率结论。
"""

from __future__ import annotations

import os
from typing import Any


class KitCpuTrace:
    """按需启停 kit 的 carb CPU profiler,把 trace 落到指定文件。"""

    _PLUGIN = "carb.profiler-cpu.plugin"

    def __init__(self, path: str):
        self.path = path
        self._profiler: Any = None
        self._settings: Any = None
        self._active = False

    def start(self) -> bool:
        try:
            import carb.profiler
            import carb.settings

            self._profiler = carb.profiler.acquire_profiler_interface(plugin_name=self._PLUGIN)
            self._settings = carb.settings.get_settings()

            # /app/profilerMask 是有符号读出的,默认全 1 位会读成 -1;
            # set_capture_mask 要无符号,故此处补回。
            mask = self._settings.get_as_int("/app/profilerMask")
            if mask is None or mask == 0:
                mask = 0x0FFFFFFFFFFFFFFFF
            elif mask < 0:
                mask += 0x010000000000000000
            self._profiler.set_capture_mask(mask)

            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            # 注意这两个键的前导斜杠与 kit 自带 profiler 窗口保持一致(filePath
            # 无前导斜杠、saveProfile 有),照抄免得写到别处去。
            self._settings.set_string(f"plugins/{self._PLUGIN}/filePath", self.path)
            self._settings.set_bool(f"/plugins/{self._PLUGIN}/saveProfile", True)
            self._active = True
            print(f"[kit_trace] capture started -> {self.path}", flush=True)
            return True
        except Exception as e:
            print(f"[kit_trace] failed to start capture: {e}", flush=True)
            self._active = False
            return False

    def stop(self) -> None:
        if not self._active:
            return
        try:
            # saveProfile 由 True→False 的这次跳变触发插件落盘
            self._settings.set_bool(f"/plugins/{self._PLUGIN}/saveProfile", False)
            self._profiler.set_capture_mask(0)
            size = os.path.getsize(self.path) if os.path.exists(self.path) else 0
            print(
                f"[kit_trace] capture stopped -> {self.path} ({size / 1024.0:.0f} KiB)"
                "; 分析: python tools/analyze_kit_trace.py <文件>",
                flush=True,
            )
        except Exception as e:
            print(f"[kit_trace] failed to stop capture: {e}", flush=True)
        finally:
            self._active = False

    @property
    def active(self) -> bool:
        return self._active


class TraceSchedule:
    """按主循环墙钟推进的"暖机 N 秒 → 抓 M 秒 → 落盘"状态机。

    放在主循环里每圈调一次 ``tick(now)``;返回 True 表示本次调用刚落盘。
    """

    def __init__(self, path: str, delay_s: float, duration_s: float):
        self._trace = KitCpuTrace(path)
        self._delay = float(delay_s)
        self._duration = float(duration_s)
        self._t0: float | None = None
        self._done = False

    def tick(self, now: float) -> bool:
        if self._done or self._duration <= 0.0:
            return False
        if self._t0 is None:
            self._t0 = now
            return False
        elapsed = now - self._t0
        if not self._trace.active:
            if elapsed >= self._delay:
                if not self._trace.start():
                    self._done = True  # 起不来就别每圈重试刷屏
            return False
        if elapsed >= self._delay + self._duration:
            self._trace.stop()
            self._done = True
            return True
        return False

    def abort(self) -> None:
        """进程退出时兜底落盘,别把已抓的数据丢掉。"""
        if self._trace.active:
            self._trace.stop()
            self._done = True
