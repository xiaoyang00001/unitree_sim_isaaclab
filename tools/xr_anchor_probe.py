# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""XR anchor 同步归因探针(诊断工具,默认关闭)。

背景:--teleop_device motion_controllers 实例化 OpenXRDevice 后主循环
50Hz → 27Hz。嫌疑有二:
  1. XrAnchorSynchronizer.sync_headset_to_anchor 挂在 XRCoreEventType.
     pre_sync_update 上,每个 XR 帧做两次 Fabric 矩阵读取 + 一次
     XRCore.set_world_transform_matrix USD layer 写入(纯 CPU 成本);
  2. xrWaitFrame / XR 锁步闸门的阻塞空转(不是 CPU 成本,优化 anchor
     写入救不回来)。
本探针把这 ~20ms/圈 的差异拆开归因,不修改 IsaacLab fork 源码——全部
通过替换 synchronizer 实例属性注入。

用法:XR_ANCHOR_PROBE=<mode> python sim_main.py ... --teleop_device motion_controllers
  off     (默认)不安装,零影响
  timing  行为不变,统计回调频率与耗时,并单独拆出
          set_world_transform_matrix 写入的耗时占比
  nowrite 保留全部矩阵读取与四元数计算,跳过最终写入(仍计时)
          → 归因"写入 + 其下游"的成本
  fabric  写入改走 usdrt/Fabric(layer_identifier=None),其余不变
          → **纯诊断档,不是修法**:用来量"USD 写 vs Fabric 写"的成本差。
            ⚠️ 已证不可行(2026-07-30,见 doc/xr_ar_judder_zh.md §2.1):kit 每帧
            用 pxr::UsdGeomImageable::ComputeLocalToWorldTransform 读 anchor,
            那条路径里没有任何 fabric 分支或回退,只写 Fabric 消费者看不到,
            视角会冻结在 XRAnchor 的 USD 初值上、不再跟随机器人。
            本档跑起来时 anchor 是冻的,主观视角行为无意义。
  noop    整个 sync_headset_to_anchor 直接返回,只计数
          → 归因"回调全部工作"的成本;anchor 静止,视角会漂,仅诊断用
  noanchor 在 noop 之上,把 kit 的 anchorMode 拨回 "scene origin"
          → 只有 noop 仍慢时才需要这一档:它把 kit C++ 侧的
            custom-anchor(锚到移动 prim)通路也一并关掉,用于区分
            "我们的 Python 回调" / "kit 的自定义锚通路" / "XR 会话本身"

统计每 ~5s 打一行 [xr_probe],回调次数/秒 ≈ pre_sync_update 派发频率
≈ XR 帧循环频率,可与 stats 窗口的 GUI render fps 对照。

注意:noop/nowrite 模式下 B 键 recenter 与 reset 的 anchor 写入同样被
跳过/保留计时,视角行为不代表正常模式,不要在这两个模式下做主观评测。
"""

from __future__ import annotations

from time import perf_counter

VALID_MODES = ("timing", "nowrite", "fabric", "noop", "noanchor")


def _fmt_ms(seconds: float) -> str:
    return f"{seconds * 1000.0:.3f}"


class _ProbeStats:
    """窗口化统计:回调总耗时,及其中"Fabric 读"与"USD 写"两段子耗时。

    三段之差即纯 Python 四元数计算,不必单独计时。
    """

    def __init__(self, mode: str, print_interval_s: float = 5.0):
        self.mode = mode
        self.print_interval_s = print_interval_s
        self._window_start = perf_counter()
        self._call_samples: list[float] = []
        self._write_samples: list[float] = []
        self._read_samples: list[float] = []

    def add_write(self, dt: float) -> None:
        self._write_samples.append(dt)

    def add_read(self, dt: float) -> None:
        self._read_samples.append(dt)

    def add_call(self, dt: float) -> None:
        self._call_samples.append(dt)
        now = perf_counter()
        window = now - self._window_start
        if window < self.print_interval_s:
            return
        self._print(window)
        self._window_start = now
        self._call_samples.clear()
        self._write_samples.clear()
        self._read_samples.clear()

    @staticmethod
    def _summary(samples: list[float]) -> str:
        if not samples:
            return "n/a"
        ordered = sorted(samples)
        mean = sum(ordered) / len(ordered)
        p95 = ordered[int(0.95 * (len(ordered) - 1))]
        return f"{_fmt_ms(mean)}/{_fmt_ms(p95)}/{_fmt_ms(ordered[-1])} ms"

    def _print(self, window: float) -> None:
        n = len(self._call_samples)
        total = sum(self._call_samples)
        line = (
            f"[xr_probe:{self.mode}] {n} calls in {window:.1f}s "
            f"({n / window:.1f}/s), sync mean/p95/max {self._summary(self._call_samples)}"
        )
        for name, samples in (("read", self._read_samples), ("write", self._write_samples)):
            if not samples:
                continue
            share = sum(samples) / total if total > 0.0 else 0.0
            # read 每次 sync 调 2 次(位置锚 + 旋转锚),故 share 是两次之和的占比
            line += (
                f", {name} mean/p95/max {self._summary(samples)}"
                f" (share {share:.0%})"
            )
        print(line, flush=True)


class _XrCoreWriteProxy:
    """包一层 XRCore 单例:拦截 set_world_transform_matrix,其余原样转发。

    timing: 计时后原样写(USD layer);
    nowrite: 只计时不写;
    fabric: 丢弃 layer_identifier,走 XRCore 默认的 usdrt/Fabric 写路径。
    """

    def __init__(self, inner, mode: str, stats: _ProbeStats):
        self._inner = inner
        self._mode = mode
        self._stats = stats

    def set_world_transform_matrix(self, prim_path, matrix, layer_identifier=None):
        t0 = perf_counter()
        if self._mode == "nowrite":
            pass
        elif self._mode == "fabric":
            self._inner.set_world_transform_matrix(prim_path, matrix)
        else:
            self._inner.set_world_transform_matrix(prim_path, matrix, layer_identifier)
        self._stats.add_write(perf_counter() - t0)

    def __getattr__(self, name):
        return getattr(self._inner, name)


def install_xr_anchor_probe(teleop_interface, mode: str) -> bool:
    """在已创建的 OpenXRDevice 上安装探针。返回是否安装成功。"""
    mode = (mode or "").strip().lower()
    if mode in ("", "off"):
        return False
    if mode not in VALID_MODES:
        raise ValueError(
            f"XR_ANCHOR_PROBE={mode!r} invalid; expected one of {VALID_MODES} or 'off'"
        )

    sync = getattr(teleop_interface, "_anchor_sync", None)
    if sync is None:
        print("[xr_probe] device has no anchor synchronizer; probe not installed")
        return False

    stats = _ProbeStats(mode)

    if mode in ("timing", "nowrite", "fabric"):
        sync._xr_core = _XrCoreWriteProxy(sync._xr_core, mode, stats)
        # 读取侧:每次 sync 调 2 次(位置锚 head_link + 旋转锚 pelvis),每次都
        # 重新 usdrt.Usd.Stage.Attach + GetFabricHierarchyWorldMatrixAttr().Get()
        original_read = sync._get_prim_world_matrix

        def timed_read(prim_path):
            t0 = perf_counter()
            try:
                return original_read(prim_path)
            finally:
                stats.add_read(perf_counter() - t0)

        sync._get_prim_world_matrix = timed_read

    if mode == "noanchor":
        # OpenXRDevice.__init__ 已经写过 anchorMode=custom anchor 与
        # customAnchor=<XRAnchor 路径>,这里拨回 scene origin。kit 的
        # XRViewportController 监听这两个键,变更会触发 _reset_anchor,
        # 把 stage anchor 从"锚在移动 prim 上"改回场景原点。
        try:
            import carb

            settings = carb.settings.get_settings()
            settings.set_string("/persistent/xr/profile/ar/anchorMode", "scene origin")
            settings.set_string("/xrstage/profile/ar/customAnchor", "")
            print(
                "[xr_probe] anchorMode → 'scene origin'(kit 自定义锚通路已关);"
                "⚠️ 视角不再跟随机器人,仅用于成本归因"
            )
        except Exception as e:
            print(f"[xr_probe] failed to revert anchorMode: {e}")

    original_sync = sync.sync_headset_to_anchor
    if mode in ("noop", "noanchor"):
        def wrapped_sync():
            stats.add_call(0.0)
    else:
        def wrapped_sync():
            t0 = perf_counter()
            original_sync()
            stats.add_call(perf_counter() - t0)

    # 订阅回调是 lambda _: self._anchor_sync.sync_headset_to_anchor(),按属性
    # 动态查找,替换实例属性即可生效,无需触碰订阅本身。
    sync.sync_headset_to_anchor = wrapped_sync

    print(
        f"[xr_probe] installed mode={mode} "
        "(timing=原行为+计时, nowrite=跳过写入, fabric=写 usdrt, noop=整体跳过)",
        flush=True,
    )
    return True
