#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025 Unitree Robotics
# SPDX-License-Identifier: Apache-2.0
"""SteamVR overlay 探针：验证"存在 overlay 能否逼 compositor 转入合成模式"。

## 为什么有这个工具

AR 卡顿的根因是 SteamVR + NOLO 这条链全程没有重投影（见 ``doc/xr_ar_judder_zh.md``）。
但 2026-07-30 实测确认了一个反例：**打开 SteamVR dashboard 时，连 Isaac 的画面也不抖**
（不只是菜单不抖，且此时机器人照常走动，说明应用仍在正常提交新帧）。

即：PC 侧的"按最新头姿逐帧重新变换应用画面"能力本来就存在，只是正常模式下不启用。
证据是 ``vrcompositor.txt`` 里的 ``Compositor Time........CPU: 0.209ms / GPU: 0.007ms``
——6 次会话一致，GPU 侧几乎为 0，compositor 在**直通**，连畸变都交给 direct-mode 驱动。

**待验证的假设**：直通是一个优化，其前提是只有单一 layer。只要存在任何一个可见 overlay，
compositor 就必须自己合成，于是顺带每帧按最新头姿重新变换 —— 若成立，代价极小
（不牺牲立体、不改 Isaac 一行代码，只需常驻一个几乎不可见的 overlay）。

## 客观判据（不需要戴头显）

本工具每秒打印 ``IVRCompositor::GetFrameTiming`` 的 ``m_flCompositorRenderGpuMs``。

- 基线（``--mode none``）应当接近 0（对应日志里的 0.007ms）
- 若 ``--mode tiny`` 下该值**显著上升**，就客观证明 compositor 转入了合成模式

主观判据仍需戴头显：转头看 Isaac 画面还抖不抖。

## 用法

    conda activate env_isaaclab
    # 需要 SteamVR 已在跑（NOLO Link 或 ALVR 拉起），且 Isaac 已作为 scene app 接入
    python tools/xr_overlay_probe.py --mode none      # 先拿基线
    python tools/xr_overlay_probe.py --mode tiny      # 假设的零代价解
    python tools/xr_overlay_probe.py --mode panel     # 测 overlay 自身是否稳
    python tools/xr_overlay_probe.py --mode blink     # 自动交替，做 A/B

依赖 ``pip install openvr``（纯 ctypes 绑定，无需编译）。本工具以
``VRApplication_Overlay`` 身份接入，**不会**抢占 Isaac 的 scene app 身份。
"""

from __future__ import annotations

import argparse
import ctypes
import math
import signal
import sys
import time
from typing import Any

# 与 vrcompositor.txt 里 "Compositor Time ... GPU: 0.007ms" 对应的判定阈值。
# 直通模式下该值 ~0.01ms；一旦 compositor 真的开始渲染，量级会完全不同。
COMPOSITE_GPU_MS_THRESHOLD = 0.10

# IVRCompositor 的重投影标志位（openvr.h）
REPROJECTION_FLAGS = (
    (0x01, "Cpu"),
    (0x02, "Gpu"),
    (0x04, "Async"),
    (0x08, "Motion"),
)


def _decode_reprojection(flags: int) -> str:
    """把 m_nReprojectionFlags 解成可读串。

    高 4 位是 "prediction frames" 计数（openvr.h: ReprojectionMotion_Prediction* 等），
    这里只解低位的原因标志，其余原样带出，避免误读。
    """
    if not flags:
        return "none"
    names = [name for bit, name in REPROJECTION_FLAGS if flags & bit]
    extra = flags & ~0x0F
    if extra:
        names.append(f"hi=0x{extra:x}")
    return "+".join(names) if names else f"0x{flags:x}"


def _make_pattern(size: int, kind: str) -> bytes:
    """生成 RGBA 测试图案。

    用高对比的硬边缘（网格 + 中心十字 + 外框）——判断"粘一下跳一下"靠的是边缘，
    渐变图案会让主观判读变难。不引入 numpy 依赖，纯 bytearray 够快（一次性生成）。
    """
    buf = bytearray(size * size * 4)
    step = max(4, size // 8)
    mid = size // 2
    edge = max(1, size // 32)
    bar = max(1, size // 24)
    for y in range(size):
        for x in range(size):
            i = (y * size + x) * 4
            on_border = x < edge or y < edge or x >= size - edge or y >= size - edge
            on_cross = abs(x - mid) < bar or abs(y - mid) < bar
            on_grid = (x % step) < 1 or (y % step) < 1
            if kind == "solid":
                r, g, b = 255, 0, 255
            elif on_border:
                r, g, b = 255, 255, 0
            elif on_cross:
                r, g, b = 255, 0, 255
            elif on_grid:
                r, g, b = 255, 255, 255
            else:
                # 深色底而非透明：透明区域可能被 compositor 优化掉，
                # 那会把"存在 overlay"这个前提本身给测糊。
                r, g, b = 12, 12, 24
            buf[i] = r
            buf[i + 1] = g
            buf[i + 2] = b
            buf[i + 3] = 255
    return bytes(buf)


def _identity_pose(openvr, x: float, y: float, z: float, pitch_deg: float = 0.0):
    """世界锚定的 overlay 位姿（绕 X 轴 pitch，便于把小 overlay 压到视野下缘）。

    必须是世界锚定：跟随头部的 overlay 相对头永远静止，测不出抖动。
    """
    m = openvr.HmdMatrix34_t()
    c = math.cos(math.radians(pitch_deg))
    s = math.sin(math.radians(pitch_deg))
    rows = (
        (1.0, 0.0, 0.0, x),
        (0.0, c, -s, y),
        (0.0, s, c, z),
    )
    for r in range(3):
        for col in range(4):
            m[r][col] = rows[r][col]
    return m


class Probe:
    def __init__(self, args):
        self.args = args
        # 惰性 import 的模块句柄；标 Any 是因为 pyopenvr 未装时这里就是 None,
        # 而 connect() 之后所有访问都在有值的路径上。
        self.openvr: Any = None
        self.overlay_handle = None
        self.visible = False
        self._stop = False

    # ---------- 生命周期 ----------

    def connect(self):
        try:
            import openvr
        except ImportError:
            sys.exit("缺少 pyopenvr。装它：pip install openvr")
        self.openvr = openvr
        try:
            # VRApplication_Overlay：以 overlay 应用身份接入，不抢 scene app。
            openvr.init(openvr.VRApplication_Overlay)
        except Exception as e:  # openvr 抛的是 OpenVRError 家族，统一兜住
            sys.exit(
                f"连接 SteamVR 失败：{e}\n"
                "先把 SteamVR 拉起来（NOLO Link 或 ALVR），确认 vrserver/vrcompositor 在跑。"
            )
        self._report_scene_app()

    def _report_scene_app(self):
        openvr = self.openvr
        try:
            apps = openvr.VRApplications()
            pid = apps.getCurrentSceneProcessId()
        except Exception:
            print("[probe] 无法查询 scene app（不影响测量）")
            return
        if not pid:
            print("[probe] ⚠️ 当前没有 scene app 接入——请先启动 Isaac（--xr），否则测的不是真实工况")
            return
        print(f"[probe] scene app pid = {pid}")

    def create_overlay(self):
        openvr = self.openvr
        args = self.args
        ov = openvr.VROverlay()
        # createOverlay 而非 createDashboardOverlay：我们要的是 in-game overlay,
        # dashboard overlay 只在菜单打开时可见，那就变成复现已知现象而不是测新假设。
        self.overlay_handle = ov.createOverlay("unitree.judder.probe", "Judder Probe")

        size = 64 if args.mode == "tiny" else 512
        kind = "solid" if args.mode == "tiny" else "grid"
        pattern = _make_pattern(size, kind)
        cbuf = (ctypes.c_char * len(pattern)).from_buffer_copy(pattern)
        ov.setOverlayRaw(self.overlay_handle, cbuf, size, size, 4)

        ov.setOverlayWidthInMeters(self.overlay_handle, args.width)
        ov.setOverlayAlpha(self.overlay_handle, args.alpha)
        pose = _identity_pose(openvr, args.x, args.y, args.z, args.pitch)
        ov.setOverlayTransformAbsolute(
            self.overlay_handle, openvr.TrackingUniverseStanding, pose
        )
        self.show()
        print(
            f"[probe] overlay 已创建 mode={args.mode} 宽={args.width}m alpha={args.alpha} "
            f"位置=({args.x}, {args.y}, {args.z}) pitch={args.pitch}°"
        )

    def show(self):
        if self.overlay_handle is None:
            return
        self.openvr.VROverlay().showOverlay(self.overlay_handle)
        self.visible = True

    def hide(self):
        if self.overlay_handle is None:
            return
        self.openvr.VROverlay().hideOverlay(self.overlay_handle)
        self.visible = False

    def shutdown(self):
        if self.openvr is None:
            return
        try:
            if self.overlay_handle is not None:
                self.openvr.VROverlay().destroyOverlay(self.overlay_handle)
        finally:
            self.openvr.shutdown()

    # ---------- 遥测 ----------

    def _frame_timing(self):
        try:
            t = self.openvr.VRCompositor().getFrameTiming()
        except Exception:
            return None
        # 字段名按 openvr.h 的 C 名字；用 getattr 兜住绑定版本差异，
        # 免得换个 pyopenvr 版本就 AttributeError 崩掉整次实验。
        return {
            "comp_gpu": getattr(t, "m_flCompositorRenderGpuMs", float("nan")),
            "comp_cpu": getattr(t, "m_flCompositorRenderCpuMs", float("nan")),
            "app_gpu": getattr(t, "m_flPreSubmitGpuMs", float("nan")),
            "presents": getattr(t, "m_nNumFramePresents", -1),
            "mispresent": getattr(t, "m_nNumMisPresented", -1),
            "dropped": getattr(t, "m_nNumDroppedFrames", -1),
            "reproj": getattr(t, "m_nReprojectionFlags", 0),
            "interval": getattr(t, "m_flClientFrameIntervalMs", float("nan")),
        }

    def run(self):
        args = self.args
        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)

        print(
            "[probe] 判据：直通模式下 comp_gpu ≈ 0.01ms；若它显著上升"
            f"（阈值 {COMPOSITE_GPU_MS_THRESHOLD}ms），说明 compositor 转入了合成模式。"
        )
        print(
            "[probe] "
            + " | ".join(
                ["t", "overlay", "comp_gpu", "comp_cpu", "app_gpu", "interval", "presents", "reproj"]
            )
        )

        t0 = time.monotonic()
        last_blink = t0
        composited_seen = False
        samples: list[float] = []

        while not self._stop:
            now = time.monotonic()
            if args.mode == "blink" and now - last_blink >= args.blink_period:
                self.hide() if self.visible else self.show()
                last_blink = now
                print(f"[probe] --- overlay -> {'ON' if self.visible else 'OFF'} ---")

            tm = self._frame_timing()
            if tm is None:
                print("[probe] getFrameTiming 不可用（compositor 未就绪？）")
            else:
                if tm["comp_gpu"] == tm["comp_gpu"]:  # 非 NaN
                    samples.append(tm["comp_gpu"])
                    if tm["comp_gpu"] > COMPOSITE_GPU_MS_THRESHOLD:
                        composited_seen = True
                print(
                    f"[probe] {now - t0:6.1f}s | {'ON ' if self.visible else 'OFF'} | "
                    f"{tm['comp_gpu']:8.3f} | {tm['comp_cpu']:8.3f} | {tm['app_gpu']:7.3f} | "
                    f"{tm['interval']:8.2f} | {tm['presents']:8d} | {_decode_reprojection(tm['reproj'])}"
                )
            time.sleep(args.interval)

        if samples:
            avg = sum(samples) / len(samples)
            peak = max(samples)
            print(
                f"\n[probe] comp_gpu 均值 {avg:.4f}ms / 峰值 {peak:.4f}ms（{len(samples)} 个样本）"
            )
            if composited_seen:
                print(
                    "[probe] ✅ compositor 出现了明显的 GPU 渲染开销 —— 它没有在直通，"
                    "假设成立的可能性大。接着戴头显做主观判读。"
                )
            else:
                print(
                    "[probe] ❌ comp_gpu 全程贴近 0 —— compositor 仍在直通，"
                    "仅存在 overlay 不足以让它转入合成模式。"
                )

    def _on_sigint(self, *_):
        self._stop = True


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="SteamVR overlay 探针：测'存在 overlay 能否逼 compositor 合成'",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--mode",
        choices=("none", "tiny", "panel", "blink"),
        default="tiny",
        help=(
            "none: 只做遥测不建 overlay（拿基线）; "
            "tiny: 极小 overlay，测假设的零代价解; "
            "panel: 大面板，测 overlay 自身稳不稳; "
            "blink: 定时交替显示/隐藏做 A/B"
        ),
    )
    p.add_argument("--width", type=float, default=None, help="overlay 宽度（米）；默认按 mode 取")
    p.add_argument("--alpha", type=float, default=None, help="不透明度 0-1；默认按 mode 取")
    p.add_argument("--x", type=float, default=0.0, help="世界系 X（米）")
    p.add_argument("--y", type=float, default=None, help="世界系 Y/高度（米）；默认按 mode 取")
    p.add_argument("--z", type=float, default=-1.8, help="世界系 Z（米，负值在前方）")
    p.add_argument("--pitch", type=float, default=0.0, help="绕 X 轴俯仰（度）")
    p.add_argument("--interval", type=float, default=1.0, help="遥测打印间隔（秒）")
    p.add_argument("--blink-period", type=float, default=8.0, help="blink 模式切换周期（秒）")
    return p


def apply_mode_defaults(args):
    """按 mode 补默认几何：tiny 要"几乎不挡视线"，panel 要"看得清边缘"。

    显式传入的值一律优先——实验里经常要手动挪位置试遮挡程度。
    """
    if args.mode == "tiny":
        args.width = 0.03 if args.width is None else args.width
        args.alpha = 0.85 if args.alpha is None else args.alpha
        args.y = 0.6 if args.y is None else args.y
    else:
        args.width = 1.2 if args.width is None else args.width
        args.alpha = 1.0 if args.alpha is None else args.alpha
        args.y = 1.5 if args.y is None else args.y
    return args


def main(argv=None):
    args = apply_mode_defaults(build_parser().parse_args(argv))

    probe = Probe(args)
    probe.connect()
    try:
        if args.mode != "none":
            probe.create_overlay()
        else:
            print("[probe] mode=none：只做遥测，不创建 overlay（这是基线）")
        probe.run()
    finally:
        probe.shutdown()
        print("[probe] 已退出")


if __name__ == "__main__":
    main()
