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

## ❌ 第一个假设已被否决（2026-07-30）

原假设是"直通的前提是单一 layer，只要存在任何可见 overlay 就能逼 compositor 每帧重新变换"。
**实测否决**：in-game overlay（``--mode tiny/panel``）与 scene layer **一起晃**
（用户原话:"这个和 isaac 是分开渲染的，问题一样，都会晃动"），且 ``comp_gpu`` 与无 overlay
的基线毫无差别（0.006–0.008ms）。

错在哪：compositor 只在**应用提交新帧时**把 overlay + scene layer 合成一张，然后整张原样
重发给 direct-mode 驱动 —— 合成 ≠ 每帧重新渲染。

## ⭐ 转向：让 compositor 待在 dashboard 模式（``--mode dashboard``）

dashboard 的机制不同:它让 compositor 成为**每 vsync 的渲染源**，Isaac 画面被逐帧按最新头姿
重绘——所以用户实测 dashboard 开着时连 Isaac 画面都不晃。要复现的是这个，而不是"存在 overlay"。

``showDashboard(key)`` 可以指定显示哪个 dashboard overlay。指向一个极小的自有 overlay，
就有机会既拿到"不晃"、又不被 SteamVR 主菜单挡住视野。**这是目前唯一被实测证明不晃、
且留在 SteamVR + NOLO 链上的方向。** 已知代价与判读要点见 ``doc/xr_ar_judder_zh.md`` §4.2。

## 客观判据（不需要戴头显）

本工具每秒打印 ``IVRCompositor::GetFrameTiming`` 的 ``m_flCompositorRenderGpuMs``。

- 基线（``--mode none``）应当接近 0（对应日志里的 0.007ms）
- 若 ``--mode tiny`` 下该值**显著上升**，就客观证明 compositor 转入了合成模式

主观判据仍需戴头显：转头看 Isaac 画面还抖不抖。

## 用法：必须两个终端，Isaac 先起

本工具**只提供一个 overlay，不提供场景画面**。单独跑它，头显里只会看到 SteamVR 的空环境
加上探针的图案——看不到 Isaac，而且此时 compositor 正在渲染 SteamVR 自己的环境，
``comp_gpu`` 必然非零，读数会被误读成"假设成立"。所以默认**没有 scene app 就拒绝测量**
（逃生门 ``--allow-no-scene``）。

    # 终端 A：先起 Isaac，等头显里看到画面
    conda activate env_isaaclab
    GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl \
    UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
    python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
        --action_source sonic_dds --device cpu --teleop_device motion_controllers --xr

    # 终端 B：确认 Isaac 画面已在头显里之后再跑
    conda activate env_isaaclab
    python tools/xr_overlay_probe.py --mode dashboard   # ⭐当前主攻
    python tools/xr_overlay_probe.py --mode none        # 基线对照
    # tiny/panel/blink 已被否决，保留仅为复核

启动时会打印 ``✅ scene app pid = ...`` 确认 Isaac 已接入；中途 Isaac 退出也会报出来，
免得后半段读数悄悄变成另一个工况。

依赖 ``pip install openvr``（纯 ctypes 绑定，无需编译）。本工具以
``VRApplication_Overlay`` 身份接入，**不会**抢占 Isaac 的 scene app 身份。
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
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
        self.dashboard_key: str | None = None
        self.visible = False
        self.scene_pid: int | None = None
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
        self._self_check_timing_fields()
        # 残留实例的检查要在这里、而不是 create_overlay 里:mode=none 也会被残留实例
        # 贴的 overlay 污染("基线"其实不是基线)。
        self._warn_on_stale_instances()
        self.scene_pid = self._query_scene_pid()
        self._require_scene_app()

    def _query_scene_pid(self) -> int | None:
        """当前 scene app 的 pid；None 表示查不到（接口不可用）。0 表示确实没有。"""
        try:
            return int(self.openvr.VRApplications().getCurrentSceneProcessId())
        except Exception:
            return None

    def _require_scene_app(self):
        """没有 scene app 就拒绝继续——这是本工具最容易出的假阳性。

        没有 scene app 时 compositor 要自己渲染 SteamVR 的默认环境，comp_gpu **必然非零**,
        看起来就像"假设成立"，实际上什么也没证明。而且头显里只会看到本探针的图案、
        看不到 Isaac 画面（2026-07-30 用户实际踩到）。
        """
        if self.scene_pid is None:
            print("[probe] ⚠️ 无法查询 scene app（IVRApplications 不可用），测量继续但请自行确认 Isaac 在跑")
            return
        if self.scene_pid:
            print(f"[probe] ✅ scene app pid = {self.scene_pid}（Isaac 已接入）")
            return
        if self.args.allow_no_scene:
            print(
                "[probe] ⚠️⚠️ 没有 scene app,但 --allow-no-scene 已指定。"
                "注意 comp_gpu 读数此时**不能**用来判断假设成立与否。"
            )
            return
        sys.exit(
            "[probe] ❌ 当前没有 scene app 接入,拒绝测量。\n"
            "\n"
            "  没有 scene app 时 compositor 会渲染 SteamVR 自己的默认环境,comp_gpu 必然非零,\n"
            "  读数看起来像'假设成立'但什么也没证明;头显里也只会看到本探针的图案。\n"
            "\n"
            "  正确流程是两个终端:\n"
            "    终端 A: GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl \\\n"
            "            UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \\\n"
            "            python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor \\\n"
            "              --robot_type g129 --action_source sonic_dds --device cpu \\\n"
            "              --teleop_device motion_controllers --xr\n"
            "    终端 B: 等头显里看到 Isaac 画面后,再跑本探针\n"
            "\n"
            "  确实要在无 scene app 下空跑,加 --allow-no-scene。"
        )

    def _check_scene_app_alive(self):
        """Isaac 中途退出/崩掉必须让人看见,否则后半段读数是另一个工况的。"""
        if self.scene_pid is None:
            return
        pid = self._query_scene_pid()
        if pid == self.scene_pid:
            return
        if not pid:
            print("[probe] ⚠️ scene app 已消失(Isaac 退出?)——此后的读数不再是真实工况")
        else:
            print(f"[probe] ⚠️ scene app 变了:{self.scene_pid} -> {pid}")
        self.scene_pid = pid

    def create_overlay(self):
        openvr = self.openvr
        args = self.args
        ov = openvr.VROverlay()
        # createOverlay 而非 createDashboardOverlay：我们要的是 in-game overlay,
        # dashboard overlay 只在菜单打开时可见，那就变成复现已知现象而不是测新假设。
        #
        # key 带 pid：overlay key 全局唯一，固定 key 会让"上一次忘了退出的实例"
        # 直接把这次 createOverlay 打成 OverlayError_KeyInUse（2026-07-30 踩过）。
        # 带 pid 后新实例总能起来，但残留实例仍会多贴一个 overlay 干扰读数，
        # 所以下面还要显式查一次并提醒。
        key = f"unitree.judder.probe.{os.getpid()}"
        self.overlay_handle = ov.createOverlay(key, "Judder Probe")

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

    def create_dashboard_overlay(self):
        """打开 dashboard，但让它显示**我们自己的**小 overlay 而不是 SteamVR 主菜单。

        这是 §4.1 假设被否决后的转向。区别在哪：

        - in-game overlay（tiny/panel）：compositor 只在**应用提交新帧时**把
          overlay + scene layer 合成一张，然后整张原样重发 → overlay 跟着 scene 一起晃。
          已被实测否决（用户:"这个和 isaac 是分开渲染的，问题一样，都会晃动"）。
        - dashboard 模式：compositor 变成**每 vsync 的渲染源**，Isaac 画面被它逐帧按
          最新头姿重绘 → 用户实测**不晃**。

        所以要复现的不是"存在 overlay"，而是让 compositor 一直待在 dashboard 模式里。
        ``showDashboard(key)`` 可以指定显示哪个 dashboard overlay——指向一个极小的
        自有 overlay，就有机会既拿到"不晃"又不被主菜单挡住视野。

        ⚠️ 这是 hack 不是正解，已知代价见 doc/xr_ar_judder_zh.md §4.2。
        """
        openvr = self.openvr
        args = self.args
        ov = openvr.VROverlay()
        key = f"unitree.judder.dash.{os.getpid()}"
        main_handle, thumb_handle = ov.createDashboardOverlay(key, "Judder ATW")
        self.overlay_handle = main_handle
        self.dashboard_key = key

        # main：越小越不挡视野——我们要的是"进入 dashboard 模式"这个副作用，不是这个 overlay 本身
        pattern = _make_pattern(64, "solid")
        cbuf = (ctypes.c_char * len(pattern)).from_buffer_copy(pattern)
        ov.setOverlayRaw(main_handle, cbuf, 64, 64, 4)
        ov.setOverlayWidthInMeters(main_handle, args.width)
        ov.setOverlayAlpha(main_handle, args.alpha)

        # thumbnail 是 dashboard 应用栏里的图标；不设会显示成空白条目
        thumb = _make_pattern(64, "grid")
        tbuf = (ctypes.c_char * len(thumb)).from_buffer_copy(thumb)
        try:
            ov.setOverlayRaw(thumb_handle, tbuf, 64, 64, 4)
        except Exception as e:
            print(f"[probe] thumbnail 设置失败（不影响主流程）：{e}")

        ov.showDashboard(key)
        self.visible = True
        print(
            f"[probe] dashboard overlay 已创建并打开:key={key} 宽={args.width}m "
            f"alpha={args.alpha}"
        )
        print(
            "[probe] 现在戴头显看:①Isaac 画面还晃不晃 ②这个 overlay 与 SteamVR 工具栏挡多少视野"
        )
        print(
            "[probe] 想做 A/B 就在头显里手动关掉 dashboard——退出时会自动分两组对比 comp_gpu"
        )

    def _reassert_dashboard(self):
        """dashboard 被系统/用户关掉后重新打开（--reassert-dashboard）。

        默认不开:它会把用户手动关 dashboard 做 A/B 的动作立刻顶回去。
        """
        if not self.dashboard_key:
            return
        try:
            if not self.openvr.VROverlay().isDashboardVisible():
                self.openvr.VROverlay().showDashboard(self.dashboard_key)
                print("[probe] dashboard 被关掉了，已重新打开（--reassert-dashboard）")
        except Exception as e:
            print(f"[probe] 重开 dashboard 失败：{e}")

    def _warn_on_stale_instances(self):
        """残留的探针实例会多贴一个 overlay，让"有几个 overlay"这个前提说不清。

        用 findOverlay 扫其他 pid 的 key 不现实（要枚举 pid），改为扫 /proc:
        这是本机诊断工具，直接看进程表最省事也最准。
        """
        me = os.getpid()
        stale = []
        for entry in os.listdir("/proc"):
            if not entry.isdigit() or int(entry) == me:
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as fh:
                    cmd = fh.read().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue
            if "xr_overlay_probe.py" not in cmd:
                continue
            # 只认真正的 python 进程：否则包裹本进程的 shell（cmdline 里带着整条命令）
            # 和 `timeout ... python ...` 的包装进程都会被误报成残留实例。
            try:
                exe = os.path.basename(os.readlink(f"/proc/{entry}/exe"))
            except OSError:
                continue
            if "python" not in exe:
                continue
            stale.append((entry, cmd.strip()))
        if not stale:
            return
        print("[probe] ⚠️ 检测到其他探针实例仍在运行——它们各自贴着一个 overlay，会污染本次判读：")
        for pid, cmd in stale:
            print(f"[probe]     pid {pid}: {cmd}")
        print("[probe]   建议先把它们停掉（kill <pid>）再测。")

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

    def _self_check_timing_fields(self):
        """启动时核对 FrameTiming 字段名，缺了就当场报错。

        ⚠️ 这里刻意**不用** ``getattr(..., default)`` 兜底：2026-07-30 第一版就是那么写的,
        结果 ``getFrameTiming()`` 返回的是 ``(result, timing)`` **元组**而不是 struct,
        getattr 全部落到默认值,整屏读数打成 ``nan`` 却不报错——白跑了三轮实验。
        静默兜底在测量工具里是负资产。
        """
        required = (
            "m_nSize",
            "m_flCompositorRenderGpuMs",
            "m_flCompositorRenderCpuMs",
            "m_flPreSubmitGpuMs",
            "m_flClientFrameIntervalMs",
            "m_nNumFramePresents",
            "m_nNumMisPresented",
            "m_nNumDroppedFrames",
            "m_nReprojectionFlags",
        )
        # 取 entry[0] 而不解包：ctypes 的 _fields_ 条目可以是 (name, type) 也可以是
        # (name, type, bitwidth)，解包成两个会在带 bitfield 的绑定上直接崩。
        have = {entry[0] for entry in self.openvr.Compositor_FrameTiming._fields_}
        missing = [f for f in required if f not in have]
        if missing:
            sys.exit(
                f"[probe] ❌ pyopenvr 的 Compositor_FrameTiming 缺字段:{missing}\n"
                "  绑定版本与本工具预期不符,读数会失真。装一个较新的:pip install -U openvr"
            )

    def _dashboard_visible(self) -> bool | None:
        """dashboard 当前是否可见；None 表示查不到。

        这一列是整个工具里最有价值的东西：dashboard 是**已知会让 compositor 自己渲染**
        的状态（§1 实证：那时连 Isaac 画面都不抖）。把它和 comp_gpu 一起采样，
        既能验证 comp_gpu 这个读数到底反映不反映 compositor 的渲染状态，
        又能自动分出"dashboard 开/关"两组做对照——不需要人工对时间戳。
        """
        try:
            return bool(self.openvr.VROverlay().isDashboardVisible())
        except Exception:
            return None

    def _frame_timing(self):
        """取一帧 timing。

        ⚠️ 不能用 pyopenvr 的 ``getFrameTiming()`` 便捷封装:它返回 ``(result, timing)``
        元组,且**不设** ``m_nSize``——而 openvr.h 明确要求调用前把 size 填好。
        这里直接走 function_table 自己填。
        """
        cls = self.openvr.Compositor_FrameTiming
        timing = cls()
        timing.m_nSize = ctypes.sizeof(cls)
        try:
            ok = self.openvr.VRCompositor().function_table.getFrameTiming(
                ctypes.byref(timing), 0
            )
        except Exception as e:
            print(f"[probe] getFrameTiming 调用失败:{e}")
            return None
        if not ok:
            # compositor 还没攒够历史,或本进程拿不到 timing。不要伪造读数。
            return None
        return {
            "comp_gpu": timing.m_flCompositorRenderGpuMs,
            "comp_cpu": timing.m_flCompositorRenderCpuMs,
            "app_gpu": timing.m_flPreSubmitGpuMs,
            "presents": timing.m_nNumFramePresents,
            "mispresent": timing.m_nNumMisPresented,
            "dropped": timing.m_nNumDroppedFrames,
            "reproj": timing.m_nReprojectionFlags,
            "interval": timing.m_flClientFrameIntervalMs,
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
            "[probe] 对照：把 dashboard 开一会儿再关掉，退出时会自动分组对比两种状态下的 comp_gpu。"
        )
        print(
            "[probe] "
            + " | ".join(
                [
                    "t",
                    "overlay",
                    "dash",
                    "comp_gpu",
                    "comp_cpu",
                    "app_gpu",
                    "interval",
                    "presents",
                    "reproj",
                ]
            )
        )

        t0 = time.monotonic()
        last_blink = t0
        composited_seen = False
        unavailable = 0
        samples: list[float] = []
        # 按 dashboard 状态分组，用于退出时的自动对照
        by_dash: dict[str, list[float]] = {"on": [], "off": []}

        while not self._stop:
            now = time.monotonic()
            if args.mode == "blink" and now - last_blink >= args.blink_period:
                self.hide() if self.visible else self.show()
                last_blink = now
                print(f"[probe] --- overlay -> {'ON' if self.visible else 'OFF'} ---")
            if args.reassert_dashboard:
                self._reassert_dashboard()

            self._check_scene_app_alive()
            tm = self._frame_timing()
            if tm is None:
                unavailable += 1
                if unavailable == 1:
                    print("[probe] ⚠️ getFrameTiming 返回 false —— compositor 未就绪或本进程拿不到 timing")
            else:
                samples.append(tm["comp_gpu"])
                if tm["comp_gpu"] > COMPOSITE_GPU_MS_THRESHOLD:
                    composited_seen = True
                dash = self._dashboard_visible()
                if dash is not None:
                    by_dash["on" if dash else "off"].append(tm["comp_gpu"])
                dash_txt = "?" if dash is None else ("ON " if dash else "off")
                print(
                    f"[probe] {now - t0:6.1f}s | {'ON ' if self.visible else 'OFF'} | {dash_txt} | "
                    f"{tm['comp_gpu']:8.3f} | {tm['comp_cpu']:8.3f} | {tm['app_gpu']:7.3f} | "
                    f"{tm['interval']:8.2f} | {tm['presents']:8d} | {_decode_reprojection(tm['reproj'])}"
                )
            time.sleep(args.interval)

        if unavailable:
            print(f"\n[probe] ⚠️ 有 {unavailable} 次采样拿不到 timing（已排除，未计入统计）")
        if not samples:
            print(
                "[probe] ❌ 一个有效样本都没有 —— 本次没有任何可用读数，不要据此下结论。"
            )
        if samples:
            avg = sum(samples) / len(samples)
            peak = max(samples)
            print(
                f"\n[probe] comp_gpu 均值 {avg:.4f}ms / 峰值 {peak:.4f}ms（{len(samples)} 个样本）"
            )
            if not self.scene_pid:
                # 没有 scene app 时 compositor 在渲染 SteamVR 自己的环境,
                # 这个读数与"overlay 能否逼它退出直通"无关。不能给结论。
                print(
                    "[probe] ⚠️ 本次没有（或中途失去）scene app —— 上面的读数**不能**用来判断假设，"
                    "compositor 此时在渲染 SteamVR 自己的环境。请先起 Isaac 再重测。"
                )
            elif self.args.mode == "dashboard":
                if composited_seen:
                    print(
                        "[probe] ✅ dashboard 模式下 comp_gpu 明显上升 —— compositor 确实成了"
                        "每帧的渲染源（符合预期）。**结论要靠主观**：Isaac 画面还晃不晃、"
                        "视野被挡多少。"
                    )
                else:
                    print(
                        "[probe] ⚠️ dashboard 模式下 comp_gpu 却没上升 —— 与 §1 的实证不符，"
                        "先确认 dashboard 真的开着（看 dash 列）再判读。"
                    )
            elif composited_seen:
                print(
                    "[probe] ✅ compositor 出现了明显的 GPU 渲染开销 —— 它没有在直通，"
                    "假设成立的可能性大。接着戴头显做主观判读。"
                )
            elif self.args.mode == "none":
                # 基线模式没建 overlay，读数只说明"此刻 compositor 在直通",
                # 不能拿来否决 overlay 假设——否决要由 tiny/panel 组给出。
                print(
                    "[probe] ℹ️ 基线：comp_gpu 贴近 0，compositor 在直通（符合预期）。"
                    "这是对照用的基线，不构成对 overlay 假设的结论。"
                )
            else:
                print(
                    "[probe] ❌ comp_gpu 全程贴近 0 —— compositor 仍在直通，"
                    f"存在 overlay（mode={self.args.mode}）不足以让它转入合成模式。"
                )
        self._report_dashboard_contrast(by_dash)

    @staticmethod
    def _report_dashboard_contrast(by_dash: dict[str, list[float]]):
        """dashboard 开/关两组 comp_gpu 的自动对照。

        这一段决定上面那个结论能不能信：
        - 两组都贴近 0 ⇒ **comp_gpu 反映不了 compositor 的渲染状态**，判据本身失效，
          得换别的观测量，别急着说"假设否决";
        - dashboard 开着时明显更高 ⇒ 判据有效，那么 overlay 组仍贴近 0 就是真的否决。
        """
        on, off = by_dash["on"], by_dash["off"]
        if not on and not off:
            return
        print("\n[probe] --- dashboard 对照 ---")
        for label, xs in (("dashboard 开", on), ("dashboard 关", off)):
            if xs:
                print(
                    f"[probe]   {label}：comp_gpu 均值 {sum(xs) / len(xs):.4f}ms / "
                    f"峰值 {max(xs):.4f}ms（{len(xs)} 样本）"
                )
            else:
                print(f"[probe]   {label}：无样本")
        if not on:
            print(
                "[probe]   ⚠️ 没采到 dashboard 打开的样本 —— 判据的有效性未被验证。"
                "请戴头显把 dashboard 开一会儿再重测。"
            )
            return
        if not off:
            return
        on_avg = sum(on) / len(on)
        off_avg = sum(off) / len(off)
        if on_avg > max(COMPOSITE_GPU_MS_THRESHOLD, off_avg * 3):
            print(
                f"[probe]   ✅ 判据有效：dashboard 开着时 comp_gpu 高出 {on_avg / max(off_avg, 1e-9):.0f}× "
                "—— 这个读数确实反映 compositor 是否在渲染。"
            )
        else:
            print(
                "[probe]   ⚠️ dashboard 开关两组没有明显差异 —— comp_gpu 可能反映不了 compositor "
                "的渲染状态（overlay 应用拿到的也许不是全局 timing）。**此时不能用它下结论**，"
                "得另找观测量。"
            )

    def _on_sigint(self, *_):
        self._stop = True


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "SteamVR overlay 探针：让 compositor 待在 dashboard 模式以消除 AR judder。"
            "（原假设'存在 overlay 即可'已被实测否决，见 doc/xr_ar_judder_zh.md §4.1.1）"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--mode",
        choices=("none", "dashboard", "tiny", "panel", "blink"),
        default="dashboard",
        help=(
            "none: 只做遥测不建 overlay（拿基线）; "
            "dashboard: ⭐打开 dashboard 但只显示自有小 overlay —— 目前唯一被实测证明"
            "不晃的方向; "
            "tiny/panel/blink: in-game overlay，**已被实测否决**（overlay 与 scene 一起晃），"
            "保留仅为复核与留档"
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
    p.add_argument(
        "--reassert-dashboard",
        action="store_true",
        help=(
            "dashboard 被关掉后自动重新打开。默认关闭——它会把你手动关 dashboard "
            "做 A/B 的动作立刻顶回去"
        ),
    )
    p.add_argument(
        "--allow-no-scene",
        action="store_true",
        help=(
            "允许在没有 scene app（Isaac 未启动）时空跑。默认拒绝，"
            "因为那时 compositor 在渲染 SteamVR 自己的环境，comp_gpu 必然非零、读数无意义"
        ),
    )
    return p


def apply_mode_defaults(args):
    """按 mode 补默认几何：tiny / dashboard 要"几乎不挡视线"，panel 要"看得清边缘"。

    显式传入的值一律优先——实验里经常要手动挪位置试遮挡程度。
    """
    if args.mode in ("tiny", "dashboard"):
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
        if args.mode == "dashboard":
            probe.create_dashboard_overlay()
        elif args.mode != "none":
            probe.create_overlay()
        else:
            print("[probe] mode=none：只做遥测，不创建 overlay（这是基线）")
        probe.run()
    finally:
        probe.shutdown()
        print("[probe] 已退出")


if __name__ == "__main__":
    main()
