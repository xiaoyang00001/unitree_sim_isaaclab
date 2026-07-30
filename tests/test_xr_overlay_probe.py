# SPDX-FileCopyrightText: Copyright (c) 2025 Unitree Robotics
# SPDX-License-Identifier: Apache-2.0
"""``tools/xr_overlay_probe.py`` 的纯逻辑测试（不连 SteamVR、不装 pyopenvr 也能跑）。

守两件事：
1. 判读用的 ``_decode_reprojection`` 不能误读——它的输出会直接进实验记录；
2. ``tiny`` 模式的默认几何不能被改大。整个假设是"几乎不可见的 overlay 也能逼
   compositor 合成"，overlay 一旦变大就变成另一个实验了。
"""

from __future__ import annotations

import importlib.util
import pathlib
import unittest

_PROBE_PATH = pathlib.Path(__file__).resolve().parents[1] / "tools" / "xr_overlay_probe.py"


def _load_probe_module():
    spec = importlib.util.spec_from_file_location("xr_overlay_probe", _PROBE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


probe = _load_probe_module()


class DecodeReprojectionTest(unittest.TestCase):
    def test_zero_is_none(self):
        self.assertEqual(probe._decode_reprojection(0), "none")

    def test_single_flags(self):
        self.assertEqual(probe._decode_reprojection(0x01), "Cpu")
        self.assertEqual(probe._decode_reprojection(0x02), "Gpu")
        self.assertEqual(probe._decode_reprojection(0x04), "Async")
        self.assertEqual(probe._decode_reprojection(0x08), "Motion")

    def test_combined_flags(self):
        self.assertEqual(probe._decode_reprojection(0x05), "Cpu+Async")

    def test_high_bits_are_surfaced_not_swallowed(self):
        # 高位是 prediction frame 计数。丢掉它会让"到底有没有做预测"读不出来。
        self.assertEqual(probe._decode_reprojection(0x24), "Async+hi=0x20")

    def test_unknown_low_bits_do_not_vanish(self):
        # 只有高位、低位无已知标志时也必须留下痕迹
        self.assertEqual(probe._decode_reprojection(0x10), "hi=0x10")


class PatternTest(unittest.TestCase):
    def test_buffer_size_is_rgba(self):
        for size in (16, 64):
            self.assertEqual(len(probe._make_pattern(size, "grid")), size * size * 4)

    def test_fully_opaque(self):
        # 透明区域可能被 compositor 优化掉，那会把"存在 overlay"这个前提测糊
        buf = probe._make_pattern(16, "grid")
        self.assertTrue(all(buf[i] == 255 for i in range(3, len(buf), 4)))

    def test_solid_kind_is_uniform(self):
        buf = probe._make_pattern(8, "solid")
        self.assertEqual(set(buf[0::4]), {255})
        self.assertEqual(set(buf[1::4]), {0})

    def test_grid_has_contrast(self):
        # 主观判读靠硬边缘，图案必须真的有明暗差
        buf = probe._make_pattern(64, "grid")
        self.assertGreater(len(set(buf[0::4])), 1)


class ModeDefaultsTest(unittest.TestCase):
    def _parse(self, argv):
        return probe.apply_mode_defaults(probe.build_parser().parse_args(argv))

    def test_default_mode_is_dashboard(self):
        # in-game overlay 假设已被否决，默认必须落在当前主攻方向上
        self.assertEqual(self._parse([]).mode, "dashboard")

    def test_dashboard_overlay_stays_small(self):
        # dashboard 模式要的是"进入 dashboard 模式"这个副作用，overlay 本身越小越不挡视野
        args = self._parse(["--mode", "dashboard"])
        self.assertLessEqual(args.width, 0.05)

    def test_tiny_stays_tiny(self):
        args = self._parse(["--mode", "tiny"])
        self.assertLessEqual(args.width, 0.05, "tiny overlay 变大就不是同一个实验了")
        self.assertEqual(args.mode, "tiny")

    def test_panel_is_large_enough_to_judge(self):
        args = self._parse(["--mode", "panel"])
        self.assertGreaterEqual(args.width, 1.0)
        self.assertEqual(args.alpha, 1.0)

    def test_explicit_values_win_over_mode_defaults(self):
        args = self._parse(["--mode", "tiny", "--width", "0.5", "--alpha", "0.2", "--y", "1.1"])
        self.assertEqual(args.width, 0.5)
        self.assertEqual(args.alpha, 0.2)
        self.assertEqual(args.y, 1.1)

    def test_none_mode_still_gets_geometry_defaults(self):
        # mode=none 不建 overlay，但参数不该是 None，否则打印/记录会出 TypeError
        args = self._parse(["--mode", "none"])
        self.assertIsNotNone(args.width)
        self.assertIsNotNone(args.alpha)
        self.assertIsNotNone(args.y)


class ThresholdTest(unittest.TestCase):
    def test_threshold_is_above_passthrough_baseline(self):
        # vrcompositor.txt 实测直通时 GPU 0.007-0.011ms，阈值必须明显高于它
        self.assertGreater(probe.COMPOSITE_GPU_MS_THRESHOLD, 0.011)


class TimingFieldsTest(unittest.TestCase):
    """守住那次 nan 事故：便捷封装返回元组，getattr 兜底把它静默成了 nan。"""

    def test_required_fields_exist_in_installed_binding(self):
        try:
            import openvr
        except ImportError:
            self.skipTest("未装 pyopenvr")
        have = {entry[0] for entry in openvr.Compositor_FrameTiming._fields_}
        for field in (
            "m_nSize",
            "m_flCompositorRenderGpuMs",
            "m_flCompositorRenderCpuMs",
            "m_flPreSubmitGpuMs",
            "m_flClientFrameIntervalMs",
            "m_nNumFramePresents",
            "m_nReprojectionFlags",
        ):
            self.assertIn(field, have)

    def test_convenience_wrapper_returns_tuple_not_struct(self):
        """锁死当初踩坑的根源，免得有人把 _frame_timing 又"简化"回便捷封装。"""
        try:
            import inspect

            import openvr
        except ImportError:
            self.skipTest("未装 pyopenvr")
        src = inspect.getsource(openvr.IVRCompositor.getFrameTiming)
        self.assertIn("return result, timing", src)


class StaleInstanceScanTest(unittest.TestCase):
    """残留实例扫描必须只认真正的 python 进程。

    第一版按 cmdline 子串匹配，把包裹自己的 shell（cmdline 里带着整条命令）和
    ``timeout ... python ...`` 的包装进程全报成了残留实例——满屏假阳性。
    """

    def _scan(self, procs, me=999):
        """procs: {pid: (cmdline_bytes, exe_path)}"""
        import contextlib
        import io
        import unittest.mock as mock

        def fake_open(path, *_):
            pid = path.split("/")[2]
            return io.BytesIO(procs[pid][0])

        def fake_readlink(path):
            return procs[path.split("/")[2]][1]

        obj = probe.Probe.__new__(probe.Probe)
        out = io.StringIO()
        with (
            mock.patch.object(probe.os, "listdir", return_value=list(procs)),
            mock.patch.object(probe.os, "getpid", return_value=me),
            mock.patch.object(probe.os, "readlink", side_effect=fake_readlink),
            mock.patch("builtins.open", fake_open),
            contextlib.redirect_stdout(out),
        ):
            probe.Probe._warn_on_stale_instances(obj)
        return out.getvalue()

    def test_reports_real_python_instance(self):
        text = self._scan(
            {"1234": (b"python\0tools/xr_overlay_probe.py\0--mode\0tiny", "/usr/bin/python3.11")}
        )
        self.assertIn("1234", text)
        self.assertIn("检测到其他探针实例", text)

    def test_ignores_shell_wrapper_with_script_name_in_cmdline(self):
        text = self._scan({"1234": (b"bash\0-c\0eval 'python tools/xr_overlay_probe.py'", "/bin/bash")})
        self.assertEqual(text, "")

    def test_ignores_timeout_wrapper(self):
        text = self._scan(
            {"1234": (b"timeout\0-s\0TERM\0python\0tools/xr_overlay_probe.py", "/usr/bin/timeout")}
        )
        self.assertEqual(text, "")

    def test_ignores_unrelated_processes(self):
        text = self._scan({"1234": (b"python\0sim_main.py\0--xr", "/usr/bin/python3.11")})
        self.assertEqual(text, "")

    def test_skips_self(self):
        text = self._scan(
            {"999": (b"python\0tools/xr_overlay_probe.py", "/usr/bin/python3.11")}, me=999
        )
        self.assertEqual(text, "")


class DashboardContrastTest(unittest.TestCase):
    """dashboard 对照是"判据到底有效吗"的自检，三个分支都要说对话。"""

    def _run(self, on, off):
        import contextlib
        import io

        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            probe.Probe._report_dashboard_contrast({"on": on, "off": off})
        return out.getvalue()

    def test_no_samples_is_silent(self):
        self.assertEqual(self._run([], []), "")

    def test_missing_dashboard_samples_says_unvalidated(self):
        text = self._run([], [0.007, 0.007])
        self.assertIn("判据的有效性未被验证", text)

    def test_clear_difference_marks_criterion_valid(self):
        text = self._run([3.5, 4.0], [0.007, 0.008])
        self.assertIn("判据有效", text)

    def test_no_difference_warns_criterion_useless(self):
        # 两组都贴近 0 ⇒ 不能据此说"假设否决"，必须提醒换观测量
        text = self._run([0.007, 0.008], [0.007, 0.008])
        self.assertIn("不能用它下结论", text)


if __name__ == "__main__":
    unittest.main()
