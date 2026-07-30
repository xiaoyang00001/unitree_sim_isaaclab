"""`tools/xr_anchor_probe.py` 的注入契约测试。

探针每跑一轮真实测量要 5 分钟(启动 Isaac + 暖机 + 采样),所以注入逻辑本身
必须先在这里跑通:装错档、写入没被拦下、noop 没真的跳过,都会白烧一轮。

用例只用假的 synchronizer / XRCore,不启动 Isaac Sim。锁住四件事:
  1. off/非法档位的行为(不装 / 抛错);
  2. timing 档不改变行为——原 sync 与原写入(含 layer_identifier)都照走;
  3. fabric 档把 layer_identifier 丢掉,从而落到 XRCore 的 usdrt 写路径;
  4. nowrite/noop 档确实跳过了对应的工作。
"""

from __future__ import annotations

import io
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.xr_anchor_probe import VALID_MODES, install_xr_anchor_probe


class _FakeXrCore:
    """记录 set_world_transform_matrix 的实参,并暴露一个转发用的额外方法。"""

    def __init__(self):
        self.writes: list[tuple] = []
        self.input_device_calls = 0

    def set_world_transform_matrix(self, prim_path, matrix, layer_identifier=None):
        self.writes.append((prim_path, matrix, layer_identifier))

    def get_input_device(self, path):
        self.input_device_calls += 1
        return f"device:{path}"


class _FakeSync:
    """够用的 XrAnchorSynchronizer 替身:sync 会读两个 prim 再写一次。"""

    LAYER = "anonymous:fake-session-layer"

    def __init__(self, xr_core):
        self._xr_core = xr_core
        self.sync_calls = 0
        self.read_paths: list[str] = []

    def _get_prim_world_matrix(self, prim_path):
        self.read_paths.append(prim_path)
        return f"matrix@{prim_path}"

    def sync_headset_to_anchor(self):
        self.sync_calls += 1
        self._get_prim_world_matrix("/World/head_link")
        self._get_prim_world_matrix("/World/pelvis")
        self._xr_core.set_world_transform_matrix("/World/XRAnchor", "M", self.LAYER)


class _FakeDevice:
    def __init__(self, sync):
        self._anchor_sync = sync


def _install(mode: str):
    core = _FakeXrCore()
    sync = _FakeSync(core)
    device = _FakeDevice(sync)
    with redirect_stdout(io.StringIO()):
        installed = install_xr_anchor_probe(device, mode)
    return core, sync, installed


class XrAnchorProbeInstallTest(unittest.TestCase):
    def test_off_and_empty_do_not_install(self):
        for mode in ("off", "", "  OFF  "):
            core, sync, installed = _install(mode)
            self.assertFalse(installed, mode)
            sync.sync_headset_to_anchor()
            # 未安装时写入应带原 layer_identifier(即完全没被碰过)
            self.assertEqual(core.writes[0][2], _FakeSync.LAYER)

    def test_invalid_mode_raises(self):
        with self.assertRaises(ValueError) as ctx:
            _install("faster")
        self.assertIn("faster", str(ctx.exception))

    def test_missing_synchronizer_is_not_fatal(self):
        class _NoSync:
            _anchor_sync = None

        with redirect_stdout(io.StringIO()):
            self.assertFalse(install_xr_anchor_probe(_NoSync(), "timing"))

    def test_timing_preserves_behavior_including_layer(self):
        core, sync, installed = _install("timing")
        self.assertTrue(installed)
        sync.sync_headset_to_anchor()
        self.assertEqual(sync.sync_calls, 1)
        self.assertEqual(sync.read_paths, ["/World/head_link", "/World/pelvis"])
        self.assertEqual(len(core.writes), 1)
        # timing 必须原样保留 USD layer 写路径,否则它就不是"原行为"基准了
        self.assertEqual(core.writes[0][2], _FakeSync.LAYER)

    def test_fabric_drops_layer_identifier(self):
        core, sync, _ = _install("fabric")
        sync.sync_headset_to_anchor()
        self.assertEqual(len(core.writes), 1)
        # layer 为 None 时 XRCore 走 set_world_transform_matrix(usdrt/Fabric)
        self.assertIsNone(core.writes[0][2])

    def test_nowrite_skips_write_but_keeps_reads(self):
        core, sync, _ = _install("nowrite")
        sync.sync_headset_to_anchor()
        self.assertEqual(core.writes, [])
        self.assertEqual(len(sync.read_paths), 2)
        self.assertEqual(sync.sync_calls, 1)

    def test_noop_skips_everything(self):
        core, sync, _ = _install("noop")
        sync.sync_headset_to_anchor()
        self.assertEqual(sync.sync_calls, 0)
        self.assertEqual(sync.read_paths, [])
        self.assertEqual(core.writes, [])

    def test_proxy_forwards_other_xrcore_methods(self):
        """recenter 路径要用 get_input_device,代理不能把它吞掉。"""
        core, sync, _ = _install("timing")
        self.assertEqual(sync._xr_core.get_input_device("/user/head"), "device:/user/head")
        self.assertEqual(core.input_device_calls, 1)

    def test_all_valid_modes_install(self):
        # noanchor 会去写 carb 设置;本机无 carb 时其内部 except 兜住,仍应装上
        for mode in VALID_MODES:
            _, _, installed = _install(mode)
            self.assertTrue(installed, mode)


class XrAnchorProbeStatsTest(unittest.TestCase):
    def test_stats_line_reports_read_and_write_shares(self):
        from tools.xr_anchor_probe import _ProbeStats

        stats = _ProbeStats("timing", print_interval_s=0.0)
        stats.add_read(0.001)
        stats.add_read(0.001)
        stats.add_write(0.004)
        buf = io.StringIO()
        with redirect_stdout(buf):
            stats.add_call(0.010)
        line = buf.getvalue()
        self.assertIn("[xr_probe:timing]", line)
        self.assertIn("read", line)
        self.assertIn("write", line)
        self.assertIn("(share 40%)", line)  # write 4ms / call 10ms

    def test_window_resets_between_prints(self):
        from tools.xr_anchor_probe import _ProbeStats

        stats = _ProbeStats("timing", print_interval_s=0.0)
        stats.add_write(0.004)
        with redirect_stdout(io.StringIO()):
            stats.add_call(0.010)
            stats.add_call(0.010)
        # 第二次打印时上一窗口的样本必须已清空,否则占比会越算越离谱
        self.assertEqual(stats._write_samples, [])
        self.assertEqual(stats._call_samples, [])


if __name__ == "__main__":
    unittest.main()
