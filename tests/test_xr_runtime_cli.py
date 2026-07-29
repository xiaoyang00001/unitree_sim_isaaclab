"""`--xr_runtime` 的命令行契约测试。

这些用例锁死一个真踩过的坑:`--xr_runtime` 必须定义在
``AppLauncher.add_app_launcher_args()`` **之后**。该函数内部会先跑一次
``parser.parse_known_args()`` 探测,而 ``--xr`` 是在探测之后才注册的。若探测时
``--xr_runtime`` 已存在,argparse 的缩写匹配会把 ``--xr`` 当成 ``--xr_runtime``
的缩写,于是 ``--xr`` 去吃下一个 token 当值,报
``argument --xr_runtime: expected one argument``——所有现有的 ``--xr`` 命令行
全部被打断。

测试不启动 Isaac Sim:只用 argparse 复刻定义顺序,以及静态检查 sim_main.py 里
两处的先后位置。
"""

from __future__ import annotations

import argparse
import re
import unittest
from pathlib import Path

_SIM_MAIN = Path(__file__).resolve().parents[1] / "sim_main.py"


def _build_parser(
    xr_runtime_first: bool, argv: list[str]
) -> tuple[argparse.ArgumentParser, bool]:
    """复刻 sim_main.py 的定义顺序。

    返回 ``(parser, probe_failed)``——``probe_failed`` 表示
    ``add_app_launcher_args()`` 内部那次 ``parse_known_args(argv)`` 探测是否炸了,
    这正是真实故障发生的位置。
    """
    parser = argparse.ArgumentParser()

    if xr_runtime_first:
        parser.add_argument("--xr_runtime", choices=("auto", "steamvr", "cloudxr"), default="auto")

    # AppLauncher.add_app_launcher_args() 的真实行为:先用命令行探测一次,再注册 --xr
    probe_failed = False
    try:
        parser.parse_known_args(argv)
    except SystemExit:
        probe_failed = True
    parser.add_argument("--xr", action="store_true", default=False)

    if not xr_runtime_first:
        parser.add_argument("--xr_runtime", choices=("auto", "steamvr", "cloudxr"), default="auto")

    return parser, probe_failed


class XrRuntimeCliOrderTest(unittest.TestCase):
    def test_definition_after_app_launcher_args_keeps_xr_working(self):
        """正确顺序下探测不炸,且 `--xr --xr_runtime cloudxr` 能完整解析。"""
        argv = ["--xr", "--xr_runtime", "cloudxr"]
        parser, probe_failed = _build_parser(xr_runtime_first=False, argv=argv)
        self.assertFalse(probe_failed, "探测期不应报错")
        args = parser.parse_args(argv)
        self.assertTrue(args.xr)
        self.assertEqual(args.xr_runtime, "cloudxr")

    def test_definition_before_app_launcher_args_breaks_probe(self):
        """错误顺序会让探测炸掉 —— 证明本测试守的不是假想问题。

        故障点在 add_app_launcher_args() 内部那次 parse_known_args:那时 --xr
        还没注册,argparse 把它当成 --xr_runtime 的缩写,于是去吃下一个 token,
        报 "expected one argument"。
        """
        argv = ["--xr", "--xr_runtime", "cloudxr"]
        _, probe_failed = _build_parser(xr_runtime_first=True, argv=argv)
        self.assertTrue(probe_failed, "错误顺序本应在探测期就失败")

    def test_bare_xr_still_parses(self):
        """只给 --xr(不给 --xr_runtime)也不能被缩写匹配吃掉。"""
        argv = ["--xr"]
        parser, probe_failed = _build_parser(xr_runtime_first=False, argv=argv)
        self.assertFalse(probe_failed)
        args = parser.parse_args(argv)
        self.assertTrue(args.xr)
        self.assertEqual(args.xr_runtime, "auto")

    def test_sim_main_defines_xr_runtime_after_app_launcher_args(self):
        """静态守卫:防止有人把参数定义挪回 add_app_launcher_args 之前。"""
        source = _SIM_MAIN.read_text(encoding="utf-8")
        launcher_at = source.find("AppLauncher.add_app_launcher_args(parser)")
        runtime_at = source.find('"--xr_runtime"')
        parse_at = source.find("args_cli = parser.parse_args()")

        self.assertGreater(launcher_at, 0, "找不到 add_app_launcher_args 调用")
        self.assertGreater(runtime_at, 0, "找不到 --xr_runtime 定义")
        self.assertGreater(
            runtime_at,
            launcher_at,
            "--xr_runtime 必须定义在 add_app_launcher_args() 之后,否则 --xr 会被"
            "argparse 当成它的缩写而失效(见本文件 docstring)",
        )
        self.assertGreater(
            parse_at, runtime_at, "--xr_runtime 必须在 parse_args() 之前定义"
        )


class XrRuntimeResolutionTest(unittest.TestCase):
    """CloudXR 就绪判据与环境快照解析——不依赖真实 runtime。"""

    def test_cloudxr_env_parsing_handles_export_prefix(self):
        """cloudxr.env 每行是 `export KEY=value`,解析要去掉 export 前缀。"""
        import tempfile
        import os
        import sys

        sys.path.insert(0, str(_SIM_MAIN.parent))
        source = _SIM_MAIN.read_text(encoding="utf-8")
        match = re.search(
            r"def _load_cloudxr_env\(run_dir: str\) -> dict:.*?\n    return parsed\n",
            source,
            re.S,
        )
        if match is None:
            self.fail("找不到 _load_cloudxr_env 定义")
        namespace: dict = {"os": os}
        exec(match.group(0), namespace)  # noqa: S102 - 受控源码片段

        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, "cloudxr.env"), "w", encoding="utf-8") as fp:
                fp.write(
                    "# comment\n"
                    "\n"
                    "export XR_RUNTIME_JSON=/home/x/.cloudxr/openxr_cloudxr.json\n"
                    "export NV_CXR_RUNTIME_DIR=/home/x/.cloudxr/run\n"
                    'export QUOTED="value"\n'
                )
            parsed = namespace["_load_cloudxr_env"](tmp)

        self.assertEqual(
            parsed["XR_RUNTIME_JSON"], "/home/x/.cloudxr/openxr_cloudxr.json"
        )
        self.assertEqual(parsed["NV_CXR_RUNTIME_DIR"], "/home/x/.cloudxr/run")
        self.assertEqual(parsed["QUOTED"], "value")
        self.assertNotIn("# comment", parsed)


if __name__ == "__main__":
    unittest.main()
