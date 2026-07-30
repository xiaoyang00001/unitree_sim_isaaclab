"""`tools/analyze_kit_trace.py` 的解析与统计测试。

trace 分析是归因的最后一环:结论直接来自这里的数字,算错了整个判断就跟着错。
两处最容易出错的逻辑锁在这里:

  1. **self 时间扣减** —— total 含子调用,嵌套 zone 会重复计入。只有 self
     (扣掉直接子 zone)才是"这段代码花的时间"。父子、兄弟、跨线程都要对。
  2. **截断容错** —— 抓取被 Ctrl-C / 组杀打断时 JSON 数组不闭合,必须还能读出
     已有事件,否则整段数据白丢(而这恰恰是最可能发生的情形)。

不启动 Isaac Sim,只喂构造好的 Chrome Trace 事件。
"""

from __future__ import annotations

import gzip
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.analyze_kit_trace import load_events, summarize


def _x(name, ts, dur, tid=1):
    return {"ph": "X", "name": name, "ts": ts, "dur": dur, "pid": 1, "tid": tid}


class SummarizeTest(unittest.TestCase):
    def test_complete_events_basic(self):
        stats, span, _ = summarize([_x("a", 0, 100), _x("a", 200, 300)])
        self.assertEqual(stats["a"]["calls"], 2)
        self.assertAlmostEqual(stats["a"]["total"], 400.0)
        self.assertAlmostEqual(stats["a"]["max"], 300.0)
        self.assertAlmostEqual(span, 0.5)  # ts 单位是 µs,span 报 ms

    def test_begin_end_pairing_and_self_subtraction(self):
        # parent 1000µs，内含 child 400µs ⇒ parent self=600, child self=400
        events = [
            {"ph": "B", "name": "parent", "ts": 0, "pid": 1, "tid": 1},
            {"ph": "B", "name": "child", "ts": 100, "pid": 1, "tid": 1},
            {"ph": "E", "name": "child", "ts": 500, "pid": 1, "tid": 1},
            {"ph": "E", "name": "parent", "ts": 1000, "pid": 1, "tid": 1},
        ]
        stats, _, _ = summarize(events)
        self.assertAlmostEqual(stats["parent"]["total"], 1000.0)
        self.assertAlmostEqual(stats["parent"]["self"], 600.0)
        self.assertAlmostEqual(stats["child"]["self"], 400.0)

    def test_two_children_both_subtracted(self):
        events = [
            {"ph": "B", "name": "p", "ts": 0, "pid": 1, "tid": 1},
            {"ph": "B", "name": "c1", "ts": 10, "pid": 1, "tid": 1},
            {"ph": "E", "name": "c1", "ts": 210, "pid": 1, "tid": 1},
            {"ph": "B", "name": "c2", "ts": 300, "pid": 1, "tid": 1},
            {"ph": "E", "name": "c2", "ts": 600, "pid": 1, "tid": 1},
            {"ph": "E", "name": "p", "ts": 1000, "pid": 1, "tid": 1},
        ]
        stats, _, _ = summarize(events)
        self.assertAlmostEqual(stats["p"]["self"], 1000.0 - 200.0 - 300.0)

    def test_complete_child_subtracted_from_begin_end_parent(self):
        """混合编码:X 型子 zone 也要从 B/E 型父 zone 的 self 里扣掉。"""
        events = [
            {"ph": "B", "name": "p", "ts": 0, "pid": 1, "tid": 1},
            _x("c", 100, 250),
            {"ph": "E", "name": "p", "ts": 1000, "pid": 1, "tid": 1},
        ]
        stats, _, _ = summarize(events)
        self.assertAlmostEqual(stats["c"]["self"], 250.0)
        self.assertAlmostEqual(stats["p"]["self"], 750.0)

    def test_stacks_are_per_thread(self):
        """不同 tid 的栈不能串,否则 self 会被扣到别的线程的 zone 上。"""
        events = [
            {"ph": "B", "name": "t1", "ts": 0, "pid": 1, "tid": 1},
            {"ph": "B", "name": "t2", "ts": 10, "pid": 1, "tid": 2},
            {"ph": "E", "name": "t2", "ts": 500, "pid": 1, "tid": 2},
            {"ph": "E", "name": "t1", "ts": 1000, "pid": 1, "tid": 1},
        ]
        stats, _, _ = summarize(events)
        self.assertAlmostEqual(stats["t1"]["self"], 1000.0)  # 未被 t2 扣减
        self.assertAlmostEqual(stats["t2"]["self"], 490.0)

    def test_unmatched_end_is_ignored(self):
        stats, _, _ = summarize([{"ph": "E", "name": "orphan", "ts": 5, "pid": 1, "tid": 1}])
        self.assertNotIn("orphan", stats)

    def test_events_without_ts_skipped(self):
        stats, _, _ = summarize([{"ph": "X", "name": "meta", "dur": 10}, _x("a", 0, 50)])
        self.assertNotIn("meta", stats)
        self.assertEqual(stats["a"]["calls"], 1)

    def test_frame_count_detected(self):
        events = [_x("carb::framework::update", i * 1000, 900) for i in range(7)]
        _, _, frames = summarize(events)
        self.assertEqual(frames, 7)


class LoadEventsTest(unittest.TestCase):
    def _write(self, text: str, gz: bool) -> Path:
        d = Path(tempfile.mkdtemp())
        p = d / ("t.gz" if gz else "t.json")
        if gz:
            with gzip.open(p, "wt") as f:
                f.write(text)
        else:
            p.write_text(text)
        return p

    def test_plain_array(self):
        p = self._write(json.dumps([_x("a", 0, 10)]), gz=False)
        self.assertEqual(len(load_events(p)), 1)

    def test_gzip_and_trace_events_dict(self):
        p = self._write(json.dumps({"traceEvents": [_x("a", 0, 10), _x("b", 0, 5)]}), gz=True)
        self.assertEqual(len(load_events(p)), 2)

    def test_truncated_array_recovered(self):
        """抓取被打断的真实情形:数组没闭合、末尾还挂着逗号。"""
        body = json.dumps([_x("a", 0, 10), _x("b", 20, 30)])
        truncated = body[:-1] + ","  # 去掉 ] 并留一个尾逗号
        p = self._write(truncated, gz=True)
        events = load_events(p)
        self.assertEqual(len(events), 2)
        stats, _, _ = summarize(events)
        self.assertIn("a", stats)
        self.assertIn("b", stats)

    def test_non_dict_entries_filtered(self):
        p = self._write(json.dumps([_x("a", 0, 10), "junk", 42]), gz=False)
        self.assertEqual(len(load_events(p)), 1)


if __name__ == "__main__":
    unittest.main()
