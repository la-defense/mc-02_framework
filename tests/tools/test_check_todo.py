"""TODO 契约、编号查重、状态和摘要统计的回归。

测试在临时文件上运行，不依赖仓库真实 TODO，也不修改任务状态。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import io
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "workflow"))

import check_todo


class TodoContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="mc02-todo-")
        self.path = Path(self.temporary.name) / "TODO.md"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write(self, body: str) -> None:
        self.path.write_text(body, encoding="utf-8")

    def run_main(self, *arguments: str) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = check_todo.main([str(self.path), *arguments])
        return code, out.getvalue(), err.getvalue()

    def test_valid_open_and_done_items_pass(self) -> None:
        self.write(
            "# TODO\n\n"
            "- [x] 历史清单项没有编号。\n"
            "- [x] **MC02-0001** [bug] 已完成项。验收：旧条件。\n"
            "- [ ] **MC02-0002** [enhancement] [ready-for-agent] 新任务。验收：可观察结果。\n"
        )
        code, out, err = self.run_main()
        self.assertEqual(code, 0)
        self.assertIn("契约有效", out)
        self.assertEqual(err, "")

    def test_missing_state_and_acceptance_are_errors(self) -> None:
        self.write("- [ ] **MC02-0003** [enhancement] 没有状态也没有验收。\n")
        code, _, err = self.run_main()
        self.assertEqual(code, 1)
        self.assertIn("分诊状态", err)
        self.assertIn("验收", err)

    def test_unknown_state_is_error(self) -> None:
        self.write("- [ ] **MC02-0004** [bug] [done] 状态不合法。验收：无。\n")
        code, _, err = self.run_main()
        self.assertEqual(code, 1)
        self.assertIn("未知状态", err)

    def test_duplicate_id_across_done_items_is_error(self) -> None:
        self.write(
            "- [x] **MC02-0005** [bug] 旧记录。验收：旧。\n"
            "- [ ] **MC02-0005** [bug] [needs-triage] 新记录。验收：新。\n"
        )
        code, _, err = self.run_main()
        self.assertEqual(code, 1)
        self.assertIn("任务编号重复", err)

    def test_long_body_and_build_reference_are_warnings(self) -> None:
        self.write(
            "- [ ] **MC02-0006** [enhancement] [needs-info] "
            + "说明" * 30
            + " 见 build/mc02-0006/report.json。验收：结果。\n"
        )
        code, _, err = self.run_main("--max-body", "40")
        self.assertEqual(code, 0)
        self.assertIn("超过 40", err)
        self.assertIn("build/", err)
        strict_code, _, strict_err = self.run_main("--max-body", "40", "--strict")
        self.assertEqual(strict_code, 1)
        self.assertIn("build/", strict_err)

    def test_summary_counts_states_and_sections(self) -> None:
        self.write(
            "## A 节\n"
            "- [ ] **MC02-0007** [bug] [needs-triage] 甲。验收：甲。\n"
            "- [ ] **MC02-0008** [bug] [ready-for-agent] 乙。验收：乙。\n"
            "## B 节\n"
            "- [x] **MC02-0009** [enhancement] 丙。验收：丙。\n"
            "- [ ] **MC02-0010** [enhancement] [ready-for-agent] 丁。验收：丁。\n"
        )
        code, out, _ = self.run_main("--summary")
        self.assertEqual(code, 0)
        self.assertIn("未完成 3，已完成 1", out)
        self.assertIn("needs-triage 1", out)
        self.assertIn("ready-for-agent 2", out)
        self.assertIn("A 节", out)
        self.assertIn("B 节", out)

    def test_json_summary_is_machine_readable(self) -> None:
        self.write("- [ ] **MC02-0011** [bug] [needs-info] 甲。验收：甲。\n")
        code, out, _ = self.run_main("--json")
        self.assertEqual(code, 0)
        self.assertIn('"open": 1', out)


if __name__ == "__main__":
    unittest.main()
