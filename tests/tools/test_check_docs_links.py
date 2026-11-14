"""文档相对链接检查的行为回归。

测试在临时目录生成 Markdown 文件，覆盖有效链接、缺失链接、外部网址、锚点、
代码块、百分号编码和 build/ 提醒；不读取仓库的真实文档。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "workflow"))

import check_docs_links


class DocsLinksTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="mc02-doc-links-")
        self.root = Path(self.temporary.name)
        (self.root / "docs" / "sub").mkdir(parents=True)
        (self.root / "README.md").write_text("# root\n", encoding="utf-8")
        (self.root / "docs" / "sub" / "target.md").write_text("# target\n", encoding="utf-8")
        (self.root / "docs" / "sub" / "a b.md").write_text("# space\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write(self, body: str, path: Path | None = None) -> Path:
        target = path or (self.root / "docs" / "page.md")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(body, encoding="utf-8")
        return target

    def test_valid_relative_links_pass(self) -> None:
        self.write(
            "见 [目标](sub/target.md#小节) 和 "
            "[空格](<sub/a b.md>)。\n"
        )
        result = check_docs_links.check(self.root)
        self.assertEqual(result.errors, [])
        self.assertEqual(result.links_checked, 2)

    def test_missing_target_is_error(self) -> None:
        self.write("见 [不存在](sub/missing.md)。\n")
        result = check_docs_links.check(self.root)
        self.assertEqual(len(result.errors), 1)
        self.assertIn("sub/missing.md", result.errors[0])

    def test_external_anchor_and_code_are_skipped(self) -> None:
        self.write(
            "[网页](https://example.com/a)\n"
            "[邮件](mailto:a@example.com)\n"
            "[锚点](#section)\n"
            "`[行内](missing.md)`\n"
            "```\n[fenced](missing.md)\n```\n"
        )
        result = check_docs_links.check(self.root)
        self.assertEqual(result.errors, [])
        self.assertEqual(result.links_checked, 0)

    def test_percent_encoded_path_is_decoded(self) -> None:
        self.write("[空格](sub/a%20b.md)\n")
        result = check_docs_links.check(self.root)
        self.assertEqual(result.errors, [])

    def test_build_reference_is_warning_not_error(self) -> None:
        self.write("[日志](../build/run/report.json)\n")
        result = check_docs_links.check(self.root)
        self.assertEqual(result.errors, [])
        self.assertEqual(len(result.warnings), 1)
        self.assertIn("build/", result.warnings[0])

    def test_outside_repository_is_warning(self) -> None:
        page = self.write("[外部](../../../outside.md)\n", self.root / "docs" / "sub" / "page.md")
        self.assertTrue(page.is_file())
        result = check_docs_links.check(self.root)
        self.assertEqual(result.errors, [])
        self.assertEqual(len(result.warnings), 1)
        self.assertIn("仓库外", result.warnings[0])

    def test_allowlisted_missing_path_is_warning(self) -> None:
        self.write("[模拟器](../tools/referee_sim/TODO.md)\n")
        rules = [{"prefix": "tools/referee_sim/", "reason": "独立仓库"}]
        result = check_docs_links.check(self.root, rules)
        self.assertEqual(result.errors, [])
        self.assertEqual(len(result.warnings), 1)
        self.assertIn("独立仓库", result.warnings[0])

    def test_deny_build_references_returns_failure(self) -> None:
        self.write("[日志](../build/report.json)\n")
        common = ["--root", str(self.root), "--max-print", "0"]
        self.assertEqual(check_docs_links.main(common), 0)
        self.assertEqual(check_docs_links.main(common + ["--deny-build-references"]), 1)


if __name__ == "__main__":
    unittest.main()
