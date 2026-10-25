"""测试构建证据解析器的输入边界，不启动编译器或修改固件。"""

from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools/workflow/capture_build_baseline.py"
SPEC = importlib.util.spec_from_file_location("capture_build_baseline", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
BASELINE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASELINE)


class BuildBaselineParsingTests(unittest.TestCase):
    """确认构建缓存和 Ninja 依赖输出只记录可复核内容。"""

    def test_cache_reader_ignores_comments_and_keeps_equals_in_values(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            cache = Path(temporary) / "CMakeCache.txt"
            cache.write_text(
                "// cache comment\nCMAKE_GENERATOR:INTERNAL=Ninja\n"
                "FLAG:STRING=-DNAME=a=b\n",
                encoding="utf-8",
            )
            self.assertEqual(
                BASELINE.parse_cache(cache),
                {"CMAKE_GENERATOR": "Ninja", "FLAG": "-DNAME=a=b"},
            )

    def test_ninja_parser_collects_only_indented_absolute_dependencies(self) -> None:
        output = (
            "CMakeFiles/a.c.obj: #deps 2, deps mtime 0 (VALID)\n"
            "    /workspace/src/a.c\n"
            "    /workspace/include/a.h\n"
            "CMakeFiles/b.c.obj: #deps 1, deps mtime 0 (VALID)\n"
            "    relative/header.h\n"
        )
        self.assertEqual(
            BASELINE.parse_ninja_dependencies(output),
            {"/workspace/src/a.c", "/workspace/include/a.h"},
        )

    def test_source_commit_requires_full_hexadecimal_identifier(self) -> None:
        BASELINE.validate_source_commit("a" * 40)
        BASELINE.validate_source_commit("b" * 64)
        with self.assertRaises(BASELINE.BaselineError):
            BASELINE.validate_source_commit("abc123")

    def test_matrix_keeps_each_configuration_dependency_fingerprint(self) -> None:
        def report(configuration: str, path: str, digest: str) -> dict[str, object]:
            return {
                "source_commit": "abc123",
                "preset_configuration": {"name": configuration},
                "toolchain": {"cmake": "3.22.1"},
                "artifacts": {"elf": {"file": f"{configuration}.elf"}},
                "size_bytes": {"text": 10},
                "third_party_packages": [
                    {
                        "name": "sample",
                        "version": "1.0",
                        "version_evidence": "test fixture",
                        "source": "https://example.invalid/sample",
                        "license": None,
                        "file_count": 1,
                        "sha256": digest,
                        "files": [{"path": path, "sha256": digest}],
                    }
                ],
                "first_party_integration_files": [],
            }

        matrix = BASELINE.assemble_matrix(
            [report("debug", "third_party/a.c", "aaa"), report("release", "third_party/b.c", "bbb")]
        )
        self.assertEqual(len(matrix["configurations"]), 2)
        fingerprints = [
            configuration["third_party_fingerprints"]["sample"]["sha256"]
            for configuration in matrix["configurations"]
        ]
        self.assertEqual(fingerprints, ["aaa", "bbb"])
        self.assertEqual(matrix["third_party_packages"][0]["union_file_count"], 2)

    def test_matrix_rejects_different_source_commits(self) -> None:
        first = {"source_commit": "a" * 40}
        second = {"source_commit": "b" * 40}
        with self.assertRaises(BASELINE.BaselineError):
            BASELINE.assemble_matrix([first, second])


if __name__ == "__main__":
    unittest.main()
