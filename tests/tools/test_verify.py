"""统一验证入口的命令构造和阶段选择回归。

测试不运行 CMake、CTest 或真实构建，只核对命令参数和 WSL 路径转换。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "workflow"))

import presets
import verify


class VerifyTests(unittest.TestCase):
    def options(self, build_root: Path) -> verify.Options:
        return verify.Options(
            root=ROOT,
            build_root=build_root,
            parallel=8,
            selected=list(presets.OFFICIAL[:2]),
            dry_run=True,
        )

    def test_fast_stages(self) -> None:
        args = verify.parse_args(["--fast"])
        self.assertEqual(verify.selected_stages(args), ["todo", "docs", "tools"])

    def test_stage_union_keeps_order(self) -> None:
        args = verify.parse_args(["--firmware", "--fast"])
        self.assertEqual(verify.selected_stages(args), ["todo", "docs", "tools", "firmware"])

    def test_host_commands_use_external_build_root_and_parallel(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            commands = verify.host_commands(self.options(Path(directory)))
        joined = [" ".join(command) for command in commands]
        self.assertTrue(any("cmake -S" in item and "-B" in item for item in joined))
        self.assertTrue(any("ctest --test-dir" in item and "--parallel 8" in item for item in joined))

    def test_firmware_commands_include_profile_check(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            options = self.options(Path(directory))
            commands = verify.firmware_commands(options)
        joined = [" ".join(command) for command in commands]
        self.assertTrue(any("--preset bench-debug -B" in item for item in joined))
        self.assertTrue(any("check_firmware_profile.py" in item for item in joined))
        self.assertTrue(any("--profile bench_safe" in item for item in joined))

    def test_archive_commands_use_matching_build_dir(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            build_root = Path(directory)
            commands = verify.archive_commands(self.options(build_root))
        joined = [" ".join(command) for command in commands]
        self.assertTrue(any("--preset bench-debug" in item for item in joined))
        self.assertTrue(any(str(build_root / "bench-debug") in item for item in joined))
        self.assertTrue(any("firmware-archives" in item for item in joined))

    def test_wsl_path_conversion(self) -> None:
        self.assertEqual(verify.to_wsl_path(Path(r"D:\repo\sub")), "/mnt/d/repo/sub")
        self.assertEqual(verify.to_wsl_path(Path("/home/user/repo")), "/home/user/repo")

    def test_dry_run_prints_commands_without_executing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            out = io.StringIO()
            with redirect_stdout(out):
                code = verify.main(["--fast", "--dry-run", "--root", str(ROOT),
                                    "--build-root", directory, "--no-wsl"])
        self.assertEqual(code, 0)
        self.assertIn("check_todo.py", out.getvalue())
        self.assertIn("check_docs_links.py", out.getvalue())


if __name__ == "__main__":
    unittest.main()
