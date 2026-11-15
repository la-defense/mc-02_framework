"""统一验证入口的命令构造和阶段选择回归。

测试不运行 CMake、CTest 或真实构建，只核对命令参数和 WSL 路径转换。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import io
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "workflow"))

import presets
import verify


class RecordingRunner:
    """记录调用参数，并按预设返回成功或超时，用来在不启动进程的情况下检查调用方式。"""

    def __init__(self, *, returncode: int = 0, timeout: bool = False) -> None:
        self.returncode = returncode
        self.timeout = timeout
        self.calls: list[tuple[list[str], dict]] = []

    def __call__(self, command: list[str], **kwargs):
        self.calls.append((command, kwargs))
        if self.timeout:
            raise subprocess.TimeoutExpired(command, kwargs.get("timeout", 0))
        return subprocess.CompletedProcess(command, self.returncode)


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

    def test_timeout_flags_have_defaults(self) -> None:
        args = verify.parse_args(["--fast"])
        self.assertEqual(args.command_timeout, verify.DEFAULT_COMMAND_TIMEOUT)
        self.assertEqual(args.wsl_timeout, verify.DEFAULT_WSL_TIMEOUT)

    def test_run_commands_passes_timeout_and_raises_on_expiry(self) -> None:
        runner = RecordingRunner(timeout=True)
        with self.assertRaises(RuntimeError) as caught:
            verify.run_commands([["cmake", "--build", "x"]], ROOT, False, 5, runner)
        self.assertEqual(runner.calls[0][1]["timeout"], 5)
        self.assertIn("超过 5 秒", str(caught.exception))
        self.assertIn("cmake --build x", str(caught.exception))

    def test_probe_wsl_accepts_reachable_session(self) -> None:
        runner = RecordingRunner(returncode=0)
        self.assertTrue(verify.probe_wsl("wsl.exe", "/mnt/d/repo", 5, runner=runner))
        self.assertIn("/bin/true", runner.calls[0][0])
        self.assertEqual(runner.calls[0][1]["timeout"], 5)

    def test_probe_wsl_rejects_unreachable_session(self) -> None:
        # 会话僵死时探测不会返回，只能靠超时判定为不可用。
        runner = RecordingRunner(timeout=True)
        self.assertFalse(verify.probe_wsl("wsl.exe", "/mnt/d/repo", 5, runner=runner))

    def test_probe_wsl_rejects_failed_probe(self) -> None:
        runner = RecordingRunner(returncode=1)
        self.assertFalse(verify.probe_wsl("wsl.exe", "/mnt/d/repo", 5, runner=runner))


if __name__ == "__main__":
    unittest.main()
