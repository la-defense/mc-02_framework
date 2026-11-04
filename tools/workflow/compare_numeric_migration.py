#!/usr/bin/env python3
"""在两个源码版本上使用同一主机编译器构建并比较有限数值轨迹。"""

import argparse
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile


def run(command: list[str]) -> subprocess.CompletedProcess[bytes]:
    """执行参数列表；失败时输出工具诊断并保留非零结果。"""
    result = subprocess.run(command, capture_output=True, check=False)
    if result.returncode:
        sys.stderr.buffer.write(result.stdout)
        sys.stderr.buffer.write(result.stderr)
        raise RuntimeError(f"命令失败：{command[0]}，退出码 {result.returncode}")
    return result


def build_trace(root: Path, build: Path, trace: Path, compiler: str, old: bool) -> Path:
    """在独立目录编译指定版本的真实算法及其同源 DSP。"""
    run(["cmake", "-S", str(root / "tests/host"), "-B", str(build), "-G", "Ninja",
         "-DCMAKE_BUILD_TYPE=Debug", f"-DCMAKE_C_COMPILER={compiler}"])
    run(["cmake", "--build", str(build), "--target", "mc02_kf", "-j2"])
    executable = build / ("trace.exe" if sys.platform == "win32" else "trace")
    algorithm = root / "modules/algorithm"
    dsp = root / "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Include"
    fusion = root / "Middlewares/Third_Party/Fusion"
    command = [compiler, "-std=c11", "-O0", "-Wall", "-Wextra", "-Werror", "-fno-fast-math",
               "-D__GNUC_PYTHON__", "-DMC02_ATTITUDE_FILTER_ENABLE_BOTH_BACKENDS=1"]
    if old:
        command.append("-DOLD_VERSION")  # 旧版矩阵接口直接使用 CMSIS 类型。
    command.extend([f"-I{algorithm}", f"-I{dsp}", f"-I{fusion}", str(trace),
                    str(algorithm / "RLS.c"), str(algorithm / "attitude_filter.c"),
                    str(algorithm / "QuaternionEKF.c"), str(fusion / "FusionAhrs.c"),
                    str(build / "libmc02_kf.a"), str(build / "libmc02_dsp_host.a"),
                    "-lm", "-o", str(executable)])
    run(command)
    return executable


def main() -> int:
    """两个源码目录必须分别指向迁移前和当前版本；输出首个差异。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path, help="迁移前源码目录")
    parser.add_argument("--current", default=Path.cwd(), type=Path, help="当前源码目录")
    parser.add_argument("--cc", default="cc", help="两个版本共同使用的主机 C 编译器")
    args = parser.parse_args()
    baseline, current = args.baseline.resolve(), args.current.resolve()
    trace = current / "tools/workflow/numeric_migration_trace.c"
    if baseline == current or not trace.is_file():
        parser.error("两份源码目录须不同，且当前目录须包含数值轨迹工具")
    try:
        with tempfile.TemporaryDirectory(prefix="mc02-numeric-compare-") as directory:
            output = Path(directory)
            old = build_trace(baseline, output / "baseline", trace, args.cc, True)
            new = build_trace(current, output / "current", trace, args.cc, False)
            old_output = run([str(old)]).stdout
            new_output = run([str(new)]).stdout
            if old_output != new_output:
                old_lines, new_lines = old_output.splitlines(), new_output.splitlines()
                for index in range(max(len(old_lines), len(new_lines))):
                    before = old_lines[index] if index < len(old_lines) else b"<missing>"
                    after = new_lines[index] if index < len(new_lines) else b"<missing>"
                    if before != after:
                        print(f"第 {index + 1} 行不同：{before!r} / {after!r}", file=sys.stderr)
                        break
                return 1
            print(f"PASS: {len(old_output.splitlines())} 行一致；SHA-256 {hashlib.sha256(old_output).hexdigest()}")
            return 0
    except (OSError, RuntimeError) as error:
        print(f"数值对照失败：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
