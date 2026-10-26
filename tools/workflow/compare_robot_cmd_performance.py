#!/usr/bin/env python3
"""比较 RobotCMD 生产路径与某个祖先提交中的基线实现。

用法：通过 ``--baseline-ref`` 指定基线提交，可设置配对次数、CPU和JSONL输出路径。
脚本构建两个Linux主机测试目标，按当前目标编译参数编译基线源文件，并保存每次结果。
复制量和100组批均值最大值的五次中位数都下降时返回0；构建、测量或输入校验失败时返回非零。
本工具测量Linux主机表现，不测量STM32H723的最坏执行时间。
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
HOST_SOURCE = ROOT / "tests" / "host"
BUILD_DIR = ROOT / "build" / "wsl" / "host-release"
ROBOT_CMD_SOURCE = ROOT / "application" / "cmd" / "robot_cmd.c"
TARGETS = ("robot_cmd_perf_test", "robot_cmd_time_test")
MEASURED_FIELDS = ("p50_batch_mean_ns", "p95_batch_mean_ns", "max_batch_mean_ns")


def run_checked(arguments: list[str], *, cwd: Path) -> subprocess.CompletedProcess[str]:
    """运行一条构建或Git命令；失败时保留命令错误并停止后续步骤。"""
    return subprocess.run(arguments, cwd=cwd, check=True, text=True, capture_output=True)


def compile_baseline_object(
    target: str, baseline_source: Path, baseline_object: Path
) -> None:
    """读取目标编译参数，并将基线 RobotCMD 源码编译为临时对象文件。"""
    database = json.loads((BUILD_DIR / "compile_commands.json").read_text(encoding="utf-8"))
    source_resolved = ROBOT_CMD_SOURCE.resolve()
    entry = next(
        item
        for item in database
        if Path(item["file"]).resolve() == source_resolved
        and f"{target}.dir" in item.get("command", " ".join(item.get("arguments", [])))
    )
    arguments = entry.get("arguments") or shlex.split(entry["command"])
    for index, argument in enumerate(arguments):
        if argument == "-o":
            arguments[index + 1] = str(baseline_object)
        elif argument == "-MF":
            arguments[index + 1] = str(baseline_object.with_suffix(".d"))
        elif argument == "-MT":
            arguments[index + 1] = str(baseline_object)
        elif argument == str(ROBOT_CMD_SOURCE):
            arguments[index] = str(baseline_source)
    if str(baseline_source) not in arguments:
        source_index = next(
            index
            for index, argument in enumerate(arguments)
            if Path(argument).resolve() == source_resolved
        )
        arguments[source_index] = str(baseline_source)
    subprocess.run(arguments, cwd=entry["directory"], check=True)


def link_baseline_binary(target: str, baseline_object: Path, output: Path) -> None:
    """复用CMake生成的链接命令，只把 RobotCMD 对象换成基线对象。"""
    commands = run_checked(["ninja", "-t", "commands", target], cwd=BUILD_DIR).stdout
    link_line = next(
        line
        for line in commands.splitlines()
        if _has_output(shlex.split(line), target)
    )
    arguments = [item for item in shlex.split(link_line) if item not in (":", "&&")]
    object_index = next(
        index
        for index, item in enumerate(arguments)
        if item.endswith("application/cmd/robot_cmd.c.o")
    )
    arguments[object_index] = str(baseline_object)
    output_index = arguments.index("-o") + 1
    arguments[output_index] = str(output)
    subprocess.run(arguments, cwd=BUILD_DIR, check=True)


def _has_output(arguments: list[str], target: str) -> bool:
    """检查Ninja命令是否生成指定的测试程序。"""
    return any(
        arguments[index] == "-o" and Path(arguments[index + 1]).name == target
        for index in range(len(arguments) - 1)
    )


def run_measurement(binary: Path, cpu: int) -> dict[str, int | str | None]:
    """将测试程序固定到指定CPU运行一次，并解析最后一行JSON结果。"""
    completed = subprocess.run(
        [str(binary)],
        check=True,
        text=True,
        capture_output=True,
        preexec_fn=lambda: os.sched_setaffinity(0, {cpu}),
    )
    return json.loads(completed.stdout.strip().splitlines()[-1])


def median_values(samples: list[dict[str, int | str | None]], field: str) -> int | float:
    """计算多次运行某项结果的中位数。"""
    values = [int(sample[field]) for sample in samples]
    median = statistics.median(values)
    return int(median) if int(median) == median else median


def parse_args() -> argparse.Namespace:
    """解析基线提交、配对次数、输出路径和可选CPU。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-ref", required=True, help="用作基线的祖先提交")
    parser.add_argument("--runs", type=int, default=5, help="每组配对运行次数，最少5次")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/mc02-0028-comparison.jsonl"),
        help="JSONL输出文件；相对路径以仓库根目录为起点",
    )
    parser.add_argument("--cpu", type=int, help="指定一个当前进程允许使用的CPU编号")
    return parser.parse_args()


def main() -> int:
    """检查输入，构建基线与当前程序，运行配对测量并写入JSONL。"""
    args = parse_args()
    if not sys.platform.startswith("linux"):
        print("此测量需要Linux CPU亲和设置和GNU链接器。", file=sys.stderr)
        return 2
    if args.runs < 5:
        print("--runs至少为5，才能比较多次运行的批次最大均值。", file=sys.stderr)
        return 2
    changed_paths = [
        "application/cmd/robot_cmd.c",
        "tests/host/robot_cmd_perf_test.c",
        "tests/host/CMakeLists.txt",
    ]
    if subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", *changed_paths], cwd=ROOT
    ).returncode != 0:
        print("请先提交 RobotCMD 和性能测试源码，再运行对比。", file=sys.stderr)
        return 2

    baseline_sha = run_checked(["git", "rev-parse", args.baseline_ref], cwd=ROOT).stdout.strip()
    current_sha = run_checked(["git", "rev-parse", "HEAD"], cwd=ROOT).stdout.strip()
    merge_base = run_checked(["git", "merge-base", baseline_sha, current_sha], cwd=ROOT).stdout.strip()
    if merge_base != baseline_sha:
        print("The baseline reference must be an ancestor of HEAD.", file=sys.stderr)
        return 2

    available_cpus = sorted(os.sched_getaffinity(0))
    cpu = available_cpus[0] if args.cpu is None else args.cpu
    if cpu not in available_cpus:
        print(f"CPU {cpu} 不在当前进程允许使用的列表 {available_cpus} 中。", file=sys.stderr)
        return 2

    output_path = args.output if args.output.is_absolute() else ROOT / args.output
    output_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        run_checked(["cmake", "--preset", "host-release"], cwd=HOST_SOURCE)
        run_checked(
            ["cmake", "--build", "--preset", "host-release", "--target", *TARGETS, "-j4"],
            cwd=HOST_SOURCE,
        )
        baseline_text = run_checked(
            ["git", "show", f"{baseline_sha}:application/cmd/robot_cmd.c"], cwd=ROOT
        ).stdout
        with tempfile.TemporaryDirectory(prefix="mc02-0028-baseline-") as temporary:
            temporary_path = Path(temporary)
            baseline_source = temporary_path / "robot_cmd_baseline.c"
            baseline_source.write_text(baseline_text, encoding="utf-8")
            baseline_binaries: dict[str, Path] = {}
            for target in TARGETS:
                baseline_object = temporary_path / f"{target}.robot_cmd.o"
                baseline_binary = temporary_path / target
                compile_baseline_object(target, baseline_source, baseline_object)
                link_baseline_binary(target, baseline_object, baseline_binary)
                baseline_binaries[target] = baseline_binary

            current_binaries = {target: BUILD_DIR / target for target in TARGETS}
            samples: dict[str, dict[str, list[dict[str, int | str | None]]]] = {
                target: {"baseline": [], "current": []} for target in TARGETS
            }
            with output_path.open("w", encoding="utf-8", newline="\n") as stream:
                metadata = {
                    "kind": "metadata",
                    "baseline_sha": baseline_sha,
                    "current_sha": current_sha,
                    "runs": args.runs,
                    "cpu": cpu,
                    "compiler": run_checked(["cc", "--version"], cwd=ROOT).stdout.splitlines()[0],
                    "batches": 100,
                    "cycles_per_batch": 50000,
                }
                stream.write(json.dumps(metadata, sort_keys=True) + "\n")
                stream.flush()
                for target in TARGETS:
                    for run_number in range(1, args.runs + 1):
                        for version, binary in (
                            ("baseline", baseline_binaries[target]),
                            ("current", current_binaries[target]),
                        ):
                            result = run_measurement(binary, cpu)
                            sample = {
                                "kind": "sample",
                                "target": target,
                                "version": version,
                                "run": run_number,
                                "result": result,
                            }
                            samples[target][version].append(result)
                            stream.write(json.dumps(sample, sort_keys=True) + "\n")
                            stream.flush()

            summaries = {}
            copies_reduced = False
            worst_batch_mean_reduced = False
            for target in TARGETS:
                baseline_samples = samples[target]["baseline"]
                current_samples = samples[target]["current"]
                if target == "robot_cmd_perf_test":
                    baseline_copy = median_values(baseline_samples, "copy_bytes_per_cycle")
                    current_copy = median_values(current_samples, "copy_bytes_per_cycle")
                    copies_reduced = current_copy < baseline_copy
                    summaries["copies"] = {
                        "baseline_bytes_per_cycle": baseline_copy,
                        "current_bytes_per_cycle": current_copy,
                        "improved": copies_reduced,
                    }
                else:
                    baseline_metrics = {
                        field: median_values(baseline_samples, field) for field in MEASURED_FIELDS
                    }
                    current_metrics = {
                        field: median_values(current_samples, field) for field in MEASURED_FIELDS
                    }
                    worst_batch_mean_reduced = (
                        current_metrics["max_batch_mean_ns"] < baseline_metrics["max_batch_mean_ns"]
                    )
                    summaries["thread_cpu_ns_per_batch_mean"] = {
                        "baseline": baseline_metrics,
                        "current": current_metrics,
                        "worst_sampled_batch_mean_improved": worst_batch_mean_reduced,
                    }

        print(json.dumps({"output": str(output_path), **summaries}, indent=2, sort_keys=True))
        return 0 if copies_reduced and worst_batch_mean_reduced else 1
    except (OSError, KeyError, StopIteration, subprocess.CalledProcessError, json.JSONDecodeError) as error:
        print(f"RobotCMD对比失败：{error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
