#!/usr/bin/env python3
"""用一个入口运行仓库的本地验证阶段。

阶段与 CI 使用同一份预设清单和脚本：
- todo：TODO 契约与摘要；
- docs：Markdown 相对链接；
- tools：`tests/tools` 下的 Python 工具回归；
- host：主机 Debug／Release 的 CMake 构建和 CTest；
- firmware：当前允许生成的裸板 ARM 预设构建与源码隔离检查；
- archive：为已构建预设生成可校验 ZIP。

在 Windows 上默认转交 WSL 执行，因为固件验收使用 WSL 工具链。构建目录可用
`--build-root` 指到 WSL 的 ext4 路径（例如 `~/mc02-build`），避免在 `/mnt/d`
上反复读写。`--dry-run` 只打印命令，适合核对本机环境。

每一步都带时限，不让一次卡顿吞掉整轮验证：转交前先跑一条 `--exec /bin/true`
探测会话，探不通就直接报错退出；单条命令超过 `--command-timeout` 秒、整次转交
超过 `--wsl-timeout` 秒都会停下。WSL 的会话层偶尔会整体僵死——虚拟机空转，
任何进入发行版的命令都不返回——这时 `wsl --shutdown` 之后重跑即可。

用法：
    python3 tools/workflow/verify.py --fast
    python3 tools/workflow/verify.py --host
    python3 tools/workflow/verify.py --firmware --presets bench-debug,rcdiag-debug
    python3 tools/workflow/verify.py --all --build-root ~/mc02-build

退出状态：任一阶段失败返回 1；参数错误返回 2。
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import presets

ROOT = Path(__file__).resolve().parents[2]
STAGES = ("todo", "docs", "tools", "host", "firmware", "archive")

# 单条命令和整次 WSL 转交的默认上限，按这台机器跑 `--all` 的实际耗时留够余量。
DEFAULT_COMMAND_TIMEOUT = 1800.0
DEFAULT_WSL_TIMEOUT = 7200.0
# 会话探测的等待时间；虚拟机可能要从关闭状态冷启动，比正常进一次宽一些。
PROBE_TIMEOUT = 60.0


@dataclass
class Options:
    """一次验证运行需要的路径、并行度与单条命令时限。"""

    root: Path
    build_root: Path
    parallel: int
    selected: list[presets.Preset]
    dry_run: bool
    command_timeout: float = DEFAULT_COMMAND_TIMEOUT


def tools_dir(root: Path) -> Path:
    """返回工作流脚本目录。"""
    return root / "tools" / "workflow"


def python_command(root: Path, script: str, *arguments: str) -> list[str]:
    """用当前解释器调用 `tools/workflow` 下的脚本。"""
    return [sys.executable, str(tools_dir(root) / script), *arguments]


def todo_commands(options: Options) -> list[list[str]]:
    """TODO 契约检查并输出分诊摘要。"""
    return [python_command(options.root, "check_todo.py", str(options.root / ".Doc" / "TODO.md"),
                           "--summary")]


def docs_commands(options: Options) -> list[list[str]]:
    """检查文档相对链接。"""
    return [python_command(options.root, "check_docs_links.py", "--root", str(options.root))]


def tool_commands(options: Options) -> list[list[str]]:
    """运行 Python 工具回归。"""
    return [[sys.executable, "-m", "unittest", "discover",
             "-s", str(options.root / "tests" / "tools"), "-p", "test_*.py"]]


def host_commands(options: Options) -> list[list[str]]:
    """构建并运行主机 Debug／Release 测试。"""
    commands: list[list[str]] = []
    for name, build_type in (("host-debug", "Debug"), ("host-release", "Release")):
        build_dir = options.build_root / name
        commands.append([
            "cmake", "-S", str(options.root / "tests" / "host"),
            "-B", str(build_dir), "-G", "Ninja",
            f"-DCMAKE_BUILD_TYPE={build_type}",
        ])
        commands.append(["cmake", "--build", str(build_dir),
                         "--parallel", str(options.parallel)])
        commands.append(["ctest", "--test-dir", str(build_dir),
                         "--output-on-failure", "--parallel", str(options.parallel)])
    return commands


def firmware_commands(options: Options) -> list[list[str]]:
    """构建选中的 ARM 预设并检查源码隔离。"""
    commands: list[list[str]] = [python_command(
        options.root, "check_production_rejection.py", "--build-root", str(options.build_root)
    )]
    for preset in options.selected:
        build_dir = options.build_root / preset.name
        elf = build_dir / presets.artifact_name(preset)
        commands.append(["cmake", "--preset", preset.name, "-B", str(build_dir)])
        commands.append(["cmake", "--build", str(build_dir),
                         "--parallel", str(options.parallel)])
        commands.append(python_command(
            options.root, "check_firmware_profile.py",
            *presets.profile_arguments(preset, build_dir, elf)
        ))
    return commands


def archive_commands(options: Options) -> list[list[str]]:
    """为选中的预设生成归档 ZIP。"""
    output_dir = options.root / "build" / "firmware-archives"
    commands: list[list[str]] = []
    for preset in options.selected:
        build_dir = options.build_root / preset.name
        commands.append(python_command(
            options.root, "archive_firmware.py",
            "--preset", preset.name,
            "--build-dir", str(build_dir),
            "--output-dir", str(output_dir),
        ))
    return commands


def stage_commands(stage: str, options: Options) -> list[list[str]]:
    """返回某个验证阶段要顺序执行的命令。"""
    builders = {
        "todo": todo_commands,
        "docs": docs_commands,
        "tools": tool_commands,
        "host": host_commands,
        "firmware": firmware_commands,
        "archive": archive_commands,
    }
    if stage not in builders:
        raise ValueError(f"未知阶段：{stage}")
    return builders[stage](options)


def selected_stages(args: argparse.Namespace) -> list[str]:
    """按命令行开关确定要运行的阶段，保持固定顺序。"""
    chosen = {
        "todo": args.fast or args.all,
        "docs": args.fast or args.all,
        "tools": args.fast or args.all,
        "host": args.host or args.all,
        "firmware": args.firmware or args.all,
        "archive": args.archive or args.all,
    }
    stages = [stage for stage in STAGES if chosen[stage]]
    return stages or ["todo", "docs", "tools"]


def to_wsl_path(path: Path) -> str:
    """把 `D:\\repo` 形式转换成 WSL 的 `/mnt/d/repo`。"""
    text = str(path)
    match = re.match(r"^([A-Za-z]):[\\/](.*)$", text)
    if match:
        rest = match.group(2).replace("\\", "/")
        return "/mnt/" + match.group(1).lower() + "/" + rest
    return text.replace("\\", "/")


def probe_wsl(wsl: str, root: str, timeout: float = PROBE_TIMEOUT,
              runner=subprocess.run) -> bool:
    """用一条不依赖工程的命令确认 WSL 会话能建起来。

    会话层僵死时，任何进入发行版的命令都不会返回，而且不会自己超时。先探
    一次，可以在探测器给的时限内失败，不必等整轮验证一起挂住。
    """
    command = [wsl, "--cd", root, "--exec", "/bin/true"]
    try:
        completed = runner(command, timeout=timeout, check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (subprocess.TimeoutExpired, OSError):
        return False
    return completed.returncode == 0


def forward_to_wsl(command: list[str], timeout: float = DEFAULT_WSL_TIMEOUT) -> int:
    """把整次验证转交给 WSL，并给这次转交加上限。"""
    try:
        return subprocess.call(command, timeout=timeout)
    except subprocess.TimeoutExpired:
        # call() 只终止 wsl.exe 本身，guest 里的命令可能还在跑。
        print(f"\n转交 WSL 超过 {timeout:.0f} 秒仍未结束，已停止等待。", file=sys.stderr)
        print("虚拟机里可能还有命令在运行；确认不需要后执行 `wsl --shutdown` 收尾。",
              file=sys.stderr)
        return 1


def run_commands(commands: list[list[str]], root: Path, dry_run: bool,
                 command_timeout: float = DEFAULT_COMMAND_TIMEOUT,
                 runner=subprocess.run) -> None:
    """顺序执行命令；打印每条命令，失败或超时抛出带命令的异常。"""
    for command in commands:
        print("$ " + " ".join(command), flush=True)
        if dry_run:
            continue
        try:
            completed = runner(command, cwd=root, timeout=command_timeout)
        except subprocess.TimeoutExpired as error:
            # 卡住的命令不会自己退出，继续等下去整轮验证都拿不到结果。
            raise RuntimeError(
                f"命令超过 {command_timeout:.0f} 秒未结束：{' '.join(command)}"
            ) from error
        if completed.returncode != 0:
            raise RuntimeError(f"命令失败（退出码 {completed.returncode}）：{' '.join(command)}")


def run_stage(stage: str, options: Options) -> float:
    """执行一个阶段并返回耗时秒数。"""
    started = time.monotonic()
    print(f"\n[{stage}] 开始", flush=True)
    run_commands(stage_commands(stage, options), options.root, options.dry_run,
                 options.command_timeout)
    elapsed = time.monotonic() - started
    print(f"[{stage}] 通过，用时 {elapsed:.1f}s", flush=True)
    return elapsed


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MC-02 本地验证入口")
    parser.add_argument("--fast", action="store_true", help="TODO、文档链接和工具回归")
    parser.add_argument("--host", action="store_true", help="主机 Debug／Release 构建与测试")
    parser.add_argument("--firmware", action="store_true", help="ARM 预设构建与源码隔离检查")
    parser.add_argument("--archive", action="store_true", help="为选中的预设生成归档")
    parser.add_argument("--all", action="store_true", help="运行全部阶段")
    parser.add_argument("--presets", help="逗号分隔的预设名，默认四种可构建的裸板配置")
    parser.add_argument("--build-root", type=Path, default=ROOT / "build" / "wsl",
                        help="构建目录根；建议放在 WSL ext4，例如 ~/mc02-build")
    parser.add_argument("--parallel", type=int, default=os.cpu_count() or 2,
                        help="CMake 和 CTest 的并行任务数")
    parser.add_argument("--dry-run", action="store_true", help="只打印命令")
    parser.add_argument("--command-timeout", type=float, default=DEFAULT_COMMAND_TIMEOUT,
                        help=f"单条命令的时限，单位秒，默认 {DEFAULT_COMMAND_TIMEOUT:.0f}")
    parser.add_argument("--wsl-timeout", type=float, default=DEFAULT_WSL_TIMEOUT,
                        help=f"整次转交 WSL 的时限，单位秒，默认 {DEFAULT_WSL_TIMEOUT:.0f}")
    parser.add_argument("--list", action="store_true", help="列出阶段和预设")
    parser.add_argument("--root", type=Path, default=ROOT, help=argparse.SUPPRESS)
    parser.add_argument("--no-wsl", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if os.name == "nt" and not args.no_wsl and not args.list:
        wsl = shutil.which("wsl.exe") or shutil.which("wsl")
        if not wsl:
            print("未找到 wsl.exe；请在 WSL 中直接运行本脚本。", file=sys.stderr)
            return 2
        forwarded = list(argv if argv is not None else sys.argv[1:])
        for index, item in enumerate(forwarded):
            if item in ("--build-root", "--root") and index + 1 < len(forwarded):
                forwarded[index + 1] = to_wsl_path(Path(forwarded[index + 1]))
        command = [wsl, "--cd", to_wsl_path(args.root), "--exec", "python3",
                   to_wsl_path(Path(__file__).resolve()), "--no-wsl", *forwarded]
        # 先花几十秒确认会话进得去：僵死的虚拟机会把后面的等待全部吞掉。
        if not probe_wsl(wsl, to_wsl_path(args.root)):
            print("WSL 会话在探测时限内没有响应，验证没有开始。", file=sys.stderr)
            print("这通常是会话层僵死：先执行 `wsl --shutdown`，再重跑本命令。",
                  file=sys.stderr)
            return 1
        print("$ " + " ".join(command), flush=True)
        return forward_to_wsl(command, args.wsl_timeout)

    if args.list:
        print("阶段：" + "、".join(STAGES))
        for preset in presets.OFFICIAL:
            print(f"  {preset.name:<22} {preset.profile:<10} {preset.role:<8} "
                  f"{preset.build_type:<15} {presets.artifact_name(preset)}")
        return 0

    try:
        chosen = presets.selected(args.presets)
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 2
    options = Options(
        root=args.root.resolve(),
        build_root=Path(args.build_root).expanduser(),
        parallel=max(1, args.parallel),
        selected=chosen,
        dry_run=args.dry_run,
        command_timeout=args.command_timeout,
    )
    stages = selected_stages(args)
    timings: list[tuple[str, float]] = []
    try:
        for stage in stages:
            timings.append((stage, run_stage(stage, options)))
    except RuntimeError as error:
        print(f"\n验证失败：{error}", file=sys.stderr)
        print("已用时间：" + "，".join(f"{name} {seconds:.1f}s" for name, seconds in timings),
              file=sys.stderr)
        return 1
    print("\n完成：" + "，".join(f"{name} {seconds:.1f}s" for name, seconds in timings))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
