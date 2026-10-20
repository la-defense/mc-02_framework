#!/usr/bin/env python3
"""在隔离副本检查增删源码后的CMake增量构建；只复制已跟踪输入，不接触板卡。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tempfile
import time


ROOT_FILES = {
    "CMakeLists.txt",
    # 配置阶段的格式正反例读取这两份文件，隔离副本也必须提供。
    "Makefile",
    "tools/workflow/check_format_diagnostics.py",
    "STM32H723VGTx_FLASH.ld",
    "startup_stm32h723xx.s",
}
SOURCE_PREFIXES = (
    "tests/host/log_fakes/",
    "Core/",
    "USB_DEVICE/",
    "Drivers/STM32H7xx_HAL_Driver/",
    "Drivers/CMSIS/Device/",
    "Drivers/CMSIS/Include/",
    "cmake/",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/",
    "Middlewares/Third_Party/FreeRTOS/Source/",
    "Middlewares/Third_Party/SEGGER/RTT/",
    "Middlewares/Third_Party/SEGGER/Config/",
    "Middlewares/ST/",
    "bsp/",
    "modules/",
    "application/",
)
TARGET = "Basic_Framework_MC02_bench_safe.elf"


class CheckFailure(RuntimeError):
    pass


class Checker:
    def __init__(self, root: Path, work_dir: Path, args: argparse.Namespace) -> None:
        self.root = root.resolve()
        self.work_dir = work_dir.resolve()
        self.args = args
        self.source = self.work_dir / "source"
        self.incremental = self.work_dir / "incremental"
        self.logs = self.work_dir / "logs"
        self.logs.mkdir(parents=True, exist_ok=True)
        self.step = 0

    def run(
        self,
        label: str,
        command: list[str],
        *,
        cwd: Path | None = None,
        save_output: bool = True,
    ) -> str:
        self.step += 1
        log_path = self.logs / f"{self.step:02d}-{label}.log"
        try:
            result = subprocess.run(
                command,
                cwd=cwd,
                text=True,
                encoding="utf-8",
                errors="replace",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=self.args.command_timeout,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            partial = exc.stdout or ""
            if isinstance(partial, bytes):
                partial = partial.decode("utf-8", errors="replace")
            log_path.write_text(
                f"$ {' '.join(command)}\n[TIMEOUT after {self.args.command_timeout}s]\n{partial}",
                encoding="utf-8",
            )
            raise CheckFailure(f"{label} timed out; see {log_path}") from exc
        log_output = result.stdout if save_output else "[command output summarized by the checker]\n"
        log_path.write_text(f"$ {' '.join(command)}\n[exit={result.returncode}]\n{log_output}", encoding="utf-8")
        if result.returncode:
            tail = "\n".join(result.stdout.splitlines()[-35:])
            raise CheckFailure(f"{label} failed; see {log_path}\n{tail}")
        return result.stdout

    def tracked_source_files(self) -> list[str]:
        result = subprocess.run(
            ["git", "-C", str(self.root), "ls-files", "-z"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=30,
        )
        if result.returncode:
            raise CheckFailure(f"git ls-files failed: {result.stderr.decode(errors='replace')}")

        paths = []
        for raw_path in result.stdout.split(b"\0"):
            if not raw_path:
                continue
            relative = raw_path.decode("utf-8", errors="surrogateescape").replace("\\", "/")
            relevant = relative in ROOT_FILES or relative.startswith(SOURCE_PREFIXES)
            source_path = self.root / Path(*PurePosixPath(relative).parts)
            if relevant and source_path.is_file():
                paths.append(relative)
        return sorted(paths)

    def create_snapshot(self) -> None:
        self.source.mkdir(parents=True)
        files = self.tracked_source_files()
        if not files:
            raise CheckFailure("no tracked source files were selected for the fixture")
        for relative in files:
            source_path = self.root / Path(*PurePosixPath(relative).parts)
            destination = self.source / Path(*PurePosixPath(relative).parts)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_path, destination)
        print(f"Copied {len(files)} tracked build inputs to {self.source}", flush=True)

    def configure(self, build_dir: Path, label: str) -> None:
        command = [
            self.args.cmake,
            "-S", str(self.source),
            "-B", str(build_dir),
            "-G", self.args.generator,
            f"-DCMAKE_BUILD_TYPE={self.args.build_type}",
            "-DMC02_PROFILE=bench_safe",
            "-DMC02_ENABLE_HEATER=OFF",
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        ]
        self.run(label, command)

    @staticmethod
    def compile_metadata(build_dir: Path, source_dir: Path) -> dict[str, set[str]]:
        database = build_dir / "compile_commands.json"
        if not database.is_file():
            raise CheckFailure(f"compile database is missing: {database}")
        root = source_dir.resolve()
        result: dict[str, set[str]] = {}
        for entry in json.loads(database.read_text(encoding="utf-8")):
            source_path = Path(entry["file"]).resolve()
            try:
                relative = source_path.relative_to(root).as_posix()
            except ValueError:
                continue
            key = relative.casefold() if os.name == "nt" else relative
            command = entry.get("command", "")
            include_dirs = set()
            for match in re.finditer(r'(?:^|\s)-I(?:\s+)?(?:"([^"]+)"|([^\s]+))', command):
                include_dir = Path(match.group(1) or match.group(2))
                if not include_dir.is_absolute():
                    include_dir = Path(entry.get("directory", build_dir)) / include_dir
                normalized = include_dir.resolve().as_posix()
                include_dirs.add(normalized.casefold() if os.name == "nt" else normalized)
            result[key] = include_dirs
        return result

    def assert_link_excludes(self, removed_sources: list[str], label: str) -> None:
        if not removed_sources:
            return
        output = self.run(
            f"{label}-ninja-link-inputs",
            [self.args.ninja, "-C", str(self.incremental), "-t", "commands", TARGET],
            save_output=False,
        )
        normalized = output.replace("\\", "/")
        link_lines = [
            line for line in normalized.splitlines()
            if re.search(r"\s-o\s+[^\s]*" + re.escape(TARGET) + r"(?:\s|$)", line)
        ]
        if not link_lines:
            raise CheckFailure(f"could not find the linker command for {TARGET}")
        link_log = self.logs / f"{self.step:02d}-{label}-link-command.log"
        link_log.write_text("\n".join(link_lines) + "\n", encoding="utf-8")
        link_command = "\n".join(link_lines)
        for source in removed_sources:
            relative = source.casefold() if os.name == "nt" else source
            object_suffix = relative + ".obj"
            if object_suffix.casefold() in link_command.casefold():
                raise CheckFailure(f"removed source remains in the link inputs: {source}")

    def stage(
        self,
        label: str,
        *,
        added: tuple[str, ...] = (),
        removed: tuple[str, ...] = (),
        excluded: tuple[str, ...] = (),
    ) -> None:
        print(f"[{label}] build existing directory, then compare against a fresh configure", flush=True)
        self.run(
            f"{label}-incremental-build",
            [self.args.cmake, "--build", str(self.incremental), "--parallel", str(self.args.jobs)],
        )

        current = self.compile_metadata(self.incremental, self.source)
        clean_dir = self.work_dir / f"clean-{self.step + 1:02d}-{label}"
        self.configure(clean_dir, f"{label}-clean-configure")
        clean = self.compile_metadata(clean_dir, self.source)

        added_keys = {path.casefold() if os.name == "nt" else path for path in added}
        removed_keys = {path.casefold() if os.name == "nt" else path for path in removed}
        excluded_keys = {path.casefold() if os.name == "nt" else path for path in excluded}
        current_sources = set(current)
        clean_sources = set(clean)
        missing = (clean_sources - current_sources) | (added_keys - current_sources)
        unexpected = (current_sources - clean_sources) | (removed_keys & current_sources) | (excluded_keys & current_sources)
        include_mismatches = {
            source: {
                "missing": sorted(clean[source] - current[source]),
                "unexpected": sorted(current[source] - clean[source]),
            }
            for source in current_sources & clean_sources
            if current[source] != clean[source]
        }
        if missing or unexpected or include_mismatches:
            raise CheckFailure(
                f"{label}: incremental source set differs from a clean configure; "
                f"missing={sorted(missing)}, unexpected={sorted(unexpected)}, "
                f"include_directories={include_mismatches}"
            )
        self.assert_link_excludes(list(removed), label)
        print(f"[{label}] PASS: {len(current_sources)} source paths and include directories match a clean configure", flush=True)

    def write_source(self, relative: str, contents: str) -> None:
        path = self.source / Path(*PurePosixPath(relative).parts)
        if path.exists():
            raise CheckFailure(f"fixture path already exists: {relative}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")

    @staticmethod
    def unique_source(relative: str) -> str:
        symbol = re.sub(r"[^A-Za-z0-9_]", "_", relative)
        return f"void mc02_cmake_regen_{symbol}(void) {{}}\n"

    def remove_file(self, relative: str) -> None:
        path = self.source / Path(*PurePosixPath(relative).parts)
        path.unlink()
        parent = path.parent
        while parent != self.source and parent.is_dir() and not any(parent.iterdir()):
            parent.rmdir()
            parent = parent.parent

    def move_file(self, source: str, destination: str) -> None:
        old = self.source / Path(*PurePosixPath(source).parts)
        new = self.source / Path(*PurePosixPath(destination).parts)
        if new.exists():
            raise CheckFailure(f"fixture destination already exists: {destination}")
        new.parent.mkdir(parents=True, exist_ok=True)
        old.replace(new)
        parent = old.parent
        while parent != self.source and parent.is_dir() and not any(parent.iterdir()):
            parent.rmdir()
            parent = parent.parent

    def run_all(self) -> None:
        self.create_snapshot()
        self.configure(self.incremental, "initial-configure")
        self.run(
            "initial-build",
            [self.args.cmake, "--build", str(self.incremental), "--parallel", str(self.args.jobs)],
        )
        self.stage("baseline")

        bsp = "bsp/mc02_cmake_regen_probe.c"
        module = "modules/mc02_cmake_regen_probe.c"
        application = "application/mc02_cmake_regen_probe.c"
        for name, path in (("bsp", bsp), ("modules", module), ("application", application)):
            self.write_source(path, self.unique_source(path))
            self.stage(f"add-{name}", added=(path,))

        for name, path in (("application", application), ("modules", module), ("bsp", bsp)):
            self.remove_file(path)
            self.stage(f"remove-{name}", removed=(path,))

        nested = "modules/mc02_cmake_regen/nested/probe.c"
        self.write_source(nested, self.unique_source(nested))
        self.stage("add-multilevel-module", added=(nested,))
        moved = "application/mc02_cmake_regen/moved_probe.c"
        self.move_file(nested, moved)
        self.stage("move-module-source", added=(moved,), removed=(nested,))
        self.remove_file(moved)
        self.stage("remove-moved-module-source", removed=(moved,))

        header_a = "modules/mc02_cmake_header_a"
        header_b = "modules/mc02_cmake_header_b"
        header_name = "mc02_cmake_regen_header.h"
        header_source = "application/mc02_cmake_header_user/probe.c"
        self.write_source(f"{header_a}/{header_name}", "#define MC02_CMAKE_REGEN_VALUE 7\n")
        self.write_source(
            header_source,
            f'#include "{header_name}"\nint mc02_cmake_regen_header_user(void) '
            "{ return MC02_CMAKE_REGEN_VALUE; }\n",
        )
        self.stage("add-header-directory-and-user", added=(header_source,))
        self.move_file(f"{header_a}/{header_name}", f"{header_b}/{header_name}")
        self.stage("move-header-only-directory")
        self.remove_file(header_source)
        self.remove_file(f"{header_b}/{header_name}")
        self.stage("remove-header-fixture", removed=(header_source,))

        template = "Core/Src/mc02_cmake_regen_probe_template.c"
        self.write_source(template, self.unique_source(template))
        self.stage("add-excluded-template", excluded=(template,))
        self.remove_file(template)
        self.stage("remove-excluded-template")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--generator", choices=("Ninja",), default="Ninja")
    parser.add_argument("--build-type", default="Debug")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--ninja", default="ninja")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--command-timeout", type=int, default=900)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    if not (root / "CMakeLists.txt").is_file():
        parser.error(f"CMake root not found from {root}")
    args.work_dir.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix=f"run-{time.strftime('%Y%m%d-%H%M%S')}-", dir=args.work_dir))
    checker = Checker(root, run_dir, args)
    try:
        checker.run_all()
    except (CheckFailure, OSError, json.JSONDecodeError, subprocess.TimeoutExpired) as exc:
        failure_path = run_dir / "failure.txt"
        failure_path.write_text(f"{exc}\n", encoding="utf-8")
        print(f"FAIL: {exc}\nEvidence: {run_dir}", file=sys.stderr)
        return 1
    print(f"PASS: all incremental CMake source discovery checks passed\nEvidence: {run_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
