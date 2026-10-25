#!/usr/bin/env python3
"""记录 CMake/Ninja 固件实际使用的第三方文件和构建产物。

在 WSL 的仓库根目录运行。工具读取编译数据库和 `ninja -t deps`，不会改写固件或构建目录。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterable


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]

PACKAGE_SPECS = (
    {
        "name": "STM32H7 HAL",
        "prefix": "Drivers/STM32H7xx_HAL_Driver/",
        "source": "https://github.com/STMicroelectronics/stm32h7xx-hal-driver",
        "license": "Drivers/STM32H7xx_HAL_Driver/LICENSE.txt",
        "version_file": "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c",
        "version_evidence": "stm32h7xx_hal.c version constant",
        "version_pattern": r"HAL Driver version number V([0-9.]+)",
    },
    {
        "name": "CMSIS Core",
        "prefix": "Drivers/CMSIS/Include/",
        "source": "https://github.com/ARM-software/CMSIS_5",
        "license": "Drivers/CMSIS/LICENSE.txt",
        "version_file": "Drivers/CMSIS/Include/cmsis_version.h",
        "version_evidence": "CMSIS Core version macros",
        "version_pattern": r"__CM_CMSIS_VERSION_MAIN\s+\(\s*(\d+)U?\).*?__CM_CMSIS_VERSION_SUB\s+\(\s*(\d+)U?\)",
    },
    {
        "name": "STM32H7 CMSIS Device",
        "prefix": "Drivers/CMSIS/Device/ST/STM32H7xx/",
        "source": "https://github.com/STMicroelectronics/cmsis-device-h7",
        "license": "Drivers/CMSIS/Device/ST/STM32H7xx/LICENSE.txt",
        "version_file": "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h",
        "version_evidence": "STM32H7 CMSIS Device version constant",
        "version_pattern": r"CMSIS Device version number V([0-9.]+)",
    },
    {
        "name": "FreeRTOS Kernel",
        "prefix": "Middlewares/Third_Party/FreeRTOS/Source/",
        "source": "https://github.com/FreeRTOS/FreeRTOS-Kernel",
        "license": "Middlewares/Third_Party/FreeRTOS/Source/LICENSE",
        "version_file": "Middlewares/Third_Party/FreeRTOS/Source/include/task.h",
        "version_evidence": "tskKERNEL_VERSION_NUMBER",
        "version_pattern": r'tskKERNEL_VERSION_NUMBER\s+"V([^\"]+)"',
    },
    {
        "name": "CMSIS-DSP",
        "prefix": "Middlewares/Third_Party/CMSIS-DSP-1.7.0/",
        "source": "https://github.com/ARM-software/CMSIS_5",
        "license": "Middlewares/Third_Party/CMSIS-DSP-1.7.0/LICENSE.txt",
        "version_file": "Middlewares/Third_Party/CMSIS-DSP-1.7.0/provenance.json",
        "version_evidence": "provenance.json with upstream commit",
        "version_pattern": None,
    },
    {
        "name": "SEGGER RTT",
        "prefix": "Middlewares/Third_Party/SEGGER/",
        "source": "https://github.com/SEGGERMicro/RTT",
        "license": None,
        "version_file": "Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.h",
        "version_evidence": "SEGGER source header",
        "version_pattern": r"RTT version:\s*([0-9.]+[a-z]?)",
    },
    {
        "name": "STM32 USB Device Library",
        "prefix": "Middlewares/ST/STM32_USB_Device_Library/",
        "source": "https://github.com/STMicroelectronics/stm32-mw-usb-device",
        "license": "Middlewares/ST/STM32_USB_Device_Library/LICENSE.txt",
        "version_file": "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c",
        "version_evidence": "no version string in selected source",
        "version_pattern": None,
    },
)

INTEGRATION_FILES = (
    "Core/Inc/FreeRTOSConfig.h",
    "Core/Inc/stm32h7xx_hal_conf.h",
    "USB_DEVICE/Target/usbd_conf.h",
    "Middlewares/Third_Party/SEGGER/Config/SEGGER_RTT_Conf.h",
    "cmake/mc02_firmware_sources.cmake",
)


class BaselineError(RuntimeError):
    """表示构建目录或来源记录不满足采集条件。"""


def sha256_file(path: Path) -> str:
    """按文件原始字节计算 SHA-256。"""
    digest = hashlib.sha256()
    with path.open("rb") as source_file:
        for block in iter(lambda: source_file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_cache(path: Path) -> dict[str, str]:
    """读取 CMakeCache 中有效的键值，不解析注释行。"""
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        if ":" in key:
            key = key.split(":", 1)[0]
        values[key] = value
    return values


def validate_source_commit(source_commit: str) -> None:
    """拒绝不完整或带格式错误的提交号，避免报告引用模糊版本。"""
    if not re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", source_commit):
        raise BaselineError("源码提交 SHA 必须是完整的 40 或 64 位小写十六进制值")


def parse_ninja_dependencies(output: str) -> set[str]:
    """从 Ninja 依赖报告中提取对象实际读取的文件路径。"""
    dependencies: set[str] = set()
    for line in output.splitlines():
        if line.startswith("    "):
            path_text = line.strip()
            if path_text.startswith("/"):
                dependencies.add(path_text)
    return dependencies


def package_version(spec: dict[str, str | None], source_root: Path) -> str:
    """从供应商版本字段读取版本；未写入源文件的组件明确返回缺失标记。"""
    source_path = source_root / str(spec["version_file"])
    source_text = source_path.read_text(encoding="utf-8", errors="replace")
    pattern = spec["version_pattern"]
    if pattern is None:
        if spec["name"] == "CMSIS-DSP":
            provenance = json.loads(source_text)
            return f"CMSIS {provenance['cmsis_version']} / DSP {provenance['dsp_version']}"
        return "源文件未声明版本；以实际依赖文件指纹锁定"
    match = re.search(str(pattern), source_text, re.DOTALL)
    if not match:
        raise BaselineError(f"无法从 {spec['version_file']} 读取版本")
    if spec["name"] == "CMSIS Core":
        return f"{match.group(1)}.{match.group(2)}.0"
    return match.group(1)


def relative_workspace_path(path_text: str, source_root: Path) -> str | None:
    """把仓库内文件化为相对路径，忽略编译器和系统头文件。"""
    candidate = Path(path_text)
    if not candidate.is_absolute():
        candidate = source_root / candidate
    try:
        return candidate.resolve().relative_to(source_root.resolve()).as_posix()
    except ValueError:
        return None


def fingerprint_group(paths: Iterable[str], source_root: Path) -> dict[str, object]:
    """按路径和文件摘要生成可重复的分组指纹。"""
    records = []
    for relative_path in sorted(set(paths)):
        absolute_path = source_root / relative_path
        if not absolute_path.is_file():
            raise BaselineError(f"实际编译依赖缺失：{relative_path}")
        records.append({"path": relative_path, "sha256": sha256_file(absolute_path)})
    if not records:
        raise BaselineError("所选构建未使用该第三方组件")
    digest = hashlib.sha256()
    for record in records:
        digest.update(f"{record['path']}\0{record['sha256']}\n".encode("utf-8"))
    return {"file_count": len(records), "sha256": digest.hexdigest(), "files": records}


def run_checked(command: list[str], cwd: Path) -> str:
    """执行版本或 Ninja 查询，失败时保留命令和诊断。"""
    completed = subprocess.run(command, cwd=cwd, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        raise BaselineError(
            f"命令失败：{' '.join(command)}\n{completed.stdout}{completed.stderr}"
        )
    return completed.stdout.strip()


def collect_build(
    build_dir: Path,
    source_root: Path = REPOSITORY_ROOT,
    source_commit: str | None = None,
) -> dict[str, object]:
    """校验固件产物并收集配置、工具链和实际第三方依赖。"""
    build_dir = build_dir.resolve()
    cache_path = build_dir / "CMakeCache.txt"
    compile_db_path = build_dir / "compile_commands.json"
    selected_path = build_dir / "mc02-selected-sources.txt"
    for required in (cache_path, compile_db_path, selected_path):
        if not required.is_file():
            raise BaselineError(f"缺少构建记录：{required}")

    cache = parse_cache(cache_path)
    source_root = source_root.resolve()
    resolved_source_commit = source_commit or run_checked(["git", "rev-parse", "HEAD"], source_root)
    validate_source_commit(resolved_source_commit)
    configured_source_root = Path(cache.get("CMAKE_HOME_DIRECTORY", "")).resolve()
    if configured_source_root != source_root:
        raise BaselineError(f"构建目录来源不匹配：{configured_source_root}")
    if cache.get("CMAKE_GENERATOR") != "Ninja":
        raise BaselineError("基线固件必须由 Ninja 构建")

    selected_lines = selected_path.read_text(encoding="utf-8").splitlines()
    selected = dict(line.split("=", 1) for line in selected_lines if "=" in line)
    target_name = selected.get("target", "")
    if not target_name.endswith(".elf"):
        raise BaselineError("mc02-selected-sources.txt 中缺少 ELF 目标名")
    basename = target_name[:-4]

    artifacts = {}
    for extension in ("elf", "hex", "bin", "map"):
        artifact_path = build_dir / f"{basename}.{extension}"
        if not artifact_path.is_file() or artifact_path.stat().st_size == 0:
            raise BaselineError(f"固件产物缺失或为空：{artifact_path}")
        artifacts[extension] = {
            "file": artifact_path.name,
            "size_bytes": artifact_path.stat().st_size,
            "sha256": sha256_file(artifact_path),
        }

    compile_commands = json.loads(compile_db_path.read_text(encoding="utf-8"))
    compiled_files: set[str] = set()
    compile_flags: set[str] = set()
    for entry in compile_commands:
        source = relative_workspace_path(str(entry["file"]), source_root)
        if source:
            compiled_files.add(source)
        command_text = entry.get("command", "")
        arguments = entry.get("arguments", [])
        compile_flags.update(arguments if arguments else command_text.split())
    if not compile_commands:
        raise BaselineError("编译数据库为空")
    required_abi_flags = {"-mcpu=cortex-m7", "-mfloat-abi=hard", "-mfpu=fpv5-d16"}
    if not required_abi_flags.issubset(compile_flags):
        raise BaselineError("编译数据库未使用 Cortex-M7 hard-float fpv5-d16 ABI")
    if "-mfloat-abi=hard" not in compile_flags or "-mfpu=fpv5-d16" not in compile_flags:
        raise BaselineError("编译数据库未同时使用 hard-float 与 fpv5-d16")
    if "-ffast-math" in compile_flags or "-Ofast" in compile_flags:
        raise BaselineError("编译数据库启用了 fast-math")

    dependency_output = run_checked(["ninja", "-t", "deps"], build_dir)
    dependency_paths = parse_ninja_dependencies(dependency_output)
    tracked_inputs = set(compiled_files)
    for path_text in dependency_paths:
        relative_path = relative_workspace_path(path_text, source_root)
        if relative_path:
            tracked_inputs.add(relative_path)

    package_records = []
    for spec in PACKAGE_SPECS:
        package_paths = [path for path in tracked_inputs if path.startswith(str(spec["prefix"]))]
        package_records.append(
            {
                "name": spec["name"],
                "version": package_version(spec, source_root),
                "version_evidence": spec["version_evidence"],
                "source": spec["source"],
                "license": spec["license"],
                **fingerprint_group(package_paths, source_root),
            }
        )

    integration = []
    for relative_path in INTEGRATION_FILES:
        path = source_root / relative_path
        if not path.is_file():
            raise BaselineError(f"缺少第一方集成配置文件：{relative_path}")
        integration.append({"path": relative_path, "sha256": sha256_file(path)})

    firmware_compiler = cache.get("CMAKE_C_COMPILER", "")
    cmake_version = run_checked(["cmake", "--version"], source_root).splitlines()[0]
    ninja_version = run_checked(["ninja", "--version"], source_root)
    compiler_version = run_checked([firmware_compiler, "--version"], source_root).splitlines()[0]
    size_report = run_checked(["arm-none-eabi-size", str(build_dir / f"{basename}.elf")], source_root)
    size_fields = size_report.splitlines()[-1].split()
    if len(size_fields) < 5:
        raise BaselineError("arm-none-eabi-size 未返回预期的 text/data/bss 列")

    return {
        "schema_version": 1,
        "source_commit": resolved_source_commit,
        "preset_configuration": {
            "build_type": cache.get("CMAKE_BUILD_TYPE"),
            "profile": cache.get("MC02_PROFILE"),
            "board_role": cache.get("MC02_BOARD_ROLE"),
            "vision_transport": cache.get("MC02_VISION_TRANSPORT"),
            "heater": cache.get("MC02_ENABLE_HEATER"),
            "target": target_name,
        },
        "toolchain": {
            "cmake": cmake_version,
            "generator": "Ninja " + ninja_version,
            "arm_compiler": compiler_version,
            "compile_commands": len(compile_commands),
            "compile_abi": "-mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16",
            "fast_math": False,
        },
        "artifacts": artifacts,
        "size_bytes": {
            "text": int(size_fields[0]),
            "data": int(size_fields[1]),
            "bss": int(size_fields[2]),
            "text_plus_data": int(size_fields[0]) + int(size_fields[1]),
            "data_plus_bss": int(size_fields[1]) + int(size_fields[2]),
        },
        "third_party_packages": package_records,
        "first_party_integration_files": integration,
    }


def assemble_matrix(reports: list[dict[str, object]]) -> dict[str, object]:
    """合并多个构建记录，保留各配置自己的依赖摘要。"""
    if not reports:
        raise BaselineError("至少需要一个固件构建记录")

    source_commits = {str(report["source_commit"]) for report in reports}
    if len(source_commits) != 1:
        raise BaselineError("构建矩阵中的源提交不一致")

    package_catalog: dict[str, dict[str, object]] = {}
    integration_files: dict[str, str] = {}
    configurations = []
    for report in reports:
        fingerprints: dict[str, dict[str, object]] = {}
        for package in report["third_party_packages"]:
            name = str(package["name"])
            record = package_catalog.setdefault(
                name,
                {
                    key: package[key]
                    for key in ("name", "version", "version_evidence", "source", "license")
                }
                | {"files": {}},
            )
            if any(record[key] != package[key] for key in ("version", "source", "license")):
                raise BaselineError(f"不同配置中的依赖信息不一致：{name}")
            catalog_files = record["files"]
            assert isinstance(catalog_files, dict)
            for file_record in package["files"]:
                path = str(file_record["path"])
                digest = str(file_record["sha256"])
                old_digest = catalog_files.setdefault(path, digest)
                if old_digest != digest:
                    raise BaselineError(f"不同配置中的文件指紋不一致：{path}")
            fingerprints[name] = {
                "file_count": package["file_count"],
                "sha256": package["sha256"],
            }
        for integration_file in report["first_party_integration_files"]:
            path = str(integration_file["path"])
            digest = str(integration_file["sha256"])
            old_digest = integration_files.setdefault(path, digest)
            if old_digest != digest:
                raise BaselineError(f"不同配置中的適配文件指紋不一致：{path}")
        configurations.append(
            {
                "preset_configuration": report["preset_configuration"],
                "toolchain": report["toolchain"],
                "artifacts": report["artifacts"],
                "size_bytes": report["size_bytes"],
                "third_party_fingerprints": fingerprints,
            }
        )

    normalized_packages = []
    for name in sorted(package_catalog):
        package = package_catalog[name]
        files = package.pop("files")
        assert isinstance(files, dict)
        file_records = [{"path": path, "sha256": files[path]} for path in sorted(files)]
        digest = hashlib.sha256()
        for file_record in file_records:
            digest.update(f"{file_record['path']}\0{file_record['sha256']}\n".encode("utf-8"))
        normalized_packages.append(
            {
                **package,
                "union_file_count": len(file_records),
                "union_sha256": digest.hexdigest(),
                "files": file_records,
            }
        )

    normalized_integrations = [
        {"path": path, "sha256": integration_files[path]}
        for path in sorted(integration_files)
    ]
    return {
        "schema_version": 1,
        "source_commit": source_commits.pop(),
        "configurations": configurations,
        "third_party_packages": normalized_packages,
        "first_party_integration_files": normalized_integrations,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True, help="WSL Ninja 固件构建目录")
    parser.add_argument("--source-root", type=Path, default=REPOSITORY_ROOT, help="对应构建缓存的源码根目录")
    parser.add_argument("--output", type=Path, help="可选 JSON 输出路径")
    parser.add_argument(
        "--matrix-build-dir",
        type=Path,
        action="append",
        default=[],
        help="同一源码提交的其他构建目录；可重复指定以记录配置矩阵",
    )
    parser.add_argument(
        "--source-commit",
        help="工作树元数据无法读取时，提供已核对的完整提交 SHA",
    )
    args = parser.parse_args()
    try:
        build_dirs = [args.build_dir, *args.matrix_build_dir]
        builds = [
            collect_build(build_dir, args.source_root, args.source_commit)
            for build_dir in build_dirs
        ]
        report = assemble_matrix(builds)
    except (BaselineError, OSError, json.JSONDecodeError) as error:
        print(f"基线采集失败：{error}", file=sys.stderr)
        return 2

    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        sys.stdout.write(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
