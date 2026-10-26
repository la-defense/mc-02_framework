#!/usr/bin/env python3
"""为已完成的 MC-02 ARM 构建生成可下载、可核对的固件归档。

在仓库根目录使用 CMake 预设构建后运行。脚本只读取构建结果，并在输出目录新建 ZIP；缺少产物、配置不匹配或身份无法确认时返回错误。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import zipfile
from pathlib import Path
from typing import Any, Callable


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ARTIFACT_SUFFIXES = (".elf", ".hex", ".bin", ".map")
CONFIG_KEYS = (
    "MC02_PROFILE",
    "MC02_BOARD_ROLE",
    "MC02_VISION_TRANSPORT",
    "MC02_VISION_PROTOCOL",
    "MC02_IMU_BACKEND",
    "MC02_ENABLE_HEATER",
    "MC02_ENABLE_REFEREE_UI",
    "CMAKE_BUILD_TYPE",
)
DIAGNOSTIC_KEYS = (
    "MC02_BENCH_RC_DIAGNOSTICS",
    "MC02_BENCH_REFEREE_DIAGNOSTICS",
    "MC02_BENCH_LCD_DIAGNOSTICS",
    "MC02_BENCH_USB_DIAGNOSTICS",
    "MC02_BENCH_CRASH_DIAGNOSTICS",
    "MC02_LOG_TEST",
)


class ArchiveError(RuntimeError):
    """表示构建信息、产物或归档内容无法通过核对。"""


CommandRunner = Callable[[list[str], Path | None], str]


def run_command(arguments: list[str], cwd: Path | None = None) -> str:
    """运行只读版本／ELF查询命令；失败时保留命令错误并停止打包。"""
    try:
        result = subprocess.run(arguments, cwd=cwd, check=True, capture_output=True,
                                text=True, encoding="utf-8", errors="replace", timeout=20)
    except (OSError, subprocess.SubprocessError) as error:
        raise ArchiveError(f"命令执行失败：{arguments[0]}：{error}") from error
    return result.stdout + result.stderr


def sha256_bytes(data: bytes) -> str:
    """返回一段字节的 SHA-256 小写十六进制值。"""
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    """分块读取文件，计算原始字节散列。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_cache(path: Path) -> dict[str, str]:
    """读取 CMakeCache 的键值，不把注释误当配置。"""
    if not path.is_file():
        raise ArchiveError(f"缺少 CMakeCache.txt：{path}")
    cache: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith("#") or line.startswith("//") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        if ":" in key:
            key = key.split(":", 1)[0]
        cache[key] = value
    return cache


def _merge_configure_preset(name: str, presets: dict[str, dict[str, Any]],
                            visiting: set[str] | None = None) -> dict[str, Any]:
    """合并预设继承项，子预设中的值覆盖父预设。"""
    if name not in presets:
        raise ArchiveError(f"CMake 配置预设不存在：{name}")
    active = set() if visiting is None else visiting
    if name in active:
        raise ArchiveError(f"CMake 预设继承形成循环：{name}")
    active.add(name)
    child = presets[name]
    merged: dict[str, Any] = {"cacheVariables": {}}
    inherited = child.get("inherits", [])
    if isinstance(inherited, str):
        inherited = [inherited]
    if not isinstance(inherited, list):
        raise ArchiveError(f"预设 inherits 格式错误：{name}")
    for parent_name in inherited:
        parent = _merge_configure_preset(str(parent_name), presets, active)
        merged.update({key: value for key, value in parent.items() if key != "cacheVariables"})
        merged["cacheVariables"].update(parent.get("cacheVariables", {}))
    merged.update({key: value for key, value in child.items()
                   if key not in ("inherits", "cacheVariables")})
    merged["cacheVariables"].update(child.get("cacheVariables", {}))
    active.remove(name)
    return merged


def read_preset(root: Path, build_preset_name: str) -> dict[str, Any]:
    """读取构建预设及其配置预设，确定预期生成器和缓存值。"""
    preset_file = root / "CMakePresets.json"
    try:
        document = json.loads(preset_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ArchiveError(f"无法读取 CMakePresets.json：{error}") from error
    configure = {item["name"]: item for item in document.get("configurePresets", [])}
    builds = {item["name"]: item for item in document.get("buildPresets", [])}
    if build_preset_name not in builds:
        raise ArchiveError(f"构建预设不存在：{build_preset_name}")
    build_preset = builds[build_preset_name]
    configure_name = build_preset.get("configurePreset")
    if not isinstance(configure_name, str):
        raise ArchiveError(f"构建预设没有 configurePreset：{build_preset_name}")
    resolved = _merge_configure_preset(configure_name, configure)
    return {
        "build_preset": build_preset_name,
        "configure_preset": configure_name,
        "generator": resolved.get("generator"),
        "cache_variables": resolved.get("cacheVariables", {}),
    }


def _preset_value(value: Any) -> str:
    """统一比较预设字符串和布尔型缓存值。"""
    if isinstance(value, bool):
        return "ON" if value else "OFF"
    if isinstance(value, dict):
        value = value.get("value")
    return str(value)


def parse_selected_sources(path: Path) -> dict[str, str]:
    """读取 CMake 生成的角色与目标摘要。"""
    if not path.is_file():
        raise ArchiveError(f"缺少源码选择摘要：{path}")
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            if key in {"profile", "role", "vision", "vision_protocol", "imu_backend", "target"}:
                values[key] = value
    required = {"profile", "role", "vision", "vision_protocol", "imu_backend", "target"}
    missing = sorted(required - values.keys())
    if missing:
        raise ArchiveError(f"源码选择摘要缺少字段：{', '.join(missing)}")
    return values


def read_git_state(root: Path) -> dict[str, Any]:
    """记录完整源码提交及已跟踪改动路径，不把未跟踪资料放入归档。"""
    try:
        commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, check=True,
                                capture_output=True, text=True, timeout=10).stdout.strip()
        timestamp_text = subprocess.run(["git", "show", "-s", "--format=%ct", "HEAD"], cwd=root,
                                        check=True, capture_output=True, text=True,
                                        timeout=10).stdout.strip()
        status = subprocess.run(["git", "-c", "core.quotepath=false", "status", "--porcelain=v1",
                                 "--untracked-files=no"], cwd=root,
                                check=True, capture_output=True, text=True,
                                encoding="utf-8", errors="replace", timeout=30).stdout
    except (OSError, subprocess.SubprocessError) as error:
        raise ArchiveError(f"无法读取 Git 构建身份：{error}") from error
    if not re.fullmatch(r"[0-9a-f]{40,64}", commit):
        raise ArchiveError("Git 未返回完整提交 SHA")
    changed_entries = [line for line in status.splitlines() if line]
    changed_paths = sorted(line[3:] if len(line) > 3 else line for line in changed_entries)
    path_fingerprint = "\n".join(changed_entries).encode("utf-8")
    return {
        "commit": commit,
        "commit_time": int(timestamp_text),
        "tracked_changes": bool(changed_entries),
        "tracked_change_count": len(changed_entries),
        "tracked_change_paths": changed_paths,
        "tracked_change_paths_sha256": sha256_bytes(path_fingerprint) if changed_entries else None,
    }


def _require_cache(cache: dict[str, str], key: str) -> str:
    """读取必需 CMake 配置；键缺失说明产物来源不完整。"""
    value = cache.get(key)
    if value is None or value == "":
        raise ArchiveError(f"CMakeCache 缺少配置：{key}")
    return value


def _read_build_id(elf: Path, readelf: str, command_runner: CommandRunner) -> str:
    """从 ELF note 读取唯一的 GNU Build ID。"""
    output = command_runner([readelf, "-n", str(elf)], None)
    matches = re.findall(r"Build ID:\s*([0-9a-fA-F]+)", output)
    if len(matches) != 1 or len(matches[0]) != 40:
        raise ArchiveError("ELF 必须包含唯一的20字节 GNU Build ID")
    return matches[0].lower()


def _safe_zip_time(timestamp: int) -> tuple[int, int, int, int, int, int]:
    """将 Git 提交时间转成 ZIP 支持的 UTC 时间。"""
    value = time.gmtime(max(timestamp, 315532800))
    return value.tm_year, value.tm_mon, value.tm_mday, value.tm_hour, value.tm_min, value.tm_sec - value.tm_sec % 2


def _archive_bytes(entries: dict[str, bytes], timestamp: int) -> bytes:
    """以固定时间和顺序生成归档，避免个人文件时间进入包内容。"""
    import io

    output = io.BytesIO()
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name in sorted(entries):
            info = zipfile.ZipInfo(name, _safe_zip_time(timestamp))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, entries[name])
    return output.getvalue()


def create_archive(root: Path, preset_name: str, build_dir: Path, output_dir: Path,
                   readelf: str = "arm-none-eabi-readelf",
                   command_runner: CommandRunner = run_command,
                   git_reader: Callable[[Path], dict[str, Any]] = read_git_state) -> Path:
    """校验构建身份并生成包含 ELF/HEX/BIN/map 的 ZIP 包。"""
    root = root.resolve()
    build_dir = build_dir.resolve()
    if not build_dir.is_dir():
        raise ArchiveError(f"构建目录不存在：{build_dir}")

    preset = read_preset(root, preset_name)
    cache = parse_cache(build_dir / "CMakeCache.txt")
    home_directory = Path(_require_cache(cache, "CMAKE_HOME_DIRECTORY")).resolve()
    if home_directory != root:
        raise ArchiveError("构建目录的 CMAKE_HOME_DIRECTORY 与当前源码根目录不匹配")
    if _require_cache(cache, "CMAKE_GENERATOR") != preset["generator"]:
        raise ArchiveError("构建目录生成器与所选预设不匹配")
    if _require_cache(cache, "CMAKE_BUILD_TYPE") not in {"Debug", "RelWithDebInfo"}:
        raise ArchiveError("固件归档只接受 Debug 或 RelWithDebInfo 构建")

    for key, expected in preset["cache_variables"].items():
        if key in CONFIG_KEYS or key in DIAGNOSTIC_KEYS:
            actual = _require_cache(cache, key)
            if actual.upper() != _preset_value(expected).upper():
                raise ArchiveError(f"缓存配置与预设不符：{key}={actual}，预期 {_preset_value(expected)}")
    profile = _require_cache(cache, "MC02_PROFILE")
    role = _require_cache(cache, "MC02_BOARD_ROLE")
    vision = _require_cache(cache, "MC02_VISION_TRANSPORT")
    if profile not in {"bench_safe", "robot"} or role not in {"single", "chassis", "gimbal"}:
        raise ArchiveError("构建身份中的 profile 或 board role 无效")
    if _require_cache(cache, "MC02_ENABLE_HEATER").upper() not in {"OFF", "FALSE", "0"}:
        raise ArchiveError("拒绝归档加热器开启的固件")
    if profile == "bench_safe" and role != "single":
        raise ArchiveError("bench_safe 固件只能使用 single 角色")

    selected = parse_selected_sources(build_dir / "mc02-selected-sources.txt")
    expected_summary = {
        "profile": profile,
        "role": role,
        "vision": vision,
        "vision_protocol": _require_cache(cache, "MC02_VISION_PROTOCOL"),
        "imu_backend": _require_cache(cache, "MC02_IMU_BACKEND"),
    }
    for key, expected in expected_summary.items():
        if selected[key] != expected:
            raise ArchiveError(f"源码选择摘要与 CMakeCache 不匹配：{key}")
    target_name = selected["target"]
    if Path(target_name).name != target_name or not target_name.endswith(".elf"):
        raise ArchiveError("源码选择摘要中的固件目标名无效")
    artifact_base = target_name[:-4]

    source_commit = git_reader(root)
    compiler = _require_cache(cache, "CMAKE_C_COMPILER")
    if Path(compiler).name != "arm-none-eabi-gcc":
        raise ArchiveError("固件归档只接受 arm-none-eabi-gcc 构建")
    compiler_output = command_runner([compiler, "--version"], None).splitlines()
    if not compiler_output:
        raise ArchiveError("交叉编译器没有返回版本信息")
    cmake_output = command_runner(["cmake", "--version"], None).splitlines()
    ninja_output = command_runner(["ninja", "--version"], None).splitlines()
    if not cmake_output or not ninja_output:
        raise ArchiveError("无法取得 CMake 或 Ninja 版本")

    toolchain_path = Path(_require_cache(cache, "CMAKE_TOOLCHAIN_FILE")).resolve()
    try:
        toolchain_relative = toolchain_path.relative_to(root).as_posix()
    except ValueError as error:
        raise ArchiveError("工具链文件不在仓库内，不能生成可复核归档") from error
    if toolchain_relative != "cmake/arm-none-eabi-toolchain.cmake":
        raise ArchiveError("构建没有使用仓库锁定的 ARM 工具链文件")
    if not toolchain_path.is_file():
        raise ArchiveError(f"工具链文件不存在：{toolchain_path}")

    artifact_paths = {suffix[1:]: build_dir / f"{artifact_base}{suffix}" for suffix in ARTIFACT_SUFFIXES}
    for path in artifact_paths.values():
        if not path.is_file() or path.stat().st_size == 0:
            raise ArchiveError(f"缺少固件归档产物或文件为空：{path.name}")

    build_id = _read_build_id(artifact_paths["elf"], readelf, command_runner)
    build_type = _require_cache(cache, "CMAKE_BUILD_TYPE")
    config_identity: dict[str, Any] = {
        "profile": profile,
        "board_role": role,
        "platform": "STM32H723xx",
        "vision_transport": vision,
        "vision_protocol": expected_summary["vision_protocol"],
        "imu_backend": expected_summary["imu_backend"],
        "build_type": build_type,
        "heater_enabled": False,
        "referee_ui_enabled": _require_cache(cache, "MC02_ENABLE_REFEREE_UI").upper() in {"ON", "TRUE", "1"},
        "diagnostics": {
            key.removeprefix("MC02_").lower(): cache.get(key, "OFF").upper() in {"ON", "TRUE", "1"}
            for key in DIAGNOSTIC_KEYS
        },
    }
    files: dict[str, bytes] = {}
    artifact_manifest: dict[str, dict[str, Any]] = {}
    for kind, path in sorted(artifact_paths.items()):
        content = path.read_bytes()
        files[path.name] = content
        artifact_manifest[path.name] = {"kind": kind, "size_bytes": len(content), "sha256": sha256_bytes(content)}

    source_list = build_dir / "mc02-selected-sources.txt"
    manifest = {
        "schema_version": 1,
        "archive_kind": "mc02-arm-firmware",
        "source": source_commit,
        "build": {
            "preset": preset_name,
            "configure_preset": preset["configure_preset"],
            "generator": preset["generator"],
            "cmake_version": cmake_output[0],
            "generator_version": ninja_output[0],
            "compiler": {"name": Path(compiler).name, "version": compiler_output[0]},
            "toolchain_file": {"path": toolchain_relative, "sha256": sha256_file(toolchain_path)},
            "selected_sources_sha256": sha256_file(source_list),
            "identity": config_identity,
        },
        "firmware": {
            "target": target_name,
            "gnu_build_id": build_id,
            "files": artifact_manifest,
        },
        "path_policy": "清单不保存本机绝对路径；ELF与map保留构建时的调试路径，供匹配源码工作区使用。",
    }
    manifest_bytes = (json.dumps(manifest, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    files["build-info.json"] = manifest_bytes
    checksummed_names = sorted(name for name in files)
    sums = "".join(f"{sha256_bytes(files[name])}  {name}\n" for name in checksummed_names).encode("ascii")
    files["SHA256SUMS"] = sums

    archive_content = _archive_bytes(files, int(source_commit["commit_time"]))
    short_commit = str(source_commit["commit"])[:12]
    manifest_id = sha256_bytes(manifest_bytes)[:8]
    destination = output_dir.resolve() / f"mc02-{preset_name}-{short_commit}-{build_id}-{manifest_id}.zip"
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if sha256_file(destination) == sha256_bytes(archive_content):
            return destination
        raise ArchiveError(f"归档已存在且内容不同，拒绝覆盖：{destination.name}")

    handle, temporary_name = tempfile.mkstemp(prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent)
    try:
        with os.fdopen(handle, "wb") as temporary:
            temporary.write(archive_content)
            temporary.flush()
            os.fsync(temporary.fileno())
        _verify_archive(Path(temporary_name), files)
        try:
            # 同目录硬链接以原子方式占用新文件名，防止并行打包覆盖旧归档。
            os.link(temporary_name, destination)
        except FileExistsError:
            if sha256_file(destination) == sha256_bytes(archive_content):
                return destination
            raise ArchiveError(f"归档已存在且内容不同，拒绝覆盖：{destination.name}")
    finally:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
    return destination


def _verify_archive(path: Path, expected: dict[str, bytes]) -> None:
    """解开临时包并逐项核对校验文件，避免写出不完整归档。"""
    try:
        with zipfile.ZipFile(path, "r") as archive:
            if archive.testzip() is not None:
                raise ArchiveError("ZIP 压缩数据校验失败")
            if set(archive.namelist()) != set(expected):
                raise ArchiveError("ZIP 文件清单与预期产物不一致")
            sums_text = archive.read("SHA256SUMS").decode("ascii")
            listed: set[str] = set()
            for line in sums_text.splitlines():
                digest, name = line.split("  ", 1)
                if name not in expected or sha256_bytes(archive.read(name)) != digest:
                    raise ArchiveError(f"归档校验和不匹配：{name}")
                if name in listed:
                    raise ArchiveError(f"归档校验文件重复列出：{name}")
                listed.add(name)
            if listed != set(expected) - {"SHA256SUMS"}:
                raise ArchiveError("SHA256SUMS 未完整列出归档内容")
            if any(Path(name).is_absolute() or ".." in Path(name).parts for name in archive.namelist()):
                raise ArchiveError("归档包含绝对路径或目录上溯路径")
    except (OSError, zipfile.BadZipFile, UnicodeDecodeError, ValueError) as error:
        raise ArchiveError(f"归档校验失败：{error}") from error


def main(argv: list[str] | None = None) -> int:
    """读取命令行参数并生成单个预设的固件包。"""
    parser = argparse.ArgumentParser(description="将一个 CMake ARM 固件构建归档为可校验 ZIP。")
    parser.add_argument("--preset", required=True, help="CMake 构建预设，例如 bench-debug")
    parser.add_argument("--build-dir", type=Path, help="可选构建目录；默认 build/wsl/<预设>")
    parser.add_argument("--output-dir", type=Path, default=Path("build/firmware-archives"),
                        help="归档输出目录；相对路径以仓库根目录为准")
    parser.add_argument("--readelf", default="arm-none-eabi-readelf", help="读取 GNU Build ID 的工具")
    args = parser.parse_args(argv)
    build_dir = args.build_dir if args.build_dir is not None else Path("build/wsl") / args.preset
    if not build_dir.is_absolute():
        build_dir = REPOSITORY_ROOT / build_dir
    output_dir = args.output_dir if args.output_dir.is_absolute() else REPOSITORY_ROOT / args.output_dir
    try:
        archive = create_archive(REPOSITORY_ROOT, args.preset, build_dir, output_dir, args.readelf)
    except ArchiveError as error:
        print(f"固件归档失败：{error}", file=sys.stderr)
        return 2
    print(f"固件归档：{archive}")
    print(f"SHA-256：{sha256_file(archive)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
