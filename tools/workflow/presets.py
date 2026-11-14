#!/usr/bin/env python3
"""集中维护官方固件构建预设、产物名和 CI 矩阵。

仓库的预设定义仍在 `CMakePresets.json`；本文件只补充“哪些预设要验证、
产物叫什么、按哪个 profile／role 检查”这些 CI 与本地脚本共用的信息。
`check_cmake` 会解析 CMakePresets，防止两边配置漂移。

用法：
    python3 tools/workflow/presets.py --list
    python3 tools/workflow/presets.py --github-matrix
    python3 tools/workflow/presets.py --check-cmake

退出状态：清单与 CMakePresets 不一致时返回 1。
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Preset:
    """一个需要构建和检查的固件配置。"""

    name: str
    profile: str
    role: str
    build_type: str
    diagnostics: str = ""


# 保留十种命名配置；robot 的实物参数尚未确认，只用于检查配置被拒绝。
OFFICIAL: tuple[Preset, ...] = (
    Preset("bench-debug", "bench_safe", "single", "Debug"),
    Preset("bench-rel", "bench_safe", "single", "RelWithDebInfo"),
    Preset("rcdiag-debug", "bench_safe", "single", "Debug", "rc"),
    Preset("rcdiag-rel", "bench_safe", "single", "RelWithDebInfo", "rc"),
    Preset("robot-single-debug", "robot", "single", "Debug"),
    Preset("robot-single-rel", "robot", "single", "RelWithDebInfo"),
    Preset("robot-chassis-debug", "robot", "chassis", "Debug"),
    Preset("robot-chassis-rel", "robot", "chassis", "RelWithDebInfo"),
    Preset("robot-gimbal-debug", "robot", "gimbal", "Debug"),
    Preset("robot-gimbal-rel", "robot", "gimbal", "RelWithDebInfo"),
)

# 默认只构建当前允许生成的裸板固件；不能用测试值绕过生产配置检查。
BUILDABLE = tuple(preset for preset in OFFICIAL if preset.profile == "bench_safe")


def artifact_name(preset: Preset) -> str:
    """按 CMakeLists 的命名规则返回 ELF 文件名。"""
    if preset.diagnostics == "rc":
        return "Basic_Framework_MC02_rcdiag_bench_safe.elf"
    if preset.profile == "bench_safe":
        return "Basic_Framework_MC02_bench_safe.elf"
    if preset.role == "single":
        return "Basic_Framework_MC02_robot.elf"
    if preset.role == "chassis":
        return "Basic_Framework_MC02_chassis_robot.elf"
    return "Basic_Framework_MC02_gimbal_robot.elf"


def profile_arguments(preset: Preset, build_dir: Path, elf: Path) -> list[str]:
    """构造检查脚本需要的参数，诊断预设额外声明 rc-diagnostics。"""
    arguments = [
        str(elf),
        "--profile", preset.profile,
        "--role", preset.role,
        "--build-dir", str(build_dir),
    ]
    if preset.diagnostics == "rc":
        arguments.append("--rc-diagnostics")
    return arguments


def github_matrix() -> list[dict[str, str]]:
    """输出 GitHub Actions 的 include 矩阵。"""
    matrix: list[dict[str, str]] = []
    for preset in BUILDABLE:
        row = asdict(preset)
        row["artifact"] = artifact_name(preset)
        matrix.append(row)
    return matrix


def _resolve(presets: dict[str, dict], name: str) -> dict:
    """沿 inherits 合并 cacheVariables，得到预设最终缓存值。"""
    preset = presets[name]
    merged: dict[str, object] = {}
    parent = preset.get("inherits")
    if parent:
        merged.update(_resolve(presets, parent))
    merged.update(preset.get("cacheVariables", {}))
    return merged


def check_cmake(path: Path | None = None) -> list[str]:
    """核对 Python 清单和 CMakePresets 的 profile、角色、构建类型。"""
    path = path or ROOT / "CMakePresets.json"
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return [f"读取 {path} 失败：{error}"]
    presets = {entry["name"]: entry for entry in data.get("configurePresets", [])}
    errors: list[str] = []
    for preset in OFFICIAL:
        if preset.name not in presets:
            errors.append(f"CMakePresets 缺少预设 {preset.name}")
            continue
        values = _resolve(presets, preset.name)
        expected = {
            "MC02_PROFILE": preset.profile,
            "MC02_BOARD_ROLE": preset.role,
            "CMAKE_BUILD_TYPE": preset.build_type,
        }
        for key, value in expected.items():
            if str(values.get(key)) != value:
                errors.append(
                    f"{preset.name}: {key} 为 {values.get(key)!r}，清单为 {value!r}"
                )
        if preset.name not in {
            entry.get("name") for entry in data.get("buildPresets", [])
        }:
            errors.append(f"CMakePresets 缺少构建预设 {preset.name}")
    return errors


def selected(names: str | None) -> list[Preset]:
    """按逗号分隔的名称筛选预设；未知名称抛 ValueError。"""
    if not names:
        return list(BUILDABLE)
    wanted = [item.strip() for item in names.split(",") if item.strip()]
    lookup = {preset.name: preset for preset in OFFICIAL}
    unknown = [name for name in wanted if name not in lookup]
    if unknown:
        raise ValueError("未知预设：" + "、".join(unknown))
    return [lookup[name] for name in wanted]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="官方固件预设清单")
    parser.add_argument("--list", action="store_true", help="按行列出预设与产物")
    parser.add_argument("--github-matrix", action="store_true", help="输出 GitHub Actions 矩阵")
    parser.add_argument("--check-cmake", action="store_true", help="核对 CMakePresets")
    parser.add_argument("--buildable-list", action="store_true", help="列出当前允许构建的预设名")
    args = parser.parse_args(argv)
    if args.buildable_list:
        print("\n".join(preset.name for preset in BUILDABLE))
        return 0
    if args.check_cmake:
        errors = check_cmake()
        if errors:
            print("\n".join(errors), file=sys.stderr)
            return 1
        print("预设清单与 CMakePresets 一致")
        return 0
    if args.github_matrix:
        print(json.dumps(github_matrix(), ensure_ascii=False))
        return 0
    for preset in OFFICIAL:
        print(f"{preset.name}\t{preset.profile}\t{preset.role}\t{preset.build_type}\t{artifact_name(preset)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
