#!/usr/bin/env python3
"""用测试配置检查机器人角色相关框架源码或头；不生成机器人固件。"""

import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

from check_firmware_profile import check_portable_includes


def main() -> None:
    """复用固件编译器和框架头路径，逐角色检查声明或源码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True, help="已构建的 Ninja 固件目录")
    parser.add_argument("--sources", action="store_true", help="检查源码语法，跳过不适用角色的文件")
    args = parser.parse_args()
    root = Path.cwd().resolve()
    build = args.build_dir.resolve()
    entries = json.loads((build / "compile_commands.json").read_text(encoding="utf-8"))
    portable = [item for item in entries if "mc02_portable_framework.dir/" in item.get("command", "")]
    if not portable:
        raise SystemExit("编译数据库缺少框架对象")
    # 防止测试复用混入厂商头目录的编译参数。
    for entry in portable:
        check_portable_includes(entry, root, entry["file"])
    arguments = shlex.split(portable[0]["command"])
    flags: list[str] = []
    index = 1
    while index < len(arguments):
        argument = arguments[index]  # 只保留头目录、预处理选择和目标架构。
        if argument in ("-I", "-D", "-isystem"):
            if index + 1 >= len(arguments):
                raise SystemExit(f"编译参数缺少取值：{argument}")
            value = arguments[index + 1]
            if argument == "-D" and value.startswith("MC02_BENCH_"):
                flags.extend((argument, value.split("=", 1)[0] + "=0"))  # 机器人角色不运行裸板诊断。
            elif not (argument == "-D" and value.startswith(("MC02_PROFILE_BENCH_SAFE", "MC02_HEATER_ENABLED", "MC02_ENABLE_REFEREE_UI"))):
                flags.extend((argument, value))  # 角色测试单独指定 robot 宏。
            index += 2
            continue
        if argument.startswith(("-I", "-D", "-isystem", "-mcpu=", "-mfpu=", "-mfloat-abi=", "-mthumb")):
            if argument.startswith("-DMC02_BENCH_"):
                flags.append(argument.split("=", 1)[0] + "=0")  # 保留宏名并关闭裸板诊断。
            elif not argument.startswith(("-DMC02_PROFILE_BENCH_SAFE", "-DMC02_HEATER_ENABLED", "-DMC02_ENABLE_REFEREE_UI")):
                flags.append(argument)
        index += 1

    # 从框架清单找与源码同名的公开头；无同名源码的头不属于本工具的覆盖范围。
    cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
    match = re.search(r"set\(MC02_PORTABLE_FRAMEWORK_SOURCES(.*?)foreach\(source IN LISTS MC02_OPTIONAL_PORTABLE_SOURCES\)", cmake, re.S)
    if not match:
        raise SystemExit("未找到框架源码清单")
    sources = re.findall(r"(?m)^\s*((?:application|modules)/[A-Za-z0-9_/]+\.c)", match.group(1))
    if args.sources:
        files = sorted({root / source for source in sources if (root / source).is_file()})
    else:
        files = sorted({(root / source).with_suffix(".h") for source in sources
                        if (root / source).with_suffix(".h").is_file()})
    if not files:
        raise SystemExit("未找到框架源码或对应头文件")
    config = (build / "generated/mc02_build_config.h").read_text(encoding="utf-8")
    # 测试副本启用可选功能，以检查条件编译声明；它不作为生产配置或参数确认。
    config = re.sub(r"(#define MC02_(?:FEATURE|SELECTED_DRIVER)_[A-Z_]+) 0\b", r"\1 1", config)
    with tempfile.TemporaryDirectory(prefix="mc02-header-config-") as directory:
        checked = 0
        for heater in (0, 1):
            # 功能选择与底层温控宏保持一致，分别检查两种分支。
            for role in ("ONE_BOARD", "CHASSIS_BOARD", "GIMBAL_BOARD"):
                ui = int(role != "GIMBAL_BOARD")  # 裁判 UI 不运行在云台板角色。
                variant = re.sub(r"(#define MC02_FEATURE_HEATER) 1\b", rf"\g<1> {heater}", config)
                variant = re.sub(r"(#define MC02_FEATURE_UI) 1\b", rf"\g<1> {ui}", variant)
                (Path(directory) / "mc02_build_config.h").write_text(variant, encoding="utf-8")
                for source_file in files:
                    if args.sources and source_file.name in ("referee_bench.c", "remote_bench.c"):
                        continue  # 裸板专用入口不属于机器人固件。
                    if args.sources and role == "ONE_BOARD" and source_file.name == "robot_can_bridge.c":
                        continue  # 板间桥接只在双板角色中使用。
                    # 测试头目录排在生成目录之前，避免读取实际禁热配置。
                    warnings = [] if args.sources else ["-Wall", "-Wextra", "-Werror"]
                    command = [arguments[0], "-std=c11", *warnings, "-I" + directory,
                               "-DMC02_PROFILE_ROBOT=1", "-D" + role + "=1",
                               f"-DMC02_HEATER_ENABLED={heater}", f"-DMC02_ENABLE_REFEREE_UI={ui}", *flags,
                               "-fsyntax-only", "-x", "c", "-"]
                    result = subprocess.run(command, input=f'#include "{source_file}"\n',
                                            text=True, capture_output=True)
                    if result.returncode:
                        raise SystemExit(f"{role} heater={heater}: {source_file.relative_to(root)}\n{result.stderr}")
                    checked += 1
    print(f"PASS: {len(files)} 份框架{'源码' if args.sources else '同名头'}，"
          f"3种测试角色和2种温控选择共 {checked} 次 C11 语法检查通过")


if __name__ == "__main__":
    main()
