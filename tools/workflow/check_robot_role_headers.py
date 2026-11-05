#!/usr/bin/env python3
"""用测试配置独立编译机器人角色相关框架头；不生成机器人固件。"""

import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

from check_firmware_profile import check_portable_includes


def main() -> None:
    """复用固件编译器和框架头路径，逐角色检查同名业务头。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True, help="已构建的 Ninja 固件目录")
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
            if not (argument == "-D" and value.startswith("MC02_PROFILE_BENCH_SAFE")):
                flags.extend((argument, value))  # 角色测试单独指定 robot 宏。
            index += 2
            continue
        if argument.startswith(("-I", "-D", "-isystem", "-mcpu=", "-mfpu=", "-mfloat-abi=", "-mthumb")):
            if not argument.startswith("-DMC02_PROFILE_BENCH_SAFE"):
                flags.append(argument)
        index += 1

    # 从框架清单找与源码同名的公开头；无同名源码的头不属于本工具的覆盖范围。
    cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
    match = re.search(r"set\(MC02_PORTABLE_FRAMEWORK_SOURCES(.*?)foreach\(source IN LISTS MC02_OPTIONAL_PORTABLE_SOURCES\)", cmake, re.S)
    if not match:
        raise SystemExit("未找到框架源码清单")
    sources = re.findall(r"(?m)^\s*((?:application|modules)/[A-Za-z0-9_/]+\.c)", match.group(1))
    headers = sorted({(root / source).with_suffix(".h") for source in sources
                      if (root / source).with_suffix(".h").is_file()})
    if not headers:
        raise SystemExit("未找到框架源码对应的头文件")
    config = (build / "generated/mc02_build_config.h").read_text(encoding="utf-8")
    # 测试副本启用可选功能，以检查条件编译声明；它不作为生产配置或参数确认。
    config = re.sub(r"(#define MC02_(?:FEATURE|SELECTED_DRIVER)_[A-Z_]+) 0\b", r"\1 1", config)
    with tempfile.TemporaryDirectory(prefix="mc02-header-config-") as directory:
        (Path(directory) / "mc02_build_config.h").write_text(config, encoding="utf-8")
        for role in ("ONE_BOARD", "CHASSIS_BOARD", "GIMBAL_BOARD"):
            for header in headers:
                # 测试头目录排在生成目录之前，避免读取实际禁热配置。
                command = [arguments[0], "-std=c11", "-Wall", "-Wextra", "-Werror",
                           "-I" + directory, "-DMC02_PROFILE_ROBOT=1", "-D" + role + "=1",
                           *flags, "-fsyntax-only", "-x", "c", "-"]
                result = subprocess.run(command, input=f'#include "{header}"\n',
                                        text=True, capture_output=True)
                if result.returncode:
                    raise SystemExit(f"{role}: {header.relative_to(root)}\n{result.stderr}")
    print(f"PASS: {len(headers)} 份同名框架头在3种测试角色下独立通过 C11 编译")


if __name__ == "__main__":
    main()
