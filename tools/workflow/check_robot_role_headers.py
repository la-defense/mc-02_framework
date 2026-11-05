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
    parser.add_argument("--dependencies", action="store_true", help="检查每种角色源码依赖的工程头")
    parser.add_argument("--protocol", choices=("sp", "v2"), default="sp", help="视觉协议")
    parser.add_argument("--transport", choices=("usb", "uart"), default="usb", help="视觉传输")
    args = parser.parse_args()
    if args.dependencies and not args.sources:
        raise SystemExit("--dependencies 需要与 --sources 一起使用")
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
    replaced_defines = (
        "MC02_PROFILE_BENCH_SAFE",  # 由测试改为机器人配置。
        "MC02_HEATER_ENABLED",  # 随温控测试选择重新指定。
        "MC02_ENABLE_REFEREE_UI",  # 随板角色重新指定。
        "MC02_VISION_PROTOCOL_",  # 随命令行协议选择重新指定。
        "VISION_USE_",  # 随命令行传输选择重新指定。
    )
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
            elif not (argument == "-D" and value.startswith(replaced_defines)):
                flags.extend((argument, value))  # 角色测试单独指定 robot 宏。
            index += 2
            continue
        if argument.startswith(("-I", "-D", "-isystem", "-mcpu=", "-mfpu=", "-mfloat-abi=", "-mthumb")):
            if argument.startswith("-DMC02_BENCH_"):
                flags.append(argument.split("=", 1)[0] + "=0")  # 保留宏名并关闭裸板诊断。
            elif not (argument.startswith("-D") and argument[2:].startswith(replaced_defines)):
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
    flags.append("-DMC02_VISION_PROTOCOL_" + args.protocol.upper() + "=1")  # 替换构建数据库的协议宏。
    flags.append("-DVISION_USE_" + ("VCP" if args.transport == "usb" else "UART"))  # 只启用一种传输。
    roles = ("ONE_BOARD",) if args.protocol == "v2" else ("ONE_BOARD", "CHASSIS_BOARD", "GIMBAL_BOARD")
    with tempfile.TemporaryDirectory(prefix="mc02-header-config-") as directory:
        checked = 0
        checked_headers = 0
        for heater in (0, 1):
            # 功能选择与底层温控宏保持一致，分别检查两种分支。
            for role in roles:
                ui = int(role != "GIMBAL_BOARD")  # 裁判 UI 不运行在云台板角色。
                variant = re.sub(r"(#define MC02_FEATURE_HEATER) 1\b", rf"\g<1> {heater}", config)
                variant = re.sub(r"(#define MC02_FEATURE_UI) 1\b", rf"\g<1> {ui}", variant)
                (Path(directory) / "mc02_build_config.h").write_text(variant, encoding="utf-8")
                dependencies: set[Path] = set()  # 每种宏组合分别收集其实际包含的头。
                for source_file in files:
                    if args.sources and source_file.name in ("referee_bench.c", "remote_bench.c"):
                        continue  # 裸板专用入口不属于机器人固件。
                    if args.sources and role == "ONE_BOARD" and source_file.name == "robot_can_bridge.c":
                        continue  # 板间桥接只在双板角色中使用。
                    # 测试头目录排在生成目录之前，避免读取实际禁热配置。
                    warnings = [] if args.sources else ["-Wall", "-Wextra", "-Werror"]
                    command = [
                        arguments[0], "-std=c11", *warnings,  # 固件编译器、C11 和适用的告警。
                        "-I" + directory,  # 优先使用临时生成的测试配置。
                        "-DMC02_PROFILE_ROBOT=1", "-D" + role + "=1",  # 指定机器人和板角色。
                        f"-DMC02_HEATER_ENABLED={heater}",  # 与测试配置中的温控选择一致。
                        f"-DMC02_ENABLE_REFEREE_UI={ui}",  # 与测试配置中的 UI 选择一致。
                        *flags,  # 复用经过检查的目录、架构和其他选择。
                        "-fsyntax-only", "-x", "c", "-",  # 只检查标准输入，不输出对象。
                    ]
                    dep_file = Path(directory) / "dependencies.d"  # 单次编译使用固定临时文件。
                    if args.dependencies:
                        # 要求编译器记录该源码实际包含的所有头，并固定依赖目标名。
                        command[-4:-4] = ["-MD", "-MF", str(dep_file), "-MT", "mc02-test"]
                    # 只包含当前源码，避免前一次编译的声明影响结果。
                    result = subprocess.run(command, input=f'#include "{source_file}"\n',
                                            text=True, capture_output=True)
                    if result.returncode:
                        # 保留角色、温控选择和编译器诊断，便于定位失败。
                        raise SystemExit(f"{role} heater={heater}: {source_file.relative_to(root)}\n{result.stderr}")
                    if args.dependencies:
                        # 展开 Make 续行，读取编译器报告的完整包含链。
                        dep_tokens = shlex.split(dep_file.read_text(encoding="utf-8").replace("\\\n", " "))
                        if not dep_tokens or dep_tokens[0] != "mc02-test:":
                            # 目标名不符或内容为空时不能将记录当作当前编译结果。
                            raise SystemExit(f"依赖记录不完整：{source_file.relative_to(root)}")
                        # 从目标名之后逐项检查本次编译的路径。
                        for token in dep_tokens[1:]:
                            candidate = Path(token).resolve()  # 编译器输出的相对路径以仓库为基准。
                            if candidate.suffix != ".h" or not candidate.is_relative_to(root):
                                continue  # 排除源文件和编译器自带头。
                            if candidate.is_relative_to(build):
                                continue  # 实际构建的生成头不属于此测试副本。
                            relative = candidate.relative_to(root)  # 使用仓库相对路径判断所属目录。
                            if relative.parts[0] in ("Core", "Drivers", "Middlewares", "USB_DEVICE"):
                                # 业务实际包含了厂商或系统层头时立即报告。
                                raise SystemExit(f"框架源码包含厂商头：{relative}")
                            dependencies.add(candidate)  # 后面为每份头生成独立翻译单元。
                    checked += 1  # 只有编译及依赖解析成功才计入本组合。
                for header in sorted(dependencies):
                    # 每份头单独包含，避免其他头预先提供缺失的类型。
                    command = [
                        arguments[0], "-std=c11",  # 沿用固件编译器并检查 C11 声明。
                        "-Wall", "-Wextra", "-Werror",  # 独立包含出现告警时报告失败。
                        "-I" + directory,  # 让当前测试配置先于实际构建目录。
                        "-DMC02_PROFILE_ROBOT=1", "-D" + role + "=1",  # 使用相同机器人角色。
                        f"-DMC02_HEATER_ENABLED={heater}",  # 保持源码编译时的温控选择。
                        f"-DMC02_ENABLE_REFEREE_UI={ui}",  # 保持源码编译时的 UI 选择。
                        *flags,  # 使用相同协议、传输、架构和框架目录。
                        "-fsyntax-only", "-x", "c", "-",  # 只检查标准输入中的单个头。
                    ]
                    # 在新的编译器进程中只包含当前头文件。
                    result = subprocess.run(command, input=f'#include "{header}"\n',
                                            text=True, capture_output=True)
                    if result.returncode:
                        # 记录具体头、角色和温控选择，保留编译器诊断。
                        raise SystemExit(f"{role} heater={heater}: {header.relative_to(root)}\n{result.stderr}")
                    checked_headers += 1  # 只有独立编译成功才计数。
    print(f"PASS: {len(files)} 份框架{'源码' if args.sources else '同名头'}，"
          f"{len(roles)}种测试角色和2种温控选择共 {checked} 次 C11 语法检查通过")
    if args.dependencies:
        print(f"PASS: 各组合实际依赖的工程头共 {checked_headers} 次独立 C11 检查通过")


if __name__ == "__main__":
    main()
