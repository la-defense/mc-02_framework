#!/usr/bin/env python3
"""逐份独立编译当前固件框架对象实际依赖的工程头文件。"""

import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

from check_firmware_profile import check_portable_includes


def fail(message: str) -> None:
    """输出具体缺失项并使检查失败。"""
    raise SystemExit(message)


def is_selected_header(candidate: Path, root: Path, build: Path) -> bool:
    """接纳源码头和构建生成头；其他构建产物及系统头不属于本次检查。"""
    if candidate.is_relative_to(build / "generated"):
        return True  # 构建目录在仓库外时也须检查生成配置头。
    return candidate.is_relative_to(root) and not candidate.is_relative_to(build)


def display_header(header: Path, root: Path, build: Path) -> str:
    """生成可读名称，支持源码树及仓库外的构建目录。"""
    if header.is_relative_to(root):
        return str(header.relative_to(root))
    return f"<build>/{header.relative_to(build)}"


def main() -> None:
    """读取实际编译参数与 Ninja 依赖，不为检查添加厂商头目录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True, help="已构建的 Ninja 固件目录")
    args = parser.parse_args()
    root = Path.cwd().resolve()
    build = args.build_dir.resolve()
    database = build / "compile_commands.json"
    if not database.is_file():
        fail("缺少编译数据库，请先配置并构建固件")
    entries = json.loads(database.read_text(encoding="utf-8"))
    portable = [entry for entry in entries if "mc02_portable_framework.dir/" in entry.get("command", "")]
    if not portable:
        fail("编译数据库没有找到框架对象")

    # 每份框架源码都先校验头路径，避免复用包含厂商目录的命令。
    expected_objects: set[str] = set()
    for entry in portable:
        check_portable_includes(entry, root, entry["file"])
        tokens = shlex.split(entry["command"])
        if "-o" not in tokens or tokens.index("-o") + 1 >= len(tokens):
            fail(f"框架编译命令缺少对象路径：{entry['file']}")
        expected_objects.add(tokens[tokens.index("-o") + 1].replace("\\", "/"))
    if len(expected_objects) != len(portable):
        fail("框架编译数据库存在重复对象路径")
    arguments = shlex.split(portable[0]["command"])
    compiler = arguments[0]  # 使用该固件实际选用的交叉编译器。
    allowed_flags = (
        "-I",  # 业务和框架头目录，事先检查过没有厂商头路径。
        "-D",  # 当前车型和功能选择的预处理宏。
        "-isystem",  # 经同一规则校验的系统头目录。
        "-mcpu=",  # 与固件一致的 CPU 架构。
        "-mfpu=",  # 与固件一致的浮点单元类型。
        "-mfloat-abi=",  # 与固件一致的浮点调用约定。
        "-mthumb",  # 使用当前 ARM 固件的 Thumb 指令集。
    )
    flags: list[str] = []
    index = 1
    while index < len(arguments):
        argument = arguments[index]  # 检查实际编译命令中的一个参数。
        if argument in ("-I", "-D", "-isystem"):
            if index + 1 >= len(arguments):
                fail(f"编译参数缺少取值：{argument}")
            flags.extend((argument, arguments[index + 1]))  # 单独传值时保留选项和值。
            index += 2
            continue
        if argument.startswith(allowed_flags):
            flags.append(argument)  # 保留合并写法和目标架构选项。
        index += 1

    # 从 Ninja 的实际依赖图收集已选框架对象的项目头，而非猜测哪些头被使用。
    result = subprocess.run(["ninja", "-C", str(build), "-t", "deps"],
                            capture_output=True, text=True, check=True)
    headers: set[Path] = set()
    objects: set[str] = set()
    active = False
    for line in result.stdout.splitlines():
        if line and not line[0].isspace():
            active = "CMakeFiles/mc02_portable_framework.dir/" in line  # 只处理框架对象。
            if active:
                if "(VALID)" not in line:
                    fail(f"框架对象的依赖记录无效：{line}")  # 不能用过期记录证明头依赖。
                objects.add(line.split(":", 1)[0])  # 保存对象名，后面与数据库比较。
            continue  # 对象记录本身不是头文件。
        if not active:
            continue  # 忽略板级和第三方对象的依赖。
        candidate = Path(line.strip())  # 去除 Ninja 输出的缩进。
        if candidate.suffix != ".h":
            continue  # 源码文件不属于独立头检查。
        candidate = (candidate if candidate.is_absolute() else build / candidate).resolve()  # 统一相对路径。
        if not is_selected_header(candidate, root, build):
            continue  # 排除编译器系统头及无关构建产物。
        if candidate.is_relative_to(root) and not candidate.is_relative_to(build / "generated"):
            relative = candidate.relative_to(root)  # 源码树中的头须检查所属目录。
            if relative.parts[0] in ("Core", "Drivers", "Middlewares", "USB_DEVICE"):
                fail(f"框架对象依赖了厂商或系统头：{relative}")
        headers.add(candidate)  # 去重后逐份编译源码头和生成头。
    if objects != expected_objects or not headers:
        fail(f"框架对象依赖不匹配：缺失 {sorted(expected_objects - objects)}，"
             f"多出 {sorted(objects - expected_objects)}，头文件 {len(headers)}")

    # 每次只包含一份头，避免前一个头偶然提供类型或宏；失败时报告编译器原始诊断。
    for header in sorted(headers):
        command = [
            compiler, "-std=c11",  # 使用固件编译器和标准 C11。
            "-Wall", "-Wextra", "-Werror",  # 将独立包含产生的告警作为失败。
            *flags,  # 复用已检查的框架路径、宏和架构选项。
            "-fsyntax-only", "-x", "c", "-",  # 从标准输入检查单份头，不生成对象。
        ]
        result = subprocess.run(command, input=f'#include "{header}"\n',
                                capture_output=True, text=True)
        if result.returncode:
            sys.stderr.write(result.stderr)
            fail(f"公开头不能独立编译：{display_header(header, root, build)}")
    print(f"PASS: {len(headers)} 份已选框架头独立通过 C11 编译；对象数 {len(objects)}")


if __name__ == "__main__":
    main()
