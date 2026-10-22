#!/usr/bin/env python3
"""核对生产格式接口的GCC正反例及实际编译命令；只写构建目录，不执行负例或访问硬件。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
HEADERS = '#include "bsp_log.h"\n#include "referee_UI.h"\n#include "oled.h"\n'
UI = 'UICharDraw((String_Data_t *)0,"abc",0u,0u,0u,1u,1u,0u,0u,'


def policy_flags() -> list[str]:
    """从唯一固件 CMake 声明读取自有源码使用的格式检查选项。"""
    cmake = (ROOT / "cmake/mc02_format_diagnostics.cmake").read_text(encoding="utf-8")
    match = re.search(r"set\(MC02_FORMAT_FLAGS\s+([^)]*)\)", cmake)
    if match is None:
        raise RuntimeError("MC02_FORMAT_FLAGS is missing")
    flags = match.group(1).split()
    return flags


def cases() -> list[tuple[str, str, bool, bool]]:
    """正例覆盖类型化日志和仍保留的UI格式接口，负例逐项验证边界。"""
    result = [("valid", 'int example(const char *text) {\n'
               'BSP_LOG_EVENT_1(EV_BSP_USB_BSP_USB_C_01, BSP_LOG_FIELD_U32(2u));\n'
               + UI + '"%s %d",text,2); OLED_printf(0u,0u,"%s %d",text,2); return 0; }', True, False)]
    calls = {
        "legacy_printlog": 'PrintLog("%s",42)',
        "legacy_log_macro": 'LOGINFO("%s",42)',
        "ui_type": UI + '"%s",42)',
        "ui_missing": UI + '"%d")',
        "oled_type": 'OLED_printf(0u,0u,"%s",42)',
        "oled_missing": 'OLED_printf(0u,0u,"%d")',
    }
    for name, call in calls.items():
        result.append((name, f'void example(const char *text) {{ (void)text; {call}; }}', False, False))
    # 长度负例只检查仍保留的OLED或其他有界格式化入口。
    result.extend([
        ("bounded_length", 'int example(void) { char b[8]; return snprintf(b,sizeof(b),"%s","123456789"); }', False, True),
        ("unbounded_length", 'int example(void) { char b[8]; return sprintf(b,"%s","123456789"); }', False, True),
    ])
    return result


def audit_commands(path: Path) -> None:
    """逐条检查自有C源确实有全部选项，厂商C源没有被附加这组策略。"""
    flags = policy_flags()
    own_count, vendor_count = 0, 0
    for entry in json.loads(path.read_text(encoding="utf-8")):
        source = Path(entry["file"])
        if not source.is_absolute():
            source = Path(entry["directory"]) / source
        if source.suffix.lower() != ".c":
            continue
        relative = source.resolve().relative_to(ROOT).as_posix()
        own = relative.startswith(("bsp/", "modules/", "application/", "Core/Src/", "USB_DEVICE/", "tests/",
                                   "docs/history/seasky/"))
        command = entry.get("arguments") or shlex.split(entry["command"], posix=os.name != "nt")
        if own:
            if not all(flag in command for flag in flags):
                raise RuntimeError("format flags missing: " + relative)
            own_count += 1
        else:
            if any(flag in command for flag in flags):
                raise RuntimeError("vendor flags were changed: " + relative)
            vendor_count += 1
    if not own_count or not vendor_count:
        raise RuntimeError("compile commands must include both owned and vendor C sources")
    print(f"Format compile commands: own={own_count}, vendor={vendor_count}; policy verified")


def main() -> int:
    """正例必须成功；反例必须有格式错误诊断，MinGW长度不足另记为覆盖限制。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--compile-commands", type=Path)
    parser.add_argument("--optimization", choices=("-O0", "-Og", "-O2"), default="-O2")
    parser.add_argument("--group", choices=("all", "legacy", "ui_oled", "macros", "length"), default="all")
    args = parser.parse_args()
    if args.compile_commands:
        audit_commands(args.compile_commands)
        return 0
    if not args.compiler or not args.output:
        parser.error("--compiler and --output are required for compile probes")
    compiler = shutil.which(args.compiler)
    if compiler is None:
        raise RuntimeError("compiler not found: " + args.compiler)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, LC_ALL="C", LANG="C")
    # Windows的cc1和汇编器依赖同目录DLL，直接调用gcc绝对路径时也必须提供此目录。
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
    version = subprocess.check_output([compiler, "--version"], env=env, text=True).splitlines()[0]
    machine = subprocess.check_output([compiler, "-dumpmachine"], env=env, text=True).strip()
    mingw = "mingw" in machine.lower()
    flags = policy_flags()
    report = {"compiler": compiler, "version": version, "machine": machine,
              "optimization": args.optimization, "flags": flags, "cases": [], "limitations": []}
    includes = [ROOT / "tests/host/log_fakes", ROOT / "bsp/log",
                ROOT / "modules/referee", ROOT / "modules/oled", ROOT / "bsp/usart"]
    for disabled in (0, 1):
        for name, body, valid, length in cases():
            group = "length" if length else ("legacy" if name.startswith("legacy_") else
                    "ui_oled" if name.startswith(("ui_", "oled_")) else "macros")
            # CTest按组启动，避免一项累计几十次编译而超过10秒；每组仍先编译合法入口。
            if not valid and args.group not in ("all", group):
                continue
            # 关闭宏无运行时输出，长度检查没有意义；类型错误仍须被拒绝。
            if disabled and length:
                continue
            tag = f"{name}_disabled{disabled}"
            source, obj = output / (tag + ".c"), output / (tag + ".o")
            source.write_text(HEADERS + body + "\n", encoding="utf-8")
            command = [compiler, "-std=c11", args.optimization, "-fdiagnostics-color=never",
                       "-fdiagnostics-show-option", "-Werror=implicit-function-declaration",
                       *flags, f"-DDISABLE_LOG_SYSTEM={disabled}"]
            command += ["-I" + str(path) for path in includes]
            command += ["-c", str(source), "-o", str(obj)]
            result = subprocess.run(command, env=env, text=True, encoding="utf-8", errors="replace",
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)
            (output / (tag + ".log")).write_text("$ " + " ".join(command) + "\n" + result.stdout, encoding="utf-8")
            format_error = re.search(r"\[-Werror=format(?:[^\]]*)\]", result.stdout) is not None
            if valid:
                accepted = result.returncode == 0
                outcome = "accepted"
            elif mingw and length and result.returncode == 0:
                accepted = True
                outcome = "length-diagnostic-unavailable"
                report["limitations"].append(tag)
            else:
                if name.startswith("legacy_"):
                    accepted = result.returncode != 0 and "implicit-function-declaration" in result.stdout
                else:
                    accepted = result.returncode != 0 and format_error
                if length:
                    accepted = accepted and ("format-truncation" in result.stdout or "format-overflow" in result.stdout)
                outcome = "rejected-with-format-error"
            report["cases"].append({"name": tag, "result": outcome, "returncode": result.returncode})
            if not accepted:
                raise RuntimeError(f"unexpected compiler result for {tag}:\n{result.stdout}")
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Format checks: {version}; {len(report['cases'])} cases verified; "
          f"length limitations={len(report['limitations'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
