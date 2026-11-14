#!/usr/bin/env python3
"""确认六种生产预设因未确认的核心配置被拒绝，不生成或烧录固件。"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess

import presets


def main() -> int:
    """逐项检查失败原因，工具链故障等其他失败不能算验收通过。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-root", type=Path, default=presets.ROOT / "build" / "wsl")
    args = parser.parse_args()
    for preset in presets.OFFICIAL:
        if preset.profile != "robot":
            continue  # 裸板配置由正常构建流程验收。
        directory = args.build_root / ("rejected-" + preset.name)
        result = subprocess.run(
            ["cmake", "--preset", preset.name, "-B", str(directory)],
            cwd=presets.ROOT, capture_output=True, text=True,
        )  # 保留本次配置输出，以区分预期拒绝和环境故障。
        output = result.stdout + result.stderr
        if result.returncode == 0 or "sentinel core configuration is unconfirmed or unavailable" not in output:
            print(output)  # 失败时交付实际原因，不能将任意非零返回当作通过。
            print(f"FAIL: {preset.name} 未按预期拒绝")
            return 1
        print(f"PASS: {preset.name} 因核心配置未确认而拒绝")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
