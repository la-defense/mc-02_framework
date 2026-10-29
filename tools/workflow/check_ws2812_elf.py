#!/usr/bin/env python3
"""核对 H723 固件的 WS2812 SRAM4 缓冲和 SPI6／BDMA 中断向量。

用法：python3 tools/workflow/check_ws2812_elf.py <firmware.elf>。
依赖 PATH 中的 arm-none-eabi-nm 与 arm-none-eabi-objcopy；只读 ELF，不连接硬件。
"""

from __future__ import annotations

import argparse
import struct
import subprocess
import tempfile
from pathlib import Path


def symbols(elf: Path) -> dict[str, int]:
    """读取已定义符号，缺少交叉工具或 ELF 无效时让调用方报告失败。"""
    result = subprocess.run(
        ["arm-none-eabi-nm", "--defined-only", str(elf)],
        check=True, capture_output=True, text=True, timeout=15,
    )
    found: dict[str, int] = {}
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) == 3:
            try:
                found[fields[2]] = int(fields[0], 16)
            except ValueError:
                continue
    return found


def check(elf: Path) -> None:
    """检查固定缓冲和两个 IRQ 的真实向量，不把符号存在当成向量已接通。"""
    names = symbols(elf)
    required = (
        "__ws2812_dma_start__", "__ws2812_dma_end__", "ws2812_dma_buffer",
        "SPI6_IRQHandler", "BDMA_Channel0_IRQHandler", "Default_Handler",
    )
    missing = [name for name in required if name not in names]
    if missing:
        raise ValueError(f"缺少符号: {', '.join(missing)}")
    start = names["__ws2812_dma_start__"]
    end = names["__ws2812_dma_end__"]
    if start != names["ws2812_dma_buffer"] or start % 32 or end - start != 128:
        raise ValueError("WS2812 缓冲地址、对齐或容量不符合约定")
    if not (0x38000000 <= start < end <= 0x38004000):
        raise ValueError("WS2812 缓冲不在 H723 SRAM4 中")

    with tempfile.TemporaryDirectory(prefix="mc02-ws2812-vector-") as directory:
        image = Path(directory) / "vector.bin"
        subprocess.run(
            ["arm-none-eabi-objcopy", "-O", "binary", "-j", ".isr_vector", str(elf), str(image)],
            check=True, capture_output=True, timeout=15,
        )
        data = image.read_bytes()
    for name, irq in (("SPI6_IRQHandler", 86), ("BDMA_Channel0_IRQHandler", 129)):
        offset = (16 + irq) * 4
        if len(data) < offset + 4:
            raise ValueError(f"中断向量表过短: {name}")
        vector = struct.unpack_from("<I", data, offset)[0]
        address = names[name]
        if (address & ~1) == (names["Default_Handler"] & ~1) or vector != (address | 1):
            raise ValueError(f"中断向量未指向实际处理函数: {name}")
    print(f"{elf}: SRAM4 {start:#010x}–{end:#010x}；SPI6／BDMA0 向量有效")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    args = parser.parse_args()
    try:
        check(args.elf)
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        print(f"WS2812 ELF 检查失败：{exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
