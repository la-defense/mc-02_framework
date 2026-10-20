#!/usr/bin/env python3
"""用标准库Decimal核对生产Float2Str；输入为binary32位形，测试不连接硬件。"""

from __future__ import annotations

import argparse
from decimal import Decimal, ROUND_HALF_UP, localcontext
import math
from pathlib import Path
import random
import struct
import subprocess


def float_bits(value: float) -> int:
    """先存为binary32，再取出对应位形，避免把Python的double当作生产输入。"""
    return struct.unpack("<I", struct.pack("<f", value))[0]


def samples() -> list[int]:
    """固定种子加半数、进位、整数范围和极小值附近的位形，便于重复结果。"""
    bits = {0, 0x80000000, 1, 0x80000001, 0x7F7FFFFF, 0xFF7FFFFF,
            0x7F800000, 0xFF800000, 0x7FC00000, 0xFFC00000}
    for value in (2147483648.0, -2147483648.0, 1.0005, 9.9995, 999.9995):
        center = float_bits(value)
        bits.update(range(center - 3, center + 4))
    # 奇数/16的千倍值恰好位于半数；加入相邻位形以检查两侧。
    for numerator in range(-4095, 4096, 2):
        center = float_bits(numerator / 16.0)
        bits.update((center - 1, center, center + 1))
    for numerator in range(-1000, 1001):
        bits.add(float_bits(numerator / 1000.0))
    for exponent in range(-149, 32):
        center = float_bits(2.0 ** exponent)
        bits.update((center - 1, center, center + 1, center | 0x80000000))
    generator = random.Random(0xB508)
    bits.update(generator.getrandbits(32) for _ in range(12000))
    return sorted(bits)


def expected(bits: int) -> str:
    """从float实际数值取得精确十进制值，用独立的十进制舍入规则生成答案。"""
    value = struct.unpack("<f", struct.pack("<I", bits))[0]
    if not math.isfinite(value):
        return "NONFINITE"
    with localcontext() as context:
        context.prec = 100
        number = Decimal.from_float(value)
        minimum, maximum = Decimal(-2147483648), Decimal(2147483647)
        if not minimum <= number <= maximum:
            return "OUT_OF_RANGE"
        rounded = number.quantize(Decimal("0.001"), rounding=ROUND_HALF_UP)
        if not minimum <= rounded <= maximum:
            return "OUT_OF_RANGE"
        if rounded == 0:
            rounded = abs(rounded)
        return f"OK {rounded:f}"


def main() -> int:
    """启动生产入口并逐份核对，首个结果不符时返回失败。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    args = parser.parse_args()
    vectors = samples()
    result = subprocess.run(
        [str(args.executable.resolve()), "reference"],
        input="".join(f"{bits:08x}\n" for bits in vectors),
        text=True, encoding="ascii", capture_output=True, timeout=8, check=False,
    )
    if result.returncode:
        raise RuntimeError(f"生产测试入口失败({result.returncode}): {result.stderr}")
    actual = result.stdout.splitlines()
    if len(actual) != len(vectors):
        raise AssertionError(f"结果数量不符: {len(actual)} != {len(vectors)}")
    for bits, output in zip(vectors, actual):
        reference = expected(bits)
        if output != reference:
            raise AssertionError(f"0x{bits:08x}: {output!r} != {reference!r}")
    print(f"PASS: {len(vectors)} binary32 vectors match Decimal (seed=0xB508)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
