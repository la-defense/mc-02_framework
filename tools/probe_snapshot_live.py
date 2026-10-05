#!/usr/bin/env python3
"""Read selected MC-02 counters over a running OpenOCD GDB server.

The OpenOCD server must use tools/openocd_running_inspection.cfg. This tool
does not issue target halt, reset, resume, step, or write commands. OpenOCD and
GDB operations have strict timeouts; only global RAM values from the ELF are
read.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ELF = ROOT / "build" / "bench-safe" / "Basic_Framework_MC02_bench_safe.elf"
EXPRESSIONS = {
    "tick_ms": "uwTick",
    "vision_mode": "recv_data.mode",
    "vision_rx_count": "vision_rx_count",
    "vision_tx_count": "vision_tx_count",
    "vision_crc_error_count": "vision_crc_error_count",
    "iwdg_reset_flag": "iwdg_reset_flag",
}
COUNTERS = ("vision_rx_count", "vision_tx_count", "vision_crc_error_count")
HOST_PATTERN = re.compile(r"[A-Za-z0-9.-]+\Z")
VALUE_PATTERN = re.compile(r"^(?P<key>[a-z_]+)=(?P<value>0x[0-9a-fA-F]+|[0-9]+)$")


class ProbeIOError(RuntimeError):
    """The GDB/OpenOCD connection or bounded memory read failed."""


def build_gdb_command(gdb: str, elf: str, host: str, port: int) -> list[str]:
    if not HOST_PATTERN.fullmatch(host):
        raise ValueError("host must be a hostname or IPv4 address")
    if not 1 <= port <= 65535:
        raise ValueError("port must be between 1 and 65535")

    commands = [
        "set pagination off",
        "set confirm off",
        "set remote interrupt-on-connect off",
        f"target extended-remote {host}:{port}",
    ]
    commands.extend(
        f'printf "{key}=%u\\n", (unsigned int)({expression})'
        for key, expression in EXPRESSIONS.items()
    )

    argv = [gdb, "--nx", "--quiet", "--batch", "--return-child-result", "--symbols", elf]
    for command in commands:
        argv.extend(("--ex", command))
    return argv


def parse_snapshot(output: str) -> dict[str, int]:
    values: dict[str, int] = {}
    for line in output.splitlines():
        match = VALUE_PATTERN.fullmatch(line.strip())
        if match and match.group("key") in EXPRESSIONS:
            values[match.group("key")] = int(match.group("value"), 0)

    missing = set(EXPRESSIONS) - values.keys()
    if missing:
        raise ProbeIOError(f"GDB snapshot is incomplete; missing: {', '.join(sorted(missing))}")
    return values


def read_snapshot(gdb: str, elf: str, host: str, port: int, timeout: float) -> dict[str, int]:
    command = build_gdb_command(gdb, elf, host, port)
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise ProbeIOError(f"GDB memory read timed out after {timeout:g}s") from error
    except OSError as error:
        raise ProbeIOError(f"could not start GDB: {error}") from error

    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip().splitlines()
        message = detail[-1] if detail else f"GDB exited with status {result.returncode}"
        raise ProbeIOError(f"GDB/OpenOCD probe I/O error: {message}")
    return parse_snapshot(result.stdout + "\n" + result.stderr)


def _small_decrease(old: int, new: int) -> bool:
    return new < old and old - new < 0x80000000


def classify_change(previous: dict[str, int], current: dict[str, int]) -> str:
    if _small_decrease(previous["tick_ms"], current["tick_ms"]):
        if current["iwdg_reset_flag"]:
            return "target-reset-iwdg"
        return "target-reset-other-or-unknown"
    if any(_small_decrease(previous[key], current[key]) for key in COUNTERS):
        return "counter-cleared"
    return "running"


def _bounded_float(value: float, name: str, minimum: float, maximum: float) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum:g} and {maximum:g}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gdb", default="arm-none-eabi-gdb")
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--samples", type=int, default=5, help="2..8 snapshots")
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between reads, 0..2")
    parser.add_argument("--timeout", type=float, default=5.0, help="per-read timeout, >0..5 seconds")
    args = parser.parse_args(argv)

    if not 2 <= args.samples <= 8:
        parser.error("--samples must be between 2 and 8")
    try:
        _bounded_float(args.interval, "--interval", 0.0, 2.0)
        _bounded_float(args.timeout, "--timeout", 0.1, 5.0)
        build_gdb_command(args.gdb, str(args.elf), args.host, args.port)
    except ValueError as error:
        parser.error(str(error))
    if not args.elf.is_file():
        parser.error(f"ELF file does not exist: {args.elf}")

    previous = None
    try:
        for index in range(args.samples):
            current = read_snapshot(args.gdb, str(args.elf), args.host, args.port, args.timeout)
            status = "baseline" if previous is None else classify_change(previous, current)
            print(json.dumps({"sample": index + 1, "status": status, **current}, sort_keys=True))
            previous = current
            if index + 1 < args.samples and args.interval > 0.0:
                time.sleep(args.interval)
    except ProbeIOError as error:
        print(f"probe_io_error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
