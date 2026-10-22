#!/usr/bin/env python3
"""使用 OpenOCD 原生 RTT 读取日志，不发送 RTT 下行数据。

按指定 ELF 解析 `_SEGGER_RTT` 的地址和大小，只在匹配符号范围内查找控制块。
脚本独立启动 OpenOCD；停止已有调试会话后再使用，避免多个进程争用 DAPLink。
"""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time
from pathlib import Path

from probe_snapshot_live import ProbeIOError, _read_elf_symbol


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ELF = ROOT / "build" / "wsl" / "bench-debug" / "Basic_Framework_MC02_bench_safe.elf"
RTT_RAM_START = 0x24000000
RTT_RAM_END = 0x24020000
RTT_ID = "SEGGER RTT"


def build_openocd_arguments(openocd: str, elf: Path, port: int) -> list[str]:
    """从 ELF 取得 RTT 控制块范围，并构造只读观测配置的启动参数。"""
    address, size, _ = _read_elf_symbol(str(elf), "_SEGGER_RTT")
    if size < 48 or size > 4096:
        raise ProbeIOError("ELF RTT control block size is outside the supported range")
    if address < RTT_RAM_START or address + size > RTT_RAM_END:
        raise ProbeIOError("ELF RTT control block is outside the approved main RAM")
    config = ROOT / "tools" / "openocd_running_inspection.cfg"
    setup = f'rtt setup 0x{address:08x} {size} "{RTT_ID}"'
    return [
        openocd,
        "-f", str(config),
        "-c", setup,
        "-c", "rtt start",
        "-c", f"rtt server start {port} 0",
    ]


def _validate_options(port: int, seconds: float, timeout: float) -> None:
    if not 1 <= port <= 65535:
        raise ValueError("RTT server port must be between 1 and 65535")
    if not 1 <= seconds <= 3600:
        raise ValueError("observation duration must be between 1 and 3600 seconds")
    if not 0.1 <= timeout <= 30:
        raise ValueError("startup timeout must be between 0.1 and 30 seconds")


def receive_only(client, seconds: float, outputs: tuple) -> None:
    """只从已连接的 RTT 上行端口接收字节；不向套接字发送数据。"""
    started = time.monotonic()
    deadline = started + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        client.settimeout(min(0.5, remaining))
        try:
            data = client.recv(2048)
        except socket.timeout:
            continue
        if not data:
            raise ProbeIOError("OpenOCD RTT server closed the receive-only connection")
        for output in outputs:
            output.write(data)
            output.flush()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--openocd", required=True, help="OpenOCD executable with STM32H7 RTT support")
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    parser.add_argument("--port", type=int, default=19021, help="local RTT channel 0 TCP port")
    parser.add_argument("--seconds", type=float, default=60.0, help="receive window, 1..3600 seconds")
    parser.add_argument("--startup-timeout", type=float, default=10.0, help="OpenOCD RTT server startup limit")
    parser.add_argument("--log", type=Path, help="optional raw RTT byte log")
    args = parser.parse_args(argv)
    try:
        _validate_options(args.port, args.seconds, args.startup_timeout)
    except ValueError as error:
        parser.error(str(error))
    if not args.elf.is_file():
        parser.error(f"ELF file does not exist: {args.elf}")

    process = None
    log = None
    try:
        openocd_args = build_openocd_arguments(args.openocd, args.elf, args.port)
        if args.log is not None:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log = args.log.open("wb")
        process = subprocess.Popen(openocd_args, cwd=ROOT, stdout=sys.stderr, stderr=sys.stderr)
        startup_deadline = time.monotonic() + args.startup_timeout
        client = None
        while time.monotonic() < startup_deadline:
            if process.poll() is not None:
                raise ProbeIOError(f"OpenOCD exited during RTT startup with status {process.returncode}")
            try:
                client = socket.create_connection(("127.0.0.1", args.port), timeout=0.2)
                break
            except OSError:
                time.sleep(0.1)
        if client is None:
            raise ProbeIOError("OpenOCD RTT server did not start before the timeout")

        with client:
            outputs = (sys.stdout.buffer,) if log is None else (sys.stdout.buffer, log)
            receive_only(client, args.seconds, outputs)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ProbeIOError) as error:
        print(f"rtt_monitor_error: {error}", file=sys.stderr)
        return 2
    finally:
        if log is not None:
            log.close()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
