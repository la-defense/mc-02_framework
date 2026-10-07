#!/usr/bin/env python3
"""Take bounded RAM snapshots without attaching GDB to the running target.

GDB is used offline to resolve ELF symbol addresses only. Live reads use the
loopback OpenOCD Telnet port configured by tools/openocd_running_inspection.cfg
and are restricted to target-state queries and single-byte/word RAM reads.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import re
import socket
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ELF = ROOT / "build" / "bench-safe" / "Basic_Framework_MC02_bench_safe.elf"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_TELNET_PORT = 4444
EXPRESSIONS = {
    "tick_ms": "uwTick",
    "vision_mode": "recv_data.mode",
    "vision_rx_count": "vision_rx_count",
    "vision_tx_count": "vision_tx_count",
    "vision_crc_error_count": "vision_crc_error_count",
    "iwdg_reset_flag": "iwdg_reset_flag",
}
COUNTERS = ("vision_rx_count", "vision_tx_count", "vision_crc_error_count")
READ_COMMANDS = {
    "tick_ms": "mdw",
    "vision_mode": "mdb",
    "vision_rx_count": "mdw",
    "vision_tx_count": "mdw",
    "vision_crc_error_count": "mdw",
    "iwdg_reset_flag": "mdb",
}
SYMBOL_VALUE_PATTERN = re.compile(r"^(?P<key>[a-z_]+)=0x(?P<value>[0-9a-fA-F]+)$")
MEMORY_VALUE_PATTERN = re.compile(
    r"^\s*0x(?P<address>[0-9a-fA-F]+):\s*(?:0x)?(?P<value>[0-9a-fA-F]+)\b"
)
MEMORY_COMMAND_PATTERN = re.compile(r"(?:mdb|mdw) 0x[0-9a-f]{1,8} 1\Z")
ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
MAX_RESPONSE_BYTES = 65536
UINT32_MAX = 0xFFFFFFFF


class ProbeIOError(RuntimeError):
    """The OpenOCD connection, state check, or bounded memory read failed."""


def build_symbol_command(gdb: str, elf: str) -> list[str]:
    """Build an offline GDB invocation; it must never connect to a target."""
    commands = ["set pagination off", "set confirm off"]
    commands.extend(
        f'printf "{key}=0x%x\\n", (unsigned int)&({expression})'
        for key, expression in EXPRESSIONS.items()
    )
    argv = [gdb, "--nx", "--quiet", "--batch", "--symbols", elf]
    for command in commands:
        argv.extend(("--ex", command))
    return argv


def parse_addresses(output: str) -> dict[str, int]:
    addresses: dict[str, int] = {}
    for line in output.splitlines():
        match = SYMBOL_VALUE_PATTERN.fullmatch(line.strip())
        if match and match.group("key") in EXPRESSIONS:
            addresses[match.group("key")] = int(match.group("value"), 16)

    missing = set(EXPRESSIONS) - addresses.keys()
    if missing:
        raise ProbeIOError(f"ELF symbol lookup is incomplete; missing: {', '.join(sorted(missing))}")
    for key, address in addresses.items():
        if not 0 < address <= UINT32_MAX:
            raise ProbeIOError(f"ELF symbol {key} has an invalid address: 0x{address:x}")
    return addresses


def resolve_symbol_addresses(gdb: str, elf: str, timeout: float) -> dict[str, int]:
    try:
        result = subprocess.run(
            build_symbol_command(gdb, elf),
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise ProbeIOError(f"offline ELF symbol lookup timed out after {timeout:g}s") from error
    except OSError as error:
        raise ProbeIOError(f"could not start offline GDB: {error}") from error

    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip().splitlines()
        message = detail[-1] if detail else f"GDB exited with status {result.returncode}"
        raise ProbeIOError(f"offline ELF symbol lookup failed: {message}")
    return parse_addresses(result.stdout + "\n" + result.stderr)


def _validate_loopback_host(host: str) -> None:
    try:
        address = ipaddress.ip_address(host)
    except ValueError as error:
        raise ValueError("host must be a loopback IP address") from error
    if not address.is_loopback:
        raise ValueError("host must be a loopback IP address")


def _validate_port(port: int) -> None:
    if not 1 <= port <= 65535:
        raise ValueError("Telnet port must be between 1 and 65535")


def _validated_address(address: int, alignment: int) -> int:
    if isinstance(address, bool) or not isinstance(address, int):
        raise ValueError("symbol addresses must be integers")
    if not 0 < address <= UINT32_MAX:
        raise ValueError("symbol address is outside the 32-bit address space")
    if address % alignment:
        raise ValueError(f"symbol address 0x{address:x} is not {alignment}-byte aligned")
    return address


def _build_read_command(key: str, address: int) -> str:
    if key not in READ_COMMANDS:
        raise ValueError(f"unsupported snapshot field: {key}")
    alignment = 4 if READ_COMMANDS[key] == "mdw" else 1
    validated = _validated_address(address, alignment)
    return f"{READ_COMMANDS[key]} 0x{validated:08x} 1"


def _parse_memory_value(output: str, expected_address: int, width_bits: int) -> int:
    for line in output.splitlines():
        match = MEMORY_VALUE_PATTERN.match(line)
        if not match:
            continue
        address = int(match.group("address"), 16)
        value = int(match.group("value"), 16)
        if address != expected_address:
            continue
        if value >= 1 << width_bits:
            raise ProbeIOError(f"OpenOCD returned an oversized value for 0x{address:x}")
        return value
    raise ProbeIOError(f"OpenOCD returned no value for RAM address 0x{expected_address:x}")


def _require_running(output: str, when: str) -> None:
    for line in output.splitlines():
        if re.search(r"\bmc02\.cpu0\b", line):
            state = line.strip().split()[-1].lower()
            if state == "running":
                return
            raise ProbeIOError(f"mc02.cpu0 is {state} {when}; snapshot aborted without resuming it")
    raise ProbeIOError(f"OpenOCD did not report mc02.cpu0 state {when}")


class OpenOCDTelnetClient:
    """Minimal bounded Telnet client with a strict OpenOCD command allowlist."""

    def __init__(self, host: str, port: int, timeout: float):
        _validate_loopback_host(host)
        _validate_port(port)
        if not 0 < timeout <= 5.0:
            raise ValueError("timeout must be greater than 0 and at most 5 seconds")
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket: socket.socket | None = None
        self._wire = bytearray()
        self._deadline = 0.0

    def __enter__(self) -> OpenOCDTelnetClient:
        self._deadline = time.monotonic() + self.timeout
        try:
            self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
        except OSError as error:
            raise ProbeIOError(f"could not connect to local OpenOCD Telnet port: {error}") from error
        try:
            self._read_until_prompt()
        except Exception:
            self.close()
            raise
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None

    def command(self, command: str) -> str:
        if command != "targets" and not MEMORY_COMMAND_PATTERN.fullmatch(command):
            raise ValueError("only targets, mdb, and mdw single-value reads are allowed")
        if self._socket is None:
            raise ProbeIOError("OpenOCD Telnet connection is closed")
        try:
            self._socket.sendall(command.encode("ascii") + b"\n")
        except OSError as error:
            raise ProbeIOError(f"OpenOCD Telnet write failed: {error}") from error
        return self._read_until_prompt()

    def _consume_telnet(self) -> bytes:
        output = bytearray()
        index = 0
        while index < len(self._wire):
            value = self._wire[index]
            if value != 0xFF:
                output.append(value)
                index += 1
                continue
            if index + 1 >= len(self._wire):
                break
            option = self._wire[index + 1]
            if option in (0xFB, 0xFC, 0xFD, 0xFE):
                if index + 2 >= len(self._wire):
                    break
                if self._socket is not None:
                    if option == 0xFB:  # WILL -> DONT
                        self._socket.sendall(bytes((0xFF, 0xFE, self._wire[index + 2])))
                    elif option == 0xFD:  # DO -> WONT
                        self._socket.sendall(bytes((0xFF, 0xFC, self._wire[index + 2])))
                index += 3
                continue
            if option == 0xFA:  # subnegotiation; discard through IAC SE
                terminator = self._wire.find(b"\xff\xf0", index + 2)
                if terminator < 0:
                    break
                index = terminator + 2
                continue
            index += 2
        del self._wire[:index]
        return bytes(output)

    def _read_until_prompt(self) -> str:
        if self._socket is None:
            raise ProbeIOError("OpenOCD Telnet connection is closed")
        output = bytearray()
        while True:
            remaining = self._deadline - time.monotonic()
            if remaining <= 0:
                raise ProbeIOError(f"OpenOCD Telnet read timed out after {self.timeout:g}s")
            self._socket.settimeout(remaining)
            try:
                chunk = self._socket.recv(4096)
            except socket.timeout as error:
                raise ProbeIOError(f"OpenOCD Telnet read timed out after {self.timeout:g}s") from error
            except OSError as error:
                raise ProbeIOError(f"OpenOCD Telnet read failed: {error}") from error
            if not chunk:
                raise ProbeIOError("OpenOCD closed the Telnet connection before its prompt")
            self._wire.extend(chunk)
            output.extend(self._consume_telnet())
            if len(output) > MAX_RESPONSE_BYTES:
                raise ProbeIOError("OpenOCD Telnet response exceeded the size limit")
            text = output.decode("utf-8", errors="replace")
            text = ANSI_ESCAPE_PATTERN.sub("", text)
            text = "".join(char for char in text if char in "\r\n\t" or ord(char) >= 32)
            if text.rstrip().endswith(">"):
                return text


def read_snapshot(addresses: dict[str, int], host: str, port: int, timeout: float) -> dict[str, int]:
    missing = set(EXPRESSIONS) - addresses.keys()
    if missing:
        raise ProbeIOError(f"snapshot addresses are incomplete; missing: {', '.join(sorted(missing))}")

    snapshot: dict[str, int] = {}
    try:
        with OpenOCDTelnetClient(host, port, timeout) as client:
            _require_running(client.command("targets"), "before the snapshot")
            for key, address in addresses.items():
                command = _build_read_command(key, address)
                output = client.command(command)
                width = 32 if READ_COMMANDS[key] == "mdw" else 8
                snapshot[key] = _parse_memory_value(output, address, width)
                _require_running(client.command("targets"), f"after reading {key}")
    except (OSError, ValueError) as error:
        if isinstance(error, ProbeIOError):
            raise
        raise ProbeIOError(str(error)) from error
    return snapshot


def _small_decrease(old: int, new: int) -> bool:
    return new < old and old - new < 0x80000000


def classify_change(previous: dict[str, int], current: dict[str, int]) -> str:
    if not previous["iwdg_reset_flag"] and current["iwdg_reset_flag"]:
        return "target-reset-iwdg"
    if _small_decrease(previous["tick_ms"], current["tick_ms"]):
        if current["iwdg_reset_flag"]:
            return "target-reset-iwdg"
        return "target-reset-other-or-unknown"
    if any(_small_decrease(previous[key], current[key]) for key in COUNTERS):
        return "counter-decrease-unclassified"
    return "running"


def _bounded_float(value: float, name: str, minimum: float, maximum: float) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum:g} and {maximum:g}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gdb", default="arm-none-eabi-gdb", help="offline ELF symbol reader")
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    parser.add_argument("--host", default=DEFAULT_HOST, help="must resolve to loopback")
    parser.add_argument("--telnet-port", type=int, default=DEFAULT_TELNET_PORT)
    parser.add_argument("--samples", type=int, default=5, help="2..8 snapshots")
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between reads, 0..2")
    parser.add_argument("--timeout", type=float, default=5.0, help="per snapshot timeout, >0..5 seconds")
    args = parser.parse_args(argv)

    if not 2 <= args.samples <= 8:
        parser.error("--samples must be between 2 and 8")
    try:
        _validate_loopback_host(args.host)
        _validate_port(args.telnet_port)
        _bounded_float(args.interval, "--interval", 0.0, 2.0)
        _bounded_float(args.timeout, "--timeout", 0.1, 5.0)
    except ValueError as error:
        parser.error(str(error))
    if not args.elf.is_file():
        parser.error(f"ELF file does not exist: {args.elf}")

    try:
        addresses = resolve_symbol_addresses(args.gdb, str(args.elf), args.timeout)
    except ProbeIOError as error:
        print(f"probe_io_error: {error}", file=sys.stderr)
        return 2

    previous = None
    try:
        for index in range(args.samples):
            current = read_snapshot(addresses, args.host, args.telnet_port, args.timeout)
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
