#!/usr/bin/env python3
"""给匹配的bench_safe诊断固件写入一份固定地址、一次性的崩溃自测许可。

仅向ELF中的bsp_crash_test_permit写入十个固定32位字；不复位、不停核，
并先核对Build ID、CMake配置和目标上的观测身份。该工具不接受任意地址或命令。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import argparse
import secrets
import socket
import struct
import sys
import time
import zlib
from pathlib import Path


PERMIT_MAGIC = 0x4D524550
PERMIT_CONSUMED = 0x4D4F4E45
PERMIT_VERSION = 1
PERMIT_WORDS = 10
PERMIT_SIZE = PERMIT_WORDS * 4
CRASH_RECORD_START = 0x24000000
CRASH_RECORD_SIZE = 336
CRASH_REGION_SIZE = 512
OBSERVATION_MAGIC = 0x4F425331
OBSERVATION_VERSION = 1
OBSERVATION_SIZE = 48
OBSERVATION_LAYOUT = 0x4F425301
OBSERVATION_RAM_START = 0x24000000
OBSERVATION_RAM_END = 0x24020000
CASE_VALUES = {"udf": 1, "assert": 2, "stack-hook": 3}


class PermitError(ValueError):
    """输入产物、目标身份或OpenOCD响应不符合固定写入流程。"""


def elf_build_id(elf_bytes: bytes) -> bytes:
    if len(elf_bytes) < 52 or elf_bytes[:4] != b"\x7fELF" or elf_bytes[4] != 1 or elf_bytes[5] != 1:
        raise PermitError("只接受小端ELF32固件")
    if struct.unpack_from("<H", elf_bytes, 18)[0] != 40:
        raise PermitError("ELF机器类型不是ARM")
    note_header = struct.pack("<III", 4, 20, 3) + b"GNU\0"
    starts = []
    index = 0
    while True:
        index = elf_bytes.find(note_header, index)
        if index < 0:
            break
        build_id = elf_bytes[index + len(note_header):index + len(note_header) + 20]
        if len(build_id) == 20:
            starts.append(build_id)
        index += 1
    if len(starts) != 1:
        raise PermitError("ELF中必须且只能包含一个20字节GNU Build ID note")
    return starts[0]


def elf_symbols(elf_bytes: bytes, requested: set[str]) -> dict[str, tuple[int, int]]:
    """只解析ELF32静态/动态符号表，按符号名返回地址和声明大小。"""
    if len(elf_bytes) < 52 or elf_bytes[:4] != b"\x7fELF" or elf_bytes[4:6] != b"\x01\x01":
        raise PermitError("只接受小端ELF32固件")
    shoff = struct.unpack_from("<I", elf_bytes, 32)[0]
    shentsize, shnum = struct.unpack_from("<HH", elf_bytes, 46)
    if shentsize != 40 or shnum == 0 or shoff + shentsize * shnum > len(elf_bytes):
        raise PermitError("ELF节表无效")
    sections = [struct.unpack_from("<IIIIIIIIII", elf_bytes, shoff + index * shentsize)
                for index in range(shnum)]
    found: dict[str, set[tuple[int, int]]] = {name: set() for name in requested}
    for section in sections:
        section_type, section_offset, section_size, link, entry_size = (
            section[1], section[4], section[5], section[6], section[9]
        )
        if section_type not in (2, 11):
            continue
        if link >= len(sections) or entry_size != 16 or section_size % entry_size:
            raise PermitError("ELF符号表布局无效")
        strings_section = sections[link]
        strings = elf_bytes[strings_section[4]:strings_section[4] + strings_section[5]]
        if len(strings) != strings_section[5]:
            raise PermitError("ELF字符串表越界")
        for offset in range(section_offset, section_offset + section_size, entry_size):
            name_offset, value, size = struct.unpack_from("<III", elf_bytes, offset)
            if name_offset >= len(strings):
                raise PermitError("ELF符号名偏移无效")
            end = strings.find(b"\0", name_offset)
            if end < 0:
                raise PermitError("ELF符号名没有结束标记")
            name = strings[name_offset:end].decode("ascii", errors="replace")
            if name in found:
                found[name].add((value, size))
    result = {}
    for name, matches in found.items():
        if len(matches) != 1:
            raise PermitError(f"ELF中{ name }符号缺失或地址不唯一")
        result[name] = next(iter(matches))
    return result


def check_cache(cache_path: Path) -> None:
    values = {}
    for line in cache_path.read_text(encoding="utf-8", errors="strict").splitlines():
        if line.startswith("//") or line.startswith("#") or ":" not in line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.split(":", 1)[0]] = value
    expected = {
        "MC02_PROFILE": "bench_safe",
        "MC02_BOARD_ROLE": "single",
        "MC02_ENABLE_REFEREE_UI": "OFF",
        "MC02_BENCH_CRASH_DIAGNOSTICS": "ON",
        "MC02_ENABLE_HEATER": "OFF",
        "MC02_BENCH_RC_DIAGNOSTICS": "OFF",
        "MC02_BENCH_REFEREE_DIAGNOSTICS": "OFF",
        "MC02_LOG_TEST": "OFF",
        "MC02_LOG_ORDINARY": "OFF",
    }
    for key, value in expected.items():
        if values.get(key) != value:
            raise PermitError(f"CMake配置{key}必须为{value}")


def build_permit(build_id: bytes, case_value: int, nonce: int | None = None) -> bytes:
    if len(build_id) != 20 or case_value not in CASE_VALUES.values():
        raise PermitError("Build ID或自测类型无效")
    if nonce is None:
        nonce = secrets.randbits(32) or 1
    words = struct.unpack("<5I", build_id)
    payload = struct.pack("<II5II", PERMIT_VERSION, case_value, *words, nonce)
    checksum = zlib.crc32(payload) & 0xFFFFFFFF
    return struct.pack("<I", PERMIT_MAGIC) + payload + struct.pack("<I", checksum)


def decode_openocd_words(output: str, start_address: int, count: int) -> list[int]:
    values = []
    for line in output.replace("\r", "").splitlines():
        line = line.strip()
        if line.endswith(">"):
            line = line[:-1].strip()
        if not line.startswith("0x") or ":" not in line:
            continue
        address_text, value_text = line.split(":", 1)
        try:
            address = int(address_text, 16)
        except ValueError:
            continue
        if address != start_address + 4 * len(values):
            raise PermitError("OpenOCD返回的RAM地址不连续")
        for token in value_text.split():
            if token == ">":
                break
            if token.lower().startswith("0x"):
                token = token[2:]
            if not token or len(token) > 8:
                raise PermitError("OpenOCD返回了无效32位字")
            try:
                values.append(int(token, 16))
            except ValueError as error:
                raise PermitError("OpenOCD返回了非十六进制数据") from error
            if len(values) > count:
                raise PermitError("OpenOCD返回的数据超过固定长度")
    if len(values) != count:
        raise PermitError(f"OpenOCD只返回{len(values)}/{count}个字")
    return values


class FixedOpenOCDClient:
    """此专用客户端仅向身份符号写入崩溃许可，不执行控制或复位命令。"""

    def __init__(self, host: str, port: int, timeout: float,
                 identity_address: int, permit_address: int):
        if host not in ("127.0.0.1", "::1") or not 1 <= port <= 65535 or not 0 < timeout <= 5:
            raise PermitError("OpenOCD连接必须是有限超时的本机回环地址")
        if (identity_address % 4 or not OBSERVATION_RAM_START <= identity_address < OBSERVATION_RAM_END or
                permit_address != CRASH_RECORD_START + CRASH_RECORD_SIZE):
            raise PermitError("OpenOCD符号地址不符合固定观测布局")
        self.host, self.port, self.timeout = host, port, timeout
        self.identity_address = identity_address
        self.permit_address = permit_address
        self.connection = None
        self.deadline = 0.0
        self.wire = bytearray()

    def __enter__(self):
        self.deadline = time.monotonic() + self.timeout
        try:
            self.connection = socket.create_connection((self.host, self.port), timeout=self.timeout)
            self._read_prompt()
        except OSError as error:
            self.close()
            raise PermitError(f"无法连接本机OpenOCD: {error}") from error
        return self

    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def __exit__(self, *_args):
        self.close()

    def command(self, command: str) -> str:
        if self.connection is None:
            raise PermitError("OpenOCD连接未打开")
        if command == "poll":
            pass
        elif command.startswith("mdw 0x"):
            parts = command.split()
            try:
                read = (int(parts[1], 16), int(parts[2])) if len(parts) == 3 else None
            except ValueError:
                read = None
            allowed = {
                (self.identity_address, 12),
                (self.permit_address, PERMIT_WORDS),
                (self.permit_address + 4, PERMIT_WORDS - 1),
            }
            if read not in allowed:
                raise PermitError("内存读取必须匹配固定身份或许可符号")
        else:
            raise PermitError("拒绝未列入许可流程的OpenOCD命令")
        self.deadline = time.monotonic() + self.timeout
        self.connection.settimeout(self.timeout)
        self.connection.sendall(command.encode("ascii") + b"\n")
        return self._read_prompt()

    def write_permit_word(self, address: int, value: int) -> str:
        if self.connection is None or address % 4 or not 0 <= value <= 0xFFFFFFFF:
            raise PermitError("固定许可写入参数无效")
        permitted_start = CRASH_RECORD_START + CRASH_RECORD_SIZE
        permitted_end = permitted_start + PERMIT_SIZE
        if not permitted_start <= address < permitted_end or (address - permitted_start) % 4:
            raise PermitError("拒绝写入固定许可符号以外的地址")
        self.deadline = time.monotonic() + self.timeout
        self.connection.settimeout(self.timeout)
        command = f"mww 0x{address:08x} 0x{value:08x}"
        self.connection.sendall(command.encode("ascii") + b"\n")
        return self._read_prompt()

    def _read_prompt(self) -> str:
        chunks = bytearray()
        while time.monotonic() < self.deadline and len(chunks) < 8192:
            if self.connection is None:
                raise PermitError("OpenOCD连接已关闭")
            self.connection.settimeout(max(0.01, self.deadline - time.monotonic()))
            block = self.connection.recv(1024)
            if not block:
                raise PermitError("OpenOCD关闭了Telnet连接")
            self.wire.extend(block)
            chunks.extend(self._consume_telnet())
            text = chunks.decode("utf-8", errors="replace")
            if text.rstrip().endswith(">"):
                return text
        raise PermitError("OpenOCD响应超时或超过固定缓冲")

    def _consume_telnet(self) -> bytes:
        output = bytearray()
        index = 0
        while index < len(self.wire):
            if self.wire[index] != 0xFF:
                output.append(self.wire[index])
                index += 1
                continue
            if index + 1 >= len(self.wire):
                break
            option = self.wire[index + 1]
            if option in (0xFB, 0xFC, 0xFD, 0xFE):
                if index + 2 >= len(self.wire):
                    break
                if self.connection is not None:
                    if option == 0xFB:
                        self.connection.sendall(bytes((0xFF, 0xFE, self.wire[index + 2])))
                    elif option == 0xFD:
                        self.connection.sendall(bytes((0xFF, 0xFC, self.wire[index + 2])))
                index += 3
                continue
            if option == 0xFA:
                end = self.wire.find(b"\xff\xf0", index + 2)
                if end < 0:
                    break
                index = end + 2
                continue
            index += 2
        del self.wire[:index]
        return bytes(output)


def arm_permit(elf: Path, cache: Path, case_name: str, host: str = "127.0.0.1",
               port: int = 4444, timeout: float = 3.0,
               client_factory=FixedOpenOCDClient) -> tuple[bytes, int]:
    check_cache(cache)
    elf_bytes = elf.read_bytes()
    build_id = elf_build_id(elf_bytes)
    symbols = elf_symbols(elf_bytes, {"g_crash_log", "bsp_crash_test_permit",
                                      "bsp_observation_identity"})
    record_address, record_size = symbols["g_crash_log"]
    permit_address, permit_size = symbols["bsp_crash_test_permit"]
    identity_address, identity_size = symbols["bsp_observation_identity"]
    if record_address != CRASH_RECORD_START or record_size != CRASH_RECORD_SIZE:
        raise PermitError("ELF崩溃记录符号不符合固定链接布局")
    if permit_address != CRASH_RECORD_START + CRASH_RECORD_SIZE or permit_size != PERMIT_SIZE:
        raise PermitError("ELF一次性许可符号不符合固定保留区布局")
    if not OBSERVATION_RAM_START <= identity_address < OBSERVATION_RAM_END or identity_size != OBSERVATION_SIZE:
        raise PermitError("ELF观测身份符号不在批准的固定RAM布局")

    with client_factory(host, port, timeout, identity_address, permit_address) as client:
        state = client.command("poll")
        if "running" not in state.lower():
            raise PermitError("目标未处于运行状态，未写入许可")
        identity_words = decode_openocd_words(
            client.command(f"mdw 0x{identity_address:08x} 12"), identity_address, 12
        )
        expected_id_words = struct.unpack("<5I", build_id)
        if (identity_words[:4] != [OBSERVATION_MAGIC, OBSERVATION_VERSION,
                                   OBSERVATION_SIZE, OBSERVATION_LAYOUT] or
                identity_words[4:9] != list(expected_id_words) or
                identity_words[9:11] != [1, 1]):
            raise PermitError("目标Build ID或bench_safe/single身份与ELF不匹配，未写入许可")

        current = decode_openocd_words(
            client.command(f"mdw 0x{permit_address:08x} {PERMIT_WORDS}"),
            permit_address, PERMIT_WORDS,
        )
        if current[0] == PERMIT_MAGIC:
            raise PermitError("目标上已有尚未消费的许可，拒绝覆盖")
        if current[0] not in (PERMIT_CONSUMED, 0):
            raise PermitError("许可区magic不是空闲或已消费状态，拒绝覆盖")

        permit = build_permit(build_id, CASE_VALUES[case_name])
        words = struct.unpack(f"<{PERMIT_WORDS}I", permit)
        # 先写完其余字段和CRC，最后发布magic，避免任务读到半份许可。
        for index in range(1, PERMIT_WORDS):
            response = client.write_permit_word(permit_address + index * 4, words[index])
            if "error" in response.lower():
                raise PermitError("OpenOCD拒绝写入许可字段")
        staged = decode_openocd_words(
            client.command(f"mdw 0x{permit_address + 4:08x} {PERMIT_WORDS - 1}"),
            permit_address + 4, PERMIT_WORDS - 1,
        )
        if staged != list(words[1:]):
            raise PermitError("许可字段回读不一致；magic仍未发布")
        response = client.write_permit_word(permit_address, PERMIT_MAGIC)
        if "error" in response.lower():
            raise PermitError("OpenOCD未确认许可magic写入")
    return permit, permit_address


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True, help="此ELF对应的CMakeCache.txt")
    parser.add_argument("--case", choices=sorted(CASE_VALUES), required=True,
                        help="每次只触发一种：udf、assert、stack-hook")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4444)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args(argv)
    try:
        permit, address = arm_permit(args.elf, args.cache, args.case,
                                     args.host, args.port, args.timeout)
    except (OSError, PermitError) as error:
        print(f"未写入崩溃自测许可：{error}", file=sys.stderr)
        return 2
    print(f"已向固定许可符号0x{address:08x}提交一次性{args.case}许可；nonce=0x{struct.unpack_from('<I', permit, 32)[0]:08x}")
    print("工具没有复位或停核；请等待目标按许可触发并复位，再读取记录。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
