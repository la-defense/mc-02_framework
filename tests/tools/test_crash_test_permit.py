"""崩溃自测许可工具仅解析匹配ELF并写入固定的一次性RAM记录。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import crash_test_permit


def _cache_text() -> str:
    values = {
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
    return "\n".join(f"{key}:BOOL={value}" for key, value in values.items()) + "\n"


class FakeClient:
    """用固定RAM窗口模拟OpenOCD，只记录专用许可工具发出的读写。"""

    def __init__(self, _host, _port, _timeout, identity_address, permit_address,
                 identity_words):
        self.identity_address = identity_address
        self.permit_address = permit_address
        self.identity_words = identity_words
        self.permit_words = [0] * crash_test_permit.PERMIT_WORDS
        self.writes = []
        self.commands = []

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def command(self, command):
        self.commands.append(command)
        if command == "poll":
            return "poll\r\n\x00background polling: on\r\nTAP: mc02.cpu (enabled)\r\n> "
        if command == "targets":
            return "TargetName Type Endian TapName State\n0* stm32h7x.cpu0 cortex_m little mc02.cpu running\n>"
        _, address_text, count_text = command.split()
        address = int(address_text, 16)
        count = int(count_text)
        if address == self.identity_address and count == 12:
            words = self.identity_words
        elif address == self.permit_address and count == crash_test_permit.PERMIT_WORDS:
            words = self.permit_words
        elif (address == self.permit_address + 4 and
              count == crash_test_permit.PERMIT_WORDS - 1):
            words = self.permit_words[1:]
        else:
            raise AssertionError("测试客户端收到未批准的读取范围")
        return "\x00" + "".join(
            f"0x{address + index * 4:08x}: 0x{value:08x}\n"
            for index, value in enumerate(words)
        ) + ">"

    def write_permit_word(self, address, value):
        self.writes.append((address, value))
        index = (address - self.permit_address) // 4
        self.permit_words[index] = value
        return ">"


class CrashTestPermitTests(unittest.TestCase):
    def test_openocd_word_decoder_accepts_telnet_nul_prefix(self):
        output = (
            "mdw 0x24001000 2\r\n\x00"
            "0x24001000: 0x12345678\r\n"
            "0x24001004: 0x90abcdef\r\n> "
        )
        self.assertEqual(
            crash_test_permit.decode_openocd_words(output, 0x24001000, 2),
            [0x12345678, 0x90ABCDEF],
        )

    def test_fixed_permit_vector_and_case_rejection(self):
        raw = crash_test_permit.build_permit(bytes(range(20)), 1, 0x12345678)
        self.assertEqual(len(raw), 40)
        self.assertEqual(raw.hex(),
                         "5045524d0100000001000000000102030405060708090a0b0c0d0e0f10111213"
                         "785634122108c51c")
        with self.assertRaises(crash_test_permit.PermitError):
            crash_test_permit.build_permit(bytes(19), 1, 1)
        with self.assertRaises(KeyError):
            crash_test_permit.CASE_VALUES["unknown"]

    def test_cache_must_be_exact_bench_safe_diagnostic_profile(self):
        with tempfile.TemporaryDirectory() as temporary:
            cache = Path(temporary) / "CMakeCache.txt"
            cache.write_text(_cache_text(), encoding="utf-8")
            crash_test_permit.check_cache(cache)
            cache.write_text(_cache_text().replace(
                "MC02_ENABLE_HEATER:BOOL=OFF", "MC02_ENABLE_HEATER:BOOL=ON"),
                encoding="utf-8")
            with self.assertRaisesRegex(crash_test_permit.PermitError, "MC02_ENABLE_HEATER"):
                crash_test_permit.check_cache(cache)

    def test_openocd_command_allowlist_and_fixed_write_window(self):
        client = crash_test_permit.FixedOpenOCDClient(
            "127.0.0.1", 4444, 1.0, 0x24001000, 0x24000150)

        class FakeSocket:
            def __init__(self):
                self.sent = []

            def settimeout(self, _timeout):
                pass

            def sendall(self, data):
                self.sent.append(data)

        client.connection = FakeSocket()
        with patch.object(client, "_read_prompt", return_value="target state: running\n>"):
            client.command("targets")
            with self.assertRaises(crash_test_permit.PermitError):
                client.command("poll")
            client.command("mdw 0x24001000 12")
            with self.assertRaises(crash_test_permit.PermitError):
                client.command("reset run")
            with self.assertRaises(crash_test_permit.PermitError):
                client.command("mdw 0x20000000 1")
            with self.assertRaises(crash_test_permit.PermitError):
                client.write_permit_word(0x24000000, 0)
        self.assertEqual(client.connection.sent,
                         [b"targets\n", b"mdw 0x24001000 12\n"])

    def test_matching_elf_and_identity_publish_magic_last(self):
        elf = ROOT / "build/wsl/crashdiag-debug/Basic_Framework_MC02_crashdiag_bench_safe.elf"
        if not elf.exists():
            self.skipTest("专用ARM诊断ELF尚未构建")
        elf_bytes = elf.read_bytes()
        build_id = crash_test_permit.elf_build_id(elf_bytes)
        symbols = crash_test_permit.elf_symbols(
            elf_bytes, {"g_crash_log", "bsp_crash_test_permit", "bsp_observation_identity"})
        self.assertEqual(symbols["g_crash_log"], (0x24000000, 336))
        self.assertEqual(symbols["bsp_crash_test_permit"], (0x24000150, 40))
        identity_address, _ = symbols["bsp_observation_identity"]
        words = [crash_test_permit.OBSERVATION_MAGIC,
                 crash_test_permit.OBSERVATION_VERSION,
                 crash_test_permit.OBSERVATION_SIZE,
                 crash_test_permit.OBSERVATION_LAYOUT,
                 *struct.unpack("<5I", build_id), 1, 1, 0]
        fake = FakeClient("127.0.0.1", 4444, 2.0, identity_address,
                          0x24000150, words)

        def factory(host, port, timeout, identity, permit):
            self.assertEqual((host, port), ("127.0.0.1", 4444))
            return fake

        with tempfile.TemporaryDirectory() as temporary:
            cache = Path(temporary) / "CMakeCache.txt"
            cache.write_text(_cache_text(), encoding="utf-8")
            permit, address = crash_test_permit.arm_permit(
                elf, cache, "udf", timeout=2.0, client_factory=factory)
        self.assertEqual(address, 0x24000150)
        self.assertEqual(fake.writes[-1], (address, crash_test_permit.PERMIT_MAGIC))
        self.assertEqual([value for _, value in fake.writes[:-1]],
                         list(struct.unpack("<10I", permit)[1:]))
        self.assertEqual(fake.permit_words, list(struct.unpack("<10I", permit)))
        self.assertEqual(fake.commands[0], "targets")
        self.assertTrue(all(command.startswith("mdw 0x") or command == "targets"
                            for command in fake.commands))

    def test_identity_mismatch_never_writes_permit(self):
        elf = ROOT / "build/wsl/crashdiag-debug/Basic_Framework_MC02_crashdiag_bench_safe.elf"
        if not elf.exists():
            self.skipTest("专用ARM诊断ELF尚未构建")
        elf_bytes = elf.read_bytes()
        build_id = crash_test_permit.elf_build_id(elf_bytes)
        symbols = crash_test_permit.elf_symbols(
            elf_bytes, {"g_crash_log", "bsp_crash_test_permit", "bsp_observation_identity"})
        identity_address, _ = symbols["bsp_observation_identity"]
        words = [crash_test_permit.OBSERVATION_MAGIC,
                 crash_test_permit.OBSERVATION_VERSION,
                 crash_test_permit.OBSERVATION_SIZE,
                 crash_test_permit.OBSERVATION_LAYOUT,
                 *struct.unpack("<5I", bytes.fromhex("ff" * 20)), 1, 1, 0]
        fake = FakeClient("127.0.0.1", 4444, 2.0, identity_address,
                          0x24000150, words)
        with tempfile.TemporaryDirectory() as temporary:
            cache = Path(temporary) / "CMakeCache.txt"
            cache.write_text(_cache_text(), encoding="utf-8")
            with self.assertRaisesRegex(crash_test_permit.PermitError, "Build ID"):
                crash_test_permit.arm_permit(
                    elf, cache, "assert", timeout=2.0,
                    client_factory=lambda *_args: fake)
        self.assertEqual(fake.writes, [])
        self.assertEqual(fake.commands[0], "targets")
        self.assertEqual(fake.commands[1],
                         f"mdw 0x{identity_address:08x} 12")
        self.assertFalse(any(command.startswith(f"mdw 0x{0x24000150:08x}")
                             for command in fake.commands))
        self.assertEqual(crash_test_permit.elf_build_id(elf_bytes), build_id)


if __name__ == "__main__":
    unittest.main()
