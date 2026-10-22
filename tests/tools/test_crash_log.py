"""固定布局崩溃记录离线解析与身份边界回归。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import crash_log


class CrashLogTests(unittest.TestCase):
    def test_fixed_v3_header_vector_keeps_legacy_payload_raw(self):
        raw = bytearray(252)
        struct.pack_into("<6I", raw, 0,
                         crash_log.V3_MAGIC_OK, 252, 3, 0x12345678,
                         0xF6F3E2C3, 4)
        struct.pack_into("<2I", raw, 24, 1, 1)
        decoded = crash_log.decode_record(bytes(raw))
        self.assertEqual(decoded["status"], "valid")
        self.assertEqual(decoded["type"], "HARDFAULT")
        self.assertEqual(decoded["firmware_id_weak"], "0x12345678")
        self.assertEqual(decoded["legacy_payload_words"], [0] * 53)
        self.assertNotIn("core_registers", decoded)

    def test_captured_v3_record_is_kept_as_weak_identity(self):
        record = ROOT / "build/observations/mc02-0068-legacy-v3-d25a36a806e060bdb94f243adab7b7063c6ab785/legacy-v3-record.bin"
        if not record.exists():
            self.skipTest("本机历史v3板上记录不存在")
        decoded = crash_log.decode_record(record.read_bytes())
        self.assertEqual(decoded["version"], 3)
        self.assertEqual(decoded["status"], "valid")
        self.assertEqual(decoded["type"], "HARDFAULT")
        self.assertEqual(len(decoded["legacy_payload_words"]), 53)
        self.assertIn("不推测故障位置", decoded["identity_limit"])
        self.assertNotIn("core_registers", decoded)

    def test_v4_fixed_vector_and_busy_final(self):
        raw = bytearray(336)
        struct.pack_into("<IIIIII", raw, 0, crash_log.V4_MAGIC_OK, 336, 4, 4, 1, 1)
        struct.pack_into("<I", raw, 332, 0x635E3450)
        decoded = crash_log.decode_record(bytes(raw))
        self.assertEqual(decoded["status"], "valid")
        self.assertEqual(decoded["type"], "HARDFAULT")
        struct.pack_into("<I", raw, 0, crash_log.V4_MAGIC_BUSY)
        self.assertEqual(crash_log.decode_record(bytes(raw))["status"], "valid-busy-final")

    def test_malformed_lengths_crc_and_stack_count_are_rejected(self):
        with self.assertRaises(crash_log.CrashRecordError):
            crash_log.decode_record(b"\0" * 11)
        with self.assertRaises(crash_log.CrashRecordError):
            crash_log.decode_record(b"\0" * 513)
        raw = bytearray(336)
        struct.pack_into("<IIIIII", raw, 0, crash_log.V4_MAGIC_OK, 336, 4, 4, 1, 1)
        struct.pack_into("<I", raw, 332, 0x635E3450)
        raw[20] ^= 1
        with self.assertRaisesRegex(crash_log.CrashRecordError, "CRC32"):
            crash_log.decode_record(bytes(raw))
        struct.pack_into("<I", raw, 200, 33)
        with self.assertRaises(crash_log.CrashRecordError):
            crash_log.decode_record(bytes(raw))

    def test_legacy_record_is_never_symbolized(self):
        record = {
            "version": 3,
            "firmware_id_weak": "0x12345678",
            "core_registers": {"pc": 0x08001234, "lr": 0x08001230},
        }
        with patch.object(crash_log, "read_elf_build_id", return_value="ab" * 20), \
             patch.object(crash_log.subprocess, "run") as run:
            decoded = crash_log.symbolize_if_matched(record, Path("test.elf"), "readelf", "addr2line")
        run.assert_not_called()
        self.assertIn("不解析地址", decoded["symbolization"])

    def test_v4_symbolization_requires_exact_build_id(self):
        build_id = "ab" * 20
        record = {"version": 4, "build_id": build_id,
                  "core_registers": {"pc": 0x08001234, "lr": 0x08001230}}
        with patch.object(crash_log, "read_elf_build_id", return_value="cd" * 20), \
             patch.object(crash_log.subprocess, "run") as run:
            decoded = crash_log.symbolize_if_matched(record, Path("test.elf"), "readelf", "addr2line")
        run.assert_not_called()
        self.assertIn("不匹配", decoded["symbolization"])
        with patch.object(crash_log, "read_elf_build_id", return_value=build_id), \
             patch.object(crash_log.subprocess, "run", return_value=type(
                 "Result", (), {"returncode": 0, "stdout": "function\nsource.c:4\n"
                                "function\nsource.c:3\n", "stderr": ""})()) as run:
            decoded = crash_log.symbolize_if_matched(record, Path("test.elf"), "readelf", "addr2line")
        run.assert_called_once()
        self.assertEqual(decoded["pc_lr_symbols"],
                         ["function", "source.c:4", "function", "source.c:3"])

    def test_archive_copies_only_matching_v4_record_with_manifest(self):
        build_id = "ab" * 20
        raw = bytearray(336)
        struct.pack_into("<IIIIII", raw, 0, crash_log.V4_MAGIC_OK, 336, 4, 4, 1, 1)
        raw[32:52] = bytes.fromhex(build_id)
        struct.pack_into("<I", raw, 52, 1)
        struct.pack_into("<I", raw, 332, zlib.crc32(raw[4:332]) & 0xFFFFFFFF)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            record_path = root / "record.bin"
            elf = root / "firmware.elf"
            map_file = root / "firmware.map"
            output = root / "archive"
            record_path.write_bytes(raw)
            elf.write_bytes(b"ELF test bytes")
            map_file.write_text("link map", encoding="ascii")
            with patch.object(crash_log, "read_elf_build_id", return_value=build_id):
                manifest = crash_log.archive_artifacts(record_path, elf, map_file, output)
            self.assertEqual(manifest["elf_build_id"], build_id)
            self.assertTrue((output / "crash-record.bin").is_file())
            self.assertTrue((output / "firmware.elf").is_file())
            self.assertTrue((output / "firmware.map").is_file())
            self.assertEqual(len(manifest["files"]), 3)
            with self.assertRaises(FileExistsError):
                with patch.object(crash_log, "read_elf_build_id", return_value=build_id):
                    crash_log.archive_artifacts(record_path, elf, map_file, output)
            mismatch = root / "mismatch"
            with self.assertRaisesRegex(crash_log.CrashRecordError, "不匹配"):
                with patch.object(crash_log, "read_elf_build_id", return_value="cd" * 20):
                    crash_log.archive_artifacts(record_path, elf, map_file, mismatch)
            self.assertFalse(mismatch.exists())

    def test_snapshot_extract_requires_v4_identity_match(self):
        build_id = "ab" * 20
        raw = bytearray(336)
        struct.pack_into("<IIIIII", raw, 0, crash_log.V4_MAGIC_OK, 336, 4, 4, 1, 1)
        raw[32:52] = bytes.fromhex(build_id)
        struct.pack_into("<I", raw, 52, 1)
        struct.pack_into("<I", raw, 332, zlib.crc32(raw[4:332]) & 0xFFFFFFFF)
        words = list(struct.unpack("<84I", raw)) + [0] * 44
        content = "\n".join((
            json_line({"type": "session", "mode": "crash-record"}),
            json_line({"type": "identity", "build_id": build_id}),
            json_line({"type": "sample", "raw_words": {"crash_record_words": words}}),
        )) + "\n"
        with tempfile.TemporaryDirectory() as temporary:
            snapshot = Path(temporary) / "raw.jsonl"
            output = Path(temporary) / "record.bin"
            snapshot.write_text(content, encoding="utf-8")
            extracted, actual_id = crash_log.extract_snapshot_record(snapshot)
            output.write_bytes(extracted)
            self.assertEqual(actual_id, build_id)
            self.assertEqual(output.read_bytes(), bytes(raw))
        with tempfile.TemporaryDirectory() as temporary:
            snapshot = Path(temporary) / "bad.jsonl"
            snapshot.write_text(content.replace(build_id, "cd" * 20), encoding="utf-8")
            with self.assertRaises(crash_log.CrashRecordError):
                crash_log.extract_snapshot_record(snapshot)


def json_line(value):
    import json
    return json.dumps(value)


if __name__ == "__main__":
    unittest.main()
