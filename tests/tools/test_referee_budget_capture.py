#!/usr/bin/env python3
"""REF-03专用工具的离线回归；不打开串口或调试器，不生成真实硬件合格证据。"""
import csv
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import referee_budget_capture as capture
import probe_snapshot_live as probe


def wire(kind=0, sender=3, receiver=4, sequence=0):
    content = b"\0\0" if kind else bytes(range(112))
    frame = bytearray((0xa5, len(content)+6, 0, sequence, 0, 1, 3))
    frame[4] = capture.crc(frame[:4], 8, 0x8c, 0xff)
    frame.extend(struct.pack("<HHH", 0x0100 if kind else 0x0200, sender, receiver))
    frame.extend(content)
    frame.extend(struct.pack("<H", capture.crc(frame, 16, 0x8408, 0xffff)))
    return bytes(frame)


class CaptureTests(unittest.TestCase):
    def test_fixed_crc_and_status(self):
        self.assertEqual(capture.crc(b"123456789", 8, 0x8c, 0xff), 0x0b)
        self.assertEqual(capture.crc(b"123456789", 16, 0x8408, 0xffff), 0x6f91)
        frame = capture.robot_frame(3, 255)
        self.assertEqual(len(frame), 26)
        self.assertEqual(frame[5:8], b"\x01\x02\x03")
        with self.assertRaises(ValueError):
            capture.robot_frame(10, 0)

    def test_stream_split_noise_bad_crc(self):
        frame = wire()
        for split in range(len(frame)+1):
            parser = capture.Frames()
            self.assertEqual(parser.feed(frame[:split])+parser.feed(frame[split:]), [frame])
        bad = bytearray(frame)
        bad[-1] ^= 1
        parser = capture.Frames()
        self.assertEqual(parser.feed(b"noise"+bad+frame+wire(1)), [frame, wire(1)])
        self.assertGreater(parser.rejected, 0)
        parser.feed(b"\xa5\xff\xff\0\0")
        self.assertLess(len(parser.buffer), 5)

    def test_trace_layout_ring_and_wrap(self):
        words = [0]*456
        words[0:2] = [0x52454633, 1]
        words[7] = 2
        words[8:15] = [1, 0, 0, 1, 0, 1, 127 | (4 << 16)]
        records, count = capture.decode_trace(words, 0)
        self.assertEqual(count, 1)
        self.assertEqual(records[0]["receiver"], 4)
        self.assertEqual(capture.decode_trace(words, 1), ([], 1))
        with self.assertRaises(ValueError):
            capture.decode_trace(words[:-1], 0)
        # 读取期间写者继续推进，但还没有绕回本批所需槽位，旧记录仍完整。
        self.assertEqual(capture.decode_trace(words, 0, 2, 8)[0][0]["id"], 1)
        with self.assertRaisesRegex(ValueError, "overrun"):
            capture.decode_trace(words, 0, 2, 129)
        words[7] = 3
        with self.assertRaises(ValueError):
            capture.decode_trace(words, 1)
        words[7] = 130
        with self.assertRaises(ValueError):
            capture.decode_trace(words, 0)
        words[0] = 0
        with self.assertRaises(ValueError):
            capture.decode_trace(words, 65)

    def test_write_and_identity_failures(self):
        with self.assertRaises(OSError):
            capture.write_frame(SimpleNamespace(write=lambda data: len(data)-1), b"abc")
        def fail(data):
            raise TimeoutError("serial timeout")
        with self.assertRaises(TimeoutError):
            capture.write_frame(SimpleNamespace(write=fail), b"abc")
        with tempfile.TemporaryDirectory() as directory:
            elf = Path(directory)/"wrong.elf"
            elf.write_bytes(b"bad")
            with self.assertRaises(ValueError):
                capture.validate_identity({"firmware": {"elf_sha256": "0"*64}}, elf)

    def test_read_allowlist(self):
        addresses = {"ref_magic": 0x24001000, "ref_count": 0x2400101c}
        client = probe.OpenOCDTelnetClient("127.0.0.1", 4444, 1, addresses, probe.REFEREE_SNAPSHOT_FIELDS)
        for command in ("halt", "resume", "mdw 0x24001000 457", "mdw 0x24001004 456"):
            with self.assertRaises(ValueError):
                client.command(command)
        # 合法读取通过命令检查，随后因没有真实连接而失败。
        with self.assertRaises(probe.ProbeIOError):
            client.command("mdw 0x24001000 456")

    def test_report_and_corruption(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            records, rows = [], []
            for i, age in enumerate((0, 29900, 30000, 60000, 71000, 74900)):
                kind = int(i >= 2)
                sender = 2 if i >= 4 else 3
                receiver = sender+0x100 if kind else 4
                frame = wire(kind, sender, receiver, i)
                records.append(dict(ordinal=i, id=i+1, submitted_ms=100+age, started_ms=100+age,
                                    completed_ms=101+age, result=0, started=1, length=len(frame),
                                    kind=kind, receiver=receiver))
                rows.append(dict(hex=frame.hex(), sender=sender))
            words = [0]*456
            words[:4] = [0x52454633, 1, 100, 3]
            words[7] = len(records)*2
            def encode_records():
                for i, r in enumerate(records):
                    words[8+i*7:15+i*7] = [r["id"], r["submitted_ms"], r["started_ms"], r["completed_ms"],
                        r["result"], r["started"], r["length"] | (r["kind"] << 8) | (r["receiver"] << 16)]
            encode_records()
            (folder/"mcu.jsonl").write_text(json.dumps(dict(words=words, records=records))+"\n")
            (folder/"capture.json").write_text(json.dumps(dict(failures=[], rejected_bytes=0, residual_bytes=0)))
            with (folder/"frames.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=("hex", "sender"))
                writer.writeheader()
                writer.writerows(rows)
            self.assertTrue(capture.report_capture(folder)["software_budget_pass"])
            records[1]["started_ms"] = 101
            encode_records()
            (folder/"mcu.jsonl").write_text(json.dumps(dict(words=words, records=records))+"\n")
            with self.assertRaisesRegex(ValueError, "34ms"):
                capture.report_capture(folder)


if __name__ == "__main__":
    unittest.main()
