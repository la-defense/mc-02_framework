#!/usr/bin/env python3
"""USB 生命周期采集工具的离线协议与边界回归。"""
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

TOOL = Path(__file__).resolve().parents[2] / "tools" / "usb_lifecycle_capture.py"
SPEC = importlib.util.spec_from_file_location("usb_lifecycle_capture", TOOL)
capture = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(capture)


class UsbLifecycleCaptureTest(unittest.TestCase):
    def test_neutral_frame_is_fixed_29_bytes_and_mode_zero(self):
        frame = capture.neutral_frame(123)
        self.assertEqual(len(frame), 29)
        self.assertEqual(frame[:3], b"SP\0")
        self.assertEqual(frame[3:27], bytes(24))
        self.assertEqual(int.from_bytes(frame[-2:], "little"), capture.crc16(frame[:-2]))
        self.assertEqual(frame, capture.neutral_frame(0))

    def test_identity_preflight_selects_normal_or_diagnostic_read_only_profile(self):
        elf = Path("build/wsl/usbtest-debug/firmware.elf")
        normal = capture.identity_command(elf, Path("build/session/identity"))
        diagnostic = capture.identity_command(elf, Path("build/session/identity"), diagnostic=True)
        self.assertIn("--usb", normal)
        self.assertNotIn("--usb-diagnostic", normal)
        self.assertIn("--usb-diagnostic", diagnostic)
        self.assertIn("--samples", diagnostic)
        self.assertIn("2", diagnostic)
        self.assertNotIn("--write", diagnostic)
        self.assertTrue(any(item.endswith("probe_snapshot_live.py") for item in diagnostic))

    def test_identity_mismatch_exits_before_opening_cdc_port(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            elf = root / "firmware.elf"
            elf.write_bytes(b"test ELF")
            output = root / "session"
            port = SimpleNamespace(device="MC02", vid=capture.USB_VID,
                                   pid=capture.USB_PID, serial_number="board-serial")
            serial_module = ModuleType("serial")
            serial_module.Serial = mock.Mock(side_effect=AssertionError("CDC opened before identity passed"))
            tools_module = ModuleType("serial.tools")
            list_ports_module = ModuleType("serial.tools.list_ports")
            list_ports_module.comports = lambda: [port]
            serial_module.tools = tools_module
            tools_module.list_ports = list_ports_module

            with mock.patch.dict(sys.modules, {
                    "serial": serial_module,
                    "serial.tools": tools_module,
                    "serial.tools.list_ports": list_ports_module,
            }), mock.patch.object(capture.subprocess, "run",
                                  return_value=SimpleNamespace(returncode=2, stdout="", stderr="身份不匹配")):
                result = capture.main([
                    "--port", "MC02", "--elf", str(elf), "--source-sha", "a" * 40,
                    "--output", str(output),
                ])

            self.assertEqual(result, 2)
            serial_module.Serial.assert_not_called()
            self.assertIn("returncode=2", (output / "identity_preflight.txt").read_text(encoding="utf-8"))

    def test_open_uses_bounded_read_timeout_for_cdc_in_requests(self):
        serial_module = SimpleNamespace(Serial=mock.Mock(return_value=object()))

        capture._open_serial_port(serial_module, "COM14")

        serial_module.Serial.assert_called_once_with(
            "COM14", 115200, timeout=capture.SERIAL_READ_TIMEOUT_S, write_timeout=0.1
        )
        self.assertGreater(capture.SERIAL_READ_TIMEOUT_S, 0.0)
        self.assertLess(capture.SERIAL_READ_TIMEOUT_S, 0.05)

    def test_reopen_matches_usb_serial_after_com_number_changes(self):
        wrong = SimpleNamespace(device="COM14", vid=capture.USB_VID,
                                pid=capture.USB_PID, serial_number="other-board")
        board = SimpleNamespace(device="COM15", vid=capture.USB_VID,
                                pid=capture.USB_PID, serial_number="board-serial")
        list_ports = SimpleNamespace(comports=mock.Mock(side_effect=[[], [wrong, board]]))
        fake_serial = object()
        serial_module = SimpleNamespace(
            SerialException=type("SerialException", (OSError,), {}),
            Serial=mock.Mock(return_value=fake_serial),
        )

        device, port = capture._reopen_serial_port(
            serial_module, list_ports, "board-serial", capture.time.monotonic() + 1.0
        )

        self.assertEqual(device, "COM15")
        self.assertIs(port, fake_serial)
        serial_module.Serial.assert_called_once_with(
            "COM15", 115200, timeout=capture.SERIAL_READ_TIMEOUT_S, write_timeout=0.1
        )

    def test_capture_reopens_same_board_after_serial_handle_fails(self):
        class Clock:
            now = 0.0

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        class Port:
            def __init__(self, clock, fail_read=False):
                self.clock = clock
                self.fail_read = fail_read
                self.closed = False

            def write(self, data):
                self.asserted_frame = data
                return len(data)

            def read(self, _size):
                if self.fail_read:
                    self.fail_read = False
                    raise serial_exception("USB device re-enumerated")
                self.clock.now += 0.5
                return b""

            def close(self):
                self.closed = True

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            elf = root / "firmware.elf"
            elf.write_bytes(b"test ELF")
            output = root / "session"
            board_before = SimpleNamespace(device="COM14", vid=capture.USB_VID,
                                           pid=capture.USB_PID, serial_number="board-serial")
            board_after = SimpleNamespace(device="COM15", vid=capture.USB_VID,
                                          pid=capture.USB_PID, serial_number="board-serial")
            clock = Clock()
            serial_exception = type("SerialException", (OSError,), {})
            first_port = Port(clock, fail_read=True)
            second_port = Port(clock)
            serial_module = ModuleType("serial")
            serial_module.SerialException = serial_exception
            serial_module.Serial = mock.Mock(side_effect=[first_port, second_port])
            tools_module = ModuleType("serial.tools")
            list_ports_module = ModuleType("serial.tools.list_ports")
            list_ports_module.comports = mock.Mock(side_effect=[[board_before], [board_after]])
            serial_module.tools = tools_module
            tools_module.list_ports = list_ports_module

            with mock.patch.dict(sys.modules, {
                    "serial": serial_module,
                    "serial.tools": tools_module,
                    "serial.tools.list_ports": list_ports_module,
            }), mock.patch.object(capture.subprocess, "run",
                                  return_value=SimpleNamespace(returncode=0, stdout="ok", stderr="")), \
                    mock.patch.object(capture.time, "monotonic", side_effect=clock.monotonic), \
                    mock.patch.object(capture.time, "sleep", side_effect=clock.sleep):
                result = capture.main([
                    "--port", "COM14", "--elf", str(elf), "--source-sha", "b" * 40,
                    "--output", str(output), "--duration", "1",
                ])

            error = (output / "error.txt").read_text(encoding="utf-8") if (output / "error.txt").exists() else ""
            self.assertEqual(result, 0, error)
            self.assertTrue(first_port.closed)
            self.assertTrue(second_port.closed)
            self.assertEqual([call.args[0] for call in serial_module.Serial.call_args_list], ["COM14", "COM15"])
            records = [json.loads(line) for line in (output / "raw.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual([record["type"] for record in records if record["type"] in ("io_error", "reopened")],
                             ["io_error", "reopened"])
            self.assertEqual(records[-1]["sent"], 2)
            self.assertEqual(records[-1]["reopen_count"], 1)
            self.assertEqual(records[-1]["failures"], 1)

    def test_status_parser_handles_split_and_multiple_frames(self):
        body = b"SP\x00" + bytes(36) + b"\x34\x12"
        frame = body + capture.crc16(body).to_bytes(2, "little")
        parser = capture.StatusFrames()
        self.assertEqual(parser.feed(frame[:17]), [])
        parsed = parser.feed(frame[17:] + frame)
        self.assertEqual(len(parsed), 2)
        self.assertEqual(parsed[0]["bullet_count"], 0x1234)
        self.assertEqual(parsed[0]["values"], (0.0,) * 9)
        self.assertEqual(len(parser.buffer), 0)

    def test_bad_crc_and_non_finite_status_are_rejected(self):
        body = b"SP\x00" + bytes(36) + b"\0\0"
        bad = body + b"\0\0"
        parser = capture.StatusFrames()
        self.assertEqual(parser.feed(bad), [])
        self.assertGreater(parser.rejected, 0)

        values = [0.0] * 9
        values[4] = math.inf
        body = b"SP\x00" + struct.pack("<9f", *values) + b"\0\0"
        frame = body + capture.crc16(body).to_bytes(2, "little")
        parser = capture.StatusFrames()
        self.assertEqual(parser.feed(frame), [])
        self.assertGreater(parser.rejected, 0)


if __name__ == "__main__":
    unittest.main()
