"""验证 OpenOCD 原生 RTT 接收工具只启动受限服务并只接收上行数据。"""

import io
import socket
import sys
from pathlib import Path
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from rtt_monitor import ProbeIOError, build_openocd_arguments, receive_only


class _ReceiveSocket:
    def __init__(self, data):
        self.data = list(data)
        self.timeout = None
        self.sent = []

    def settimeout(self, value):
        self.timeout = value

    def recv(self, _size):
        value = self.data.pop(0)
        if isinstance(value, BaseException):
            raise value
        return value


class RttMonitorTests(unittest.TestCase):
    def test_openocd_commands_use_exact_elf_symbol_and_do_not_control_target(self):
        with patch("rtt_monitor._read_elf_symbol", return_value=(0x24001234, 168, b"")):
            args = build_openocd_arguments("openocd.exe", __file__, 19021)
        self.assertIn("rtt setup 0x24001234 168 \"SEGGER RTT\"", args)
        self.assertIn("rtt server start 19021 0", args)
        self.assertFalse(any(value in {"halt", "reset", "resume", "mww", "flash"} for value in args))
        config = Path(args[2]).read_text(encoding="utf-8")
        self.assertFalse(any(line.strip().startswith(("halt", "reset", "resume", "mww", "flash"))
                             for line in config.splitlines()))

    def test_receive_only_client_copies_bytes_without_sending(self):
        client = _ReceiveSocket([b"boot ready\r\n"])
        output = io.BytesIO()
        with patch("rtt_monitor.time.monotonic", side_effect=[0.0, 0.1, 2.0]):
            receive_only(client, 1.0, (output,))
        self.assertEqual(output.getvalue(), b"boot ready\r\n")
        self.assertEqual(client.sent, [])

    def test_receive_only_reports_server_close(self):
        client = _ReceiveSocket([b""])
        with patch("rtt_monitor.time.monotonic", side_effect=[0.0, 0.1]), \
             self.assertRaisesRegex(ProbeIOError, "closed"):
            receive_only(client, 1.0, (io.BytesIO(),))

    def test_invalid_receive_only_symbol_range_is_rejected(self):
        with patch("rtt_monitor._read_elf_symbol", return_value=(0x20000000, 168, b"")), \
             self.assertRaisesRegex(ProbeIOError, "outside"):
            build_openocd_arguments("openocd.exe", __file__, 19021)

    def test_receive_only_waits_through_idle_period_without_sending(self):
        client = _ReceiveSocket([socket.timeout(), b"next\n"])
        output = io.BytesIO()
        with patch("rtt_monitor.time.monotonic", side_effect=[0.0, 0.1, 0.2, 1.5]):
            receive_only(client, 1.0, (output,))
        self.assertEqual(output.getvalue(), b"next\n")
        self.assertEqual(client.sent, [])


if __name__ == "__main__":
    unittest.main()
