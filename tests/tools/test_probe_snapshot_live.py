import re
import socketserver
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import probe_snapshot_live


EXPECTED_EXPRESSIONS = {
    "tick_ms": "uwTick",
    "vision_mode": "recv_data.mode",
    "vision_rx_count": "vision_rx_count",
    "vision_tx_count": "vision_tx_count",
    "vision_crc_error_count": "vision_crc_error_count",
    "iwdg_reset_flag": "iwdg_reset_flag",
}

TEST_ADDRESSES = {
    "tick_ms": 0x2400AC30,
    "vision_mode": 0x2400E5DE,
    "vision_rx_count": 0x2400E5A8,
    "vision_tx_count": 0x2400E5A4,
    "vision_crc_error_count": 0x2400E59C,
    "iwdg_reset_flag": 0x2400D0FC,
}

TEST_VALUES = {
    "tick_ms": 1200,
    "vision_mode": 0,
    "vision_rx_count": 15,
    "vision_tx_count": 30,
    "vision_crc_error_count": 1,
    "iwdg_reset_flag": 0,
}


class FakeOpenOCDServer:
    def __init__(self, state_responses=None, initial_delay=0.0):
        self.commands = []
        self.initial_delay = initial_delay
        self.state_responses = list(
            state_responses if state_responses is not None else ["running"] * (len(TEST_ADDRESSES) + 1)
        )
        self.memory_by_address = {
            address: TEST_VALUES[key] for key, address in TEST_ADDRESSES.items()
        }

        owner = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                if owner.initial_delay:
                    time.sleep(owner.initial_delay)
                self.wfile.write(b"Open On-Chip Debugger\r\n> ")
                self.wfile.flush()
                while True:
                    line = self.rfile.readline()
                    if not line:
                        return
                    command = line.decode("ascii").strip()
                    owner.commands.append(command)
                    if command == "targets":
                        state = owner.state_responses.pop(0)
                        response = f"0* mc02.cpu0 cortex_m little mc02.cpu {state}\r\n"
                    else:
                        match = re.fullmatch(r"(?:mdw|mdb) (0x[0-9a-f]+) 1", command)
                        if not match:
                            response = "invalid command\r\n"
                        else:
                            address = int(match.group(1), 16)
                            value = owner.memory_by_address[address]
                            width = 2 if command.startswith("mdb") else 8
                            response = f"{match.group(1)}: {value:0{width}x}\r\n"
                    self.wfile.write(response.encode("ascii") + b"> ")
                    self.wfile.flush()

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

            def handle_error(self, request, client_address):
                return

        self.server = Server(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    @property
    def port(self):
        return self.server.server_address[1]

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)


class ProbeSnapshotLiveTests(unittest.TestCase):
    def test_gdb_command_resolves_addresses_offline_without_target_connection(self):
        command = probe_snapshot_live.build_symbol_command("arm-none-eabi-gdb", "firmware.elf")
        gdb_commands = [command[index + 1] for index, arg in enumerate(command[:-1]) if arg == "--ex"]

        self.assertEqual(probe_snapshot_live.EXPRESSIONS, EXPECTED_EXPRESSIONS)
        self.assertEqual(len(gdb_commands), len(EXPECTED_EXPRESSIONS) + 2)
        self.assertEqual(gdb_commands[:2], ["set pagination off", "set confirm off"])
        for key, expression in EXPECTED_EXPRESSIONS.items():
            self.assertIn(f'printf "{key}=0x%x\\n", (unsigned int)&({expression})', gdb_commands)
        joined = " ".join(gdb_commands).lower()
        self.assertNotIn("target ", joined)
        self.assertNotIn("remote", joined)
        self.assertNotIn("x/", joined)

    def test_symbol_address_parser_requires_all_addresses(self):
        output = "\n".join(
            f"{key}=0x{address:x}" for key, address in TEST_ADDRESSES.items()
        )
        self.assertEqual(probe_snapshot_live.parse_addresses(output), TEST_ADDRESSES)
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.parse_addresses("tick_ms=0x2400ac30")

    def test_offline_symbol_lookup_has_a_hard_timeout(self):
        with patch.object(
            probe_snapshot_live.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired(cmd="gdb", timeout=0.1),
        ) as run:
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                probe_snapshot_live.resolve_symbol_addresses("gdb", "firmware.elf", timeout=0.1)

        self.assertEqual(run.call_args.kwargs["timeout"], 0.1)
        self.assertFalse(run.call_args.kwargs.get("shell", False))

    def test_running_inspection_config_disables_gdb_and_limits_openocd_to_loopback(self):
        config = (ROOT / "tools" / "openocd_running_inspection.cfg").read_text(encoding="utf-8")
        commands = [line.strip().split() for line in config.splitlines()
                    if line.strip() and not line.lstrip().startswith("#")]

        self.assertIn(["bindto", "127.0.0.1"], commands)
        self.assertIn(["telnet", "port", "4444"], commands)
        self.assertIn(["gdb", "port", "disabled"], commands)
        self.assertIn(["tcl", "port", "disabled"], commands)
        self.assertIn(["adapter", "speed", "400"], commands)
        self.assertLess(
            commands.index(["source", "[find", "target/stm32h7x.cfg]"]),
            commands.index(["adapter", "speed", "400"]),
        )
        self.assertIn(
            ["mc02.cpu0", "configure", "-event", "examine-end", "{}"],
            commands,
        )
        self.assertFalse(any(parts[0] in {"halt", "reset", "resume", "shutdown"} for parts in commands))

    def test_telnet_snapshot_reads_only_ram_and_preserves_running_target(self):
        server = FakeOpenOCDServer()
        try:
            snapshot = probe_snapshot_live.read_snapshot(
                TEST_ADDRESSES, "127.0.0.1", server.port, timeout=1.0
            )
        finally:
            server.close()

        self.assertEqual(snapshot, TEST_VALUES)
        self.assertEqual(server.commands[0], "targets")
        self.assertEqual(server.commands[-1], "targets")
        self.assertTrue(all(command == "targets" or command.startswith(("mdb ", "mdw "))
                            for command in server.commands))
        self.assertEqual(len(server.commands), len(TEST_ADDRESSES) * 2 + 1)

    def test_snapshot_aborts_before_memory_reads_if_target_is_not_running(self):
        server = FakeOpenOCDServer(state_responses=["halted"])
        try:
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                probe_snapshot_live.read_snapshot(
                    TEST_ADDRESSES, "127.0.0.1", server.port, timeout=1.0
                )
        finally:
            server.close()

        self.assertEqual(server.commands, ["targets"])

    def test_snapshot_reports_if_target_stops_during_reads_without_resuming_it(self):
        server = FakeOpenOCDServer(state_responses=["running", "halted"])
        try:
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                probe_snapshot_live.read_snapshot(
                    TEST_ADDRESSES, "127.0.0.1", server.port, timeout=1.0
                )
        finally:
            server.close()

        self.assertEqual(server.commands[0], "targets")
        self.assertEqual(server.commands[-1], "targets")
        self.assertFalse(any(command in {"halt", "reset", "resume"} for command in server.commands))

    def test_telnet_command_whitelist_rejects_run_control(self):
        server = FakeOpenOCDServer()
        try:
            with probe_snapshot_live.OpenOCDTelnetClient(
                "127.0.0.1", server.port, timeout=1.0
            ) as client:
                with self.assertRaises(ValueError):
                    client.command("halt")
        finally:
            server.close()

        self.assertEqual(server.commands, [])

    def test_telnet_read_timeout_is_reported(self):
        server = FakeOpenOCDServer(initial_delay=0.2)
        try:
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                with probe_snapshot_live.OpenOCDTelnetClient(
                    "127.0.0.1", server.port, timeout=0.05
                ):
                    self.fail("delayed prompt unexpectedly connected")
        finally:
            server.close()

    def test_telnet_client_rejects_non_loopback_hosts(self):
        for host in ("192.0.2.1", "localhost"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                probe_snapshot_live.OpenOCDTelnetClient(host, 4444, timeout=1.0)

    def test_snapshot_change_classification_distinguishes_reset_and_counter_decrease(self):
        before = {
            "tick_ms": 1000,
            "vision_rx_count": 25,
            "vision_tx_count": 50,
            "vision_crc_error_count": 1,
            "iwdg_reset_flag": 0,
        }
        watchdog_reset = {
            "tick_ms": 20,
            "vision_rx_count": 0,
            "vision_tx_count": 0,
            "vision_crc_error_count": 0,
            "iwdg_reset_flag": 1,
        }
        counter_decrease_without_reset_evidence = {
            "tick_ms": 1200,
            "vision_rx_count": 0,
            "vision_tx_count": 0,
            "vision_crc_error_count": 0,
            "iwdg_reset_flag": 0,
        }
        natural_wrap = {
            "tick_ms": 0xFFFFFFF0,
            "vision_rx_count": 0xFFFFFFFE,
            "vision_tx_count": 0xFFFFFFFE,
            "vision_crc_error_count": 0xFFFFFFFE,
            "iwdg_reset_flag": 0,
        }
        after_wrap = {
            "tick_ms": 0x20,
            "vision_rx_count": 2,
            "vision_tx_count": 2,
            "vision_crc_error_count": 2,
            "iwdg_reset_flag": 0,
        }

        self.assertEqual(probe_snapshot_live.classify_change(before, watchdog_reset), "target-reset-iwdg")
        self.assertEqual(
            probe_snapshot_live.classify_change(before, counter_decrease_without_reset_evidence),
            "counter-decrease-unclassified",
        )
        self.assertEqual(probe_snapshot_live.classify_change(natural_wrap, after_wrap), "running")


if __name__ == "__main__":
    unittest.main()
