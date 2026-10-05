import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import probe_snapshot_live


class ProbeSnapshotLiveTests(unittest.TestCase):
    def test_gdb_commands_only_attach_and_read(self):
        command = probe_snapshot_live.build_gdb_command(
            "arm-none-eabi-gdb", "firmware.elf", "127.0.0.1", 3333
        )
        gdb_commands = [command[index + 1] for index, arg in enumerate(command[:-1]) if arg == "--ex"]
        command_names = [line.strip().split()[0] for line in gdb_commands]

        self.assertIn("target", command_names)
        self.assertIn("printf", command_names)
        self.assertNotIn("halt", command_names)
        self.assertNotIn("reset", command_names)
        self.assertNotIn("resume", command_names)
        self.assertNotIn("continue", command_names)
        self.assertNotIn("step", command_names)
        self.assertNotIn("next", command_names)
        self.assertIn("--nx", command)
        self.assertIn("--return-child-result", command)

    def test_running_target_config_disables_intrusive_attach_events(self):
        config = (ROOT / "tools" / "openocd_running_inspection.cfg").read_text(encoding="utf-8")
        commands = [line.strip().split() for line in config.splitlines()
                    if line.strip() and not line.lstrip().startswith("#")]

        self.assertIn(["mc02.cpu0", "configure", "-event", "gdb-attach", "{}"], commands)
        self.assertIn(["mc02.cpu0", "configure", "-event", "gdb-detach", "{}"], commands)
        self.assertIn(["gdb", "memory_map", "disable"], commands)
        self.assertFalse(any(parts[0] in {"halt", "reset", "resume", "shutdown"} for parts in commands))

    def test_snapshot_parser_reads_required_counters(self):
        output = """tick_ms=1200
vision_mode=2
vision_rx_count=15
vision_tx_count=30
vision_crc_error_count=1
iwdg_reset_flag=0
"""

        snapshot = probe_snapshot_live.parse_snapshot(output)

        self.assertEqual(snapshot["tick_ms"], 1200)
        self.assertEqual(snapshot["vision_mode"], 2)
        self.assertEqual(snapshot["vision_rx_count"], 15)
        self.assertEqual(snapshot["vision_crc_error_count"], 1)
        self.assertEqual(snapshot["iwdg_reset_flag"], 0)

    def test_counter_decrease_is_classified_using_tick_and_reset_flag(self):
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
        counter_clear = {
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
        self.assertEqual(probe_snapshot_live.classify_change(before, counter_clear), "counter-cleared")
        self.assertEqual(probe_snapshot_live.classify_change(natural_wrap, after_wrap), "running")

    def test_gdb_read_uses_a_process_timeout(self):
        completed = subprocess.CompletedProcess(
            args=[], returncode=0,
            stdout=("tick_ms=1\nvision_mode=0\nvision_rx_count=0\nvision_tx_count=0\n"
                    "vision_crc_error_count=0\niwdg_reset_flag=0\n"),
            stderr="",
        )
        with patch.object(probe_snapshot_live.subprocess, "run", return_value=completed) as run:
            result = probe_snapshot_live.read_snapshot(
                "gdb", "firmware.elf", "127.0.0.1", 3333, timeout=2.0
            )

        self.assertEqual(result["tick_ms"], 1)
        self.assertEqual(run.call_args.kwargs["timeout"], 2.0)
        self.assertFalse(run.call_args.kwargs.get("shell", False))

    def test_gdb_timeout_is_reported_as_probe_io_error(self):
        with patch.object(
            probe_snapshot_live.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired(cmd="gdb", timeout=0.1),
        ):
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                probe_snapshot_live.read_snapshot(
                    "gdb", "firmware.elf", "127.0.0.1", 3333, timeout=0.1
                )


if __name__ == "__main__":
    unittest.main()
