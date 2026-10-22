"""验证只读快照工具的身份校验、固定字段读取与报告边界。"""

import contextlib
import io
import json
import re
import socketserver
import struct
import subprocess
import sys
import tempfile
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

EXPECTED_FIELD_SPECS = {
    "tick_ms": ("uwTick", "mdw", 32, 4),
    "vision_mode": ("recv_data.mode", "mdb", 8, 1),
    "vision_rx_count": ("vision_rx_count", "mdw", 32, 4),
    "vision_tx_count": ("vision_tx_count", "mdw", 32, 4),
    "vision_crc_error_count": ("vision_crc_error_count", "mdw", 32, 4),
    "iwdg_reset_flag": ("iwdg_reset_flag", "mdb", 8, 1),
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

TEST_IMU_ADDRESSES = {
    key: 0x24001000 + index * 4
    for index, key in enumerate(probe_snapshot_live.IMU_SNAPSHOT_FIELDS)
}
TEST_IMU_VALUES = {
    key: index + 1 for index, key in enumerate(probe_snapshot_live.IMU_SNAPSHOT_FIELDS)
}
TEST_INIT_ADDRESSES = {
    key: 0x24004000 + index * 4
    for index, key in enumerate(probe_snapshot_live.INIT_SNAPSHOT_FIELDS)
}
TEST_INIT_VALUES = {
    "tick_ms": 10,
    "init_failed": 1,
    "init_stage": 2,
    "init_module": 6,
    "init_reason": 1,
    "init_resource_id_lo": 0x34,
    "init_resource_id_hi": 0x12,
    "init_failure_count": 3,
    "init_heartbeat": 99,
}


def _pack_u32(byte_values):
    return [int.from_bytes(bytes(byte_values[index:index + 4]), "little")
            for index in range(0, len(byte_values), 4)]


def _rtos_fixture():
    fields = probe_snapshot_live.RTOS_SNAPSHOT_FIELDS
    names = bytearray(12 * probe_snapshot_live.RTOS_TASK_CAPACITY)
    names[:4] = b"IDLE"
    names[12:21] = b"motortask"
    priorities = bytes([0, 3] + [0] * 10)
    stacks = [120, 80] + [0] * 10
    stack_bytes = b"".join(value.to_bytes(2, "little") for value in stacks)
    values = {}
    address = 0x24005000
    addresses = {}
    for key, field in fields.items():
        addresses[key] = address
        address += field.count * 4 + 4
        if field.count > 1:
            values[key] = [0] * field.count
        else:
            values[key] = 0
    values.update(
        tick_ms=123456,
        stats_layout=probe_snapshot_live.RTOS_TASK_STATS_LAYOUT,
        stats_generation_before=8,
        stats_generation_after=8,
        stats_sample_id=1,
        stats_captured_ms=123456,
        stats_count=2,
        stats_task_total=2,
        stats_truncated=0,
        stats_names_words=_pack_u32(names),
        stats_runtime=[11, 22] + [0] * 10,
        stats_priority_words=_pack_u32(priorities),
        stats_stack_words=_pack_u32(stack_bytes),
        motor_task_max_us=125,
        motor_task_calls=1000,
        ins_task_max_us=600,
        ins_task_calls=1000,
        daemon_task_max_us=220,
        daemon_task_calls=100,
        robot_task_max_us=1100,
        robot_task_calls=200,
        uart_tx_task_max_us=85,
        uart_tx_task_calls=1000,
        motor_control_max_us=123,
        ins_compute_max_us=490,
        can_rx_dispatch_max_us=700,
        can_rx_dispatch_calls=31,
        can_rx_drop_total=0,
        can_rx_peak_max=31,
        lcd_sample_max_us=72,
        lcd_update_max_us=300,
        lcd_draw_max_us=620,
    )
    return addresses, values


def _test_identity_info():
    build_id = "0123456789abcdef0123456789abcdef01234567"
    build_id_bytes = bytes.fromhex(build_id)
    expected = {
        "magic": probe_snapshot_live.OBSERVATION_IDENTITY_MAGIC,
        "version": probe_snapshot_live.OBSERVATION_IDENTITY_VERSION,
        "size_bytes": probe_snapshot_live.OBSERVATION_IDENTITY_WORDS * 4,
        "layout": probe_snapshot_live.OBSERVATION_IDENTITY_LAYOUT,
        "build_id_words": [int.from_bytes(build_id_bytes[i:i + 4], "little")
                            for i in range(0, len(build_id_bytes), 4)],
        "profile": 1,
        "role": 1,
        "module_mask": sum(probe_snapshot_live.OBSERVATION_MODULES.values()),
        "build_id": build_id,
        "profile_name": "bench_safe",
        "role_name": "single",
    }
    return {
        "expected": expected,
        "identity_address": 0x24001000,
        "ram_bounds": (probe_snapshot_live.SNAPSHOT_RAM_START, probe_snapshot_live.SNAPSHOT_RAM_END),
    }


def _identity_words(expected):
    return [
        expected["magic"], expected["version"], expected["size_bytes"], expected["layout"],
        *expected["build_id_words"], expected["profile"], expected["role"], expected["module_mask"],
    ]


def _write_identity_elf(path, template_values=None):
    if template_values is None:
        expected = _test_identity_info()["expected"]
        template_values = [expected[key] for key in (
            "magic", "version", "size_bytes", "layout", "profile", "role", "module_mask",
        )]
    section_names = b"\0.shstrtab\0.strtab\0.symtab\0.rodata\0.bss\0"
    section_name_offsets = {
        ".shstrtab": section_names.index(b".shstrtab"),
        ".strtab": section_names.index(b".strtab"),
        ".symtab": section_names.index(b".symtab"),
        ".rodata": section_names.index(b".rodata"),
        ".bss": section_names.index(b".bss"),
    }
    symbol_names = b"\0bsp_observation_identity_template\0bsp_observation_identity\0"
    template_name = symbol_names.index(b"bsp_observation_identity_template")
    identity_name = symbol_names.index(b"bsp_observation_identity", template_name + 1)
    shstr_offset = 52
    strtab_offset = shstr_offset + len(section_names)
    symtab_offset = (strtab_offset + len(symbol_names) + 3) & ~3
    symtab = bytes(16) + struct.pack("<IIIBBH", template_name, 0x08001000, 28, 0x11, 0, 4)
    symtab += struct.pack("<IIIBBH", identity_name, 0x24001000, 48, 0x11, 0, 5)
    rodata_offset = (symtab_offset + len(symtab) + 3) & ~3
    template_data = struct.pack("<7I", *template_values)
    bss_offset = rodata_offset + len(template_data)
    section_offset = (bss_offset + 3) & ~3
    section_headers = [bytes(40)]
    section_headers.append(struct.pack("<IIIIIIIIII", section_name_offsets[".shstrtab"], 3, 0, 0,
                                       shstr_offset, len(section_names), 0, 0, 1, 0))
    section_headers.append(struct.pack("<IIIIIIIIII", section_name_offsets[".strtab"], 3, 0, 0,
                                       strtab_offset, len(symbol_names), 0, 0, 1, 0))
    section_headers.append(struct.pack("<IIIIIIIIII", section_name_offsets[".symtab"], 2, 0, 0,
                                       symtab_offset, len(symtab), 2, 1, 4, 16))
    section_headers.append(struct.pack("<IIIIIIIIII", section_name_offsets[".rodata"], 1, 2,
                                       0x08001000, rodata_offset, len(template_data), 0, 0, 4, 0))
    section_headers.append(struct.pack("<IIIIIIIIII", section_name_offsets[".bss"], 8, 3,
                                       0x24001000, bss_offset, 48, 0, 0, 4, 0))
    ident = b"\x7fELF" + bytes([1, 1, 1]) + bytes(9)
    header = struct.pack("<HHIIIIIHHHHHH", 2, 40, 1, 0, 0, section_offset, 0,
                         52, 0, 0, 40, len(section_headers), 1)
    image = bytearray(ident + header)
    image.extend(section_names)
    image.extend(symbol_names)
    image.extend(bytes(symtab_offset - len(image)))
    image.extend(symtab)
    image.extend(bytes(rodata_offset - len(image)))
    image.extend(template_data)
    image.extend(bytes(section_offset - len(image)))
    for section in section_headers:
        image.extend(section)
    path.write_bytes(image)


def _performance_raw(sample_id, tick_ms, base, module_mask=None):
    if module_mask is None:
        module_mask = sum(probe_snapshot_live.OBSERVATION_MODULES.values())
    fields = probe_snapshot_live.PERFORMANCE_SNAPSHOT_FIELDS
    raw = {
        key: ([0] * field.count if field.count > 1 else 0)
        for key, field in fields.items()
    }
    names = bytearray(12 * probe_snapshot_live.RTOS_TASK_CAPACITY)
    names[:4] = b"IDLE"
    names[12:23] = b"defaultTask"
    priorities = bytes([0, 3] + [0] * 10)
    stacks = [120, 100] + [0] * 10
    stack_bytes = b"".join(value.to_bytes(2, "little") for value in stacks)
    generation = sample_id * 2
    raw.update({
        "tick_ms": tick_ms,
        "iwdg_reset_flag": 0,
        "stats_layout": probe_snapshot_live.RTOS_TASK_STATS_LAYOUT,
        "stats_generation_before": generation,
        "stats_generation_after": generation,
        "stats_sample_id": sample_id,
        "stats_captured_ms": tick_ms,
        "stats_count": 2,
        "stats_task_total": 2,
        "stats_truncated": 0,
        "stats_names_words": _pack_u32(names),
        "stats_runtime": [base * 10, base * 15] + [0] * 10,
        "stats_priority_words": _pack_u32(priorities),
        "stats_stack_words": _pack_u32(stack_bytes),
    })
    probes = []
    for index in range(len(probe_snapshot_live.OBSERVATION_PROBE_NAMES)):
        probes.extend([10 + index, 30 + index, base * 100 + index, base * 10])
    raw.update({
        "timing_generation_before": generation,
        "timing_generation_after": generation,
        "timing_words": [
            probe_snapshot_live.OBSERVATION_TIMING_LAYOUT, generation, sample_id,
            tick_ms, module_mask, *probes,
        ],
    })
    buses = []
    for index in range(probe_snapshot_live.OBSERVATION_CAN_BUS_COUNT):
        bus = {key: 0 for key in probe_snapshot_live.OBSERVATION_CAN_FIELDS}
        if index == 0:
            bus.update(active=1, tx_ok=base * 5, tx_done=base * 5,
                       rx_frames=base * 3, status=0)
        buses.extend(bus[key] for key in probe_snapshot_live.OBSERVATION_CAN_FIELDS)
    active_mask = 1 if module_mask & probe_snapshot_live.OBSERVATION_MODULES["can"] else 0
    raw.update({
        "can_generation_before": generation if active_mask else 0,
        "can_generation_after": generation if active_mask else 0,
        "can_words": ([probe_snapshot_live.OBSERVATION_CAN_LAYOUT, generation, sample_id,
                       tick_ms, active_mask, *buses] if active_mask else [0] * probe_snapshot_live.OBSERVATION_CAN_WORDS),
    })
    lcd_values = {key: 0 for key in probe_snapshot_live.OBSERVATION_LCD_FIELDS}
    lcd_values.update(sample_last_us=10 + base, sample_max_us=20 + base,
                      update_last_us=30 + base, update_max_us=40 + base,
                      draw_last_us=50 + base, draw_max_us=60 + base,
                      busy_us=base * 1000, cycles=base * 2, rows=base * 3,
                      spi_bytes=base * 4, spi_errors=base, recover_count=base,
                      init_done=1)
    lcd_enabled = bool(module_mask & probe_snapshot_live.OBSERVATION_MODULES["lcd"])
    raw.update({
        "lcd_generation_before": generation if lcd_enabled else 0,
        "lcd_generation_after": generation if lcd_enabled else 0,
        "lcd_words": ([probe_snapshot_live.OBSERVATION_LCD_LAYOUT, generation, sample_id,
                       tick_ms, module_mask, *[lcd_values[key] for key in probe_snapshot_live.OBSERVATION_LCD_FIELDS]]
                      if lcd_enabled else [0] * probe_snapshot_live.OBSERVATION_LCD_WORDS),
    })
    return raw


def _performance_record(sample_id, tick_ms, base, identity):
    decoded = probe_snapshot_live.decode_performance_snapshot(
        _performance_raw(sample_id, tick_ms, base), identity
    )
    return {"tick_ms": tick_ms, **{key: decoded[key] for key in ("task", "timing", "can", "lcd")}}


class RemoteProfileTests(unittest.TestCase):
    def test_rc_input_preserves_raw_z_and_decodes_signed_boundary(self):
        fields = probe_snapshot_live.RC_INPUT_SNAPSHOT_FIELDS
        addresses = {key: 0x24003000 + i * 4 for i, key in enumerate(fields)}
        values = {key: 0 for key in fields}
        values['raw_mouse_z'] = 0x8000
        server = FakeOpenOCDServer(addresses=addresses, values=values)
        try:
            raw = probe_snapshot_live.read_snapshot(addresses, '127.0.0.1', server.port, 2, fields)
            actual = probe_snapshot_live.decode_rc_input(raw)
        finally:
            server.close()
        self.assertEqual(actual['raw_mouse_z'], 0x8000)
        self.assertEqual(actual['mouse_z'], -32768)
        self.assertIn(f"mdw 0x{addresses['raw_mouse_z']:08x} 1", server.commands)
        self.assertFalse(any(c in {'halt', 'resume', 'reset'} for c in server.commands))

    def test_rc_input_cli_selects_profile_and_keeps_raw_evidence(self):
        fields = probe_snapshot_live.RC_INPUT_SNAPSHOT_FIELDS
        values = {key: 0 for key in fields}
        values.update(mouse_x_lo=0xfe, mouse_x_hi=0xff, key_bitmap_hi=2)
        identity_info = _test_identity_info()
        target_words = _identity_words(identity_info["expected"])
        output = io.StringIO()
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        output_root = temporary.name
        with patch.object(probe_snapshot_live, "resolve_observation_identity", return_value=identity_info), \
             patch.object(probe_snapshot_live, "resolve_symbol_addresses", return_value={}) as resolve, \
             patch.object(probe_snapshot_live, "read_verified_snapshot", return_value=(values, target_words)) as read, \
             contextlib.redirect_stdout(output):
            result = probe_snapshot_live.main([
                "--rc-input", "--elf", __file__, "--samples", "2", "--interval", "0",
                "--output-root", output_root,
            ])
        self.assertEqual(result, 0)
        self.assertIs(resolve.call_args.args[3], fields)
        self.assertIs(read.call_args.args[6], fields)
        session = next((Path(output_root) / "observations").iterdir())
        rows = [json.loads(line) for line in (session / "raw.jsonl").read_text(encoding="utf-8").splitlines()]
        samples = [row for row in rows if row["type"] == "sample"]
        self.assertEqual([row["status"] for row in samples], ["baseline", "running"])
        self.assertEqual(samples[0]["decoded"]["key_bitmap"], 512)
        self.assertEqual(samples[0]["decoded"]["mouse_x"], -2)
        self.assertEqual(samples[0]["decoded"]["mouse_x_hi"], 255)

    def test_rc_input_reads_byte_pairs_and_decodes_signed_mouse(self):
        fields = probe_snapshot_live.RC_INPUT_SNAPSHOT_FIELDS
        addresses = {key: 0x24003000 + i * 4 for i, key in enumerate(fields)}
        values = {key: 0 for key in fields}
        values.update(key_bitmap_lo=0x81, key_bitmap_hi=0x02,
                      mouse_x_lo=0xff, mouse_x_hi=0xff,
                      mouse_y_lo=0x00, mouse_y_hi=0x80,
                      mouse_left=1, mouse_right=0, e_edges=3, f_edges=2)
        server = FakeOpenOCDServer(addresses=addresses, values=values)
        try:
            raw = probe_snapshot_live.read_snapshot(addresses, "127.0.0.1", server.port, 2, fields)
            actual = probe_snapshot_live.decode_rc_input(raw)
        finally:
            server.close()
        self.assertEqual(actual["key_bitmap"], 0x0281)
        self.assertEqual(actual["mouse_x"], -1)
        self.assertEqual(actual["mouse_y"], -32768)
        self.assertEqual((actual["mouse_left"], actual["e_edges"], actual["f_edges"]), (1, 3, 2))
        self.assertEqual(raw, values)
        byte_fields = ("key_bitmap_lo", "key_bitmap_hi", "mouse_x_lo", "mouse_x_hi",
                       "mouse_y_lo", "mouse_y_hi", "mouse_left", "mouse_right", "e_edges", "f_edges")
        for key in byte_fields:
            self.assertIn(f"mdb 0x{addresses[key]:08x} 1", server.commands)
        self.assertEqual(len(server.commands), 2 * len(fields) + 1)
        self.assertFalse(any(c in {"halt", "resume", "reset"} for c in server.commands))

    def test_rc_reads_only_fixed_addresses_and_checks_running(self):
        fields = probe_snapshot_live.RC_SNAPSHOT_FIELDS
        addresses = {key: 0x24001000 + i * 4 for i, key in enumerate(fields)}
        values = {key: 0 for key in fields}
        values.update(accepted_frames=200, online=1, estop_latched=1, fault_flags=28)
        server = FakeOpenOCDServer(addresses=addresses, values=values)
        try:
            actual = probe_snapshot_live.read_snapshot(addresses, "127.0.0.1", server.port, 2, fields)
        finally:
            server.close()
        self.assertEqual(actual, values)
        self.assertEqual(len(server.commands), 2 * len(fields) + 1)
        self.assertFalse(any(c in {"halt", "resume", "reset"} for c in server.commands))

    def test_rc_aborts_on_stop_without_run_control(self):
        fields = probe_snapshot_live.RC_SNAPSHOT_FIELDS
        addresses = {key: 0x24002000 + i * 4 for i, key in enumerate(fields)}
        values = {key: 0 for key in fields}
        server = FakeOpenOCDServer(["running", "halted"], addresses=addresses, values=values)
        try:
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                probe_snapshot_live.read_snapshot(addresses, "127.0.0.1", server.port, 2, fields)
        finally:
            server.close()
        self.assertEqual(len(server.commands), 3)


class FakeOpenOCDServer:
    def __init__(self, state_responses=None, initial_delay=0.0, addresses=None, values=None,
                 update_on_bulk=None):
        self.commands = []
        self.initial_delay = initial_delay
        self.addresses = addresses if addresses is not None else TEST_ADDRESSES
        self.values = values if values is not None else TEST_VALUES
        self.state_responses = list(
            state_responses if state_responses is not None else ["running"] * (len(self.addresses) + 1)
        )
        self.memory_by_address = {}
        for key, address in self.addresses.items():
            value = self.values[key]
            values = value if isinstance(value, list) else [value]
            for index, word in enumerate(values):
                self.memory_by_address[address + index * 4] = word
        self.update_on_bulk = dict(update_on_bulk or {})

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
                        match = re.fullmatch(r"(mdw|mdb) (0x[0-9a-f]+) ([1-9][0-9]*)", command)
                        if not match:
                            response = "invalid command\r\n"
                        else:
                            read_command = match.group(1)
                            address = int(match.group(2), 16)
                            count = int(match.group(3))
                            if count == 1:
                                value = owner.memory_by_address[address]
                                width = 2 if read_command == "mdb" else 8
                                response = f"0x{address:08x}: {value:0{width}x}\r\n"
                            elif read_command == "mdw":
                                words = [owner.memory_by_address[address + index * 4]
                                         for index in range(count)]
                                lines = []
                                for base_index in range(0, count, 8):
                                    base = address + base_index * 4
                                    row = " ".join(f"{word:08x}" for word in words[base_index:base_index + 8])
                                    lines.append(f"0x{base:08x}: {row}")
                                response = "\r\n".join(lines) + "\r\n"
                                owner.memory_by_address.update(owner.update_on_bulk)
                            else:
                                response = "invalid command\r\n"
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
    def test_summary_and_adc_help_use_complete_chinese_text(self):
        summary, complete = probe_snapshot_live._short_summary(
            [{"status": "running"}],
            {"profile_name": "bench_safe", "role_name": "single", "build_id": "abc"},
            None,
        )
        self.assertTrue(complete)
        self.assertIn("观测摘要", summary)
        self.assertIn("原始样本数：1", summary)
        self.assertNotIn("???", summary)

        output = io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(SystemExit):
            probe_snapshot_live.main(["--help"])
        self.assertIn("读取固定 ADC 诊断字段", output.getvalue())

    def test_performance_window_skips_missed_samples_without_catch_up(self):
        identity_info = _test_identity_info()

        class FakeClock:
            now = 0.0

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        clock = FakeClock()
        target_words = _identity_words(identity_info["expected"])
        reads = []

        def slow_snapshot(*_args):
            clock.now += 2.5
            reads.append(clock.now)
            return {}, target_words

        with tempfile.TemporaryDirectory() as output_root:
            with patch.object(probe_snapshot_live, "resolve_observation_identity",
                              return_value=identity_info), \
                 patch.object(probe_snapshot_live, "resolve_symbol_addresses", return_value={}), \
                 patch.object(probe_snapshot_live, "read_verified_snapshot", side_effect=slow_snapshot), \
                 patch.object(probe_snapshot_live, "decode_performance_snapshot",
                              return_value={"tick_ms": 100}), \
                 patch.object(probe_snapshot_live, "classify_change", return_value="running"), \
                 patch.object(probe_snapshot_live, "summarize_performance",
                              return_value=("complete\n", True)), \
                 patch.object(probe_snapshot_live.time, "monotonic", side_effect=clock.monotonic), \
                 patch.object(probe_snapshot_live.time, "sleep", side_effect=clock.sleep), \
                 contextlib.redirect_stdout(io.StringIO()):
                result = probe_snapshot_live.main([
                    "--performance", "--duration", "5", "--interval", "1", "--elf", __file__,
                    "--output-root", output_root,
                ])
            self.assertEqual(result, 0)
            self.assertEqual(reads, [2.5, 5.5])
            session = next((Path(output_root) / "observations").iterdir())
            samples = [
                json.loads(line) for line in (session / "raw.jsonl").read_text(encoding="utf-8").splitlines()
                if json.loads(line)["type"] == "sample"
            ]
            self.assertEqual([item["missed_schedule_slots_before_sample"] for item in samples], [0, 2])

    def test_performance_window_rejects_duration_outside_limits(self):
        for duration in ("4.99", "300.01"):
            with tempfile.TemporaryDirectory() as output_root:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                    probe_snapshot_live.main([
                        "--performance", "--duration", duration, "--elf", __file__,
                        "--output-root", output_root,
                    ])
                self.assertEqual(raised.exception.code, 2)
                self.assertFalse((Path(output_root) / "observations").exists())

    def test_identity_record_validates_fixed_layout_and_build_id_words(self):
        expected = _test_identity_info()["expected"]
        actual = probe_snapshot_live.decode_identity_words(_identity_words(expected))
        self.assertEqual(actual["build_id_words"], expected["build_id_words"])
        self.assertEqual(actual["module_mask"], expected["module_mask"])
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.decode_identity_words(_identity_words(expected)[:-1])
        invalid = _identity_words(expected)
        invalid[0] ^= 1
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.decode_identity_words(invalid)

    def test_offline_identity_resolver_reads_build_id_template_and_ram_layout(self):
        identity = _test_identity_info()["expected"]
        template = [identity[key] for key in (
            "magic", "version", "size_bytes", "layout", "profile", "role", "module_mask",
        )]
        with tempfile.TemporaryDirectory() as temporary:
            elf = Path(temporary) / "identity.elf"
            _write_identity_elf(elf, template)
            gdb_output = "\n".join((
                "identity_words=0x24001000", "ram_start=0x24000000",
                "ram_end=0x24020000", "template_address=0x8001000",
            ))
            readelf_result = subprocess.CompletedProcess([], 0, f"Build ID: {identity['build_id']}\n", "")
            with patch.object(probe_snapshot_live.subprocess, "run", return_value=readelf_result), \
                 patch.object(probe_snapshot_live, "_run_gdb", return_value=gdb_output):
                resolved = probe_snapshot_live.resolve_observation_identity("gdb", "readelf", str(elf), 1.0)
            self.assertEqual(resolved["expected"]["build_id"], identity["build_id"])
            self.assertEqual(resolved["expected"]["module_mask"], identity["module_mask"])
            self.assertEqual(resolved["identity_address"], 0x24001000)

            invalid_template = list(template)
            invalid_template[3] ^= 1
            _write_identity_elf(elf, invalid_template)
            with patch.object(probe_snapshot_live.subprocess, "run", return_value=readelf_result), \
                 patch.object(probe_snapshot_live, "_run_gdb", return_value=gdb_output), \
                 self.assertRaisesRegex(probe_snapshot_live.ProbeIOError, "layout"):
                probe_snapshot_live.resolve_observation_identity("gdb", "readelf", str(elf), 1.0)

    def test_target_identity_mismatch_stops_before_business_field_reads(self):
        identity_info = _test_identity_info()
        wrong_words = _identity_words(identity_info["expected"])
        wrong_words[9] = 2
        identity_address = identity_info["identity_address"]
        business_address = identity_address + 0x100
        fields = {"tick_ms": probe_snapshot_live.SnapshotField("uwTick", "mdw", 32)}
        addresses = {"tick_ms": business_address}
        server = FakeOpenOCDServer(
            state_responses=["running"] * 8,
            addresses={"identity_words": identity_address, "tick_ms": business_address},
            values={"identity_words": wrong_words, "tick_ms": 99},
        )
        try:
            with self.assertRaisesRegex(probe_snapshot_live.ProbeIOError, "does not match"):
                probe_snapshot_live.read_verified_snapshot(
                    addresses, identity_address, identity_info["expected"],
                    "127.0.0.1", server.port, 1.0, fields, identity_info["ram_bounds"],
                )
        finally:
            server.close()
        self.assertEqual(server.commands[0], "targets")
        self.assertEqual(server.commands[1], f"mdw 0x{identity_address:08x} 12")
        self.assertNotIn(f"mdw 0x{business_address:08x} 1", server.commands)

    def test_performance_decoder_unpacks_all_four_groups_and_publication_time(self):
        identity = _test_identity_info()["expected"]
        decoded = probe_snapshot_live.decode_performance_snapshot(
            _performance_raw(3, 15000, 7), identity
        )
        self.assertEqual(decoded["task"]["sample_id"], 3)
        self.assertEqual(decoded["task"]["captured_ms"], 15000)
        self.assertEqual(decoded["timing"]["sample_id"], 3)
        self.assertEqual(decoded["timing"]["probes"]["motor_task"]["calls"], 70)
        self.assertEqual(decoded["can"]["buses"][0]["tx_ok"], 35)
        self.assertEqual(decoded["lcd"]["values"]["draw_last_us"], 57)

    def test_performance_decoder_distinguishes_unpublished_and_disabled_groups(self):
        identity = _test_identity_info()["expected"]
        raw = _performance_raw(1, 1000, 1)
        raw.update(stats_generation_before=0, stats_generation_after=0,
                   stats_sample_id=0, stats_captured_ms=0, stats_count=0,
                   stats_task_total=0, stats_truncated=0)
        raw["timing_generation_before"] = raw["timing_generation_after"] = 0
        raw["timing_words"] = [0] * probe_snapshot_live.OBSERVATION_TIMING_WORDS
        decoded = probe_snapshot_live.decode_performance_snapshot(raw, identity)
        self.assertEqual(decoded["task"]["status"], "not-published")
        self.assertEqual(decoded["timing"]["status"], "not-published")

        mask = identity["module_mask"] & ~probe_snapshot_live.OBSERVATION_MODULES["can"]
        raw = _performance_raw(2, 6000, 2, module_mask=mask)
        disabled = dict(identity, module_mask=mask)
        decoded = probe_snapshot_live.decode_performance_snapshot(raw, disabled)
        self.assertEqual(decoded["can"]["status"], "not-enabled")
        self.assertEqual(decoded["can"]["buses"], [])

    def test_performance_summary_ignores_duplicate_publication_and_recomputes_from_jsonl(self):
        identity = _test_identity_info()["expected"]
        first_raw = _performance_raw(1, 1000, 10)
        duplicate_raw = _performance_raw(1, 3000, 50)
        last_raw = _performance_raw(2, 6000, 20)
        decoded = [probe_snapshot_live.decode_performance_snapshot(raw, identity)
                   for raw in (first_raw, duplicate_raw, last_raw)]
        rows = []
        for index, item in enumerate(decoded):
            rows.append({"type": "sample", "tick_ms": item["tick_ms"], "decoded": item,
                         "raw_words": (first_raw, duplicate_raw, last_raw)[index]})
        with tempfile.TemporaryDirectory() as temporary:
            raw_path = Path(temporary) / "raw.jsonl"
            raw_path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            saved = [json.loads(line) for line in raw_path.read_text(encoding="utf-8").splitlines()]
        summary, complete = probe_snapshot_live.summarize_performance(saved, identity)
        self.assertTrue(complete)
        self.assertIn("40.0%", summary)
        self.assertIn('"tx_ok": 50', summary)
        self.assertIn("窗口增加 20", summary)
        self.assertEqual(probe_snapshot_live._counter_delta(0xFFFFFFFE, 3), 5)

    def test_performance_summary_rejects_one_record_or_stale_publication(self):
        identity = _test_identity_info()["expected"]
        decoded = probe_snapshot_live.decode_performance_snapshot(
            _performance_raw(1, 1000, 1), identity
        )
        summary, complete = probe_snapshot_live.summarize_performance(
            [{"tick_ms": 13000, **decoded}], identity
        )
        self.assertFalse(complete)
        self.assertIn("不可计算", summary)

    def test_observation_session_saves_identity_failure_and_interruption(self):
        with tempfile.TemporaryDirectory() as output_root:
            with patch.object(probe_snapshot_live, "resolve_observation_identity",
                              side_effect=probe_snapshot_live.ProbeIOError("no Build ID")), \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                result = probe_snapshot_live.main([
                    "--elf", __file__, "--output-root", output_root,
                ])
            self.assertEqual(result, 2)
            session = next((Path(output_root) / "observations").iterdir())
            self.assertIn("no Build ID", (session / "summary.txt").read_text(encoding="utf-8"))
            rows = [json.loads(line) for line in (session / "raw.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual(rows[-1]["type"], "error")

        identity_info = _test_identity_info()
        target_words = _identity_words(identity_info["expected"])
        with tempfile.TemporaryDirectory() as output_root:
            with patch.object(probe_snapshot_live, "resolve_observation_identity", return_value=identity_info), \
                 patch.object(probe_snapshot_live, "resolve_symbol_addresses", return_value={}), \
                 patch.object(probe_snapshot_live, "read_verified_snapshot", side_effect=KeyboardInterrupt), \
                 contextlib.redirect_stdout(io.StringIO()):
                result = probe_snapshot_live.main([
                    "--performance", "--duration", "5", "--interval", "10", "--elf", __file__,
                    "--output-root", output_root,
                ])
            self.assertEqual(result, 130)
            session = next((Path(output_root) / "observations").iterdir())
            rows = [json.loads(line) for line in (session / "raw.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual(rows[-1]["type"], "interrupted")
            self.assertTrue((session / "summary.txt").is_file())
    def test_rtos_profile_unpacks_stack_runtime_and_fixed_task_names(self):
        addresses, values = _rtos_fixture()
        decoded = probe_snapshot_live.decode_rtos_snapshot(values)
        self.assertEqual(decoded["task_count"], 2)
        self.assertEqual(decoded["task_total"], 2)
        self.assertFalse(decoded["task_table_truncated"])
        self.assertEqual(decoded["tasks"], [
            {"name": "IDLE", "runtime_counter": 11, "priority": 0,
             "stack_high_water_words": 120, "stack_high_water_bytes": 480,
             "stack_capacity_words": 128, "stack_margin_permille": 937},
            {"name": "motortask", "runtime_counter": 22, "priority": 3,
             "stack_high_water_words": 80, "stack_high_water_bytes": 320,
             "stack_capacity_words": 512, "stack_margin_permille": 156},
        ])
        self.assertEqual(decoded["can_rx_dispatch_max_us"], 700)

    def test_rtos_profile_reads_only_fixed_ram_blocks_while_running(self):
        fields = probe_snapshot_live.RTOS_SNAPSHOT_FIELDS
        addresses, values = _rtos_fixture()
        server = FakeOpenOCDServer(addresses=addresses, values=values)
        try:
            raw = probe_snapshot_live.read_snapshot(
                addresses, "127.0.0.1", server.port, timeout=2.0, fields=fields
            )
        finally:
            server.close()
        self.assertEqual(raw, values)
        self.assertIn(f"mdw 0x{addresses['stats_names_words']:08x} 36", server.commands)
        self.assertIn(f"mdw 0x{addresses['stats_runtime']:08x} 12", server.commands)
        self.assertTrue(all(command == "targets" or command.startswith(("mdb ", "mdw "))
                            for command in server.commands))
        self.assertFalse(any(command in {"halt", "resume", "reset"} for command in server.commands))

    def test_rtos_profile_rejects_changed_or_unpublished_task_table(self):
        _, values = _rtos_fixture()
        changed = dict(values, stats_generation_after=10)
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.decode_rtos_snapshot(changed)
        unpublished = dict(values, stats_generation_before=9, stats_generation_after=9)
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.decode_rtos_snapshot(unpublished)

    def test_rtos_profile_reports_twelve_entry_truncation_without_claiming_completeness(self):
        _, values = _rtos_fixture()
        values.update(stats_count=12, stats_task_total=13, stats_truncated=1)
        decoded = probe_snapshot_live.decode_rtos_snapshot(values)
        self.assertTrue(decoded["task_table_truncated"])
        self.assertEqual(len(decoded["tasks"]), 12)
        values["stats_truncated"] = 0
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.decode_rtos_snapshot(values)

    def test_rtos_profile_rejects_layout_and_capacity_mismatch(self):
        _, values = _rtos_fixture()
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.decode_rtos_snapshot(dict(values, stats_layout=0))
        too_much_free_stack = dict(values)
        too_much_free_stack["stats_stack_words"] = [129 | (80 << 16)] + [0] * 5
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.decode_rtos_snapshot(too_much_free_stack)
        addresses, _ = _rtos_fixture()
        addresses["stats_names_words"] = probe_snapshot_live.SNAPSHOT_RAM_END - 4
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live._validate_snapshot_addresses(addresses,
                                                              probe_snapshot_live.RTOS_SNAPSHOT_FIELDS)

    def test_init_profile_resolves_fixed_startup_diagnostics(self):
        fields = probe_snapshot_live.INIT_SNAPSHOT_FIELDS
        command = probe_snapshot_live.build_symbol_command("arm-none-eabi-gdb", "firmware.elf", fields)
        joined = " ".join(command).lower()
        self.assertEqual(len([arg for arg in command if arg.startswith("printf ")]), len(fields))
        self.assertIn("init_diagnostic_snapshot.failure_count", joined)
        self.assertIn("init_diagnostic_snapshot.heartbeat", joined)
        self.assertNotIn("target ", joined)

    def test_init_profile_reads_only_fixed_fields_while_running(self):
        fields = probe_snapshot_live.INIT_SNAPSHOT_FIELDS
        server = FakeOpenOCDServer(addresses=TEST_INIT_ADDRESSES, values=TEST_INIT_VALUES)
        try:
            actual = probe_snapshot_live.read_snapshot(
                TEST_INIT_ADDRESSES, "127.0.0.1", server.port, 1.0, fields
            )
        finally:
            server.close()
        self.assertEqual(actual, TEST_INIT_VALUES)
        self.assertEqual(len(server.commands), 2 * len(fields) + 1)
        reads = [command for command in server.commands if command != "targets"]
        self.assertTrue(all(command.startswith("mdb ") or command.startswith("mdw ") for command in reads))
        self.assertFalse(any(command in {"halt", "reset", "resume"} for command in server.commands))

    def test_init_cli_selects_fixed_profile(self):
        fields = probe_snapshot_live.INIT_SNAPSHOT_FIELDS
        identity_info = _test_identity_info()
        target_words = _identity_words(identity_info["expected"])
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        output_root = temporary.name
        with patch.object(probe_snapshot_live, "resolve_observation_identity", return_value=identity_info), \
             patch.object(probe_snapshot_live, "resolve_symbol_addresses", return_value=TEST_INIT_ADDRESSES) as resolve, \
             patch.object(probe_snapshot_live, "read_verified_snapshot", return_value=(TEST_INIT_VALUES, target_words)) as read, \
             contextlib.redirect_stdout(io.StringIO()):
            result = probe_snapshot_live.main([
                "--init", "--elf", __file__, "--samples", "2", "--interval", "0",
                "--output-root", output_root,
            ])
        self.assertEqual(result, 0)
        self.assertIs(resolve.call_args.args[3], fields)
        self.assertIs(read.call_args.args[6], fields)
        session = next((Path(output_root) / "observations").iterdir())
        rows = [json.loads(line) for line in (session / "raw.jsonl").read_text(encoding="utf-8").splitlines()]
        samples = [row for row in rows if row["type"] == "sample"]
        self.assertEqual(samples[-1]["decoded"]["init_failure_count"], 3)
        self.assertEqual(samples[-1]["status"], "running")

    def test_gdb_command_resolves_addresses_offline_without_target_connection(self):
        command = probe_snapshot_live.build_symbol_command("arm-none-eabi-gdb", "firmware.elf")
        gdb_commands = [command[index + 1] for index, arg in enumerate(command[:-1]) if arg == "--ex"]

        self.assertEqual(
            {
                key: (field.expression, field.command, field.width_bits, field.alignment_bytes)
                for key, field in probe_snapshot_live.SNAPSHOT_FIELDS.items()
            },
            EXPECTED_FIELD_SPECS,
        )
        self.assertEqual(len(gdb_commands), len(EXPECTED_EXPRESSIONS) + 2)
        self.assertEqual(gdb_commands[:2], ["set pagination off", "set confirm off"])
        for key, expression in EXPECTED_EXPRESSIONS.items():
            self.assertIn(f'printf "{key}=0x%x\\n", (unsigned int)&({expression})', gdb_commands)
        joined = " ".join(gdb_commands).lower()
        self.assertNotIn("target ", joined)
        self.assertNotIn("remote", joined)
        self.assertNotIn("x/", joined)

    def test_imu_profile_resolves_only_fixed_diagnostic_symbols(self):
        fields = probe_snapshot_live.IMU_SNAPSHOT_FIELDS
        command = probe_snapshot_live.build_symbol_command(
            "arm-none-eabi-gdb", "firmware.elf", fields
        )
        gdb_commands = [command[index + 1] for index, arg in enumerate(command[:-1]) if arg == "--ex"]
        joined = " ".join(gdb_commands)

        self.assertEqual(len(gdb_commands), len(fields) + 2)
        for key, field in fields.items():
            self.assertIn(f'printf "{key}=0x%x\\n", (unsigned int)&({field.expression})', gdb_commands)
        self.assertIn("bmi088_init_stage", joined)
        self.assertIn("bmi088_sample_sequence_diag", joined)
        self.assertIn("imu_heater_duty_diag", joined)
        self.assertNotIn("vision_", joined)
        self.assertNotIn("target ", joined.lower())

    def test_imu_profile_reads_only_approved_aligned_symbols_while_running(self):
        fields = probe_snapshot_live.IMU_SNAPSHOT_FIELDS
        server = FakeOpenOCDServer(
            addresses=TEST_IMU_ADDRESSES,
            values=TEST_IMU_VALUES,
        )
        try:
            snapshot = probe_snapshot_live.read_snapshot(
                TEST_IMU_ADDRESSES,
                "127.0.0.1",
                server.port,
                timeout=1.0,
                fields=fields,
            )
        finally:
            server.close()

        self.assertEqual(snapshot, TEST_IMU_VALUES)
        reads = [command for command in server.commands if command != "targets"]
        self.assertEqual(len(reads), len(fields))
        self.assertTrue(all(command.startswith("mdw ") for command in reads))
        self.assertFalse(any(command in {"halt", "reset", "resume"} for command in server.commands))

    def test_imu_profile_rejects_unlisted_symbol_before_connecting(self):
        server = FakeOpenOCDServer()
        invalid = dict(TEST_IMU_ADDRESSES)
        invalid["vision_rx_count"] = TEST_ADDRESSES["vision_rx_count"]
        try:
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                probe_snapshot_live.read_snapshot(
                    invalid,
                    "127.0.0.1",
                    server.port,
                    timeout=1.0,
                    fields=probe_snapshot_live.IMU_SNAPSHOT_FIELDS,
                )
        finally:
            server.close()

        self.assertEqual(server.commands, [])

    def test_symbol_address_parser_requires_all_addresses(self):
        output = "\n".join(
            f"{key}=0x{address:x}" for key, address in TEST_ADDRESSES.items()
        )
        self.assertEqual(probe_snapshot_live.parse_addresses(output), TEST_ADDRESSES)
        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.parse_addresses("tick_ms=0x2400ac30")

    def test_symbol_address_parser_rejects_addresses_outside_linker_ram(self):
        invalid = dict(TEST_ADDRESSES)
        invalid["vision_rx_count"] = 0x40000000
        output = "\n".join(f"{key}=0x{address:x}" for key, address in invalid.items())

        with self.assertRaises(probe_snapshot_live.ProbeIOError):
            probe_snapshot_live.parse_addresses(output)

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

    def test_snapshot_rejects_addresses_outside_linker_ram_before_connecting(self):
        server = FakeOpenOCDServer()
        invalid = dict(TEST_ADDRESSES)
        invalid["vision_rx_count"] = 0x40000000
        try:
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                probe_snapshot_live.read_snapshot(
                    invalid, "127.0.0.1", server.port, timeout=1.0
                )
        finally:
            server.close()

        self.assertEqual(server.commands, [])

    def test_telnet_command_whitelist_rejects_run_control(self):
        server = FakeOpenOCDServer()
        try:
            with probe_snapshot_live.OpenOCDTelnetClient(
                "127.0.0.1", server.port, timeout=1.0,
                allowed_addresses=TEST_ADDRESSES,
            ) as client:
                with self.assertRaises(ValueError):
                    client.command("halt")
        finally:
            server.close()

        self.assertEqual(server.commands, [])

    def test_telnet_memory_command_must_use_a_resolved_elf_symbol_address(self):
        server = FakeOpenOCDServer()
        try:
            with probe_snapshot_live.OpenOCDTelnetClient(
                "127.0.0.1", server.port, timeout=1.0,
                allowed_addresses=TEST_ADDRESSES,
            ) as client:
                with self.assertRaises(ValueError):
                    client.command("mdw 0x24000000 1")
                with self.assertRaises(ValueError):
                    client.command(f"mdb 0x{TEST_ADDRESSES['tick_ms']:08x} 1")
        finally:
            server.close()

        self.assertEqual(server.commands, [])

    def test_telnet_read_timeout_is_reported(self):
        server = FakeOpenOCDServer(initial_delay=0.2)
        try:
            with self.assertRaises(probe_snapshot_live.ProbeIOError):
                with probe_snapshot_live.OpenOCDTelnetClient(
                    "127.0.0.1", server.port, timeout=0.05,
                    allowed_addresses=TEST_ADDRESSES,
                ):
                    self.fail("delayed prompt unexpectedly connected")
        finally:
            server.close()

    def test_telnet_client_rejects_non_loopback_hosts(self):
        for host in ("192.0.2.1", "localhost"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                probe_snapshot_live.OpenOCDTelnetClient(
                    host, 4444, timeout=1.0, allowed_addresses=TEST_ADDRESSES
                )

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
        watchdog_flag_without_tick_rollback = {
            "tick_ms": 1200,
            "vision_rx_count": 25,
            "vision_tx_count": 50,
            "vision_crc_error_count": 1,
            "iwdg_reset_flag": 1,
        }
        tick_rollback_without_watchdog_flag = {
            "tick_ms": 20,
            "vision_rx_count": 25,
            "vision_tx_count": 50,
            "vision_crc_error_count": 1,
            "iwdg_reset_flag": 0,
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
            probe_snapshot_live.classify_change(before, watchdog_flag_without_tick_rollback),
            "target-reset-iwdg",
        )
        self.assertEqual(
            probe_snapshot_live.classify_change(before, tick_rollback_without_watchdog_flag),
            "target-reset-other-or-unknown",
        )
        self.assertEqual(
            probe_snapshot_live.classify_change(before, counter_decrease_without_reset_evidence),
            "counter-decrease-unclassified",
        )
        self.assertEqual(probe_snapshot_live.classify_change(natural_wrap, after_wrap), "running")


if __name__ == "__main__":
    unittest.main()
