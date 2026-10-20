#!/usr/bin/env python3
"""MC02-0037 只读 ADC 采集和 VIN 误差报告的可移植回归。"""
from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(Path(__file__).parent))

import adc_vin_measurement as measurement
import probe_snapshot_live as probe
import test_probe_snapshot_live as probe_tests


TEST_SOURCE_SHA = "a" * 40
TEST_ELF_BYTES = b"test-only firmware ELF identity"


def volts_bits(raw: int, vref_mv: int = 3300, divider_milli: int = 11000) -> int:
    volts = raw * vref_mv * divider_milli / (65535 * 1_000_000)
    return struct.unpack("<I", struct.pack("<f", volts))[0]


def adc_record(sequence: int = 1, tick: int = 100, captured: int | None = None,
               timestamp: int | None = None, raw_vcc: int = 21845,
               status: int = 1, config_valid: int = 1) -> dict[str, int]:
    captured = tick if captured is None else captured
    timestamp = captured if timestamp is None else timestamp
    record = {key: 0 for key in probe.ADC_SNAPSHOT_FIELDS}
    record.update({
        "adc_version": 2,
        "adc_layout": probe.ADC_DIAGNOSTIC_LAYOUT,
        "adc_captured_ms": captured,
        "adc_status": status,
        "adc_raw_vcc_in": raw_vcc,
        "adc_raw_key": 777,
        "adc_sample_sequence": sequence,
        "adc_sample_timestamp_ms": timestamp,
        "adc_error_flags": 0,
        "adc_hal_error_code": 0,
        "adc_voltage_config_valid": config_valid,
        "adc_volts_bits": volts_bits(raw_vcc) if config_valid else 0,
        "adc_vref_mv": 3300 if config_valid else 0,
        "adc_full_scale": 65535 if config_valid else 0,
        "adc_divider_milli": 11000 if config_valid else 0,
        "tick_ms": tick,
        "iwdg_reset_flag": 0,
        "adc_version_after": 2,
    })
    return record


def qualified_session(elf_digest: str) -> dict:
    """构造仅供数学单测使用的完整依据，不代表当前实物登记已经满足条件。"""
    return {
        "schema": 1,
        "board": {"schematic_match_confirmed": True, "verified_range_v": [12, 24]},
        "meter": {"model": "synthetic", "resolution_v": 0.1, "accuracy_percent": 1,
                  "accuracy_digits": 1, "accuracy_source": "synthetic spec",
                  "calibration_record": "synthetic calibration"},
        "power": {"model": "synthetic"},
        "firmware": {"source_sha": TEST_SOURCE_SHA, "elf_sha256": elf_digest,
                     "flash_verified": True},
        "budget": {"source": "synthetic applicable maximum specification", "adc_applicable": True,
                   "adc_max_counts": 4, "reference_min_v": 3.2, "reference_max_v": 3.4,
                   "top_ohm": 100000, "bottom_ohm": 10000, "top_tolerance": 0.01,
                   "bottom_tolerance": 0.01, "resistor_spec_confirmed": True},
    }


def write_point(directory: Path, session_bytes: bytes, elf_digest: str,
                phase: str, setpoint: float, meter_v: float | None = None,
                rows: int = 30) -> Path:
    path = directory / f"{phase}-{str(setpoint).replace('.', '-')}.csv"
    session_path = path.with_suffix(".session.json")
    session_path.write_bytes(session_bytes)
    session_digest = hashlib.sha256(session_bytes).hexdigest()
    target_v = setpoint if meter_v is None else meter_v
    raw = round(target_v * 65535 / (3.3 * 11))
    actual_v = raw * 3300 * 11000 / (65535 * 1_000_000)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=measurement.COLUMNS)
        writer.writeheader()
        for index in range(rows):
            captured = 1000 + index * 250
            record = adc_record(index + 1, captured, raw_vcc=raw)
            metadata = {
                "point_id": path.stem,
                "phase": phase,
                "setpoint_v": setpoint,
                "meter_v": actual_v,
                "current_a": 0.16,
                "reference_v": 3.3,
                "elf_sha256": elf_digest,
                "session_sha256": session_digest,
            }
            writer.writerow({**metadata, **record, "adc_volts": probe.decode_adc(record)["adc_volts"]})
    return path


class ADCRecordTests(unittest.TestCase):
    def test_fixed_elf_fields_use_one_bounded_block_and_recheck_the_version(self):
        fields = probe.ADC_SNAPSHOT_FIELDS
        self.assertEqual(fields["adc_raw_vcc_in"].expression, "bsp_adc_diagnostic.raw_vcc_in")
        self.assertEqual(fields["adc_version"].width_bits, 32)
        values = adc_record()
        addresses = {key: 0x24001000 + index * 4 for index, key in enumerate(fields)}
        server_values = {key: values[key] for key in fields}
        server = probe_tests.FakeOpenOCDServer(
            addresses=addresses,
            values=server_values,
            state_responses=["running", "running"],
        )
        try:
            result = probe.read_snapshot(addresses, "127.0.0.1", server.port, 1.0, fields)
        finally:
            server.close()
        self.assertEqual(result["adc_version_after"], result["adc_version"])
        reads = [c for c in server.commands if c.startswith(("mdw ", "mdb "))]
        self.assertEqual(len(reads), 4)
        self.assertEqual(reads, [
            f"mdw 0x{addresses['adc_version']:08x} 15",
            f"mdw 0x{addresses['adc_version']:08x} 1",
            f"mdw 0x{addresses['tick_ms']:08x} 1",
            f"mdb 0x{addresses['iwdg_reset_flag']:08x} 1",
        ])
        self.assertEqual(server.commands[0], "targets")
        self.assertEqual(server.commands[-1], "targets")
        self.assertFalse(any(c in {"halt", "reset", "resume"} for c in server.commands))
        self.assertAlmostEqual(probe.decode_adc(result)["adc_volts"], 12.1, places=6)

    def test_fixed_elf_adc_block_rejects_a_version_change_during_read(self):
        fields = probe.ADC_SNAPSHOT_FIELDS
        record = adc_record()
        addresses = {key: 0x24001000 + index * 4 for index, key in enumerate(fields)}
        server = probe_tests.FakeOpenOCDServer(
            addresses=addresses,
            values={key: record[key] for key in fields},
            state_responses=["running", "running"],
            update_on_bulk={addresses["adc_version"]: 4},
        )
        try:
            result = probe.read_snapshot(addresses, "127.0.0.1", server.port, 1.0, fields)
        finally:
            server.close()
        self.assertEqual(result["adc_version"], 2)
        self.assertEqual(result["adc_version_after"], 4)
        with self.assertRaisesRegex(probe.ProbeIOError, "changed during reading"):
            probe.decode_adc(result)

    def test_adc_block_read_is_limited_to_the_complete_elf_resolved_record(self):
        fields = probe.ADC_SNAPSHOT_FIELDS
        addresses = {key: 0x24001000 + index * 4 for index, key in enumerate(fields)}
        server = probe_tests.FakeOpenOCDServer(addresses=addresses,
            values={key: 0 for key in fields})
        try:
            with probe.OpenOCDTelnetClient(
                "127.0.0.1", server.port, timeout=1.0,
                allowed_addresses=addresses, fields=fields,
            ) as client:
                with self.assertRaises(ValueError):
                    client.command(f"mdw 0x{addresses['adc_version']:08x} 14")
                with self.assertRaises(ValueError):
                    client.command("mdw 0x24000000 15")
        finally:
            server.close()
        self.assertEqual(server.commands, [])

    def test_decoder_accepts_zero_raw_and_twenty_ms_boundary(self):
        record = adc_record(raw_vcc=0, tick=120, captured=120, timestamp=100)
        decoded = probe.decode_adc(record)
        self.assertTrue(decoded["adc_usable"])
        self.assertEqual(decoded["adc_volts"], 0.0)
        self.assertEqual(decoded["adc_sample_age_ms"], 20)
        record["adc_sample_timestamp_ms"] = 99
        with self.assertRaises(probe.ProbeIOError):
            probe.decode_adc(record)

    def test_decoder_handles_tick_wrap_and_rejects_invalid_or_torn_records(self):
        record = adc_record(tick=10, captured=9, timestamp=probe.UINT32_MAX - 10)
        self.assertEqual(probe.decode_adc(record)["adc_sample_age_ms"], 20)
        for key, value in (("adc_version", 0), ("adc_version", 3), ("adc_layout", 0),
                           ("adc_raw_key", 65536), ("adc_voltage_config_valid", 2),
                           ("iwdg_reset_flag", 2), ("adc_status", 4)):
            broken = adc_record()
            broken[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(probe.ProbeIOError):
                probe.decode_adc(broken)
        broken = adc_record()
        broken["adc_volts_bits"] = 0x7FC00000
        with self.assertRaisesRegex(probe.ProbeIOError, "not finite"):
            probe.decode_adc(broken)
        broken = adc_record(config_valid=0)
        broken["adc_volts_bits"] = volts_bits(21845)
        with self.assertRaisesRegex(probe.ProbeIOError, "nonzero voltage"):
            probe.decode_adc(broken)
        broken = adc_record()
        del broken["adc_raw_key"]
        with self.assertRaisesRegex(probe.ProbeIOError, "missing or unexpected"):
            probe.decode_adc(broken)

    def test_acquisition_retries_unpublished_and_duplicate_records(self):
        records = [adc_record(), adc_record(), adc_record(sequence=2, tick=350, captured=350)]
        logs = []
        with patch.object(measurement.time, "sleep"):
            rows = measurement.collect(lambda: records.pop(0), 2, 0.0, logs.append)
        self.assertEqual([row["adc_sample_sequence"] for row in rows], [1, 2])
        self.assertTrue(any(row.get("rejected") == "duplicate sample" for row in logs))
        torn = adc_record()
        torn["adc_version"] = 1
        logs = []
        with patch.object(measurement.time, "sleep"):
            with self.assertRaisesRegex(ValueError, "only 0/1"):
                measurement.collect(lambda: torn, 1, 0.0, logs.append)
        self.assertEqual(len(logs), 3)


class MeasurementToolTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.elf = self.root / "firmware.elf"
        self.elf.write_bytes(TEST_ELF_BYTES)
        self.elf_digest = hashlib.sha256(TEST_ELF_BYTES).hexdigest()
        self.session_path = self.root / "session.json"
        self.session_path.write_text(json.dumps(qualified_session(self.elf_digest)), encoding="utf-8")

    def test_capture_requires_exact_elf_identity_and_fixed_measurement_point(self):
        session = measurement.load_session(self.session_path)
        self.assertEqual(measurement.validate_identity(session, self.elf), self.elf_digest)
        args = SimpleNamespace(
            session=self.session_path, elf=self.elf, gdb="arm-none-eabi-gdb", host="127.0.0.1",
            telnet_port=4444, output=self.root / "bad-point.csv", point="bad-point", phase="up",
            setpoint=13.0, meter_v=13.0, current_a=0.16, reference_v=3.3,
            samples=30, interval=0.5, settle=5.0, timeout=1.0,
        )
        with patch.object(measurement.probe, "resolve_symbol_addresses") as resolve:
            with self.assertRaisesRegex(ValueError, "required up test point"):
                measurement.capture(args)
        resolve.assert_not_called()
        data = json.loads(self.session_path.read_text(encoding="utf-8"))
        data["firmware"]["elf_sha256"] = "0" * 64
        self.session_path.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "verified ELF"):
            measurement.validate_identity(measurement.load_session(self.session_path), self.elf)

    def test_capture_writes_only_complete_csv_and_preserves_raw_attempts(self):
        args = SimpleNamespace(
            session=self.session_path, elf=self.elf, gdb="arm-none-eabi-gdb", host="127.0.0.1",
            telnet_port=4444, output=self.root / "baseline.csv", point="baseline-12-5",
            phase="baseline", setpoint=12.5, meter_v=12.5, current_a=0.16, reference_v=3.3,
            samples=30, interval=0.5, settle=5.0, timeout=1.0,
        )
        samples = iter(adc_record(i + 1, 1000 + i * 250) for i in range(args.samples))
        addresses = {key: 0x24001000 + index * 4 for index, key in enumerate(probe.ADC_SNAPSHOT_FIELDS)}
        with patch.object(measurement.probe, "resolve_symbol_addresses", return_value=addresses), \
             patch.object(measurement.probe, "read_snapshot", side_effect=lambda *args: next(samples)), \
             patch.object(measurement.time, "sleep"):
            measurement.capture(args)
        with args.output.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 30)
        self.assertEqual(rows[0]["session_sha256"], measurement.sha256(args.output.with_suffix(".session.json")))
        raw_lines = args.output.with_suffix(".jsonl").read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(raw_lines), 30)
        self.assertTrue(json.loads(raw_lines[0])["accepted"])

    def test_capture_failure_keeps_attempt_log_without_publishing_partial_csv(self):
        output = self.root / "incomplete.csv"
        args = SimpleNamespace(
            session=self.session_path, elf=self.elf, gdb="arm-none-eabi-gdb", host="127.0.0.1",
            telnet_port=4444, output=output, point="incomplete", phase="baseline",
            setpoint=12.5, meter_v=12.5, current_a=0.16, reference_v=3.3,
            samples=30, interval=0.5, settle=5.0, timeout=1.0,
        )
        addresses = {key: 0x24001000 + index * 4 for index, key in enumerate(probe.ADC_SNAPSHOT_FIELDS)}
        with patch.object(measurement.probe, "resolve_symbol_addresses", return_value=addresses), \
             patch.object(measurement.probe, "read_snapshot", return_value=adc_record()), \
             patch.object(measurement.time, "sleep"):
            with self.assertRaisesRegex(ValueError, "only 1/30"):
                measurement.capture(args)
        self.assertFalse(output.exists())
        attempts = output.with_suffix(".jsonl").read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(attempts), 90)
        self.assertTrue(any("duplicate sample" in row for row in attempts))

    def test_budget_uses_maximum_bounds_and_counts_meter_uncertainty_once(self):
        session = qualified_session(self.elf_digest)
        limit, meter_error, reasons = measurement.budget_limit(
            session, 12.5, {"adc_vref_mv": 3300, "adc_divider_milli": 11000, "adc_full_scale": 65535}, 3.3
        )
        self.assertAlmostEqual(limit, 0.6410050560810555, places=12)
        self.assertAlmostEqual(meter_error, 0.225, places=12)
        self.assertEqual(reasons, [])
        _, _, outside = measurement.budget_limit(
            session, 12.5, {"adc_vref_mv": 3300, "adc_divider_milli": 11000, "adc_full_scale": 65535}, 3.5
        )
        self.assertIn("measured ADC reference is outside the declared bounds", outside)
        incomplete = measurement.load_session(ROOT / "docs/workflow/tickets/MC02-0037-session.json")
        missing_limit, missing_meter, missing = measurement.budget_limit(
            incomplete, 12.5, {"adc_vref_mv": 3300, "adc_divider_milli": 11000, "adc_full_scale": 65535}
        )
        self.assertIsNone(missing_limit)
        self.assertIsNone(missing_meter)
        self.assertTrue(any("missing meter calibration record" in reason for reason in missing))
        self.assertTrue(any("unconfirmed schematic match" in reason for reason in missing))
        self.assertTrue(any("incomplete error budget: missing meter accuracy percent" in reason
                            for reason in missing))
        self.assertFalse(any("NoneType" in reason for reason in missing))

    def test_report_rejects_wrong_identity_raw_mismatch_and_incomplete_points(self):
        session = qualified_session(self.elf_digest)
        session_bytes = json.dumps(session, sort_keys=True).encode()
        baseline = write_point(self.root, session_bytes, self.elf_digest, "baseline", 12.5)
        stats, reasons = measurement.summarize([baseline])
        self.assertEqual(stats[0]["result"], "pass")
        self.assertTrue(any("missing measurement points" in reason for reason in reasons))

        with baseline.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
            columns = rows[0].keys()
        rows[0]["adc_volts"] = "99.0"
        with baseline.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        with self.assertRaisesRegex(ValueError, "voltage does not match raw bits"):
            measurement.summarize([baseline])

    def test_out_of_range_reference_keeps_point_unqualified(self):
        session = qualified_session(self.elf_digest)
        session_bytes = json.dumps(session, sort_keys=True).encode()
        point = write_point(self.root, session_bytes, self.elf_digest, "baseline", 12.5)
        with point.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
            columns = rows[0].keys()
        for row in rows:
            row["reference_v"] = "3.5"
        with point.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        stats, reasons = measurement.summarize([point])
        self.assertEqual(stats[0]["result"], "not-qualified")
        self.assertTrue(any("reference is outside" in reason for reason in reasons))

    def test_report_requires_all_points_and_qualifies_only_with_complete_evidence(self):
        session = qualified_session(self.elf_digest)
        session_bytes = json.dumps(session, sort_keys=True).encode()
        paths = [write_point(self.root, session_bytes, self.elf_digest, "baseline", 12.5)]
        for phase, points in (("up", measurement.POINTS), ("down", measurement.POINTS),
                              ("reboot", measurement.REBOOT_POINTS)):
            paths.extend(write_point(self.root, session_bytes, self.elf_digest, phase, point) for point in points)
        stats, reasons = measurement.summarize(paths)
        self.assertEqual(len(stats), 16)
        self.assertEqual(reasons, [])
        self.assertTrue(all(point["result"] == "pass" for point in stats))

        bad_identity = paths[-1]
        with bad_identity.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
            columns = rows[0].keys()
        rows[0]["elf_sha256"] = "0" * 64
        with bad_identity.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        with self.assertRaisesRegex(ValueError, "identity mismatch"):
            measurement.summarize(paths)

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "report graph extra is not installed")
    def test_report_writes_csv_json_and_error_graph(self):
        session = qualified_session(self.elf_digest)
        session_bytes = json.dumps(session, sort_keys=True).encode()
        paths = [write_point(self.root, session_bytes, self.elf_digest, "baseline", 12.5)]
        for phase, points in (("up", measurement.POINTS), ("down", measurement.POINTS),
                              ("reboot", measurement.REBOOT_POINTS)):
            paths.extend(write_point(self.root, session_bytes, self.elf_digest, phase, point) for point in points)
        output = self.root / "report"
        result = measurement.report(SimpleNamespace(inputs=paths, output=output))
        self.assertEqual(result, "pass")
        self.assertTrue((output / "summary.csv").is_file())
        self.assertTrue((output / "summary.json").is_file())
        self.assertTrue((output / "vin_error.png").is_file())
        self.assertTrue((output / "vin_error.svg").is_file())


if __name__ == "__main__":
    unittest.main(verbosity=2)
