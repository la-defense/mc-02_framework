"""验证SP/V2禁火帧、参数检查及发送失败即停止。"""
import contextlib
import io
import math
import struct
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import vision_sim


class FrameTests(unittest.TestCase):
    def test_sp_neutral_frame_has_fixed_bytes_and_no_fire_mode(self):
        frame = vision_sim.build_sp_frame(0, 0, 0, 0, 0, 0, 0)
        self.assertEqual(
            frame.hex(),
            "5350000000000000000000000000000000000000000000000000009971",
        )
        self.assertEqual(len(frame), 29)

    def test_sp_aim_frame_uses_radians_on_wire_and_rejects_fire_mode(self):
        frame = vision_sim.build_sp_frame(
            1, math.degrees(0.25), math.degrees(-0.125),
            math.degrees(-0.5), math.degrees(0.75),
            math.degrees(0.5), math.degrees(-1.0),
        )
        self.assertEqual(frame[3:7], struct.pack("<f", 0.25))
        self.assertEqual(frame[15:19], struct.pack("<f", -0.125))
        with self.assertRaises(ValueError):
            vision_sim.build_sp_frame(2, 0, 0, 0, 0, 0, 0)

    def test_v2_aim_matches_independent_firmware_vector(self):
        frame = vision_sim.build_v2_frame(
            mode=1,
            yaw_deg=math.degrees(0.25),
            pitch_deg=math.degrees(-0.125),
            yaw_rate_deg_s=math.degrees(-0.5),
            yaw_accel_deg_s2=math.degrees(0.75),
            pitch_rate_deg_s=math.degrees(0.5),
            pitch_accel_deg_s2=math.degrees(-1.0),
            sequence=0x0102,
            source_time_ms=0x11223344,
        )
        self.assertEqual(
            frame.hex(),
            "4d4302011900020144332211010000803e000000bf0000403f"
            "000000be0000003f000080bf1d10",
        )
        self.assertEqual(len(frame), 39)

    def test_invalid_parameters_and_fire_mode_are_rejected(self):
        for protocol in ("sp", "v2"):
            with self.subTest(protocol=protocol), self.assertRaises(ValueError):
                vision_sim.build_frame(protocol, mode=2, values=vision_sim.DEFAULT_VALUES)
        with self.assertRaises(ValueError):
            vision_sim.build_v2_frame(mode=0, yaw_deg=float("nan"))
        with self.assertRaises(ValueError):
            vision_sim.build_sp_frame(0, 10**500, 0)


class Transport:
    def __init__(self, failure=None, clock=None, write_delay=0.08):
        self.failure = failure
        self.writes = []
        self.clock = clock
        self.write_delay = write_delay
        self.write_times = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        if self.clock is not None:
            self.write_times.append(self.clock.now)
        if self.clock is not None:
            self.clock.advance(self.write_delay)
        if isinstance(self.failure, Exception):
            raise self.failure
        if self.failure is not None:
            return self.failure
        return len(data)

    def close(self):
        self.closed = True


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def advance(self, amount):
        self.now += amount

    def sleep(self, amount):
        self.advance(amount)


class SendTests(unittest.TestCase):
    def test_short_write_and_transport_errors_stop_without_retry(self):
        for failure in (10, TimeoutError("write timeout"), OSError("disconnected")):
            with self.subTest(failure=failure):
                transport = Transport(failure)
                with self.assertRaises(vision_sim.SendError):
                    vision_sim.send_periodic(
                        transport, protocol="sp", mode=0, seconds=1, hz=10,
                        values=vision_sim.DEFAULT_VALUES,
                    )
                self.assertEqual(len(transport.writes), 1)

    def test_delayed_write_skips_backlog(self):
        clock = FakeClock()
        transport = Transport(clock=clock, write_delay=0.25)
        sent = vision_sim.send_periodic(
            transport, protocol="sp", mode=0, seconds=0.5, hz=10,
            values=vision_sim.DEFAULT_VALUES,
            monotonic=clock.monotonic, sleep=clock.sleep,
        )
        self.assertEqual(sent, len(transport.writes))
        self.assertEqual(sent, 2)
        self.assertGreaterEqual(transport.write_times[1] - transport.write_times[0], 0.349)


class CliTests(unittest.TestCase):
    def test_port_is_required_and_port_open_failure_is_reported_once(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            vision_sim.parse_args([])

        error_text = io.StringIO()
        with patch.object(vision_sim, "open_transport", side_effect=OSError("port busy")) as opener, \
             contextlib.redirect_stderr(error_text):
            result = vision_sim.main(["--port", "COM42", "--seconds", "0.1"])
        self.assertEqual(result, 2)
        opener.assert_called_once()
        self.assertIn("可能被占用", error_text.getvalue())

    def test_serial_open_receives_bounded_write_timeout(self):
        factory = Mock(return_value=Transport())
        fake_serial = types.SimpleNamespace(Serial=factory)
        args = vision_sim.parse_args(["--port", "COM7", "--write-timeout", "0.25"])
        with patch.dict(sys.modules, {"serial": fake_serial}):
            vision_sim.open_transport(args)
        factory.assert_called_once_with(
            "COM7", baudrate=115200, timeout=0, write_timeout=0.25,
        )

    def test_cli_stops_after_short_write_and_closes_transport(self):
        transport = Transport(failure=4)
        error_text = io.StringIO()
        with patch.object(vision_sim, "open_transport", return_value=transport) as opener, \
             contextlib.redirect_stderr(error_text):
            result = vision_sim.main(["--port", "COM8", "--seconds", "0.1"])
        self.assertEqual(result, 2)
        opener.assert_called_once()
        self.assertEqual(len(transport.writes), 1)
        self.assertTrue(transport.closed)
        self.assertIn("未重试", error_text.getvalue())

    def test_fire_mode_is_rejected_before_opening_port(self):
        with patch.object(vision_sim, "open_transport") as opener, \
             contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            vision_sim.main(["--port", "COM9", "--mode", "2"])
        opener.assert_not_called()


if __name__ == "__main__":
    unittest.main()
