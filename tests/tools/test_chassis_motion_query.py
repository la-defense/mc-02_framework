"""用固件固定字节样例和串口替身检查只读底盘查询。"""

import io
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import chassis_motion_query as motion


QUERY = bytes.fromhex("d3 6c 01 05 47 00 a1 bc")  # 与固件诊断帧契约一致的固定请求。
REPLY = bytes.fromhex(
    "d3 6c 01 85 47 26 01 03 12 34 56 78 00 00 80 3f 00 00 00 c0 00 00 00 3f "
    "09 00 00 00 02 00 00 00 00 00 00 00 00 00 c0 3f 00 00 00 bf 97 71"
)  # 固件 log_test.c 的样例负载及固定 CRC。
REFUSAL = bytes.fromhex("d3 6c 01 85 47 01 07 14 2a")  # 未登记返回一个状态字节。


def with_crc(frame_without_crc: bytes) -> bytes:
    """故意改动字段时重算 CRC，让用例实际进入字段检查。"""
    return frame_without_crc + motion._crc16(frame_without_crc)


class MockPort:
    """记录请求并逐字节交付测试输入，无需设备依赖。"""

    def __init__(self, data: bytes, write_count: int = 8, write_delay: float = 0.0):
        self.incoming = io.BytesIO(data)
        self.write_count = write_count
        self.write_delay = write_delay
        self.writes = []
        self.read_count = 0
        self.reset_done = False
        self.now = 10.0

    def reset_input_buffer(self):
        self.reset_done = True

    def write(self, data):
        self.writes.append(data)
        self.now += self.write_delay
        return self.write_count

    def read(self, count):
        self.read_count += 1
        self.now += 0.001
        return self.incoming.read(count)

    def clock(self):
        return self.now


class MotionQueryTests(unittest.TestCase):
    def test_firmware_vectors_and_units(self):
        self.assertEqual(motion.make_query(0x47), QUERY)
        result = motion.decode_reply(REPLY, 0x47)
        self.assertEqual(result["sequence"], 0x47)
        self.assertEqual(result["velocity"], {
            "source_time_ms": 0x78563412, "vx_m_s": 1.0, "vy_m_s": -2.0, "wz_rad_s": 0.5,
        })
        self.assertEqual(result["odometry"], {
            "source_time_ms": 9, "segment": 2, "x_m": 0.0, "y_m": 1.5, "yaw_rad": -0.5,
        })

    def test_missing_and_zero_are_distinct(self):
        header = REPLY[:6]
        for flags in (0, 1, 2):
            with self.subTest(flags=flags):
                frame = with_crc(header + bytes((1, flags)) + bytes(36))
                result = motion.decode_reply(frame, 0x47)
                if flags & 1:
                    self.assertEqual(result["velocity"]["vx_m_s"], 0.0)
                else:
                    self.assertIsNone(result["velocity"])
                if flags & 2:
                    self.assertEqual(result["odometry"]["x_m"], 0.0)
                else:
                    self.assertIsNone(result["odometry"])

    def test_rejects_bad_envelopes_and_motion_data(self):
        invalid_number = bytearray(REPLY[:-2])
        struct.pack_into("<f", invalid_number, 12, float("nan"))  # 直接改速度字段的二进制内容。
        cases = (
            (REPLY, 0x48, "序号"),  # 同一命令的旧回复不能交付给本次读取。
            (REPLY[:-1], 0x47, "长度"),  # 数据长度少一个字节。
            (REPLY[:-1] + b"\x00", 0x47, "CRC"),  # 字段完整但 CRC 被改写。
            (REFUSAL, 0x47, "诊断状态 7"),  # 固件拒绝独立于数据缺测。
            (with_crc(REPLY[:6] + b"\x02" + REPLY[7:-2]), 0x47, "版本"),  # 未知数据格式。
            (with_crc(REPLY[:7] + b"\x83" + REPLY[8:-2]), 0x47, "有效位"),  # 使用保留位。
            (with_crc(REPLY[:7] + b"\x02" + REPLY[8:-2]), 0x47, "缺测项"),  # 无效速度仍带数据。
            (with_crc(invalid_number), 0x47, "非有限"),  # CRC 合法的 NaN 也不能交付。
        )
        for frame, sequence, error in cases:
            with self.subTest(error=error), self.assertRaisesRegex(ValueError, error):
                motion.decode_reply(frame, sequence)

    def test_serial_syncs_past_noise_bad_crc_and_unrelated_reply(self):
        other = with_crc(REPLY[:4] + b"\x46" + REPLY[5:-2])  # 来自其他序号的合法帧。
        bad_length = REPLY[:5] + b"\xff"  # 超出应答格式长度，不能阻塞后续帧。
        bad_crc = REPLY[:-1] + b"\x00"
        port = MockPort(b"noise\xd3" + other + bad_length + bad_crc + REPLY)
        result = motion.request_motion(port, 0x47, 0.5, port.clock)
        self.assertTrue(port.reset_done)
        self.assertEqual(port.writes, [QUERY])
        self.assertEqual(result, motion.decode_reply(REPLY, 0x47))

    def test_serial_timeout_and_short_write_have_no_retry(self):
        noise = MockPort(b"?" * 100 + REPLY)
        for port, timeout, exception, message in (
            (MockPort(REPLY[:25]), 0.5, TimeoutError, "超时"),  # 不完整应答不能成功。
            (MockPort(REPLY, write_delay=0.6), 0.5, TimeoutError, "超时"),  # 写入消耗共同期限。
            (MockPort(REPLY, write_count=7), 0.5, OSError, "短写"),  # 查询少写一字节就停止。
            (noise, 0.05, TimeoutError, "超时"),  # 连续噪声不延长期限。
        ):
            with self.subTest(exception=exception, timeout=timeout):
                with self.assertRaisesRegex(exception, message):
                    motion.request_motion(port, 0x47, timeout, port.clock)
                self.assertEqual(port.writes, [QUERY])
        self.assertLess(noise.read_count, 100)  # 有界退出前尚未读完噪声。


if __name__ == "__main__":
    unittest.main()
