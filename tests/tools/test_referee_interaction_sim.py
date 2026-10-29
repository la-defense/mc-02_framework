#!/usr/bin/env python3
"""使用独立固定帧核对裁判模拟器的通用组帧；不连接串口。"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "referee_sim"))

from referee_sim_app.protocol.frame import FrameParser, build_frame  # noqa: E402


class InteractionFrameTest(unittest.TestCase):
    """固定字节验证编码、拆分接收和损坏帧处理。"""

    def test_fixed_frame_and_bad_crc(self):
        expected = bytes.fromhex("a5090042b701033402030001001122337488")
        content = bytes.fromhex("340203000100112233")
        self.assertEqual(build_frame(0x0301, content, 0x42), expected)
        self.assertEqual(build_frame(0x0301, content, 0),
                         bytes.fromhex("a50900004d0103340203000100112233e02f"))
        for split in range(1, len(expected)):
            parser = FrameParser(max_data_len=119)
            self.assertEqual(parser.feed(expected[:split]), [])
            self.assertEqual(parser.feed(expected[split:])[0].raw, expected)
        corrupted = bytearray(expected)
        corrupted[-1] ^= 1
        self.assertEqual(FrameParser().feed(corrupted), [])


if __name__ == "__main__":
    unittest.main()
