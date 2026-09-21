#!/usr/bin/env python3
"""USB-CAN 适配器（私有 CDC 协议）串口监视器 —— 只用 pyserial，不依赖 python-can。

背景（2026-09 实测，设备 VID_2E88&PID_4603 / COM15）：
  - 该适配器**不是 slcan/CANable**，python-can 无法直接驱动（ASCII 命令无响应）；
  - 它上电后会持续以私有二进制协议输出"收到的 CAN 帧"，格式（由观测推断）：

        55 AA | 11 08 | ID_L ID_H | DATA0..DATA7 | xx xx
        ↑头     ↑类型/DLC   ↑11bit 标准ID  ↑8 字节数据

    实测：MCU 运行时约 900~1400 帧/s、ID=0x200（底盘控制组）、数据全 0（电机停机）；
    把 MCU 暂停后流几乎归零 → 证明它报的确实是总线上的帧。
  - 该适配器目前**不回 ACK**（MCU 侧 TEC 一直涨到 BusOff），推测是"通道未打开/静默模式"，
    需要厂商上位机或协议文档才能打开正常收发。

用法：
    python tools/usbcan_serial_monitor.py                      # 监视 10 秒
    python tools/usbcan_serial_monitor.py --seconds 30
    python tools/usbcan_serial_monitor.py --port COM15 --raw   # 打印原始字节便于继续逆向
"""

import argparse
import collections
import time

import serial

HEADER = b"\x55\xaa"
FRAME_LEN = 16


def parse_frame(f: bytes):
    """返回 (id, dlc, data)；布局不符时返回 None。"""
    if len(f) != FRAME_LEN or f[0:2] != HEADER:
        return None
    dlc = f[3]
    can_id = f[4] | (f[5] << 8)
    data = f[6:6 + min(dlc, 8)]
    return can_id, dlc, data


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM15")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--raw", action="store_true", help="打印原始 16 字节帧")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()
    print(f"# 已打开 {args.port} @{args.baud}，监视 {args.seconds:g}s（Ctrl-C 退出）")

    buf = b""
    stats = collections.Counter()
    last_report = time.time()
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        buf += ser.read(8192)
        # 按同步字对齐切帧
        while True:
            i = buf.find(HEADER)
            if i < 0:
                buf = buf[-1:]
                break
            if len(buf) - i < FRAME_LEN:
                buf = buf[i:]
                break
            frame = buf[i:i + FRAME_LEN]
            buf = buf[i + FRAME_LEN:]
            parsed = parse_frame(frame)
            if parsed is None:
                continue
            can_id, dlc, data = parsed
            stats[can_id] += 1
            if args.raw:
                print(f"  [{can_id:#05x}] dlc={dlc} data={data.hex(' ')}  raw={frame.hex(' ')}")
        now = time.time()
        if now - last_report >= 1.0:
            total = sum(stats.values())
            rate = total / (now - t0)
            detail = " ".join(f"0x{k:03x}={v}" for k, v in stats.most_common(4))
            print(f"  {now - t0:5.1f}s  累计 {total} 帧  ({rate:.0f} 帧/s)  {detail}")
            last_report = now
    ser.close()
    print(f"# 结束：共 {sum(stats.values())} 帧，ID 分布 "
          + ", ".join(f"{hex(k)}={v}" for k, v in stats.most_common()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
