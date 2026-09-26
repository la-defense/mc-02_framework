#!/usr/bin/env python3
"""通过虚拟串口(USB CDC / VCP)向下位机发送模拟视觉帧, 用于验证视觉接收链路。

为什么需要它: 视觉正式链路走板载 USB 虚拟串口(COM10), 没有上位机时链路是"死的";
用这个脚本按 200Hz 往那个串口灌合法的 SP 帧, 就能在不开视觉程序的情况下验证
"USB 中断 → 队列 → VCPProcessRx → DecodeVision" 整条链路。

帧格式见 modules/master_machine/master_process.h 的 Vision_Recv_s
(#pragma pack(1), 共 29 字节, 小端):
    head[2] = 'S','P' | mode(u8) | yaw,yaw_vel,yaw_acc,pitch,pitch_vel,pitch_acc (float32) | crc16(u16)
CRC 与工程里 VisionCRC16() 一致: 初值 0xFFFF、多项式反向 0x8408、**不取反**,
校验范围是除最后 2 字节外的全部内容。

用法:
    python tools/vision_sim.py --port COM10 --seconds 10            # 200Hz 发 10 秒
    python tools/vision_sim.py --port COM10 --sweep --seconds 30    # yaw 做 0.5Hz 正弦扫描
    python tools/vision_sim.py --port COM10 --mode 0               # 只发"不控制"帧
"""

import argparse
import math
import struct
import time

import serial  # pyserial


def crc16(data: bytes) -> int:
    """与下位机 VisionCRC16() 完全一致的实现。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(mode: int, yaw: float, pitch: float,
                yaw_vel: float = 0.0, yaw_acc: float = 0.0,
                pitch_vel: float = 0.0, pitch_acc: float = 0.0) -> bytes:
    body = struct.pack("<2sBffffff", b"SP", mode,
                       yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)
    return body + struct.pack("<H", crc16(body))


def main() -> int:
    ap = argparse.ArgumentParser(description="模拟上位机视觉帧投喂")
    ap.add_argument("--port", default="COM10", help="虚拟串口(默认 COM10)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="VCP 的波特率只是摆设, 随便填")
    ap.add_argument("--hz", type=float, default=200.0, help="发送频率(默认 200Hz)")
    ap.add_argument("--seconds", type=float, default=10.0, help="持续时长")
    ap.add_argument("--mode", type=int, default=2, help="0=不控制 1=控制不开火 2=控制并开火")
    ap.add_argument("--sweep", action="store_true", help="yaw 做 0.5Hz 正弦扫描(便于肉眼看变化)")
    args = ap.parse_args()

    period = 1.0 / max(args.hz, 1.0)
    sent = 0
    t0 = time.time()
    next_t = t0

    with serial.Serial(args.port, args.baud, timeout=0) as ser:
        print(f"开始向 {args.port} 发送视觉帧: {args.hz:.0f}Hz, {args.seconds:.0f}s, mode={args.mode}")
        while True:
            elapsed = time.time() - t0
            if elapsed >= args.seconds:
                break
            yaw = 30.0 * math.sin(2.0 * math.pi * 0.5 * elapsed) if args.sweep else 10.0
            ser.write(build_frame(args.mode, yaw, 5.0))
            sent += 1
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)

    print(f"已发送 {sent} 帧 (实际 {sent / max(time.time() - t0, 1e-3):.0f} Hz)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
