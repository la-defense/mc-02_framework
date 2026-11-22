#!/usr/bin/env python3
"""单次读取受限底盘 USART10 的速度与里程计，或离线解析一帧应答。

在线模式只发送一条固定格式的无负载查询。输出 JSON 中，缺测项为 null。
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import struct
import sys
import time


FRAME_PREFIX = b"\xd3\x6c\x01"  # 两个同步字节及版本 1。
QUERY = 0x05  # 只读底盘反馈查询，无负载。
REPLY = 0x85  # 固件加上应答位后的命令编号。
HEADER_SIZE = 6  # 同步、版本、命令、序号、负载长度。
CRC_SIZE = 2  # CRC 按低字节在前保存。
MOTION_SIZE = 38  # 格式版本、有效位、速度和里程计的总长度。
BAUD_RATE = 921600  # USART10 的固件波特率，使用 8N1，无流控。
MAX_TIMEOUT_S = 5.0  # 一次写入及等待应答允许的最长秒数。


def _crc16(data: bytes) -> bytes:
    """按固件 CRC16_MODBUS_A5 的参数计算并返回两个线上字节。"""
    value = 0xFFFF  # 固件的 CRC 初值。
    for byte in data:
        value ^= byte  # 将下一个字节合入低位。
        for _ in range(8):
            value = (value >> 1) ^ (0xA001 if value & 1 else 0)  # 按反射多项式逐位计算。
    return value.to_bytes(CRC_SIZE, "little")  # 固件按低位字节、高位字节发送。


def make_query(sequence: int) -> bytes:
    """构造带一字节序号的无负载读取请求。"""
    header = FRAME_PREFIX + bytes((QUERY, sequence, 0))
    return header + _crc16(header)


def decode_reply(frame: bytes, sequence: int) -> dict:
    """验证一条完整应答并按 SI 单位返回数据，非法或拒绝应答抛出 ValueError。"""
    prefix = FRAME_PREFIX + bytes((REPLY, sequence))  # 本次请求只匹配回显同一序号的底盘应答。
    if len(frame) < HEADER_SIZE + CRC_SIZE or not frame.startswith(prefix):
        raise ValueError("应答帧头、版本、命令或序号不匹配")  # 不读取未通过基本检查的负载字段。
    payload = frame[HEADER_SIZE:-CRC_SIZE]  # 正文不包含头和末尾 CRC。
    if len(payload) != frame[5] or len(payload) not in (1, MOTION_SIZE):
        raise ValueError("应答长度不匹配")  # 固件只返回一个状态字节或完整运动数据。
    if _crc16(frame[:-CRC_SIZE]) != frame[-CRC_SIZE:]:
        raise ValueError("应答 CRC 不匹配")  # 不输出可能被改写的读数。
    if len(payload) == 1:
        raise ValueError(f"固件拒绝底盘查询，诊断状态 {payload[0]}")  # 未登记或处理失败不是缺测。
    schema, flags = payload[:2]  # 格式版本和两个独立有效位。
    if schema != 1 or flags & ~3:
        raise ValueError("底盘数据版本或有效位无效")  # 保留位不能作为有效数据解释。

    speed_time, vx, vy, wz = struct.unpack_from("<Ifff", payload, 2)  # 速度使用 ms、m/s、rad/s。
    pose_time, segment, x, y, yaw = struct.unpack_from("<IIfff", payload, 18)  # 位姿使用 ms、段号、m、rad。
    if not all(math.isfinite(value) for value in (vx, vy, wz, x, y, yaw)):
        raise ValueError("底盘数据含非有限值")  # 禁止用 NaN 或无穷表示读数。
    if (not (flags & 1) and any(payload[2:18])) or (not (flags & 2) and any(payload[18:])):
        raise ValueError("缺测项含非零字节")  # 固件将缺测项整组清零，发现矛盾时拒绝整帧。
    return {
        "sequence": sequence,
        "velocity": {"source_time_ms": speed_time, "vx_m_s": vx, "vy_m_s": vy, "wz_rad_s": wz}
        if flags & 1 else None,  # 有效的零速度仍输出数值。
        "odometry": {"source_time_ms": pose_time, "segment": segment, "x_m": x, "y_m": y, "yaw_rad": yaw}
        if flags & 2 else None,  # 缺测位姿不假定原点或段号。
    }


def _find_reply(buffer: bytearray, prefix: bytes) -> bytes | None:
    """从串口流查找匹配的完整应答，只保留可能补齐的尾部。"""
    while (start := buffer.find(prefix)) >= 0:
        del buffer[:start]  # 跳过噪声或其他序号的帧。
        if len(buffer) < HEADER_SIZE:
            return None  # 序号到齐但长度字段尚未到达。
        length = buffer[5]  # 仅接收固件定义的两种长度。
        if length in (1, MOTION_SIZE):
            end = HEADER_SIZE + length + CRC_SIZE  # 一帧最多 46 字节。
            if len(buffer) < end:
                return None  # 还在接收负载或 CRC。
            candidate = bytes(buffer[:end])  # 从当前同步字节开始复核。
            if _crc16(candidate[:-CRC_SIZE]) == candidate[-CRC_SIZE:]:
                return candidate  # 数据区的版本、状态和有效位由 decode_reply 检查。
        del buffer[0]  # 坏长度或坏 CRC 时从后一字节继续查找，保留后续帧。
    del buffer[:max(0, len(buffer) - len(prefix) + 1)]  # 留出可能跨两次读取的帧头。
    return None


def request_motion(port, sequence: int, timeout_s: float, clock=time.monotonic) -> dict:
    """清空接收缓存后发一条查询，在共同期限内返回数据或报错。"""
    if not 0 < timeout_s <= MAX_TIMEOUT_S:
        raise ValueError(f"超时应大于 0 且不超过 {MAX_TIMEOUT_S:g} 秒")
    request = make_query(sequence)  # 保持发送命令及其无负载格式固定。
    prefix = FRAME_PREFIX + bytes((REPLY, sequence))  # 读取时只认本次序号。
    port.reset_input_buffer()  # 丢弃发送前留在主机串口缓存的字节。
    port.write_timeout = timeout_s  # 串口写入也有上限。
    deadline = clock() + timeout_s  # 写入和接收使用同一个截止时刻。
    if port.write(request) != len(request):
        raise OSError("串口短写，查询未完整发送")  # 不重新发送部分请求。
    buffer = bytearray()  # 逐字节读取，扫描器只保留一帧以内的候选数据。
    while (remaining := deadline - clock()) > 0:
        port.timeout = remaining  # 分帧收到、连续收到噪声都不会刷新总期限。
        byte = port.read(1)  # 单次应答很小，读到末字节即可返回。
        if not byte or clock() > deadline:
            break  # 串口超时或过期后才收到字节时，不交付结果。
        buffer.extend(byte)  # 原样累积字节直到完整应答出现。
        frame = _find_reply(buffer, prefix)  # 跳过序号不符、已损坏或无关的帧。
        if frame is not None:
            return decode_reply(frame, sequence)  # 合法的固件拒绝也在这里明确上报。
    raise TimeoutError("等待匹配的底盘应答超时")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    offline = commands.add_parser("decode", help="离线校验一帧十六进制应答")
    offline.add_argument("frame", help="十六进制应答，可包含空格")
    offline.add_argument("--sequence", required=True, type=lambda value: int(value, 0), help="请求序号，可用 0x47 等写法")
    online = commands.add_parser("query", help="从指定 USART10 主机串口查询一次")
    online.add_argument("port", help="显式指定串口，例如 COM7 或 /dev/ttyUSB0")
    online.add_argument("--timeout", type=float, default=0.5, help="写入与等待的共同期限，单位秒，默认 0.5")
    args = parser.parse_args(argv)
    try:
        if args.mode == "decode":
            result = decode_reply(bytes.fromhex(args.frame), args.sequence)
        else:
            if not 0 < args.timeout <= MAX_TIMEOUT_S:
                raise ValueError(f"超时应大于 0 且不超过 {MAX_TIMEOUT_S:g} 秒")
            import serial  # 离线解析仅依赖 Python 标准库。

            port = serial.Serial(baudrate=BAUD_RATE)  # 显式指定设备后再打开。
            port.dtr = False  # 查询不使用线路控制信号。
            port.rts = False  # 关闭请求发送控制信号。
            port.port = args.port  # 只打开调用者指定的串口。
            with port:
                result = request_motion(port, secrets.randbelow(256), args.timeout)
    except (ImportError, OSError, ValueError) as error:
        print(f"底盘查询失败：{error}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
