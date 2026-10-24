#!/usr/bin/env python3
"""生成SP或V2禁火瞄准帧，并以有限期限发送到显式指定的串口。

默认发送中立帧。模式只允许0（无目标）或1（使用瞄准数据）；SP的供弹模式2
无法由本工具生成，V2瞄准消息本身也没有开火字段。命令行角度统一使用度，编码前
转换成SP和V2线上使用的弧度。串口打开、写入忙、超时、短写或错误均立即停止，不重试。
本工具用于主机链路诊断；不要连接带执行器的机器人进行测试。
"""

import argparse
import math
import struct
import sys
import time

DEFAULT_VALUES = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
MAX_SECONDS = 3600.0
MAX_HZ = 200.0
MAX_WRITE_TIMEOUT_S = 5.0
MAX_BAUD = 3_000_000
FLOAT32_MAX = 3.4028234663852886e38


class SendError(RuntimeError):
    """发送未完整提交；调用方必须结束本轮，不可重试旧帧。"""


def crc16_sp(data: bytes) -> int:
    """SP CRC：初值FFFF、反射多项式8408、无最终异或。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def crc16_modbus(data: bytes) -> int:
    """V2 CRC16/MODBUS：初值FFFF、多项式A001、无最终异或。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def _wire_values(values):
    if len(values) != 6:
        raise ValueError("瞄准数据必须有6个角度、角速度或角加速度值")
    converted = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("瞄准参数必须是有限数值")
        try:
            degrees = float(value)
        except OverflowError as error:
            raise ValueError("瞄准参数超出数值范围") from error
        if not math.isfinite(degrees):
            raise ValueError("瞄准参数必须是有限数值")
        radians = math.radians(degrees)
        if not math.isfinite(radians) or abs(radians) > FLOAT32_MAX:
            raise ValueError("角度转换超出float32范围")
        converted.append(radians)
    return converted


def _validate_mode(mode: int) -> None:
    if type(mode) is not int or mode not in (0, 1):
        raise ValueError("mode只允许0=无目标或1=使用瞄准数据；开火模式不受支持")


def build_sp_frame(mode: int, yaw_deg: float, pitch_deg: float,
                   yaw_rate_deg_s: float = 0.0, yaw_accel_deg_s2: float = 0.0,
                   pitch_rate_deg_s: float = 0.0,
                   pitch_accel_deg_s2: float = 0.0) -> bytes:
    """编码29字节SP命令；六个命令行角度量先转弧度再写入小端float32。"""
    _validate_mode(mode)
    yaw, pitch, yaw_rate, yaw_accel, pitch_rate, pitch_accel = _wire_values(
        (yaw_deg, pitch_deg, yaw_rate_deg_s, yaw_accel_deg_s2,
         pitch_rate_deg_s, pitch_accel_deg_s2)
    )
    body = struct.pack("<2sBffffff", b"SP", mode, yaw, yaw_rate, yaw_accel,
                       pitch, pitch_rate, pitch_accel)
    return body + struct.pack("<H", crc16_sp(body))


def build_v2_frame(*, mode: int, yaw_deg: float = 0.0, pitch_deg: float = 0.0,
                   yaw_rate_deg_s: float = 0.0, yaw_accel_deg_s2: float = 0.0,
                   pitch_rate_deg_s: float = 0.0, pitch_accel_deg_s2: float = 0.0,
                   sequence: int = 0, source_time_ms: int = 0) -> bytes:
    """编码39字节V2 AIM_COMMAND；头和负载整数为小端，角度量为弧度。"""
    _validate_mode(mode)
    if type(sequence) is not int or not 0 <= sequence <= 0xFFFF:
        raise ValueError("V2序号必须在0..65535")
    if type(source_time_ms) is not int or not 0 <= source_time_ms <= 0xFFFFFFFF:
        raise ValueError("V2来源时刻必须在0..4294967295毫秒")
    yaw, pitch, yaw_rate, yaw_accel, pitch_rate, pitch_accel = _wire_values(
        (yaw_deg, pitch_deg, yaw_rate_deg_s, yaw_accel_deg_s2,
         pitch_rate_deg_s, pitch_accel_deg_s2)
    )
    payload = struct.pack("<Bffffff", mode, yaw, yaw_rate, yaw_accel,
                          pitch, pitch_rate, pitch_accel)
    header = struct.pack("<2sBBHHI", b"MC", 2, 0x01, len(payload),
                         sequence, source_time_ms)
    body = header + payload
    return body + struct.pack("<H", crc16_modbus(body))


def build_frame(protocol: str, *, mode: int, values=DEFAULT_VALUES,
                sequence: int = 0, source_time_ms: int = 0) -> bytes:
    """按六个命令行度制参数构造指定协议的瞄准帧。"""
    if len(values) != 6:
        raise ValueError("瞄准数据必须有6个值")
    if protocol == "sp":
        return build_sp_frame(mode, *values)
    if protocol == "v2":
        return build_v2_frame(mode=mode, yaw_deg=values[0], pitch_deg=values[1],
            yaw_rate_deg_s=values[2], yaw_accel_deg_s2=values[3],
            pitch_rate_deg_s=values[4], pitch_accel_deg_s2=values[5],
            sequence=sequence, source_time_ms=source_time_ms)
    raise ValueError("protocol必须为sp或v2")


def send_periodic(transport, *, protocol: str, mode: int, seconds: float, hz: float,
                  values=DEFAULT_VALUES, sweep: bool = False,
                  monotonic=time.monotonic, sleep=time.sleep) -> int:
    """周期写入完整帧；短写、超时和异常均停止，不补发积压帧。"""
    if not math.isfinite(seconds) or not 0.0 < seconds <= MAX_SECONDS:
        raise ValueError(f"seconds必须在0..{MAX_SECONDS:g}秒内")
    if not math.isfinite(hz) or not 0.0 < hz <= MAX_HZ:
        raise ValueError(f"hz必须在0..{MAX_HZ:g}内")
    _validate_mode(mode)
    _wire_values(values)

    started = monotonic()
    deadline = started + seconds
    next_due = started
    period = 1.0 / hz
    sequence = 0
    sent = 0
    while True:
        now = monotonic()
        if now >= deadline:
            return sent
        if now < next_due:
            sleep(min(next_due - now, deadline - now))
            continue

        elapsed = now - started
        current_values = list(values)
        if sweep:
            # 扫描只改变目标角，幅度10度、频率0.5Hz，其余参数保持不变。
            current_values[0] += 10.0 * math.sin(2.0 * math.pi * 0.5 * elapsed)
        try:
            frame = build_frame(protocol, mode=mode, values=current_values,
                                sequence=sequence, source_time_ms=int(now * 1000) & 0xFFFFFFFF)
            written = transport.write(frame)
            if written != len(frame):
                raise SendError(f"串口短写：{written}/{len(frame)}字节")
        except Exception as error:
            if isinstance(error, SendError):
                raise
            raise SendError(f"串口发送失败：{error}") from error

        sent += 1
        sequence = (sequence + 1) & 0xFFFF
        next_due += period
        # 写入耗时跨过发送时点时从当前时刻重新排期，不连续追赶补发旧帧。
        after_write = monotonic()
        if next_due < after_write:
            next_due = after_write + period


def _finite_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("必须是数字") from error
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("必须是有限数值")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="必须显式指定的串口名，例如 COM12 或 /dev/ttyACM0")
    parser.add_argument("--protocol", choices=("sp", "v2"), default="sp",
                        help="线上格式；默认sp。V2仅发送AIM_COMMAND")
    parser.add_argument("--baud", type=int, default=115200,
                        help="串口波特率；USB CDC设备可能忽略此值")
    parser.add_argument("--hz", type=_finite_float, default=20.0,
                        help=f"发送频率，范围(0,{MAX_HZ:g}]Hz")
    parser.add_argument("--seconds", type=_finite_float, default=10.0,
                        help=f"发送时长，范围(0,{MAX_SECONDS:g}]秒")
    parser.add_argument("--write-timeout", type=_finite_float, default=0.1,
                        help=f"单次串口写入期限，范围(0,{MAX_WRITE_TIMEOUT_S:g}]秒")
    parser.add_argument("--mode", type=int, choices=(0, 1), default=0,
                        help="0=无目标（默认），1=瞄准数据；不支持任何开火模式")
    parser.add_argument("--yaw-deg", type=_finite_float, default=0.0,
                        help="偏航角，度；SP/V2线上转换为弧度")
    parser.add_argument("--pitch-deg", type=_finite_float, default=0.0,
                        help="俯仰角，度；SP/V2线上转换为弧度")
    parser.add_argument("--yaw-rate-deg-s", type=_finite_float, default=0.0,
                        help="偏航角速度，度/秒")
    parser.add_argument("--yaw-accel-deg-s2", type=_finite_float, default=0.0,
                        help="偏航角加速度，度/秒²")
    parser.add_argument("--pitch-rate-deg-s", type=_finite_float, default=0.0,
                        help="俯仰角速度，度/秒")
    parser.add_argument("--pitch-accel-deg-s2", type=_finite_float, default=0.0,
                        help="俯仰角加速度，度/秒²")
    parser.add_argument("--sweep", action="store_true",
                        help="偏航目标叠加10度、0.5Hz正弦扫描")
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1200 <= args.baud <= MAX_BAUD:
        parser.error(f"baud必须在1200..{MAX_BAUD}之间")
    if not 0.0 < args.hz <= MAX_HZ:
        parser.error(f"hz必须在0..{MAX_HZ:g}之间")
    if not 0.0 < args.seconds <= MAX_SECONDS:
        parser.error(f"seconds必须在0..{MAX_SECONDS:g}之间")
    if not 0.0 < args.write_timeout <= MAX_WRITE_TIMEOUT_S:
        parser.error(f"write-timeout必须在0..{MAX_WRITE_TIMEOUT_S:g}之间")
    values = (args.yaw_deg, args.pitch_deg, args.yaw_rate_deg_s,
              args.yaw_accel_deg_s2, args.pitch_rate_deg_s,
              args.pitch_accel_deg_s2)
    try:
        _wire_values(values)
    except ValueError as error:
        parser.error(str(error))
    args.values = values
    return args


def open_transport(args):
    """只打开用户指定端口，并给每次写入设置有限超时。"""
    import serial
    return serial.Serial(args.port, baudrate=args.baud, timeout=0,
                         write_timeout=args.write_timeout)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        # 串口忙、端口不存在和驱动错误都只报告一次，不切换端口或重试。
        transport = open_transport(args)
    except Exception as error:
        print(f"无法打开串口{args.port}（可能被占用）：{error}", file=sys.stderr)
        return 2

    print(f"端口={args.port} 协议={args.protocol} 波特率={args.baud}Hz 频率={args.hz:g}Hz "
          f"时长={args.seconds:g}s 写超时={args.write_timeout:g}s mode={args.mode} 禁火")
    try:
        sent = send_periodic(transport, protocol=args.protocol, mode=args.mode,
                             seconds=args.seconds, hz=args.hz, values=args.values,
                             sweep=args.sweep)
    except (SendError, ValueError) as error:
        print(f"发送停止，未重试：{error}", file=sys.stderr)
        return 2
    finally:
        try:
            transport.close()
        except Exception:
            pass
    print(f"完成：{sent}帧")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
