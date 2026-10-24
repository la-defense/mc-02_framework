#!/usr/bin/env python3
"""有限采集 MC02 USB CDC 状态流，只发送固定的中性视觉帧。

用途：记录 CDC 收发、序号和主机侧错误，供 MC02-0062 裸板回归使用。
入口：命令行 main；使用前需显式指定端口和输出目录，工具会先运行只读身份核对。
限制：短时阻塞读取以保持CDC IN读请求；设备重枚举后仅按USB序列号有限重开，不重发失败帧。
      不发送模式切换或执行器命令；主机时间不是USB线端时序保证。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import time

USB_VID = 0x0483
USB_PID = 0x5740
FRAME_PERIOD_S = 0.01
MAX_SECONDS = 300.0
SERIAL_READ_TIMEOUT_S = 0.02
RECONNECT_TIMEOUT_S = 10.0
RECONNECT_POLL_S = 0.02
UINT32_MASK = 0xFFFFFFFF


def crc16(data: bytes) -> int:
    """独立计算 SP 协议 CRC16，避免采集工具依赖目标固件实现。"""
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value & 0xFFFF


def neutral_frame(sequence: int) -> bytes:
    """生成固定29字节 mode=0、9个零浮点字段的中性帧。"""
    body = b"SP\x00" + bytes(6 * 4)
    return body + struct.pack("<H", crc16(body))


def identity_command(elf: Path, output_root: Path, *, diagnostic: bool = False,
                     gdb: str = "arm-none-eabi-gdb", readelf: str = "arm-none-eabi-readelf",
                     host: str = "127.0.0.1", telnet_port: int = 4444) -> list[str]:
    """构造固定字段的只读身份核对命令，不允许外部传入任意调试表达式。"""
    tool = Path(__file__).with_name("probe_snapshot_live.py")
    mode = "--usb-diagnostic" if diagnostic else "--usb"
    return [sys.executable, str(tool), mode, "--samples", "2", "--elf", str(elf.resolve()),
            "--gdb", gdb, "--readelf", readelf, "--host", host,
            "--telnet-port", str(telnet_port), "--output-root", str(output_root)]


def _open_serial_port(serial_module, device: str):
    """打开指定CDC端口并保留短时读请求，便于设备发送IN状态帧。"""
    return serial_module.Serial(device, 115200, timeout=SERIAL_READ_TIMEOUT_S, write_timeout=0.1)


def _matching_serial_device(list_ports, serial_number: str):
    """按稳定的USB序列号找当前COM名；多个匹配项时拒绝猜测。"""
    matches = [item for item in list_ports.comports()
               if item.vid == USB_VID and item.pid == USB_PID and item.serial_number == serial_number]
    if len(matches) > 1:
        raise OSError("USB serial number matched more than one CDC port")
    return matches[0] if matches else None


def _reopen_serial_port(serial_module, list_ports, serial_number: str, deadline: float):
    """在期限内等待同一块板重新出现，返回当前COM名和新句柄。"""
    serial_exception = getattr(serial_module, "SerialException", OSError)
    error_types = (OSError,) if issubclass(serial_exception, OSError) else (OSError, serial_exception)
    while time.monotonic() < deadline:
        item = _matching_serial_device(list_ports, serial_number)
        if item is not None:
            try:
                return item.device, _open_serial_port(serial_module, item.device)
            except error_types:
                # 设备刚出现时驱动可能还没准备好，短暂停顿后再检查同一序列号。
                pass
        time.sleep(RECONNECT_POLL_S)
    raise OSError("matching STM32 CDC did not reopen before the reconnect deadline")


class StatusFrames:
    """解析固定43字节状态帧并保留未消费字节。"""
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, data: bytes) -> list[dict]:
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0:2] != b"SP":
                del self.buffer[0]
                self.rejected += 1
                continue
            if len(self.buffer) < 43:
                break
            candidate = bytes(self.buffer[:43])
            expected = int.from_bytes(candidate[-2:], "little")
            if crc16(candidate[:-2]) != expected:
                del self.buffer[0]
                self.rejected += 1
                continue
            values = struct.unpack("<9f", candidate[3:39])
            bullet_count = int.from_bytes(candidate[39:41], "little")
            if not all(math.isfinite(value) for value in values):
                del self.buffer[:43]
                self.rejected += 1
                continue
            frames.append({"mode": candidate[2], "values": values, "bullet_count": bullet_count})
            del self.buffer[:43]
        return frames


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="显式指定 MC02 USB CDC 端口")
    parser.add_argument("--elf", type=Path, required=True, help="供记录身份的本地 ELF")
    parser.add_argument("--source-sha", required=True, help="40位固件源码提交 SHA")
    parser.add_argument("--output", type=Path, required=True, help="新建的会话目录")
    parser.add_argument("--duration", type=float, default=120.0, help="采集秒数，1～300")
    parser.add_argument("--gdb", default="arm-none-eabi-gdb", help="用于只读固件身份核对的 GDB")
    parser.add_argument("--readelf", default="arm-none-eabi-readelf", help="用于读取 ELF Build ID 的 readelf")
    parser.add_argument("--host", default="127.0.0.1", help="本机 OpenOCD telnet 地址")
    parser.add_argument("--telnet-port", type=int, default=4444, help="本机 OpenOCD 只读服务端口")
    parser.add_argument("--diagnostic-elf", action="store_true", help="按 bench_safe USB 诊断固件字段核对身份")
    args = parser.parse_args(argv)
    if not args.elf.is_file() or len(args.source_sha) != 40 or any(c not in "0123456789abcdef" for c in args.source_sha):
        parser.error("ELF 或源码 SHA 无效")
    if not 1.0 <= args.duration <= MAX_SECONDS:
        parser.error("duration 必须在1～300秒之间")
    if args.output.exists():
        parser.error("输出目录必须不存在，避免覆盖旧证据")

    try:
        import serial
        from serial.tools import list_ports
    except ImportError as error:
        print(f"pyserial unavailable: {error}", file=sys.stderr)
        return 2
    matches = [item for item in list_ports.comports() if item.device == args.port]
    if len(matches) != 1 or matches[0].vid != USB_VID or matches[0].pid != USB_PID or not matches[0].serial_number:
        print("指定端口未唯一匹配带序列号的 STM32 USB CDC (0483:5740)", file=sys.stderr)
        return 2

    args.output.mkdir(parents=True, exist_ok=False)
    digest = hashlib.sha256(args.elf.read_bytes()).hexdigest()
    identity_root = args.output / "identity"
    identity_mode = "--usb-diagnostic" if args.diagnostic_elf else "--usb"
    command = identity_command(args.elf, identity_root, diagnostic=args.diagnostic_elf,
                               gdb=args.gdb, readelf=args.readelf, host=args.host,
                               telnet_port=args.telnet_port)
    try:
        identity_result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as error:
        (args.output / "identity_error.txt").write_text(str(error) + "\n", encoding="utf-8")
        print(f"只读固件身份核对未能完成：{error}", file=sys.stderr)
        return 2
    (args.output / "identity_preflight.txt").write_text(
        f"command={command!r}\nreturncode={identity_result.returncode}\n"
        f"stdout:\n{identity_result.stdout}\nstderr:\n{identity_result.stderr}\n",
        encoding="utf-8",
    )
    if identity_result.returncode != 0:
        print("板上固件身份核对失败；未打开 CDC 串口或发送测试帧", file=sys.stderr)
        return 2
    parser_frames = StatusFrames()
    sent = received = failures = reopen_count = 0
    capture_error = None
    start = time.monotonic()
    deadline = start + args.duration
    next_send = start
    session = {"schema": 1, "port": args.port, "vid": USB_VID, "pid": USB_PID,
               "usb_serial": matches[0].serial_number, "source_sha": args.source_sha,
               "elf": str(args.elf.resolve()), "elf_sha256": digest,
               "duration_limit_s": args.duration, "tx": "fixed-neutral-mode0-only",
               "read_timeout_s": SERIAL_READ_TIMEOUT_S,
               "reconnect_timeout_s": RECONNECT_TIMEOUT_S,
               "identity_preflight": str(identity_root.resolve()),
               "identity_mode": identity_mode}
    port = None
    try:
        with (args.output / "raw.bin").open("xb") as raw, (args.output / "raw.jsonl").open("x", encoding="utf-8") as log:
            log.write(json.dumps({"type": "session", **session}, sort_keys=True) + "\n")
            try:
                port = _open_serial_port(serial, args.port)
            except (OSError, serial.SerialException) as error:
                capture_error = str(error)
            # 短时等待读结果能让Windows主机持续请求IN数据；重枚举后只发下一份新帧。
            while capture_error is None and time.monotonic() < deadline:
                try:
                    now = time.monotonic()
                    if now >= next_send:
                        frame = neutral_frame(sent)
                        if port.write(frame) != len(frame):
                            raise OSError("short CDC write; stop without replaying partial frame")
                        sent += 1
                        next_send = now + FRAME_PERIOD_S
                    data = port.read(512)
                    if data:
                        raw.write(data)
                        for item in parser_frames.feed(data):
                            received += 1
                            log.write(json.dumps({"type": "status", "elapsed_s": round(time.monotonic()-start, 6),
                                                  **item}, allow_nan=False) + "\n")
                    time.sleep(0.001)
                except (OSError, serial.SerialException) as error:
                    failures += 1
                    log.write(json.dumps({"type": "io_error", "elapsed_s": round(time.monotonic()-start, 6),
                                          "error": str(error)}, ensure_ascii=False) + "\n")
                    try:
                        port.close()
                    except (OSError, serial.SerialException):
                        pass
                    port = None
                    reconnect_deadline = min(deadline, time.monotonic() + RECONNECT_TIMEOUT_S)
                    try:
                        active_port, port = _reopen_serial_port(
                            serial, list_ports, matches[0].serial_number, reconnect_deadline
                        )
                    except (OSError, serial.SerialException) as reopen_error:
                        capture_error = f"CDC reconnect failed after {error}: {reopen_error}"
                        break
                    reopen_count += 1
                    next_send = time.monotonic() + FRAME_PERIOD_S
                    log.write(json.dumps({"type": "reopened", "elapsed_s": round(time.monotonic()-start, 6),
                                          "port": active_port}, ensure_ascii=False) + "\n")
                    log.flush()
            if port is not None:
                port.close()
            raw.flush()
            log.write(json.dumps({"type": "summary", "sent": sent, "received": received,
                                  "rejected_bytes": parser_frames.rejected,
                                  "partial_bytes": len(parser_frames.buffer), "failures": failures,
                                  "reopen_count": reopen_count, "complete": capture_error is None}) + "\n")
            log.flush()
        if capture_error is not None:
            (args.output / "error.txt").write_text(capture_error + "\n", encoding="utf-8")
            return 2
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, serial.SerialException) as error:
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass
        (args.output / "error.txt").write_text(str(error) + "\n", encoding="utf-8")
        return 2
    return 0 if capture_error is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
