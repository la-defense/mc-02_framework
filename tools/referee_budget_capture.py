#!/usr/bin/env python3
"""REF-03有限串口回归：发送机器人状态，保存线上字节及运行态只读MCU记录。

仅用于已verify的具名refdiag禁热固件和3.3V TTL裁判口；不发送控制、使能或开火命令。
session沿用固件身份登记字段。串口到达时间不是精确线端时间，额度判定使用MCU记录。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import re
import json
from pathlib import Path
import struct
import threading
import time

import probe_snapshot_live as probe

MASK = 0xffffffff


def validate_identity(session: dict, elf: Path) -> str:
    """只核对本轮登记的准确软件SHA与已verify的ELF；不替代物理接线或Flash校验。"""
    if session.get("schema") != 1 or not isinstance(session.get("firmware"), dict):
        raise ValueError("requires schema 1 firmware identity session")
    firmware = session["firmware"]
    digest = hashlib.sha256(elf.read_bytes()).hexdigest()
    if firmware.get("flash_verified") is not True or firmware.get("elf_sha256") != digest:
        raise ValueError("session must record the verified ELF SHA-256")
    if not re.fullmatch(r"[0-9a-f]{40}", firmware.get("source_sha") or ""):
        raise ValueError("session requires the exact firmware source SHA")
    return digest


def crc(data: bytes, width: int, poly: int, initial: int) -> int:
    """独立逐位反射算法，对照生产CRC8/CRC16；不使用被测模块的查表。"""
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (poly if value & 1 else 0)
    return value & ((1 << width) - 1)


def robot_frame(robot: int, sequence: int) -> bytes:
    """只生成0x0201合法身份帧，其他字段置零，不包含机器人控制。"""
    if robot not in (2, 3):
        raise ValueError("diagnostic sender must be red robot 2 or 3")
    header = bytearray((0xa5, 17, 0, sequence & 255))
    header.append(crc(header, 8, 0x8c, 0xff))
    frame = bytes(header) + b"\x01\x02" + bytes((robot,)) + bytes(16)
    return frame + struct.pack("<H", crc(frame, 16, 0x8408, 0xffff))


class Frames:
    """有界流解析：坏CRC逐字节重同步，不把半帧认作完整记录。"""
    def __init__(self):
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, data: bytes) -> list[bytes]:
        result = []
        self.buffer.extend(data)
        while self.buffer:
            if self.buffer[0] != 0xa5:
                del self.buffer[0]
                self.rejected += 1
                continue
            if len(self.buffer) < 5:
                break
            length = int.from_bytes(self.buffer[1:3], "little") + 9
            if length > 127 or length < 15 or crc(self.buffer[:4], 8, 0x8c, 0xff) != self.buffer[4]:
                del self.buffer[0]
                self.rejected += 1
                continue
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            if crc(frame[:-2], 16, 0x8408, 0xffff) != int.from_bytes(frame[-2:], "little"):
                del self.buffer[0]
                self.rejected += 1
                continue
            result.append(frame)
            del self.buffer[:length]
        return result


def decode_trace(words: list[int], previous: int, before: int | None = None,
                 after: int | None = None) -> tuple[list[dict], int]:
    """解码固定456字记录；拒绝错ELF布局、丢环记录和未发布内容。"""
    if len(words) != 456 or words[0] != 0x52454633 or words[1] != 1:
        raise ValueError("referee trace layout mismatch")
    if before is None or after is None:
        if words[7] & 1:
            raise ValueError("MCU completion publication in progress")
        before = after = words[7]
    if not 0 <= before <= words[7] <= after <= MASK:
        raise ValueError("MCU publication reset or invalid read order")
    count = before // 2
    # 不强求整块读取期间停止更新；只取读取前已发布的记录。若写者已经绕环
    # 覆盖到本批所需槽位，就拒绝。ceil包括尚未发布但正在改写的那一个槽位。
    if (after + 1) // 2 - previous > 64:
        raise ValueError("MCU trace overrun during read")
    difference = (count - previous) & MASK
    if difference > 64:
        raise ValueError("MCU trace overrun or reset; evidence is incomplete")
    records = []
    for ordinal in range(previous, previous + difference):
        item = words[8 + (ordinal % 64) * 7:15 + (ordinal % 64) * 7]
        ident, submitted, started, completed, result, flag, tail = item
        length, kind, receiver = tail & 255, (tail >> 8) & 255, tail >> 16
        if not ident or result > 4 or flag not in (0, 1) or length not in (17, 127) or kind > 1:
            raise ValueError("invalid MCU completion record")
        records.append(dict(ordinal=ordinal, id=ident, submitted_ms=submitted, started_ms=started,
                            completed_ms=completed, result=result, started=flag, length=length, kind=kind, receiver=receiver))
    return records, count


def trace_snapshot(addresses: dict, host: str, port: int) -> dict:
    """MCU保持运行，固定对象整块只读；前后版本用于判断所需槽位是否已被覆盖。"""
    fields = probe.REFEREE_SNAPSHOT_FIELDS
    with probe.OpenOCDTelnetClient(host, port, 2.0, addresses, fields) as client:
        probe._require_running(client.command("targets"), "before referee read")
        count_address = addresses["ref_count"]
        before = probe._parse_memory_value(client.command(f"mdw 0x{count_address:08x} 1"), count_address, 32)
        base = addresses["ref_magic"]
        words = probe._parse_memory_words(client.command(f"mdw 0x{base:08x} 456"), base, 456)
        after = probe._parse_memory_value(client.command(f"mdw 0x{count_address:08x} 1"), count_address, 32)
        probe._require_running(client.command("targets"), "after referee read")
    return dict(words=words, before=before, after=after)


def write_frame(port, frame: bytes) -> None:
    """有界发送完整身份帧；短写/超时中止，避免持续写入残帧。"""
    if port.write(frame) != len(frame):
        raise OSError("partial serial write; capture aborted")


def report_capture(folder: Path) -> dict:
    """逐项核对MCU预算和线上完整帧；不以PC时间判定34ms或精确线速。"""
    capture = json.loads((folder / "capture.json").read_text(encoding="utf-8"))
    if capture["failures"] or capture["rejected_bytes"] or capture["residual_bytes"]:
        raise ValueError("serial evidence contains errors or incomplete frames")
    records = []
    snapshots = []
    previous_count = 0
    for line in (folder / "mcu.jsonl").read_text(encoding="utf-8").splitlines():
        item = json.loads(line)
        snapshots.append(item["words"])
        decoded, previous_count = decode_trace(item["words"], previous_count, item.get("before"), item.get("after"))
        if decoded != item["records"]:
            raise ValueError("decoded MCU records disagree with raw words")
        records.extend(decoded)
    if not snapshots or snapshots[-1][3] != 3:
        raise ValueError("finite diagnostic has not finished")
    if snapshots[-1][7] // 2 != len(records):
        raise ValueError("missing completion records")
    with (folder / "frames.csv").open(newline="", encoding="utf-8") as stream:
        frames = list(csv.DictReader(stream))
    successful = [r for r in records if r["result"] == 0]
    if len(successful) != len(frames) or any(r["result"] in (1, 4) for r in records):
        raise ValueError("wire frames and successful completions disagree or UART failed")
    previous = None
    history = []
    maximum = 0
    for r in records:
        if not r["started"]:
            continue
        now = r["started_ms"]
        if previous is not None and ((now - previous) & MASK) < 34:
            raise ValueError("MCU start interval below 34ms")
        previous = now
        history = [h for h in history if ((now - h["started_ms"]) & MASK) < 1000]
        total = r["length"] + sum(h["length"] for h in history if h["receiver"] == r["receiver"])
        cap = 5120 if r["receiver"] in (7, 9, 107, 109) else 3720
        if r["receiver"] < 256 and total > cap:
            raise ValueError("rolling receiver byte budget exceeded")
        maximum = max(maximum, total)
        history.append(r)
    last_ui = None
    for r, frame in zip(successful, frames):
        wire = bytes.fromhex(frame["hex"])
        if Frames().feed(wire) != [wire]:
            raise ValueError("saved wire CRC or length invalid")
        receiver = int.from_bytes(wire[11:13], "little")
        if len(wire) != r["length"] or receiver != r["receiver"]:
            raise ValueError("MCU completion order does not match wire frame")
        if r["kind"]:
            if wire[7:9] != b"\x00\x01" or wire[13:-2] != b"\0\0":
                raise ValueError("unexpected UI diagnostic content")
            if last_ui is not None and ((r["started_ms"]-last_ui) & MASK) < 115:
                raise ValueError("UI start interval below 115ms")
            last_ui = r["started_ms"]
        elif wire[7:9] != b"\x00\x02" or wire[13:-2] != bytes(range(112)):
            raise ValueError("unexpected full-length interaction content")
    begin = snapshots[-1][2]
    full = [r for r in successful if ((r["started_ms"]-begin) & MASK) < 30000]
    mixed = [r for r in successful if 30000 <= ((r["started_ms"]-begin) & MASK) < 75000]
    if not full or not mixed or not any(r["kind"] for r in mixed):
        raise ValueError("missing full-length or mixed phase")
    if ((full[-1]["started_ms"]-begin) & MASK) < 29000 or \
            ((mixed[-1]["started_ms"]-mixed[0]["started_ms"]) & MASK) < 30000:
        raise ValueError("insufficient measured phase coverage")
    if {int(frame["sender"]) for frame in frames} != {2, 3}:
        raise ValueError("identity change and receive recovery not observed")
    return dict(frames=len(frames), completions=len(records), max_receiver_window_bytes=maximum,
                full_phase_frames=len(full), mixed_phase_frames=len(mixed), software_budget_pass=True,
                limit="MCU timestamps; no physical waveform or client-display guarantee")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--elf", type=Path, required=True)
    ap.add_argument("--session", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--gdb", default="arm-none-eabi-gdb")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--telnet-port", type=int, default=4444)
    args = ap.parse_args()
    session = json.loads(args.session.read_text(encoding="utf-8"))
    digest = validate_identity(session, args.elf)
    if "_refdiag_bench_safe.elf" not in args.elf.name:
        ap.error("requires the named bench_safe referee diagnostic ELF")
    addresses = probe.resolve_symbol_addresses(args.gdb, str(args.elf), 5.0, probe.REFEREE_SNAPSHOT_FIELDS)
    import serial
    args.output.mkdir(parents=True, exist_ok=False)
    stop = threading.Event()
    failures = []

    def observe():
        previous = 0
        try:
            with (args.output / "mcu.jsonl").open("w", encoding="utf-8") as stream:
                while not stop.is_set():
                    snapshot = trace_snapshot(addresses, args.host, args.telnet_port)
                    records, previous = decode_trace(snapshot["words"], previous,
                                                     snapshot["before"], snapshot["after"])
                    stream.write(json.dumps(dict(pc_monotonic=time.monotonic(), **snapshot,
                                                 records=records)) + "\n")
                    stream.flush()
                    stop.wait(0.2)
        except Exception as error:
            failures.append(str(error))
            stop.set()

    thread = threading.Thread(target=observe)
    thread.start()
    parser = Frames()
    rows = []
    identity_rows = []
    try:
        with serial.Serial(args.port, 115200, timeout=0.01, write_timeout=0.1) as port, \
                (args.output / "uart.bin").open("wb") as raw:
            begin = time.monotonic()
            next_send = begin
            sequence = 0
            # 75秒固件负载，额外5秒收尾；65秒停更1秒，70秒变更身份。
            while not stop.is_set() and time.monotonic() - begin < 80.0:
                now = time.monotonic()
                age = now - begin
                if now >= next_send:
                    next_send = now + 0.1
                    if not 65.0 <= age < 66.0:
                        robot = 3 if age < 70.0 else 2
                        write_frame(port, robot_frame(robot, sequence))
                        identity_rows.append(dict(pc_s=age, robot=robot, sequence=sequence))
                        sequence = (sequence + 1) & 255
                data = port.read(4096)
                raw.write(data)
                for frame in parser.feed(data):
                    if frame[5:7] != b"\x01\x03":
                        raise ValueError("unexpected command on dedicated referee TX")
                    content, sender, receiver = struct.unpack("<HHH", frame[7:13])
                    rows.append(dict(pc_s=time.monotonic()-begin, sequence=frame[3], length=len(frame),
                                     content_id=content, sender=sender, receiver=receiver, hex=frame.hex()))
    except Exception as error:
        failures.append(str(error))
        raise
    finally:
        stop.set()
        thread.join(timeout=6.0)
        with (args.output / "frames.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=("pc_s", "sequence", "length", "content_id", "sender", "receiver", "hex"))
            writer.writeheader()
            writer.writerows(rows)
        (args.output / "identity.json").write_text(json.dumps(identity_rows, indent=2), encoding="utf-8")
        (args.output / "capture.json").write_text(json.dumps(dict(elf_sha256=digest,
            source_sha=session["firmware"]["source_sha"], frames=len(rows), rejected_bytes=parser.rejected,
            residual_bytes=len(parser.buffer), failures=failures), indent=2), encoding="utf-8")
    if failures:
        raise RuntimeError("; ".join(failures))
    report = report_capture(args.output)
    (args.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
