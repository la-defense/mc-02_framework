#!/usr/bin/env python3
"""离线检查 MC02 固定布局崩溃记录，并为现场保存匹配构建产物。

仅读取输入记录、ELF 和 map；v3 记录没有完整 Build ID，因此不会按当前 ELF
解析地址。archive 子命令只在新目录保存证据，不覆盖已有会话。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import struct
import subprocess
import sys
import zlib
from datetime import datetime, timezone
from pathlib import Path


V3_MAGIC_BUSY = 0x59535542
V3_MAGIC_OK = 0x4B4F5243
V4_MAGIC_BUSY = 0x34495342
V4_MAGIC_OK = 0x344F5243
V3_LENGTH = 252
V4_LENGTH = 336
CAPACITY = 512
UINT32_MAX = 0xFFFFFFFF
CRASH_TYPES = {
    0: "NONE", 1: "HARDFAULT", 2: "MEMMANAGE", 3: "BUSFAULT",
    4: "USAGEFAULT", 5: "NMI", 6: "ASSERT", 7: "STACK_OVF",
}


class CrashRecordError(ValueError):
    """输入记录不符合本工具支持的固定布局。"""


def _u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def _text(data: bytes, length_offset: int, text_offset: int, capacity: int) -> str:
    length = _u32(data, length_offset)
    if length > capacity:
        raise CrashRecordError("文字长度超过固定字段容量")
    field = data[text_offset:text_offset + capacity]
    if len(field) != capacity or (length < capacity and field[length] != 0):
        raise CrashRecordError("文字字段长度或结束标记无效")
    return field[:length].decode("ascii", errors="replace")


def decode_record(data: bytes) -> dict:
    """检查并解码一条 v3/v4 记录；不对记录里的 PC 做地址推测。"""
    if not isinstance(data, bytes) or not data or len(data) > CAPACITY:
        raise CrashRecordError("记录长度须为1至512字节")
    if len(data) < 12:
        raise CrashRecordError("记录头不足12字节")

    magic, declared_length, version = struct.unpack_from("<III", data)
    if version == 3 and magic in (V3_MAGIC_BUSY, V3_MAGIC_OK):
        if declared_length != V3_LENGTH or len(data) != V3_LENGTH:
            raise CrashRecordError("v3记录必须恰好为252字节")
        stored_crc = _u32(data, 16)
        calculated_crc = zlib.crc32(data[40:]) & UINT32_MAX
        if stored_crc != calculated_crc:
            raise CrashRecordError("v3 CRC32-IEEE不匹配")
        # v3只有弱fw_id；不依赖无法核实的历史ELF解释寄存器和栈字段。
        payload_words = list(struct.unpack_from("<53I", data, 40))
        return {
            "version": 3,
            "status": "valid" if magic == V3_MAGIC_OK else ("valid-busy-final" if _u32(data, 20) == 4 else "writing"),
            "length": declared_length,
            "firmware_id_weak": f"0x{_u32(data, 12):08x}",
            "count": _u32(data, 24),
            "type": CRASH_TYPES.get(_u32(data, 28), f"UNKNOWN_{_u32(data, 28)}"),
            "arg0": _u32(data, 32),
            "arg1": _u32(data, 36),
            "legacy_payload_words": payload_words,
            "crc32": f"0x{stored_crc:08x}",
            "identity_limit": "v3只有弱fw_id；寄存器和栈仅保留原始字，不推测故障位置。",
        }

    if version != 4:
        raise CrashRecordError(f"不支持记录版本{version}")
    if declared_length != V4_LENGTH or len(data) != V4_LENGTH:
        raise CrashRecordError("v4记录必须恰好为336字节")
    if magic not in (V4_MAGIC_OK, V4_MAGIC_BUSY):
        raise CrashRecordError("v4 magic无效")
    progress = _u32(data, 12)
    if magic == V4_MAGIC_BUSY and progress != 4:
        raise CrashRecordError(f"记录尚未完成，写入阶段={progress}")
    stored_crc = _u32(data, 332)
    calculated_crc = zlib.crc32(data[4:332]) & UINT32_MAX
    if stored_crc != calculated_crc:
        raise CrashRecordError("v4 CRC32-IEEE不匹配")

    build_id = data[32:52].hex()
    stack_words = _u32(data, 200)
    if stack_words > 32:
        raise CrashRecordError("栈快照长度超过32字")
    return {
        "version": 4,
        "status": "valid" if magic == V4_MAGIC_OK else "valid-busy-final",
        "length": declared_length,
        "progress": progress,
        "count": _u32(data, 16),
        "type": CRASH_TYPES.get(_u32(data, 20), f"UNKNOWN_{_u32(data, 20)}"),
        "arg0": _u32(data, 24),
        "arg1": _u32(data, 28),
        "build_id": build_id if _u32(data, 52) & 1 else None,
        "identity_flags": _u32(data, 52),
        "profile": _u32(data, 56),
        "board_role": _u32(data, 60),
        "module_mask": _u32(data, 64),
        "config_version": _u32(data, 68),
        "task_name": _text(data, 72, 76, 16),
        "source_name": _text(data, 92, 96, 32),
        "fault_registers": dict(zip(("cfsr", "hfsr", "bfar", "mmfar", "dfsr"),
                                     struct.unpack_from("<5I", data, 128))),
        "core_registers": dict(zip(("r0", "r1", "r2", "r3", "r12", "lr", "pc", "xpsr"),
                                   struct.unpack_from("<8I", data, 148))),
        "frame_status": _u32(data, 180),
        "exception_return": _u32(data, 184),
        "sp": _u32(data, 188),
        "psp": _u32(data, 192),
        "msp": _u32(data, 196),
        "stack_snapshot": list(struct.unpack_from(f"<{stack_words}I", data, 204)) if stack_words else [],
        "crc32": f"0x{stored_crc:08x}",
    }


def read_elf_build_id(elf: Path, readelf: str = "arm-none-eabi-readelf") -> str:
    """通过GNU readelf读取ELF的GNU Build ID；缺工具或缺标识均明确失败。"""
    result = subprocess.run([readelf, "-n", str(elf)], check=False, capture_output=True, text=True)
    if result.returncode != 0:
        raise CrashRecordError(f"readelf读取失败: {result.stderr.strip()}")
    for line in result.stdout.splitlines():
        if "Build ID:" in line:
            value = line.split("Build ID:", 1)[1].strip().lower()
            if len(value) == 40 and all(char in "0123456789abcdef" for char in value):
                return value
    raise CrashRecordError("ELF中没有20字节GNU Build ID")


def symbolize_if_matched(record: dict, elf: Path | None, readelf: str, addr2line: str) -> dict:
    """仅当v4完整Build ID相等时才调用addr2line。"""
    result = dict(record)
    if elf is None:
        result["symbolization"] = "未提供ELF，保留原始地址"
        return result
    elf_id = read_elf_build_id(elf, readelf)
    result["elf_build_id"] = elf_id
    record_id = record.get("build_id")
    if record.get("version") != 4 or record_id is None:
        result["symbolization"] = "记录没有可用于匹配的完整Build ID；不解析地址"
        return result
    if record_id != elf_id:
        result["symbolization"] = "Build ID不匹配；不解析地址"
        return result
    addresses = record.get("core_registers", {})
    pc = addresses.get("pc", 0)
    lr = addresses.get("lr", 0)
    cmd = [addr2line, "-f", "-C", "-e", str(elf), f"0x{pc:x}", f"0x{lr:x}"]
    symbols = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if symbols.returncode != 0:
        result["symbolization"] = f"Build ID匹配，但addr2line失败: {symbols.stderr.strip()}"
        return result
    result["symbolization"] = "Build ID匹配"
    result["pc_lr_symbols"] = symbols.stdout.splitlines()
    return result


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def extract_snapshot_record(path: Path) -> tuple[bytes, str]:
    """从受限快照JSONL中提取固定记录，仅接受已核对身份的v4内容。"""
    session = None
    identity = None
    samples = []
    errors = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError as error:
            raise CrashRecordError("快照JSONL包含损坏行") from error
        if item.get("type") == "session":
            session = item
        elif item.get("type") == "identity":
            identity = item
        elif item.get("type") == "sample":
            samples.append(item)
        elif item.get("type") == "error":
            errors.append(item)
    if session is None or session.get("mode") != "crash-record" or identity is None:
        raise CrashRecordError("输入不是已完成身份检查的crash-record快照")
    if errors or not samples:
        raise CrashRecordError("快照读取不完整或包含错误")
    words = samples[-1].get("raw_words", {}).get("crash_record_words")
    if (not isinstance(words, list) or len(words) != CAPACITY // 4 or
            any(type(word) is not int or not 0 <= word <= UINT32_MAX for word in words)):
        raise CrashRecordError("固定记录快照必须包含128个32位字")
    raw = struct.pack("<128I", *words)
    declared_length = _u32(raw, 4)
    if declared_length not in (V3_LENGTH, V4_LENGTH):
        raise CrashRecordError("快照中没有受支持长度的崩溃记录")
    record_bytes = raw[:declared_length]
    record = decode_record(record_bytes)
    expected_build_id = identity.get("build_id")
    if record.get("version") != 4 or record.get("build_id") != expected_build_id:
        raise CrashRecordError("记录Build ID与快照所选ELF不一致")
    return record_bytes, expected_build_id


def archive_artifacts(record_path: Path, elf: Path, map_file: Path, output: Path,
                      readelf: str = "arm-none-eabi-readelf") -> dict:
    """将记录及匹配的ELF/map复制到新会话目录，并保存文件指纹。"""
    record_bytes = record_path.read_bytes()
    decoded = decode_record(record_bytes)
    if decoded.get("version") != 4:
        raise CrashRecordError("仅含完整Build ID的v4记录可以与ELF/map建立归档关联")
    build_id = read_elf_build_id(elf, readelf)
    if decoded.get("version") == 4 and decoded.get("build_id") != build_id:
        raise CrashRecordError("记录和ELF Build ID不匹配，拒绝归档为同一现场")
    if not elf.is_file() or not map_file.is_file():
        raise CrashRecordError("ELF或map文件不存在")
    output.mkdir(parents=True, exist_ok=False)
    copied = []
    for source, name in ((record_path, "crash-record.bin"),
                         (elf, elf.name), (map_file, map_file.name)):
        destination = output / name
        shutil.copyfile(source, destination)
        copied.append({"name": name, "size_bytes": destination.stat().st_size,
                       "sha256": sha256_file(destination)})
    manifest = {
        "format": "mc02-crash-artifact-v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "record_version": decoded["version"],
        "record_status": decoded["status"],
        "record_identity": decoded.get("build_id") or decoded.get("firmware_id_weak"),
        "elf_build_id": build_id,
        "elf_sha256": sha256_file(elf),
        "files": copied,
        "record": decoded,
        "note": ("v3身份仅为弱fw_id；本归档不授权用该ELF符号化。" if decoded["version"] == 3
                 else "Build ID确认记录与ELF属于同一链接映像；SHA-256确认归档文件内容。"),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                                          encoding="utf-8")
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--record", type=Path, help="固定布局原始二进制记录")
    source.add_argument("--snapshot-jsonl", type=Path, help="由probe_snapshot_live --crash-record生成的原始快照")
    parser.add_argument("--extract-to", type=Path, help="从快照提取记录时保存到不存在的新文件")
    parser.add_argument("--elf", type=Path, help="可选ELF；只有v4完整Build ID匹配时才符号化")
    parser.add_argument("--readelf", default="arm-none-eabi-readelf")
    parser.add_argument("--addr2line", default="arm-none-eabi-addr2line")
    parser.add_argument("--archive-dir", type=Path, help="可选新目录，同时复制记录、ELF和map")
    parser.add_argument("--map", type=Path, help="与ELF匹配的map文件，使用--archive-dir时必需")
    args = parser.parse_args(argv)
    try:
        if args.snapshot_jsonl is not None:
            if args.extract_to is None:
                raise CrashRecordError("--snapshot-jsonl需要--extract-to")
            record_bytes, snapshot_build_id = extract_snapshot_record(args.snapshot_jsonl)
            args.extract_to.parent.mkdir(parents=True, exist_ok=True)
            with args.extract_to.open("xb") as stream:
                stream.write(record_bytes)
            args.record = args.extract_to
        else:
            if args.extract_to is not None:
                raise CrashRecordError("--extract-to仅用于--snapshot-jsonl")
            record_bytes = args.record.read_bytes()
            snapshot_build_id = None
        decoded = decode_record(record_bytes)
        if snapshot_build_id is not None and decoded.get("build_id") != snapshot_build_id:
            raise CrashRecordError("快照身份与记录Build ID不匹配")
        if args.archive_dir is not None:
            if args.elf is None or args.map is None:
                raise CrashRecordError("--archive-dir需要同时提供--elf和--map")
            archive_artifacts(args.record, args.elf, args.map, args.archive_dir, args.readelf)
        decoded = symbolize_if_matched(decoded, args.elf, args.readelf, args.addr2line)
    except (OSError, CrashRecordError) as error:
        print(f"崩溃记录处理失败：{error}", file=sys.stderr)
        return 2
    print(json.dumps(decoded, ensure_ascii=False, indent=2))
    return 0 if decoded["status"].startswith("valid") else 2


if __name__ == "__main__":
    raise SystemExit(main())
