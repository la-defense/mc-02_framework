#!/usr/bin/env python3
"""通过 OpenOCD 读取目标板上的 SEGGER RTT 上行缓冲并打印日志文本。

为什么需要它：本机的 xPack OpenOCD 构建没有编译 rtt 命令（`rtt setup` 会报
"Unexpected command line argument"），而 RTT 的上行缓冲本来就是普通 RAM，
所以直接用 memory read 读出来解码即可，不依赖任何 RTT 客户端。

用法：
    python tools/rtt_dump.py                     # 采集 10 秒，只打印新增日志
    python tools/rtt_dump.py --seconds 30        # 采集 30 秒
    python tools/rtt_dump.py --filter cpu        # 只打印含 "cpu" 的行
    python tools/rtt_dump.py --all               # 连历史内容一起打印

依赖：arm-none-eabi-nm（用于定位 _SEGGER_RTT）、OpenOCD（路径见 --openocd）。
"""

import argparse
import re
import subprocess
import sys
import time

DEFAULT_OPENOCD = (
    r"C:\Users\Administrator\AppData\Roaming\Code\User\globalStorage"
    r"\bmd.stm32-for-vscode\@xpack-dev-tools\openocd\0.12.0-7.1\.content\bin\openocd.EXE"
)
DEFAULT_ELF = r"build\Basic_Framework_MC02.elf"


def addr_of_rtt_cb(elf: str) -> int:
    out = subprocess.run(
        ["arm-none-eabi-nm", "-C", "-n", elf],
        capture_output=True, text=True, check=True,
    ).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == "_SEGGER_RTT":
            return int(parts[0], 16)
    raise SystemExit("未在 ELF 中找到 _SEGGER_RTT 符号")


def ocd_read(openocd: str, cmds: list[str]) -> str:
    args = [openocd, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32h7x.cfg",
            "-c", "init", "-c", "halt"] + [c for cmd in cmds for c in ("-c", cmd)] + \
           ["-c", "resume", "-c", "exit"]
    # OpenOCD 把 mdw 的结果写到 stdout、日志写到 stderr，这里两者都取
    res = subprocess.run(args, capture_output=True, text=True)
    return res.stdout + res.stderr


def parse_words(out: str, addr: int, count: int) -> list[int]:
    """从 OpenOCD mdw 输出里取出 count 个 32bit 值。"""
    words: list[int] = []
    for line in out.splitlines():
        m = re.match(r"^(0x[0-9a-fA-F]+):\s+(.*)$", line.strip())
        if not m:
            continue
        for tok in m.group(2).split():
            if re.fullmatch(r"[0-9a-fA-F]{8}", tok):
                words.append(int(tok, 16))
    return words[:count]


def read_rtt_header(openocd: str, cb: int) -> dict:
    out = ocd_read(openocd, [f"mdw {hex(cb)} 12"])
    w = parse_words(out, cb, 12)
    if len(w) < 12:
        raise SystemExit("读取 RTT 控制块失败")
    idb = b"".join(x.to_bytes(4, "little") for x in w[0:4])
    if b"SEGGER RTT" not in idb:
        raise SystemExit(f"控制块 ID 不正确: {idb!r}")
    return {
        "max_up": w[4], "max_down": w[5],
        "up_name": w[6], "up_buf": w[7], "up_size": w[8],
        "up_wr": w[9], "up_rd": w[10], "up_flags": w[11],
    }


def read_buffer(openocd: str, addr: int, size: int) -> bytes:
    """按 4 字节读回整段缓冲。"""
    out = ocd_read(openocd, [f"mdw {hex(addr)} {size // 4}"])
    w = parse_words(out, addr, size // 4)
    return b"".join(x.to_bytes(4, "little") for x in w)[:size]


def write_rd(openocd: str, cb: int, rd: int) -> None:
    """把读指针写回目标: 否则缓冲很快写满, 固件的新日志会被静默丢弃。"""
    ocd_read(openocd, [f"mww {hex(cb + 40)} {hex(rd)}"])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--openocd", default=DEFAULT_OPENOCD)
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--filter", default=None, help="只打印包含该子串的行")
    ap.add_argument("--all", action="store_true", help="打印缓冲内全部内容而不是仅新增")
    args = ap.parse_args()

    cb = addr_of_rtt_cb(args.elf)
    hdr = read_rtt_header(args.openocd, cb)
    size = hdr["up_size"]
    base = hdr["up_buf"]
    print(f"# RTT 控制块 {hex(cb)}  上行缓冲 {hex(base)}  {size} 字节  "
          f"WrOff={hdr['up_wr']} RdOff={hdr['up_rd']}", file=sys.stderr)

    # 默认只看"从现在开始"的新日志; --all 才把缓冲里的历史内容一起打印
    if not args.all:
        # 缓冲区可能已经被塞满(没人读时固件会静默丢日志), 先把读指针推到写指针处
        write_rd(args.openocd, cb, hdr["up_wr"])
    deadline = time.time() + args.seconds
    pending = b""
    while True:
        out = ocd_read(args.openocd, [f"mdw {hex(cb + 36)} 2"])
        w = parse_words(out, cb + 36, 2)
        # 以目标里的 RdOff 为准(它就是"已经被读走的位置"), 这样回绕也安全
        wr = w[0] if len(w) == 2 else hdr["up_wr"]
        rd_now = w[1] if len(w) == 2 else hdr["up_rd"]
        if wr != rd_now:
            buf = read_buffer(args.openocd, base, size)
            chunk = buf[rd_now:wr] if wr >= rd_now else buf[rd_now:] + buf[:wr]
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                text = re.sub(r"\x1b\[[0-9;]*m", "", text).strip()
                if not text:
                    continue
                if args.filter and args.filter not in text:
                    continue
                print(text)
            sys.stdout.flush()
            write_rd(args.openocd, cb, wr)
        if time.time() >= deadline:
            break
        time.sleep(0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
