#!/usr/bin/env python3
"""读取 MC-02 上的性能探针, 打印两次快照之间的 CPU 占用率与分段耗时。

用途：CPU 饱和排查/优化前后对比。即使 RTT 日志被热路径日志刷爆，探针变量
仍然是可靠的（它们只是普通 RAM）。

用法：
    python tools/probe_snapshot.py                 # 采样 30 秒
    python tools/probe_snapshot.py --seconds 60
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

# (名字, 地址, 32bit 字数) —— 与 nm 输出对应, 详见 docs/代码审查报告_MC-02.md
REGIONS = [
    ("can", 0x2400A8A4, 12),
    ("ins", 0x2400AEEC, 16),
    ("lcd_buf", 0x2400AFF0, 5),
    ("task_name", 0x2400B4CC, 36),   # 12 个任务名(每个 12 字节, 只用前 count 个)
    ("task_rt", 0x2400B568, 12),
    ("lcd_prof", 0x2400B598, 12),
    ("dji", 0x2400B6B4, 12),
    ("motor", 0x2400B724, 12),
]

RUNTIME_UNIT_US = 1000.0 / 480.0  # RTOS 计数器单位(千周期)换算成 us
PROBE_UNIT_US = 1.0               # 探针 total_us 本身就是 us
PROBE_LEN = 4                     # 每个 DWT_Probe_t 是 4 个 32bit: last/max/total/calls


def task_names(words: list[int]) -> list[str]:
    """lcd_stats_name 是 12 个 12 字节的名字。"""
    names = []
    for i in range(12):
        chunk = b"".join(w.to_bytes(4, "little") for w in words[i * 3:i * 3 + 3])[:12]
        names.append(chunk.split(b"\x00")[0].decode("ascii", errors="replace"))
    return names


def read_all(openocd: str) -> dict[str, list[int]]:
    cmds = ["init", "halt"]
    for _, addr, count in REGIONS:
        cmds.append(f"mdw {hex(addr)} {count}")
    cmds += ["resume", "exit"]
    args = [openocd, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32h7x.cfg"]
    for c in cmds:
        args += ["-c", c]
    res = subprocess.run(args, capture_output=True, text=True)
    out = res.stdout + res.stderr
    words = []
    for line in out.splitlines():
        m = re.match(r"^(0x[0-9a-fA-F]+):\s+(.*)$", line.strip())
        if not m:
            continue
        words += [int(t, 16) for t in m.group(2).split() if re.fullmatch(r"[0-9a-fA-F]{8}", t)]
    result = {}
    pos = 0
    for name, _, count in REGIONS:
        result[name] = words[pos:pos + count]
        pos += count
    if pos == 0 or len(words) < pos:
        raise SystemExit(f"读取探针失败(得到 {len(words)} 个值, 期望 {pos})")
    return result


def diff(now: list[int], before: list[int]) -> list[int]:
    return [(n - b) & 0xFFFFFFFF for n, b in zip(now, before)]


def avg_us(total_delta: int, calls_delta: int, unit: float = PROBE_UNIT_US) -> float:
    return total_delta * unit / calls_delta if calls_delta else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--openocd", default=DEFAULT_OPENOCD)
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()

    a = read_all(args.openocd)
    t0 = time.time()
    time.sleep(args.seconds)
    b = read_all(args.openocd)
    win = time.time() - t0

    print(f"=== 采样窗口 {win:.1f}s ===")

    # 各任务占用率
    rt = diff(b["task_rt"], a["task_rt"])
    names = task_names(b["task_name"])
    total = sum(rt)
    print("\n[任务占用率] (按 RTOS 运行时间统计, 千分比)")
    for name, share in sorted(((n, r * 1000.0 / total if total else 0.0)
                               for n, r in zip(names, rt) if n), key=lambda x: -x[1]):
        print(f"  {name:<12} {share:7.1f}")

    # 分段耗时
    def probe(region: str, idx: int, label: str):
        off = idx * PROBE_LEN
        _, mx, _, _ = b[region][off:off + PROBE_LEN]          # 绝对值: last/max
        _, _, tot, calls = diff(b[region], a[region])[off:off + PROBE_LEN]  # 增量算平均
        print(f"  {label:<14} avg={avg_us(tot, calls):9.1f}us  max={mx:>8}us  "
              f"calls={calls:<8} ({calls / win:6.0f}/s)")

    print("\n[分段耗时] (us)")
    print(" motor 任务:")
    probe("motor", 0, "LKMotorControl")
    probe("motor", 1, "DJIMotorControl")
    probe("motor", 2, "MotorControlTask")
    print(" DJI 内部:")
    probe("dji", 0, "CAN 发送")
    probe("dji", 1, "PID+组帧")
    probe("dji", 2, "DJI 合计")
    print(" INS:")
    probe("ins", 0, "温控")
    probe("ins", 1, "EKF")
    probe("ins", 2, "BMI088读取")
    probe("ins", 3, "INS_Task合计")
    print(" CAN:")
    c = diff(b["can"], a["can"])
    print(f"  rx_frames={c[0]}  tx_calls={c[10]}  tx_full={c[5]}  tx_spins={c[6]}  "
          f"tx_avg={avg_us(c[9], c[10]):.1f}us  tx_max={b['can'][8]}us  rx_isr_max={b['can'][2]}us")
    print(" LCD:")
    lp = diff(b["lcd_prof"], a["lcd_prof"])
    lb = b["lcd_buf"]
    print(f"  update_last={b['lcd_prof'][9]}us update_max={b['lcd_prof'][8]}us "
          f"refreshes={lp[0]} ({lp[0] / win:.1f}/s) clear_last={lb[0]}us "
          f"last_row={lb[3]}B/{lb[4]}us")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
