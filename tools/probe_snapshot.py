#!/usr/bin/env python3
"""读取 MC-02 上的性能探针, 输出两次快照之间的 CPU 占用率与分段耗时。

用途：CPU 饱和排查 / 优化前后对比。即使 RTT 日志被热路径日志刷爆，探针变量
仍然是可靠的（它们只是普通 RAM）。

用法：
    python tools/probe_snapshot.py                 # 采样 30 秒
    python tools/probe_snapshot.py --seconds 60

注意：符号地址每次编译都会变，所以本工具从 ELF 动态解析（arm-none-eabi-nm）。
"""

import argparse
import re
import subprocess
import time

DEFAULT_OPENOCD = (
    r"C:\Users\Administrator\AppData\Roaming\Code\User\globalStorage"
    r"\bmd.stm32-for-vscode\@xpack-dev-tools\openocd\0.12.0-7.1\.content\bin\openocd.EXE"
)
DEFAULT_ELF = r"build\Basic_Framework_MC02.elf"

# 每个 DWT_Probe_t = 4 个 32bit: last / max / total_us / calls
PROBES = [
    "motor_prof_all", "motor_prof_dji", "motor_prof_lk",
    "dji_prof_all", "dji_prof_send", "dji_prof_pid",
    "ins_prof_all", "ins_prof_temp", "ins_prof_ekf", "ins_prof_read",
    "can_prof_tx", "can_prof_rx_isr",
]
# 单个 32bit 的符号
SCALARS = [
    "can_prof_tx_full", "can_prof_tx_spins", "can_prof_rx_frames",
    "lcd_prof_clear_us", "lcd_prof_rows", "lcd_prof_spi_us",
    "lcd_prof_row_bytes", "lcd_prof_row_us", "lcd_prof_cycles",
    "lcd_prof_update_last_us", "lcd_prof_update_max_us",
    "lcd_prof_draw_last_us", "lcd_prof_draw_max_us",
    "lcd_prof_init_us", "lcd_prof_recover_us", "lcd_prof_busy_us",
    "lcd_stats_count", "lcd_stats_total",
]
TASK_NAME = "lcd_stats_name"   # 12 个任务名 × 12 字节
TASK_RT = "lcd_stats_runtime"  # 12 个 uint32
CAN_BUS = "can_bus"            # CANBusState_t can_bus[3]
# ARM EABI 默认枚举取"能容纳的最小类型"(这里 status 是 1 字节):
# 结构体 = 4(handle) + 20(tx) + 20(bus) + 8(log_ms/log_cnt) = 52 字节(实测步长一致)
CAN_BUS_STRIDE = 52
RUNTIME_UNIT_US = 1000.0 / 480.0  # RTOS 计数器单位(千周期) → us


def symbols(elf: str) -> dict:
    out = subprocess.run(["arm-none-eabi-nm", "-C", "-n", elf],
                         capture_output=True, text=True, check=True).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and re.fullmatch(r"[0-9a-fA-F]+", parts[0]):
            table[parts[2]] = int(parts[0], 16)
    return table


def read_words(openocd: str, reqs: list) -> dict:
    args = [openocd, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32h7x.cfg",
            "-c", "init", "-c", "halt"]
    for addr, count in reqs:
        args += ["-c", f"mdw {hex(addr)} {count}"]
    args += ["-c", "resume", "-c", "exit"]
    res = subprocess.run(args, capture_output=True, text=True)
    # OpenOCD 对一次 mdw 会打印多行(每行 4 个 32bit), 按命令顺序把词全部收集起来再切片
    flat = []
    for line in (res.stdout + res.stderr).splitlines():
        m = re.match(r"^(0x[0-9a-fA-F]+):\s+((?:[0-9a-fA-F]{8}\s*)+)$", line.strip())
        if not m:
            continue
        flat += [int(t, 16) for t in m.group(2).split() if re.fullmatch(r"[0-9a-fA-F]{8}", t)]
    data = {}
    pos = 0
    for addr, count in reqs:
        data[addr] = flat[pos:pos + count]
        pos += count
    return data


def gather(openocd: str, sym: dict) -> dict:
    reqs = [(sym[p], 4) for p in PROBES]
    reqs += [(sym[s], 1) for s in SCALARS]
    reqs += [(sym[TASK_NAME], 36), (sym[TASK_RT], 12)]
    bus_base = sym.get(CAN_BUS, 0)
    reqs += [(bus_base + i * CAN_BUS_STRIDE, 14) for i in range(3)]
    raw = read_words(openocd, reqs)

    def words(addr):
        return raw.get(addr, [])

    snap = {"probe": {}, "scalar": {}, "bus": []}
    for p in PROBES:
        w = words(sym[p])
        snap["probe"][p] = w[:4] if len(w) >= 4 else [0, 0, 0, 0]
    for s in SCALARS:
        w = words(sym[s])
        snap["scalar"][s] = w[0] if w else 0
    names = []
    nw = words(sym[TASK_NAME])
    for i in range(12):
        chunk = b"".join(x.to_bytes(4, "little") for x in nw[i * 3:i * 3 + 3])[:12]
        names.append(chunk.split(b"\x00")[0].decode("ascii", errors="replace"))
    snap["names"] = names
    snap["runtime"] = words(sym[TASK_RT])[:12]
    snap["bus"] = [words(bus_base + i * CAN_BUS_STRIDE)[:14] for i in range(3)]
    return snap


def d(now: int, before: int) -> int:
    return (now - before) & 0xFFFFFFFF


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--openocd", default=DEFAULT_OPENOCD)
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()

    sym = symbols(args.elf)
    a = gather(args.openocd, sym)
    t0 = time.time()
    time.sleep(args.seconds)
    b = gather(args.openocd, sym)
    win = time.time() - t0
    print(f"=== 采样窗口 {win:.1f}s ===")

    # 两次快照之间任务顺序可能变化, 必须按名字配对(与 lcd_task.c 里的做法一致)
    now_rt = dict(zip(b["names"], b["runtime"]))
    old_rt = dict(zip(a["names"], a["runtime"]))
    deltas = {n: d(now_rt[n], old_rt[n]) for n in now_rt if n in old_rt}
    total = sum(deltas.values())
    print("[任务占用率] (千分比, 1000 = 100%)")
    for name, share in sorted(((n, r * 1000.0 / total if total else 0.0)
                               for n, r in deltas.items() if n), key=lambda x: -x[1]):
        print(f"  {name:<12} {share:7.1f}")

    print("[分段耗时] avg=窗口增量/次数, max=绝对最大值, 单位 us")
    for p in PROBES:
        _, mx, _, _ = b["probe"][p]
        _, _, tot, calls = [d(x, y) for x, y in zip(b["probe"][p], a["probe"][p])]
        avg = tot / calls if calls else 0.0
        print(f"  {p:<16} avg={avg:9.1f}  max={mx:>8}  calls={calls:<9} ({calls / win:7.0f}/s)")

    print("[CAN]")
    sa, s = a["scalar"], b["scalar"]
    print(f"  累计丢弃={s['can_prof_tx_full']} (本窗口 +{d(s['can_prof_tx_full'], sa['can_prof_tx_full'])})  "
          f"自旋(应为0)={s['can_prof_tx_spins']}  rx_frames+={d(s['can_prof_rx_frames'], sa['can_prof_rx_frames'])}")
    for i, label in enumerate(("CAN1", "CAN2", "CAN3")):
        now, old = b["bus"][i], a["bus"][i]
        if len(now) < 13:
            continue
        # 字段顺序: 0=handle 1=tx_ok 2=tx_drop 3=tx_error 4=tx_done 5=tfe事件
        #          6=busoff 7=busoff_recover 8=rx_lost 9=rx_frames
        #          10=字节打包[TEC | REC<<8 | status<<16] 11=log_ms 12=log_cnt
        print(f"  {label}: tx_ok+={d(now[1], old[1])} tx_drop+={d(now[2], old[2])} "
              f"tx_err+={d(now[3], old[3])} tx_done+={d(now[4], old[4])} "
              f"busoff=+{d(now[6], old[6])} recover=+{d(now[7], old[7])} "
              f"rx_lost={d(now[8], old[8])} TEC={now[10] & 0xFF} REC={(now[10] >> 8) & 0xFF}")

    print("[LCD]")
    print(f"  update_last={s['lcd_prof_update_last_us']}us update_max={s['lcd_prof_update_max_us']}us "
          f"refreshes+={d(s['lcd_prof_cycles'], sa['lcd_prof_cycles'])} "
          f"({d(s['lcd_prof_cycles'], sa['lcd_prof_cycles']) / win:.1f}/s)")
    print(f"  init={s['lcd_prof_init_us']}us clear={s['lcd_prof_clear_us']}us "
          f"last_row={s['lcd_prof_row_bytes']}B/{s['lcd_prof_row_us']}us")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
