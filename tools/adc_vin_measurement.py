#!/usr/bin/env python3
"""MC02-0037 电压测量：capture 只读运行态 ADC；report 汇总 CSV、误差表和图。

先备份、烧录并 verify 对应 ELF，再在会话 JSON 登记身份和仪表依据。工具只核对
所给身份记录与文件，不代替 Flash verify、物理接线或仪表校准。缺少依据仍可记录
探索数据，但 report 不会给出合格结论。原始逐次读取保存在 CSV 同名的 JSONL 中。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import sys
import time

import probe_snapshot_live as probe

POINTS = (12.0, 12.5, 15.0, 18.0, 21.0, 24.0)
REBOOT_POINTS = (12.0, 18.0, 24.0)
PHASES = ("baseline", "up", "down", "reboot")
PHASE_POINTS = {
    "baseline": (12.5,),
    "up": POINTS,
    "down": POINTS,
    "reboot": REBOOT_POINTS,
}
PREFIX = ("point_id", "phase", "setpoint_v", "meter_v", "current_a", "reference_v",
          "elf_sha256", "session_sha256")
COLUMNS = PREFIX + tuple(probe.ADC_SNAPSHOT_FIELDS) + ("adc_version_after", "adc_volts")


def finite(value, name: str, minimum: float = 0.0, maximum: float = 100.0) -> float:
    """拒绝非有限值、布尔值和超出允许范围的数据。"""
    if isinstance(value, bool):
        raise ValueError(f"{name}: boolean is not a measurement")
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(f"{name}: outside {minimum}..{maximum}")
    return number


def sha256(path: Path) -> str:
    """记录文件内容身份，文件不存在时由调用方报告错误。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_session(path: Path) -> dict:
    """读取版本为 1 的登记表；未知仪表参数使用 null，禁止代填精度。"""
    session = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("schema") != 1:
        raise ValueError("unsupported session schema")
    for key in ("board", "meter", "power", "firmware", "budget"):
        if not isinstance(session.get(key), dict):
            raise ValueError(f"session requires {key} object")
    return session


def validate_identity(session: dict, elf: Path) -> str:
    """核对已登记的烧录身份；不把格式编号检查称为整片固件校验。"""
    firmware = session["firmware"]
    digest = sha256(elf)
    if firmware.get("flash_verified") is not True or firmware.get("elf_sha256") != digest:
        raise ValueError("session must record the verified ELF SHA-256")
    if not re.fullmatch(r"[0-9a-f]{40}", firmware.get("source_sha") or ""):
        raise ValueError("session requires the exact firmware source SHA")
    return digest


def collect(read, count: int, interval: float, log) -> list[dict]:
    """采够指定份数；最多尝试三倍次数，只重试未发布、跨更新和重复记录。"""
    if not 1 <= count <= 60:
        raise ValueError("count must be 1..60")
    finite(interval, "interval", 0.0, 2.0)
    accepted: list[dict] = []
    previous = None
    for attempt in range(count * 3):
        if attempt:
            time.sleep(interval)
        try:
            raw = read()
        except (OSError, probe.ProbeIOError) as error:
            log({"attempt": attempt + 1, "rejected": f"read failed: {error}"})
            raise
        try:
            record = probe.decode_adc(raw)
        except probe.ProbeIOError as error:
            log({"attempt": attempt + 1, "raw": raw, "rejected": str(error)})
            message = str(error)
            if "changed during reading" in message or "not been published" in message:
                continue
            raise
        if not record["adc_usable"] or record["iwdg_reset_flag"]:
            log({"attempt": attempt + 1, "raw": raw, "rejected": "ADC invalid or watchdog reset"})
            raise ValueError("ADC invalid or watchdog reset; stop this measurement")
        if previous:
            sequence_delta = (record["adc_sample_sequence"] - previous["adc_sample_sequence"]) & probe.UINT32_MAX
            if not sequence_delta:
                log({"attempt": attempt + 1, "raw": raw, "rejected": "duplicate sample"})
                continue
            if sequence_delta >= 0x80000000 or probe.classify_change(previous, record) != "running":
                log({"attempt": attempt + 1, "raw": raw, "rejected": "target reset or sequence reversed"})
                raise ValueError("target reset or ADC sequence reversed")
        log({"attempt": attempt + 1, "raw": raw, "accepted": True})
        accepted.append(record)
        previous = record
        if len(accepted) == count:
            return accepted
    raise ValueError(f"only {len(accepted)}/{count} independent ADC records obtained")


def capture(args) -> None:
    """保留原始证据，再写完整测点 CSV；失败时保留 JSONL，不生成成功 CSV。"""
    session = load_session(args.session)
    elf_digest = validate_identity(session, args.elf)
    probe._validate_loopback_host(args.host)
    probe._validate_port(args.telnet_port)
    if not 30 <= args.samples <= 60:
        raise ValueError("samples must be 30..60")
    finite(args.interval, "interval", 0.25, 2.0)
    finite(args.timeout, "timeout", 0.1, 5.0)
    finite(args.settle, "settle", 5.0, 60.0)
    meter_v = finite(args.meter_v, "meter_v", 0.001, 30.0)
    finite(args.setpoint, "setpoint", 12.0, 24.0)
    if args.current_a is not None:
        finite(args.current_a, "current_a", 0.0, 2.0)
    if args.reference_v is not None:
        finite(args.reference_v, "reference_v", 0.001, 5.0)
    if args.setpoint not in PHASE_POINTS[args.phase]:
        raise ValueError(f"setpoint is not a required {args.phase} test point")
    if abs(meter_v - args.setpoint) > 0.5:
        raise ValueError("DMM reading is too far from the selected test point")
    if args.output.suffix.lower() != ".csv":
        raise ValueError("output must use a .csv suffix")
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", args.point):
        raise ValueError("point must use 1..64 ASCII letters, digits, _, . or -")
    raw_path = args.output.with_suffix(".jsonl")
    session_path = args.output.with_suffix(".session.json")
    if any(p.exists() for p in (args.output, raw_path, session_path)):
        raise ValueError("output already exists; use a new point filename")
    addresses = probe.resolve_symbol_addresses(str(args.gdb), str(args.elf), args.timeout, probe.ADC_SNAPSHOT_FIELDS)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # 保存本点的登记表副本，后续改动会话文件也不会改写旧测量的仪表依据。
    session_path.write_bytes(args.session.read_bytes())
    time.sleep(args.settle)
    with raw_path.open("x", encoding="utf-8") as stream:
        def log(row):
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
        rows = collect(lambda: probe.read_snapshot(addresses, args.host, args.telnet_port,
                       args.timeout, probe.ADC_SNAPSHOT_FIELDS), args.samples, args.interval, log)
    metadata = dict(zip(PREFIX, (args.point, args.phase, args.setpoint, args.meter_v,
                    args.current_a if args.current_a is not None else "",
                    args.reference_v if args.reference_v is not None else "",
                    elf_digest, sha256(session_path))))
    with args.output.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows({**row, **metadata} for row in rows)
    print(json.dumps({"point": args.point, "records": len(rows), "csv": str(args.output)}))


def budget_limit(session: dict, measured_v: float, config: dict,
                 measured_reference_v: float | None = None) -> tuple[float | None, float | None, list[str]]:
    """计算电路误差上界和仪表误差；资料不足返回原因，绝不拿典型值代填最大值。"""
    reasons = []
    meter, board, budget = (session[k] for k in ("meter", "board", "budget"))
    for key, value in (("meter calibration record", meter.get("calibration_record")),
                       ("meter accuracy source", meter.get("accuracy_source")),
                       ("ADC maximum error source", budget.get("source"))):
        if not isinstance(value, str) or not value.strip():
            reasons.append(f"missing {key}")
    for key, value in (("schematic match", board.get("schematic_match_confirmed")),
                       ("applicable ADC maximum specification", budget.get("adc_applicable")),
                       ("resistor specification", budget.get("resistor_spec_confirmed"))):
        if value is not True:
            reasons.append(f"unconfirmed {key}")
    if board.get("verified_range_v") != [12, 24]:
        reasons.append("12..24V board range not verified")
    try:
        meter_error = (measured_v * finite(meter["accuracy_percent"], "meter percent", 0, 10) / 100
                       + finite(meter["accuracy_digits"], "meter digits", 0, 100)
                       * finite(meter["resolution_v"], "meter resolution", 1e-9, 1))
        ref_min = finite(budget["reference_min_v"], "reference minimum", 0.001, 5)
        ref_max = finite(budget["reference_max_v"], "reference maximum", ref_min, 5)
        top = finite(budget["top_ohm"], "top resistor", 1, 1e7)
        bottom = finite(budget["bottom_ohm"], "bottom resistor", 1, 1e7)
        top_tol = finite(budget["top_tolerance"], "top tolerance", 0, 0.1)
        bottom_tol = finite(budget["bottom_tolerance"], "bottom tolerance", 0, 0.1)
        adc_counts = finite(budget["adc_max_counts"], "ADC maximum counts", 0, 65535)
    except (KeyError, TypeError, ValueError) as error:
        reasons.append(f"incomplete error budget: {error}")
        return None, None, reasons
    if measured_reference_v is not None and not ref_min <= measured_reference_v <= ref_max:
        reasons.append("measured ADC reference is outside the declared bounds")
    cfg_reference = config["adc_vref_mv"] / 1000
    cfg_divider = config["adc_divider_milli"] / 1000
    divider_min = 1 + top * (1 - top_tol) / (bottom * (1 + bottom_tol))
    divider_max = 1 + top * (1 + top_tol) / (bottom * (1 - bottom_tol))
    gain_min = cfg_reference * cfg_divider / (ref_max * divider_max)
    gain_max = cfg_reference * cfg_divider / (ref_min * divider_min)
    # ADC 最大误差使用一项总误差；另计整批平均向下取整不足 1 个计数及浮点运算余量。
    limit = ((measured_v + meter_error) * max(abs(gain_min - 1), abs(gain_max - 1))
             + (adc_counts + 1) * cfg_reference * cfg_divider / config["adc_full_scale"]
             + 4 * 2**-23 * cfg_reference * cfg_divider)
    return limit, meter_error, reasons


def summarize(paths: list[Path]) -> tuple[list[dict], list[str]]:
    """拒绝无效、重复、改错身份或内容不完整的 CSV；返回逐点统计和未完成条件。"""
    stats, reasons, seen = [], [], set()
    common_elf = None
    common_session_digest = None
    common_config = None
    for path in paths:
        session_path = path.with_suffix(".session.json")
        session = load_session(session_path)
        with path.open(encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != list(COLUMNS):
                raise ValueError(f"{path}: CSV fields mismatch")
            rows = list(reader)
        if len(rows) < 30 or len(rows) > 60:
            raise ValueError(f"{path}: expected 30..60 rows")
        first = rows[0]
        point_key = (first["phase"], finite(first["setpoint_v"], "setpoint", 12, 24))
        if (first["phase"] not in PHASES or point_key[1] not in PHASE_POINTS[first["phase"]]
                or point_key in seen):
            raise ValueError("duplicate or invalid measurement point")
        seen.add(point_key)
        if common_elf is None:
            common_elf = first["elf_sha256"]
            common_session_digest = first["session_sha256"]
        if (first["elf_sha256"] != common_elf or first["session_sha256"] != sha256(session_path)
                or first["session_sha256"] != common_session_digest
                or first["elf_sha256"] != session["firmware"].get("elf_sha256")
                or session["firmware"].get("flash_verified") is not True
                or not re.fullmatch(r"[0-9a-f]{40}", session["firmware"].get("source_sha") or "")):
            raise ValueError("session/firmware identity mismatch")
        readings, previous, config = [], None, None
        for row in rows:
            if any(row[k] != first[k] for k in PREFIX):
                raise ValueError("metadata changes within one point")
            raw = {k: int(row[k]) for k in probe.ADC_SNAPSHOT_FIELDS}
            raw["adc_version_after"] = int(row["adc_version_after"])
            decoded = probe.decode_adc(raw)
            if not decoded["adc_usable"] or decoded["iwdg_reset_flag"]:
                raise ValueError("CSV contains invalid ADC data")
            if abs(finite(row["adc_volts"], "ADC volts", 0, 500) - decoded["adc_volts"]) > 1e-7:
                raise ValueError("CSV voltage does not match raw bits")
            this_config = {k: raw[k] for k in ("adc_vref_mv", "adc_divider_milli", "adc_full_scale")}
            if config is not None and config != this_config:
                raise ValueError("ADC configuration changes within one point")
            config = this_config
            if common_config is None:
                common_config = this_config
            elif common_config != this_config:
                raise ValueError("ADC configuration changes between measurement points")
            if previous:
                delta = (raw["adc_sample_sequence"] - previous["adc_sample_sequence"]) & probe.UINT32_MAX
                if not 0 < delta < 0x80000000 or probe.classify_change(previous, raw) != "running":
                    raise ValueError("duplicate or reversed ADC samples in CSV")
            previous = raw
            readings.append(decoded["adc_volts"])
        meter_v = finite(first["meter_v"], "meter volts", 0.001, 30)
        if abs(meter_v - point_key[1]) > 0.5:
            raise ValueError("DMM reading is too far from the recorded test point")
        point_reasons = []
        reference_v = None
        for key, maximum in (("current_a", 2), ("reference_v", 5)):
            if not first[key]:
                point_reasons.append(f"missing {key}")
            else:
                try:
                    measured = finite(first[key], key, 0, maximum)
                    if key == "reference_v":
                        reference_v = measured
                except ValueError as error:
                    point_reasons.append(str(error))
        limit, meter_error, missing = budget_limit(session, meter_v, config, reference_v)
        point_reasons.extend(missing)
        reasons.extend(f"{first['point_id']}: {reason}" for reason in point_reasons)
        # 用最大单次偏差加仪表误差作保守比较，平均值不能掩盖单次超差。
        worst = max(abs(v - meter_v) for v in readings)
        qualified = not point_reasons and limit is not None and meter_error is not None
        result = "not-qualified" if not qualified else "pass" if worst + meter_error <= limit else "fail"
        stats.append({"point_id": first["point_id"], "phase": point_key[0], "setpoint_v": point_key[1],
                      "records": len(rows), "meter_v": meter_v, "mean_v": statistics.mean(readings),
                      "min_v": min(readings), "max_v": max(readings), "stdev_v": statistics.pstdev(readings),
                      "mean_error_v": statistics.mean(readings) - meter_v,
                      "relative_error_percent": (statistics.mean(readings) / meter_v - 1) * 100,
                      "worst_error_v": worst, "circuit_limit_v": limit, "meter_error_v": meter_error,
                      "measurement_error_allowance_v": (max(0.0, limit - meter_error)
                                                         if limit is not None and meter_error is not None else None),
                      "qualification_reasons": "; ".join(point_reasons),
                      "result": result})
    required = {("baseline", 12.5)}
    required |= {(phase, point) for phase in ("up", "down") for point in POINTS}
    required |= {("reboot", point) for point in REBOOT_POINTS}
    missing_points = required - seen
    if missing_points:
        reasons.append("missing measurement points: " + str(sorted(missing_points)))
    return stats, sorted(set(reasons))


def report(args) -> str:
    """生成可独立查看的统计和标准绘图库图像；不自行关闭 TODO。"""
    stats, reasons = summarize(args.inputs)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "summary.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(stats[0]))
        writer.writeheader()
        writer.writerows(stats)
    result = ("fail" if any(s["result"] == "fail" for s in stats) else
              "not-qualified" if reasons or any(s["result"] == "not-qualified" for s in stats) else "pass")
    (args.output / "summary.json").write_text(json.dumps({"result": result, "reasons": reasons,
        "points": stats}, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for phase in PHASES:
        data = sorted((s for s in stats if s["phase"] == phase), key=lambda s: s["setpoint_v"])
        if data:
            xs = [s["meter_v"] for s in data]
            errors = [s["mean_error_v"] for s in data]
            deviations = [s["stdev_v"] for s in data]
            ax.errorbar(xs, errors, yerr=deviations, fmt="o-", capsize=3, label=phase)
            if all(s["measurement_error_allowance_v"] is not None for s in data):
                limits = [s["measurement_error_allowance_v"] for s in data]
                ax.fill_between(xs, [-v for v in limits], limits, alpha=0.08)
    ax.axhline(0, color="black", linewidth=0.7)
    ax.set(xlabel="DMM input (V)", ylabel="Firmware minus DMM (V)", title=f"MC02 VIN: {result}")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output / "vin_error.png", dpi=180)
    fig.savefig(args.output / "vin_error.svg")
    plt.close(fig)
    print(json.dumps({"result": result, "missing": reasons}, ensure_ascii=False))
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture", help="已 verify 的固件上只读采集一个稳定测点")
    capture_parser.add_argument("--session", required=True, type=Path)
    capture_parser.add_argument("--elf", required=True, type=Path)
    capture_parser.add_argument("--gdb", default="arm-none-eabi-gdb")
    capture_parser.add_argument("--host", default="127.0.0.1")
    capture_parser.add_argument("--telnet-port", default=4444, type=int)
    capture_parser.add_argument("--output", required=True, type=Path)
    capture_parser.add_argument("--point", required=True)
    capture_parser.add_argument("--phase", required=True, choices=PHASES)
    capture_parser.add_argument("--setpoint", required=True, type=float)
    capture_parser.add_argument("--meter-v", required=True, type=float)
    capture_parser.add_argument("--current-a", type=float)
    capture_parser.add_argument("--reference-v", type=float)
    capture_parser.add_argument("--samples", default=30, type=int)
    capture_parser.add_argument("--interval", default=0.5, type=float)
    capture_parser.add_argument("--settle", default=5.0, type=float)
    capture_parser.add_argument("--timeout", default=5.0, type=float)
    report_parser = commands.add_parser("report", help="汇总完整 CSV；缺少精度依据只输出探索统计")
    report_parser.add_argument("inputs", nargs="+", type=Path)
    report_parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "capture":
            capture(args)
            return 0
        result = report(args)
    except (ValueError, TypeError, KeyError, OSError, probe.ProbeIOError, ImportError) as error:
        print(f"measurement_error: {error}", file=sys.stderr)
        return 2
    return 0 if result == "pass" else 1 if result == "fail" else 3


if __name__ == "__main__":
    raise SystemExit(main())
