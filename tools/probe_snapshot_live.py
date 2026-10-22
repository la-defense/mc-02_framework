#!/usr/bin/env python3
"""Take bounded RAM snapshots without attaching GDB to the running target.

GDB is used offline to resolve ELF symbol addresses only. Live reads use the
loopback OpenOCD Telnet port configured by tools/openocd_running_inspection.cfg
and are restricted to target-state queries and single-byte/word RAM reads.

ADC 模式另检查记录前后的版本和格式编号，防止把更新中的字段拼在一起。
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import re
import socket
import math
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ELF = ROOT / "build" / "bench-safe" / "Basic_Framework_MC02_bench_safe.elf"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_TELNET_PORT = 4444
COUNTERS = ("vision_rx_count", "vision_tx_count", "vision_crc_error_count")
# STM32H723VGTx_FLASH.ld defines RAM at 0x24000000 with length 128 KiB.
# Live inspection is intentionally limited to this linker RAM region.
SNAPSHOT_RAM_START = 0x24000000
SNAPSHOT_RAM_END = 0x24020000  # exclusive
SYMBOL_VALUE_PATTERN = re.compile(r"^(?P<key>[a-z_]+)=0x(?P<value>[0-9a-fA-F]+)$")
MEMORY_VALUE_PATTERN = re.compile(
    r"^\s*0x(?P<address>[0-9a-fA-F]+):\s*(?:0x)?(?P<value>[0-9a-fA-F]+)\b"
)
MEMORY_COMMAND_PATTERN = re.compile(
    r"(?P<command>mdb|mdw) 0x(?P<address>[0-9a-f]{1,8}) (?P<count>[1-9][0-9]*)\Z"
)
MEMORY_WORDS_PATTERN = re.compile(r"^\s*0x(?P<address>[0-9a-fA-F]{1,8}):(?P<values>.*)$")
ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
MAX_RESPONSE_BYTES = 65536
UINT32_MAX = 0xFFFFFFFF


@dataclass(frozen=True)
class SnapshotField:
    expression: str
    command: str
    width_bits: int
    count: int = 1

    @property
    def alignment_bytes(self) -> int:
        return self.width_bits // 8


SNAPSHOT_FIELDS = {
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    "vision_mode": SnapshotField("recv_data.mode", "mdb", 8),
    "vision_rx_count": SnapshotField("vision_rx_count", "mdw", 32),
    "vision_tx_count": SnapshotField("vision_tx_count", "mdw", 32),
    "vision_crc_error_count": SnapshotField("vision_crc_error_count", "mdw", 32),
    "iwdg_reset_flag": SnapshotField("iwdg_reset_flag", "mdb", 8),
}

IMU_SNAPSHOT_FIELDS = {
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    "init_stage": SnapshotField("bmi088_init_stage", "mdw", 32),
    "init_failure_stage": SnapshotField("bmi088_init_failure_stage", "mdw", 32),
    "init_error": SnapshotField("bmi088_init_error", "mdw", 32),
    "init_retry_count": SnapshotField("bmi088_init_retry_count", "mdw", 32),
    "init_acc_status": SnapshotField("bmi088_init_acc_status", "mdw", 32),
    "init_gyro_status": SnapshotField("bmi088_init_gyro_status", "mdw", 32),
    "init_acc_chip_id": SnapshotField("bmi088_init_acc_chip_id", "mdw", 32),
    "init_gyro_chip_id": SnapshotField("bmi088_init_gyro_chip_id", "mdw", 32),
    "imu_sample_sequence": SnapshotField("bmi088_sample_sequence_diag", "mdw", 32),
    "imu_sample_timestamp_ms": SnapshotField("bmi088_sample_timestamp_ms_diag", "mdw", 32),
    "imu_sample_valid": SnapshotField("imu_sample_valid_diag", "mdw", 32),
    "imu_heater_duty": SnapshotField("imu_heater_duty_diag", "mdw", 32),
}

# 每个地址都从给定 ELF 离线解析；不接受运行时提供的任意地址。
ADC_SNAPSHOT_FIELDS = {
    **{f"adc_{key}": SnapshotField(f"bsp_adc_diagnostic.{key}", "mdw", 32)
       for key in ("version", "layout", "captured_ms", "status", "raw_vcc_in", "raw_key",
                   "sample_sequence", "sample_timestamp_ms", "error_flags", "hal_error_code",
                   "voltage_config_valid", "volts_bits", "vref_mv", "full_scale", "divider_milli")},
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    "iwdg_reset_flag": SnapshotField("iwdg_reset_flag", "mdb", 8),
}
# REF-03只允许读取此固定诊断对象；布局在固件中由_Static_assert核对。
REFEREE_DIAGNOSTIC_WORD_COUNT = 456
REFEREE_SNAPSHOT_FIELDS = {
    "ref_magic": SnapshotField("referee_bench_diag.magic", "mdw", 32),
    "ref_count": SnapshotField("referee_bench_diag.write_count", "mdw", 32),
}

# 固定读取 RTOS 任务快照和已登记的 DWT 最大耗时；数组块长度来自固件编译常量。
RTOS_SNAPSHOT_FIELDS = {
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    "stats_layout": SnapshotField("lcd_stats_layout", "mdw", 32),
    "stats_generation_before": SnapshotField("lcd_stats_generation", "mdw", 32),
    "stats_count": SnapshotField("lcd_stats_count", "mdb", 8),
    "stats_task_total": SnapshotField("lcd_stats_task_total", "mdw", 32),
    "stats_truncated": SnapshotField("lcd_stats_truncated", "mdb", 8),
    "stats_names_words": SnapshotField("((uint32_t *)&lcd_stats_name[0][0])[0]", "mdw", 32, 36),
    "stats_runtime": SnapshotField("((uint32_t *)&lcd_stats_runtime[0])[0]", "mdw", 32, 12),
    "stats_priority_words": SnapshotField("((uint32_t *)&lcd_stats_prio[0])[0]", "mdw", 32, 3),
    "stats_stack_words": SnapshotField("((uint32_t *)&lcd_stats_stack[0])[0]", "mdw", 32, 6),
    "motor_task_max_us": SnapshotField("r25_task_motor.max_us", "mdw", 32),
    "motor_task_calls": SnapshotField("r25_task_motor.calls", "mdw", 32),
    "ins_task_max_us": SnapshotField("r25_task_ins.max_us", "mdw", 32),
    "ins_task_calls": SnapshotField("r25_task_ins.calls", "mdw", 32),
    "daemon_task_max_us": SnapshotField("r25_task_daemon.max_us", "mdw", 32),
    "daemon_task_calls": SnapshotField("r25_task_daemon.calls", "mdw", 32),
    "robot_task_max_us": SnapshotField("r25_task_robot.max_us", "mdw", 32),
    "robot_task_calls": SnapshotField("r25_task_robot.calls", "mdw", 32),
    "uart_tx_task_max_us": SnapshotField("usart_tx_task_probe.max_us", "mdw", 32),
    "uart_tx_task_calls": SnapshotField("usart_tx_task_probe.calls", "mdw", 32),
    "motor_control_max_us": SnapshotField("motor_prof_all.max_us", "mdw", 32),
    "ins_compute_max_us": SnapshotField("ins_prof_all.max_us", "mdw", 32),
    "can_rx_dispatch_max_us": SnapshotField("can_prof_rx_dispatch.max_us", "mdw", 32),
    "can_rx_dispatch_calls": SnapshotField("can_prof_rx_dispatch.calls", "mdw", 32),
    "can_rx_drop_total": SnapshotField("can_rxq_drop_total", "mdw", 32),
    "can_rx_peak_max": SnapshotField("can_rxq_peak_max", "mdw", 32),
    "lcd_sample_max_us": SnapshotField("lcd_prof_sample_max_us", "mdw", 32),
    "lcd_update_max_us": SnapshotField("lcd_prof_update_max_us", "mdw", 32),
    "lcd_draw_max_us": SnapshotField("lcd_prof_draw_max_us", "mdw", 32),
    "stats_generation_after": SnapshotField("lcd_stats_generation", "mdw", 32),
}
RTOS_TASK_CAPACITY = 12
RTOS_TASK_STATS_LAYOUT = 0x52543102
RTOS_TASK_STACK_WORDS = {
    "defaultTask": 128,
    "IDLE": 128,
    "Tmr Svc": 256,
    "instask": 1024,
    "motortask": 512,
    "uarttxtask": 512,
    "logtask": 1024,
    "daemontask": 512,
    "robottask": 1024,
    "uitask": 512,
    "lcdtask": 1024,
}


def decode_rtos_snapshot(snapshot: dict[str, int | list[int]]) -> dict:
    """Validate the published task table and unpack its fixed-width arrays."""
    if set(snapshot) != set(RTOS_SNAPSHOT_FIELDS):
        raise ProbeIOError("RTOS record has missing or unexpected fields")
    for key, field in RTOS_SNAPSHOT_FIELDS.items():
        value = snapshot[key]
        values = value if field.count > 1 else [value]
        if not isinstance(values, list) or len(values) != field.count:
            raise ProbeIOError(f"RTOS field {key} has an invalid word count")
        if any(type(word) is not int or not 0 <= word <= UINT32_MAX for word in values):
            raise ProbeIOError(f"RTOS field {key} contains an invalid unsigned word")

    before = snapshot["stats_generation_before"]
    after = snapshot["stats_generation_after"]
    if snapshot["stats_layout"] != RTOS_TASK_STATS_LAYOUT:
        raise ProbeIOError("RTOS task table layout mismatch: use the verified firmware ELF")
    if before != after or before & 1:
        raise ProbeIOError("RTOS task table changed during reading or is being published")
    count = snapshot["stats_count"]
    total = snapshot["stats_task_total"]
    truncated = snapshot["stats_truncated"]
    if count > RTOS_TASK_CAPACITY or total < count or truncated not in (0, 1):
        raise ProbeIOError("RTOS task counts are inconsistent")
    if bool(truncated) != (total > count):
        raise ProbeIOError("RTOS truncation flag disagrees with the task counts")

    name_bytes = b"".join(word.to_bytes(4, "little") for word in snapshot["stats_names_words"])
    priority_bytes = b"".join(word.to_bytes(4, "little") for word in snapshot["stats_priority_words"])
    stack_bytes = b"".join(word.to_bytes(4, "little") for word in snapshot["stats_stack_words"])
    stacks = struct.unpack("<12H", stack_bytes)
    tasks = []
    for index in range(count):
        raw_name = name_bytes[index * 12:(index + 1) * 12]
        name = raw_name.split(b"\0", 1)[0].decode("ascii", errors="replace")
        stack_words = stacks[index]
        stack_capacity_words = RTOS_TASK_STACK_WORDS.get(name)
        if stack_capacity_words is not None and stack_words > stack_capacity_words:
            raise ProbeIOError(f"task {name} reports more free stack than its configured depth")
        tasks.append({
            "name": name,
            "runtime_counter": snapshot["stats_runtime"][index],
            "priority": priority_bytes[index],
            "stack_high_water_words": stack_words,
            "stack_high_water_bytes": stack_words * 4,
            "stack_capacity_words": stack_capacity_words,
            "stack_margin_permille": (
                (stack_words * 1000) // stack_capacity_words
                if stack_capacity_words is not None else None
            ),
        })
    return {
        **{key: value for key, value in snapshot.items()
           if key != "stats_layout" and not key.startswith("stats_")},
        "snapshot_generation": after,
        "task_count": count,
        "task_total": total,
        "task_table_capacity": RTOS_TASK_CAPACITY,
        "task_table_truncated": bool(truncated),
        "tasks": tasks,
    }
ADC_DIAGNOSTIC_LAYOUT = 0xADC10001
ADC_DIAGNOSTIC_WORD_COUNT = 15


def decode_adc(snapshot: dict[str, int]) -> dict:
    """验证一份完整的 ADC 诊断记录；失败抛出 ProbeIOError，保留调用方原数据。"""
    expected = set(ADC_SNAPSHOT_FIELDS) | {"adc_version_after"}
    if set(snapshot) != expected:
        raise ProbeIOError("ADC record has missing or unexpected fields")
    if any(type(value) is not int or not 0 <= value <= UINT32_MAX for value in snapshot.values()):
        raise ProbeIOError("ADC record contains an invalid unsigned word")
    version = snapshot["adc_version"]
    if not version or version & 1 or version != snapshot["adc_version_after"]:
        raise ProbeIOError("ADC record changed during reading or has not been published")
    if snapshot["adc_layout"] != ADC_DIAGNOSTIC_LAYOUT:
        raise ProbeIOError("ADC layout mismatch: use the verified firmware ELF")
    if snapshot["adc_status"] not in range(4):
        raise ProbeIOError("invalid ADC status")
    if snapshot["iwdg_reset_flag"] not in (0, 1):
        raise ProbeIOError("invalid IWDG reset flag")
    if snapshot["adc_raw_vcc_in"] > 65535 or snapshot["adc_raw_key"] > 65535:
        raise ProbeIOError("ADC raw value is outside the 16-bit range")
    if snapshot["adc_voltage_config_valid"] not in (0, 1):
        raise ProbeIOError("invalid ADC configuration flag")
    # 诊断更新比 ADC 慢；这里分别检查诊断是否停更、采样在记录时是否新鲜。
    diagnostic_age = (snapshot["tick_ms"] - snapshot["adc_captured_ms"]) & UINT32_MAX
    sample_age = (snapshot["adc_captured_ms"] - snapshot["adc_sample_timestamp_ms"]) & UINT32_MAX
    if diagnostic_age > 1000:
        raise ProbeIOError("ADC diagnostic record is older than 1000ms")
    volts = struct.unpack("<f", struct.pack("<I", snapshot["adc_volts_bits"]))[0]
    if not math.isfinite(volts):
        raise ProbeIOError("ADC voltage is not finite")
    if snapshot["adc_voltage_config_valid"]:
        if not (0 < snapshot["adc_vref_mv"] <= 5000 and snapshot["adc_full_scale"] == 65535
                and 0 < snapshot["adc_divider_milli"] <= 100000):
            raise ProbeIOError("ADC configuration fields do not match the valid flag")
        expected_volts = (snapshot["adc_raw_vcc_in"] * snapshot["adc_vref_mv"]
                          * snapshot["adc_divider_milli"] / (65535 * 1_000_000))
        if abs(volts - expected_volts) > 0.00002:
            raise ProbeIOError("ADC voltage and raw/configuration fields disagree")
    elif volts != 0.0:
        raise ProbeIOError("invalid ADC configuration has a nonzero voltage")
    if snapshot["adc_status"] == 1 and (sample_age > 20 or snapshot["adc_error_flags"]):
        raise ProbeIOError("ADC valid status disagrees with its age or errors")
    return {**snapshot, "adc_volts": volts, "adc_sample_age_ms": sample_age,
            "adc_diagnostic_age_ms": diagnostic_age,
            "adc_usable": snapshot["adc_status"] == 1 and snapshot["adc_voltage_config_valid"] == 1}

INIT_SNAPSHOT_FIELDS = {
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    "init_failed": SnapshotField("init_diagnostic_snapshot.failed", "mdb", 8),
    "init_stage": SnapshotField("init_diagnostic_snapshot.stage", "mdb", 8),
    "init_module": SnapshotField("init_diagnostic_snapshot.module", "mdb", 8),
    "init_reason": SnapshotField("init_diagnostic_snapshot.reason", "mdb", 8),
    "init_resource_id_lo": SnapshotField(
        "((unsigned char *)&(init_diagnostic_snapshot.resource_id))[0]", "mdb", 8
    ),
    "init_resource_id_hi": SnapshotField(
        "((unsigned char *)&(init_diagnostic_snapshot.resource_id))[1]", "mdb", 8
    ),
    "init_failure_count": SnapshotField("init_diagnostic_snapshot.failure_count", "mdw", 32),
    "init_heartbeat": SnapshotField("init_diagnostic_snapshot.heartbeat", "mdw", 32),
}

RC_SNAPSHOT_FIELDS = {
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    **{key: SnapshotField(f"remote_control_diag.{key}", "mdw", 32) for key in (
        "accepted_frames", "rejected_length", "rejected_fields", "last_valid_tick",
        "offline_tick", "offline_last_valid_tick", "offline_events", "has_valid_frame", "online")},
    "uart_oversize": SnapshotField("usart_rx_oversize_total", "mdw", 32),
    "switch_left": SnapshotField("rc_ctrl[0].rc.switch_left", "mdb", 8),
    "switch_right": SnapshotField("rc_ctrl[0].rc.switch_right", "mdb", 8),
    "estop_latched": SnapshotField("estop_latched", "mdb", 8),
    "enable_requested": SnapshotField("enable_requested", "mdb", 8),
    "safety_state": SnapshotField("robot_state", "mdb", 8),
    "fault_flags": SnapshotField("fault_flags", "mdw", 32),
    "iwdg_reset_flag": SnapshotField("iwdg_reset_flag", "mdb", 8),
}

# Halfword reads are deliberately excluded from the live command whitelist.
# Read the little-endian storage as two bytes; the target keeps running, so
# these diagnostic pairs are not an atomic input snapshot.
RC_INPUT_SNAPSHOT_FIELDS = {
    **RC_SNAPSHOT_FIELDS,
    # A zero-extended 32-bit diagnostic word preserves the 16-bit wire pattern.
    "raw_mouse_z": SnapshotField("remote_control_diag.raw_mouse_z", "mdw", 32),
    **{name: SnapshotField(f"remote_control_diag.{name}", "mdw", 32) for name in (
        "raw_key_bitmap", "raw_mouse_x", "raw_mouse_y", "raw_mouse_left", "raw_mouse_right",
        "pc_authorized", "pc_heartbeat", "pc_recovery_count", "pc_last_advance_tick",
        "pc_expire_tick", "pc_expirations", "pc_revoke_tick", "pc_revoke_reason")},
    **{f"{name}_{part}": SnapshotField(
        f"((unsigned char *)&({expression}))[{offset}]", "mdb", 8)
       for name, expression in (
           ("key_bitmap", "rc_ctrl[0].key[0].keys"),
           ("mouse_x", "rc_ctrl[0].mouse.x"),
           ("mouse_y", "rc_ctrl[0].mouse.y"))
       for part, offset in (("lo", 0), ("hi", 1))},
    "mouse_left": SnapshotField("rc_ctrl[0].mouse.press_l", "mdb", 8),
    "mouse_right": SnapshotField("rc_ctrl[0].mouse.press_r", "mdb", 8),
    # KEY_PRESS=0, Key_E=7, Key_F=9 in the current firmware header.
    "e_edges": SnapshotField("rc_ctrl[0].key_count[0][7]", "mdb", 8),
    "f_edges": SnapshotField("rc_ctrl[0].key_count[0][9]", "mdb", 8),
}


def decode_rc_input(raw: dict[str, int]) -> dict[str, int]:
    """Keep raw byte evidence and add interpreted keyboard/signed mouse fields."""
    result = dict(raw)
    for name in ("key_bitmap", "mouse_x", "mouse_y"):
        word = raw[f"{name}_lo"] | (raw[f"{name}_hi"] << 8)
        result[name] = word - 0x10000 if name != "key_bitmap" and word & 0x8000 else word
    if "raw_mouse_z" in raw:
        word = raw["raw_mouse_z"]
        result["mouse_z"] = word - 0x10000 if word & 0x8000 else word
    return result


class ProbeIOError(RuntimeError):
    """The OpenOCD connection, state check, or bounded memory read failed."""


def build_symbol_command(
    gdb: str, elf: str, fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS
) -> list[str]:
    """Build an offline GDB invocation; it must never connect to a target."""
    commands = ["set pagination off", "set confirm off"]
    commands.extend(
        f'printf "{key}=0x%x\\n", (unsigned int)&({field.expression})'
        for key, field in fields.items()
    )
    argv = [gdb, "--nx", "--quiet", "--batch", "--symbols", elf]
    for command in commands:
        argv.extend(("--ex", command))
    return argv


def parse_addresses(
    output: str, fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS
) -> dict[str, int]:
    addresses: dict[str, int] = {}
    for line in output.splitlines():
        match = SYMBOL_VALUE_PATTERN.fullmatch(line.strip())
        if match and match.group("key") in fields:
            addresses[match.group("key")] = int(match.group("value"), 16)

    _validate_snapshot_addresses(addresses, fields)
    return addresses


def resolve_symbol_addresses(
    gdb: str,
    elf: str,
    timeout: float,
    fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS,
) -> dict[str, int]:
    try:
        result = subprocess.run(
            build_symbol_command(gdb, elf, fields),
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise ProbeIOError(f"offline ELF symbol lookup timed out after {timeout:g}s") from error
    except OSError as error:
        raise ProbeIOError(f"could not start offline GDB: {error}") from error

    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip().splitlines()
        message = detail[-1] if detail else f"GDB exited with status {result.returncode}"
        raise ProbeIOError(f"offline ELF symbol lookup failed: {message}")
    return parse_addresses(result.stdout + "\n" + result.stderr, fields)


def _validate_loopback_host(host: str) -> None:
    try:
        address = ipaddress.ip_address(host)
    except ValueError as error:
        raise ValueError("host must be a loopback IP address") from error
    if not address.is_loopback:
        raise ValueError("host must be a loopback IP address")


def _validate_port(port: int) -> None:
    if not 1 <= port <= 65535:
        raise ValueError("Telnet port must be between 1 and 65535")


def _validated_address(address: int, alignment: int) -> int:
    if isinstance(address, bool) or not isinstance(address, int):
        raise ValueError("symbol addresses must be integers")
    if not SNAPSHOT_RAM_START <= address < SNAPSHOT_RAM_END or address > UINT32_MAX:
        raise ValueError("symbol address is outside the approved linker RAM region")
    if address % alignment:
        raise ValueError(f"symbol address 0x{address:x} is not {alignment}-byte aligned")
    return address


def _validate_snapshot_addresses(
    addresses: dict[str, int], fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS
) -> None:
    expected = set(fields)
    supplied = set(addresses)
    missing = expected - supplied
    extra = supplied - expected
    if missing or extra:
        details = []
        if missing:
            details.append(f"missing: {', '.join(sorted(missing))}")
        if extra:
            details.append(f"unexpected: {', '.join(sorted(extra))}")
        raise ProbeIOError("snapshot addresses do not match the approved fields (" + "; ".join(details) + ")")

    for key, field in fields.items():
        try:
            _validated_address(addresses[key], field.alignment_bytes)
            if not 1 <= field.count <= 64:
                raise ValueError("read length must be between 1 and 64 words")
            if field.count > 1 and field.command != "mdw":
                raise ValueError("multiword reads must use aligned 32-bit words")
            if addresses[key] + field.count * 4 > SNAPSHOT_RAM_END:
                raise ValueError("read extends beyond the approved linker RAM region")
        except ValueError as error:
            raise ProbeIOError(f"ELF symbol {key} has an invalid address: {error}") from error


def _build_read_command(
    key: str, address: int, fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS
) -> str:
    if key not in fields:
        raise ValueError(f"unsupported snapshot field: {key}")
    field = fields[key]
    validated = _validated_address(address, field.alignment_bytes)
    return f"{field.command} 0x{validated:08x} {field.count}"


def _parse_memory_value(output: str, expected_address: int, width_bits: int) -> int:
    for line in output.splitlines():
        match = MEMORY_VALUE_PATTERN.match(line)
        if not match:
            continue
        address = int(match.group("address"), 16)
        value = int(match.group("value"), 16)
        if address != expected_address:
            continue
        if value >= 1 << width_bits:
            raise ProbeIOError(f"OpenOCD returned an oversized value for 0x{address:x}")
        return value
    raise ProbeIOError(f"OpenOCD returned no value for RAM address 0x{expected_address:x}")


def _parse_memory_words(output: str, start_address: int, count: int) -> list[int]:
    """解析受限的 mdw 块读取；每行可包含多个连续的 32 位字。"""
    values: list[int] = []
    for line in output.replace("\r", "").splitlines():
        line = line.strip()
        if line.endswith(">"):
            line = line[:-1].strip()
        match = MEMORY_WORDS_PATTERN.fullmatch(line)
        if not match:
            continue
        address = int(match.group("address"), 16)
        tokens = match.group("values").split()
        if address != start_address + len(values) * 4:
            raise ProbeIOError("OpenOCD returned a non-contiguous fixed diagnostic block")
        for token in tokens:
            if token == ">":
                break
            if token.lower().startswith("0x"):
                token = token[2:]
            if not token or len(token) > 8 or not re.fullmatch(r"[0-9a-fA-F]+", token):
                raise ProbeIOError("OpenOCD returned an invalid fixed diagnostic word")
            if len(values) >= count:
                raise ProbeIOError("OpenOCD returned too many fixed diagnostic words")
            values.append(int(token, 16))
    if len(values) != count:
        raise ProbeIOError(f"OpenOCD returned {len(values)} of {count} fixed diagnostic words")
    return values


def _require_running(output: str, when: str) -> None:
    for line in output.splitlines():
        if re.search(r"\bmc02\.cpu0\b", line):
            state = line.strip().split()[-1].lower()
            if state == "running":
                return
            raise ProbeIOError(f"mc02.cpu0 is {state} {when}; snapshot aborted without resuming it")
    raise ProbeIOError(f"OpenOCD did not report mc02.cpu0 state {when}")


class OpenOCDTelnetClient:
    """Minimal bounded Telnet client with a strict OpenOCD command allowlist."""

    def __init__(
        self,
        host: str,
        port: int,
        timeout: float,
        allowed_addresses: dict[str, int],
        fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS,
    ):
        _validate_loopback_host(host)
        _validate_port(port)
        if not 0 < timeout <= 5.0:
            raise ValueError("timeout must be greater than 0 and at most 5 seconds")
        self.host = host
        self.port = port
        self.timeout = timeout
        _validate_snapshot_addresses(allowed_addresses, fields)
        self.fields = fields
        self.allowed_addresses = dict(allowed_addresses)
        self.allowed_reads = frozenset(
            (field.command, allowed_addresses[key], field.count)
            for key, field in fields.items()
        )
        self._socket: socket.socket | None = None
        self._wire = bytearray()
        self._deadline = 0.0

    def __enter__(self) -> OpenOCDTelnetClient:
        self._deadline = time.monotonic() + self.timeout
        try:
            self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
        except OSError as error:
            raise ProbeIOError(f"could not connect to local OpenOCD Telnet port: {error}") from error
        try:
            self._read_until_prompt()
        except Exception:
            self.close()
            raise
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None

    def command(self, command: str) -> str:
        if command == "targets":
            pass
        else:
            match = MEMORY_COMMAND_PATTERN.fullmatch(command)
            if match is None:
                raise ValueError("only targets and bounded ELF-resolved memory reads are allowed")
            read = (match.group("command"), int(match.group("address"), 16))
            count = int(match.group("count"))
            fixed_read = (*read, count) in self.allowed_reads
            adc_block_read = (
                self.fields is ADC_SNAPSHOT_FIELDS
                and match.group("command") == "mdw"
                and read[1] == self.allowed_addresses["adc_version"]
                and count == ADC_DIAGNOSTIC_WORD_COUNT
                and all(
                    self.allowed_addresses[f"adc_{key}"] == read[1] + index * 4
                    for index, key in enumerate((
                        "version", "layout", "captured_ms", "status", "raw_vcc_in", "raw_key",
                        "sample_sequence", "sample_timestamp_ms", "error_flags", "hal_error_code",
                        "voltage_config_valid", "volts_bits", "vref_mv", "full_scale", "divider_milli",
                    ))
                )
            )
            referee_block_read = (
                self.fields is REFEREE_SNAPSHOT_FIELDS
                and match.group("command") == "mdw"
                and read[1] == self.allowed_addresses["ref_magic"]
                and count == REFEREE_DIAGNOSTIC_WORD_COUNT
                and self.allowed_addresses["ref_count"] == read[1] + 28
                and read[1] + count * 4 <= SNAPSHOT_RAM_END
            )
            if not fixed_read and not adc_block_read and not referee_block_read:
                raise ValueError("memory reads must match an ELF-resolved symbol address and width")
        if self._socket is None:
            raise ProbeIOError("OpenOCD Telnet connection is closed")
        try:
            self._socket.sendall(command.encode("ascii") + b"\n")
        except OSError as error:
            raise ProbeIOError(f"OpenOCD Telnet write failed: {error}") from error
        return self._read_until_prompt()

    def _consume_telnet(self) -> bytes:
        output = bytearray()
        index = 0
        while index < len(self._wire):
            value = self._wire[index]
            if value != 0xFF:
                output.append(value)
                index += 1
                continue
            if index + 1 >= len(self._wire):
                break
            option = self._wire[index + 1]
            if option in (0xFB, 0xFC, 0xFD, 0xFE):
                if index + 2 >= len(self._wire):
                    break
                if self._socket is not None:
                    if option == 0xFB:  # WILL -> DONT
                        self._socket.sendall(bytes((0xFF, 0xFE, self._wire[index + 2])))
                    elif option == 0xFD:  # DO -> WONT
                        self._socket.sendall(bytes((0xFF, 0xFC, self._wire[index + 2])))
                index += 3
                continue
            if option == 0xFA:  # subnegotiation; discard through IAC SE
                terminator = self._wire.find(b"\xff\xf0", index + 2)
                if terminator < 0:
                    break
                index = terminator + 2
                continue
            index += 2
        del self._wire[:index]
        return bytes(output)

    def _read_until_prompt(self) -> str:
        if self._socket is None:
            raise ProbeIOError("OpenOCD Telnet connection is closed")
        output = bytearray()
        while True:
            remaining = self._deadline - time.monotonic()
            if remaining <= 0:
                raise ProbeIOError(f"OpenOCD Telnet read timed out after {self.timeout:g}s")
            self._socket.settimeout(remaining)
            try:
                chunk = self._socket.recv(4096)
            except socket.timeout as error:
                raise ProbeIOError(f"OpenOCD Telnet read timed out after {self.timeout:g}s") from error
            except OSError as error:
                raise ProbeIOError(f"OpenOCD Telnet read failed: {error}") from error
            if not chunk:
                raise ProbeIOError("OpenOCD closed the Telnet connection before its prompt")
            self._wire.extend(chunk)
            output.extend(self._consume_telnet())
            if len(output) > MAX_RESPONSE_BYTES:
                raise ProbeIOError("OpenOCD Telnet response exceeded the size limit")
            text = output.decode("utf-8", errors="replace")
            text = ANSI_ESCAPE_PATTERN.sub("", text)
            text = "".join(char for char in text if char in "\r\n\t" or ord(char) >= 32)
            if text.rstrip().endswith(">"):
                return text


def read_snapshot(
    addresses: dict[str, int],
    host: str,
    port: int,
    timeout: float,
    fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS,
) -> dict[str, int | list[int]]:
    _validate_snapshot_addresses(addresses, fields)

    snapshot: dict[str, int | list[int]] = {}
    try:
        with OpenOCDTelnetClient(
            host, port, timeout, allowed_addresses=addresses, fields=fields
        ) as client:
            _require_running(client.command("targets"), "before the snapshot")
            if fields is ADC_SNAPSHOT_FIELDS:
                address = addresses["adc_version"]
                # 块中的首字就是版本号；连续读取后再单独复读，拒绝跨更新的混合数据。
                block = client.command(f"mdw 0x{address:08x} {ADC_DIAGNOSTIC_WORD_COUNT}")
                words = _parse_memory_words(block, address, ADC_DIAGNOSTIC_WORD_COUNT)
                for index, key in enumerate((
                    "version", "layout", "captured_ms", "status", "raw_vcc_in", "raw_key",
                    "sample_sequence", "sample_timestamp_ms", "error_flags", "hal_error_code",
                    "voltage_config_valid", "volts_bits", "vref_mv", "full_scale", "divider_milli",
                )):
                    snapshot[f"adc_{key}"] = words[index]
                after = client.command(_build_read_command("adc_version", address, fields))
                snapshot["adc_version_after"] = _parse_memory_value(after, address, 32)
                for key in ("tick_ms", "iwdg_reset_flag"):
                    field_address = addresses[key]
                    output = client.command(_build_read_command(key, field_address, fields))
                    snapshot[key] = _parse_memory_value(
                        output, field_address, fields[key].width_bits
                    )
                _require_running(client.command("targets"), "after checking ADC version")
            else:
                for key, address in addresses.items():
                    command = _build_read_command(key, address, fields)
                    output = client.command(command)
                    if fields[key].count > 1:
                        snapshot[key] = _parse_memory_words(output, address, fields[key].count)
                    else:
                        snapshot[key] = _parse_memory_value(
                            output, address, fields[key].width_bits
                        )
                    _require_running(client.command("targets"), f"after reading {key}")
    except (OSError, ValueError) as error:
        if isinstance(error, ProbeIOError):
            raise
        raise ProbeIOError(str(error)) from error
    return snapshot


def _small_decrease(old: int, new: int) -> bool:
    return new < old and old - new < 0x80000000


def classify_change(previous: dict[str, int], current: dict[str, int]) -> str:
    if (
        "iwdg_reset_flag" in previous
        and "iwdg_reset_flag" in current
        and not previous["iwdg_reset_flag"]
        and current["iwdg_reset_flag"]
    ):
        return "target-reset-iwdg"
    if _small_decrease(previous["tick_ms"], current["tick_ms"]):
        if current.get("iwdg_reset_flag", 0):
            return "target-reset-iwdg"
        return "target-reset-other-or-unknown"
    if any(
        key in previous and key in current and _small_decrease(previous[key], current[key])
        for key in COUNTERS
    ):
        return "counter-decrease-unclassified"
    return "running"


def _bounded_float(value: float, name: str, minimum: float, maximum: float) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum:g} and {maximum:g}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gdb", default="arm-none-eabi-gdb", help="offline ELF symbol reader")
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    parser.add_argument("--host", default=DEFAULT_HOST, help="must resolve to loopback")
    parser.add_argument("--telnet-port", type=int, default=DEFAULT_TELNET_PORT)
    parser.add_argument("--samples", type=int, default=5, help="2..8 snapshots")
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between reads, 0..2")
    parser.add_argument("--timeout", type=float, default=5.0, help="per snapshot timeout, >0..5 seconds")
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument("--imu", action="store_true", help="read the fixed BMI088/INS/heater diagnostic profile")
    profile.add_argument("--adc", action="store_true", help="读取并核对固定 ADC 诊断记录")
    profile.add_argument("--init", action="store_true", help="read the fixed startup-failure diagnostic profile")
    profile.add_argument("--rc", action="store_true", help="read the fixed remote/safety diagnostic profile")
    profile.add_argument("--rc-input", action="store_true", help="read fixed remote/safety plus keyboard and mouse fields (non-atomic)")
    profile.add_argument("--rtos", action="store_true", help="read the fixed task-stack and timing profile")
    args = parser.parse_args(argv)

    if not 2 <= args.samples <= 8:
        parser.error("--samples must be between 2 and 8")
    try:
        _validate_loopback_host(args.host)
        _validate_port(args.telnet_port)
        _bounded_float(args.interval, "--interval", 0.0, 2.0)
        _bounded_float(args.timeout, "--timeout", 0.1, 5.0)
    except ValueError as error:
        parser.error(str(error))
    if not args.elf.is_file():
        parser.error(f"ELF file does not exist: {args.elf}")

    try:
        fields = (RTOS_SNAPSHOT_FIELDS if args.rtos else ADC_SNAPSHOT_FIELDS if args.adc else RC_INPUT_SNAPSHOT_FIELDS if args.rc_input else RC_SNAPSHOT_FIELDS if args.rc
                  else INIT_SNAPSHOT_FIELDS if args.init else IMU_SNAPSHOT_FIELDS if args.imu
                  else SNAPSHOT_FIELDS)
        addresses = resolve_symbol_addresses(args.gdb, str(args.elf), args.timeout, fields)
    except ProbeIOError as error:
        print(f"probe_io_error: {error}", file=sys.stderr)
        return 2

    previous = None
    try:
        for index in range(args.samples):
            current = read_snapshot(addresses, args.host, args.telnet_port, args.timeout, fields)
            if args.rc_input:
                current = decode_rc_input(current)
            elif args.adc:
                current = decode_adc(current)
            elif args.rtos:
                current = decode_rtos_snapshot(current)
            status = "baseline" if previous is None else classify_change(previous, current)
            print(json.dumps({"sample": index + 1, "status": status, **current}, sort_keys=True))
            previous = current
            if index + 1 < args.samples and args.interval > 0.0:
                time.sleep(args.interval)
    except ProbeIOError as error:
        print(f"probe_io_error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
