#!/usr/bin/env python3
"""Take bounded RAM snapshots without attaching GDB to the running target.

GDB is used offline to resolve ELF symbol addresses only. Live reads use the
loopback OpenOCD Telnet port configured by tools/openocd_running_inspection.cfg
and are restricted to target-state queries and single-byte/word RAM reads.

ADC 模式另检查记录前后的版本和格式编号，防止把更新中的字段拼在一起。
"""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import os
import re
import socket
import math
import struct
import subprocess
import sys
import time
from datetime import datetime, timezone
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ELF = ROOT / "build" / "wsl" / "bench-debug" / "Basic_Framework_MC02_bench_safe.elf"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_TELNET_PORT = 4444
COUNTERS = ("vision_rx_count", "vision_tx_count", "vision_crc_error_count")
# STM32H723VGTx_FLASH.ld defines RAM at 0x24000000 with length 128 KiB.
# Live inspection is intentionally limited to this linker RAM region.
SNAPSHOT_RAM_START = 0x24000000
SNAPSHOT_RAM_END = 0x24020000  # exclusive
SYMBOL_VALUE_PATTERN = re.compile(r"^(?P<key>[a-z][a-z0-9_]*)=0x(?P<value>[0-9a-fA-F]+)$")
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

# v3 历史记录固定为 252 字节；只用所选 ELF 的 g_crash_log 符号读取。
CRASH_V3_WORD_COUNT = 63
CRASH_V3_SNAPSHOT_FIELDS = {
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    "crash_v3_words": SnapshotField("g_crash_log", "mdw", 32, CRASH_V3_WORD_COUNT),
    "iwdg_reset_flag": SnapshotField("iwdg_reset_flag", "mdb", 8),
}
CRASH_RECORD_SNAPSHOT_FIELDS = {
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    "crash_record_words": SnapshotField("g_crash_log", "mdw", 32, 128),
    "iwdg_reset_flag": SnapshotField("iwdg_reset_flag", "mdb", 8),
}

# 固定读取 RTOS 任务快照和已登记的 DWT 最大耗时；数组块长度来自固件编译常量。
RTOS_SNAPSHOT_FIELDS = {
    "tick_ms": SnapshotField("uwTick", "mdw", 32),
    "stats_layout": SnapshotField("lcd_stats_layout", "mdw", 32),
    "stats_generation_before": SnapshotField("lcd_stats_generation", "mdw", 32),
    "stats_sample_id": SnapshotField("lcd_stats_sample_id", "mdw", 32),
    "stats_captured_ms": SnapshotField("lcd_stats_captured_ms", "mdw", 32),
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
OBSERVATION_IDENTITY_MAGIC = 0x4F425331
OBSERVATION_IDENTITY_VERSION = 1
OBSERVATION_IDENTITY_LAYOUT = 0x4F425301
OBSERVATION_TIMING_LAYOUT = 0x4F425302
OBSERVATION_CAN_LAYOUT = 0x4F425303
OBSERVATION_LCD_LAYOUT = 0x4F425304
OBSERVATION_IDENTITY_WORDS = 12
OBSERVATION_TIMING_WORDS = 61
OBSERVATION_CAN_WORDS = 50
OBSERVATION_LCD_WORDS = 25
OBSERVATION_CAN_BUS_COUNT = 3
OBSERVATION_MODULES = {
    "ins": 1 << 0,
    "motor": 1 << 1,
    "dji": 1 << 2,
    "can": 1 << 3,
    "uart": 1 << 4,
    "usb": 1 << 5,
    "lcd": 1 << 6,
}
OBSERVATION_PROFILE_NAMES = {1: "bench_safe", 2: "robot"}
OBSERVATION_ROLE_NAMES = {1: "single", 2: "chassis", 3: "gimbal"}
OBSERVATION_PROBE_NAMES = (
    "motor_task", "ins_task", "daemon_task", "robot_task", "motor_all",
    "dji_pid", "dji_send", "ins_all", "ins_read", "ins_ekf", "ins_temp",
    "can_tx", "can_rx_dispatch", "uart_tx_task",
)
OBSERVATION_CAN_FIELDS = (
    "active", "tx_ok", "tx_drop", "tx_error", "tx_done", "tx_fifo_empty_events",
    "busoff", "busoff_recover", "rx_lost", "rx_frames", "rx_queue_drop",
    "rx_queue_peak", "tx_error_count", "rx_error_count", "status",
)
OBSERVATION_LCD_FIELDS = (
    "sample_last_us", "sample_max_us", "update_last_us", "update_max_us",
    "draw_last_us", "draw_max_us", "init_us", "recover_us", "busy_us", "cycles",
    "clear_us", "row_us", "row_bytes", "spi_us", "rows", "spi_bytes", "spi_errors",
    "module_absent", "recover_count", "init_done",
)
IDENTITY_SNAPSHOT_FIELDS = {
    "identity_words": SnapshotField(
        "((uint32_t *)&bsp_observation_identity)[0]", "mdw", 32, OBSERVATION_IDENTITY_WORDS
    ),
}
PERFORMANCE_TASK_KEYS = (
    "tick_ms", "stats_layout", "stats_generation_before", "stats_sample_id",
    "stats_captured_ms", "stats_count", "stats_task_total", "stats_truncated",
    "stats_names_words", "stats_runtime", "stats_priority_words", "stats_stack_words",
    "stats_generation_after",
)
PERFORMANCE_TASK_FIELDS = {
    **{key: RTOS_SNAPSHOT_FIELDS[key] for key in PERFORMANCE_TASK_KEYS},
    "iwdg_reset_flag": SNAPSHOT_FIELDS["iwdg_reset_flag"],
}
PERFORMANCE_SNAPSHOT_FIELDS = {
    **PERFORMANCE_TASK_FIELDS,
    "timing_generation_before": SnapshotField("bsp_observation_timing.generation", "mdw", 32),
    "timing_words": SnapshotField(
        "((uint32_t *)&bsp_observation_timing)[0]", "mdw", 32, OBSERVATION_TIMING_WORDS
    ),
    "timing_generation_after": SnapshotField("bsp_observation_timing.generation", "mdw", 32),
    "can_generation_before": SnapshotField("bsp_observation_can.generation", "mdw", 32),
    "can_words": SnapshotField(
        "((uint32_t *)&bsp_observation_can)[0]", "mdw", 32, OBSERVATION_CAN_WORDS
    ),
    "can_generation_after": SnapshotField("bsp_observation_can.generation", "mdw", 32),
    "lcd_generation_before": SnapshotField("bsp_observation_lcd.generation", "mdw", 32),
    "lcd_words": SnapshotField(
        "((uint32_t *)&bsp_observation_lcd)[0]", "mdw", 32, OBSERVATION_LCD_WORDS
    ),
    "lcd_generation_after": SnapshotField("bsp_observation_lcd.generation", "mdw", 32),
}
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


def decode_rtos_snapshot(
    snapshot: dict[str, int | list[int]],
    fields: dict[str, SnapshotField] = RTOS_SNAPSHOT_FIELDS,
    allow_unpublished: bool = False,
) -> dict:
    """Validate the published task table and unpack its fixed-width arrays."""
    if set(snapshot) != set(fields):
        raise ProbeIOError("RTOS record has missing or unexpected fields")
    for key, field in fields.items():
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
        raise ObservationSnapshotChangedError("RTOS task table changed during reading or is being published")
    unpublished = (
        "stats_sample_id" in snapshot
        and snapshot["stats_sample_id"] == 0
        and before == 0
        and after == 0
        and snapshot["stats_captured_ms"] == 0
    )
    if unpublished and not allow_unpublished:
        raise ProbeIOError("RTOS task table has not published a nonzero sample id")
    count = snapshot["stats_count"]
    total = snapshot["stats_task_total"]
    truncated = snapshot["stats_truncated"]
    if unpublished:
        count = total = truncated = 0
    elif count > RTOS_TASK_CAPACITY or total < count or truncated not in (0, 1):
        raise ProbeIOError("RTOS task counts are inconsistent")
    if not unpublished and bool(truncated) != (total > count):
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
    result = {
        **{key: value for key, value in snapshot.items()
           if key != "stats_layout" and not key.startswith("stats_")},
        "snapshot_generation": after,
        "task_count": count,
        "task_total": total,
        "task_table_capacity": RTOS_TASK_CAPACITY,
        "task_table_truncated": bool(truncated),
        "published": not unpublished,
        "tasks": tasks,
    }
    if "stats_sample_id" in snapshot:
        result["sample_id"] = snapshot["stats_sample_id"]
        result["captured_ms"] = snapshot["stats_captured_ms"]
    return result
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


class ObservationSnapshotChangedError(ProbeIOError):
    """A publication seqlock changed during a group read."""


def build_symbol_command(
    gdb: str,
    elf: str,
    fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS,
    value_expressions: dict[str, str] | None = None,
) -> list[str]:
    """Build an offline GDB invocation; it must never connect to a target."""
    commands = ["set pagination off", "set confirm off"]
    commands.extend(
        f'printf "{key}=0x%x\\n", (unsigned int)&({field.expression})'
        for key, field in fields.items()
    )
    if value_expressions:
        commands.extend(
            f'printf "{key}=0x%x\\n", (unsigned int)({expression})'
            for key, expression in value_expressions.items()
        )
    argv = [gdb, "--nx", "--quiet", "--batch", "--symbols", elf]
    for command in commands:
        argv.extend(("--ex", command))
    return argv


def parse_addresses(
    output: str,
    fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS,
    ram_bounds: tuple[int, int] = (SNAPSHOT_RAM_START, SNAPSHOT_RAM_END),
) -> dict[str, int]:
    addresses: dict[str, int] = {}
    for line in output.splitlines():
        match = SYMBOL_VALUE_PATTERN.fullmatch(line.strip())
        if match and match.group("key") in fields:
            addresses[match.group("key")] = int(match.group("value"), 16)

    _validate_snapshot_addresses(addresses, fields, ram_bounds)
    return addresses


def resolve_symbol_addresses(
    gdb: str,
    elf: str,
    timeout: float,
    fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS,
    ram_bounds: tuple[int, int] = (SNAPSHOT_RAM_START, SNAPSHOT_RAM_END),
) -> dict[str, int]:
    output = _run_gdb(gdb, elf, timeout, fields)
    return parse_addresses(output, fields, ram_bounds)


def _run_gdb(
    gdb: str,
    elf: str,
    timeout: float,
    fields: dict[str, SnapshotField],
    value_expressions: dict[str, str] | None = None,
) -> str:
    try:
        result = subprocess.run(
            build_symbol_command(gdb, elf, fields, value_expressions),
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
    return result.stdout + "\n" + result.stderr


def parse_symbol_values(output: str, expected: set[str]) -> dict[str, int]:
    values: dict[str, int] = {}
    for line in output.splitlines():
        match = SYMBOL_VALUE_PATTERN.fullmatch(line.strip())
        if match and match.group("key") in expected:
            values[match.group("key")] = int(match.group("value"), 16)
    if set(values) != expected:
        missing = sorted(expected - set(values))
        raise ProbeIOError("ELF is missing required identity values: " + ", ".join(missing))
    return values


def _read_elf_symbol(elf: str, symbol_name: str) -> tuple[int, int, bytes]:
    """Read one ELF32 little-endian symbol from the file, without target access."""
    try:
        image = Path(elf).read_bytes()
    except OSError as error:
        raise ProbeIOError(f"could not read ELF file: {error}") from error
    if len(image) < 52 or image[:4] != b"\x7fELF" or image[4] != 1 or image[5] != 1:
        raise ProbeIOError("selected ELF must be a complete ELF32 little-endian image")
    try:
        header = struct.unpack_from("<HHIIIIIHHHHHH", image, 16)
        section_offset, section_entry_size, section_count, section_names_index = (
            header[5], header[10], header[11], header[12]
        )
        if section_entry_size < 40 or section_count == 0 or section_names_index >= section_count:
            raise ProbeIOError("ELF section table is missing or invalid")
        if section_offset + section_entry_size * section_count > len(image):
            raise ProbeIOError("ELF section table extends beyond the file")
        sections = [
            struct.unpack_from("<IIIIIIIIII", image, section_offset + index * section_entry_size)
            for index in range(section_count)
        ]
        string_section = sections[section_names_index]
        section_strings = image[string_section[4]:string_section[4] + string_section[5]]

        def string_at(data: bytes, offset: int) -> str:
            if offset >= len(data):
                raise ProbeIOError("ELF string-table offset is invalid")
            end = data.find(b"\0", offset)
            if end < 0:
                raise ProbeIOError("ELF string-table entry is unterminated")
            return data[offset:end].decode("ascii", errors="strict")

        for table in sections:
            if table[1] != 2:  # SHT_SYMTAB
                continue
            linked_index = table[6]
            entry_size = table[9]
            if linked_index >= section_count or entry_size < 16 or table[5] % entry_size:
                raise ProbeIOError("ELF symbol table is invalid")
            names_section = sections[linked_index]
            names = image[names_section[4]:names_section[4] + names_section[5]]
            for offset in range(table[4], table[4] + table[5], entry_size):
                name_offset, value, size, _info, _other, section_index = struct.unpack_from(
                    "<IIIBBH", image, offset
                )
                if string_at(names, name_offset) != symbol_name:
                    continue
                if section_index == 0 or section_index >= section_count:
                    raise ProbeIOError(f"ELF symbol {symbol_name} is undefined or has no file data")
                section = sections[section_index]
                if value < section[3] or value + size > section[3] + section[5]:
                    raise ProbeIOError(f"ELF symbol {symbol_name} lies outside its section")
                if section[1] == 8:  # SHT_NOBITS, including .bss
                    return value, size, b""
                file_offset = section[4] + (value - section[3])
                if file_offset + size > len(image):
                    raise ProbeIOError(f"ELF symbol {symbol_name} extends beyond the file")
                return value, size, image[file_offset:file_offset + size]
    except (IndexError, struct.error, UnicodeError) as error:
        raise ProbeIOError(f"ELF symbol table could not be decoded: {error}") from error
    raise ProbeIOError(f"ELF is missing required symbol: {symbol_name}")


def resolve_observation_identity(
    gdb: str, readelf: str, elf: str, timeout: float
) -> dict:
    """核对 ELF 内的 Build ID、身份模板和主 RAM 链接范围。"""
    try:
        result = subprocess.run(
            [readelf, "-n", elf], capture_output=True, text=True,
            timeout=timeout, check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise ProbeIOError(f"ELF Build ID lookup timed out after {timeout:g}s") from error
    except OSError as error:
        raise ProbeIOError(f"could not start ELF reader: {error}") from error
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip().splitlines()
        message = detail[-1] if detail else f"readelf exited with status {result.returncode}"
        raise ProbeIOError(f"ELF Build ID lookup failed: {message}")
    matches = re.findall(r"Build ID:\s*([0-9a-fA-F]+)", result.stdout + result.stderr)
    if len(matches) != 1 or len(matches[0]) != 40:
        raise ProbeIOError("ELF must contain exactly one 20-byte GNU Build ID")
    build_id = matches[0].lower()

    value_expressions = {
        "ram_start": "(unsigned int)&__mc02_main_ram_start__",
        "ram_end": "(unsigned int)&__mc02_main_ram_end__",
        "template_address": "(unsigned int)&bsp_observation_identity_template",
    }
    output = _run_gdb(gdb, elf, timeout, IDENTITY_SNAPSHOT_FIELDS, value_expressions)
    values = parse_symbol_values(output, set(value_expressions))
    addresses = parse_addresses(output, IDENTITY_SNAPSHOT_FIELDS)
    ram_bounds = (values["ram_start"], values["ram_end"])
    if ram_bounds != (SNAPSHOT_RAM_START, SNAPSHOT_RAM_END):
        raise ProbeIOError("ELF main RAM range does not match the approved H723 read area")
    _validate_snapshot_addresses(addresses, IDENTITY_SNAPSHOT_FIELDS, ram_bounds)

    template_address, template_size, template_bytes = _read_elf_symbol(
        elf, "bsp_observation_identity_template"
    )
    identity_symbol_address, identity_size, _identity_bytes = _read_elf_symbol(
        elf, "bsp_observation_identity"
    )
    if values["template_address"] != template_address:
        raise ProbeIOError("ELF identity template address disagrees with its symbol table")
    if template_size != 7 * 4 or len(template_bytes) != template_size:
        raise ProbeIOError("ELF observation identity template has an invalid size")
    if identity_symbol_address != addresses["identity_words"] or identity_size != OBSERVATION_IDENTITY_WORDS * 4:
        raise ProbeIOError("ELF observation identity object has an invalid address or size")
    template_values = struct.unpack("<7I", template_bytes)
    template = dict(zip(("magic", "version", "size_bytes", "layout", "profile", "role", "module_mask"), template_values))
    if template["magic"] != OBSERVATION_IDENTITY_MAGIC:
        raise ProbeIOError("ELF observation identity magic mismatch")
    if template["version"] != OBSERVATION_IDENTITY_VERSION:
        raise ProbeIOError("ELF observation identity version mismatch")
    if template["size_bytes"] != OBSERVATION_IDENTITY_WORDS * 4:
        raise ProbeIOError("ELF observation identity size mismatch")
    if template["layout"] != OBSERVATION_IDENTITY_LAYOUT:
        raise ProbeIOError("ELF observation identity layout mismatch")
    if template["profile"] not in OBSERVATION_PROFILE_NAMES or template["role"] not in OBSERVATION_ROLE_NAMES:
        raise ProbeIOError("ELF observation profile or role is unknown")
    valid_module_mask = sum(OBSERVATION_MODULES.values())
    if template["module_mask"] & ~valid_module_mask:
        raise ProbeIOError("ELF observation module mask has unknown bits")

    build_id_bytes = bytes.fromhex(build_id)
    build_id_words = [
        int.from_bytes(build_id_bytes[index:index + 4], "little")
        for index in range(0, len(build_id_bytes), 4)
    ]
    expected = {
        **template,
        "build_id_words": build_id_words,
        "build_id": build_id,
        "profile_name": OBSERVATION_PROFILE_NAMES[template["profile"]],
        "role_name": OBSERVATION_ROLE_NAMES[template["role"]],
    }
    return {
        "expected": expected,
        "identity_address": addresses["identity_words"],
        "ram_bounds": ram_bounds,
        "build_id": build_id,
    }


def decode_identity_words(words: list[int]) -> dict[str, int | list[int]]:
    if len(words) != OBSERVATION_IDENTITY_WORDS or any(
        type(word) is not int or not 0 <= word <= UINT32_MAX for word in words
    ):
        raise ProbeIOError("target observation identity has an invalid word count or value")
    actual = {
        "magic": words[0],
        "version": words[1],
        "size_bytes": words[2],
        "layout": words[3],
        "build_id_words": words[4:9],
        "profile": words[9],
        "role": words[10],
        "module_mask": words[11],
    }
    if actual["magic"] != OBSERVATION_IDENTITY_MAGIC:
        raise ProbeIOError("target has no initialized observation identity")
    if actual["version"] != OBSERVATION_IDENTITY_VERSION:
        raise ProbeIOError("target observation identity version mismatch")
    if actual["size_bytes"] != OBSERVATION_IDENTITY_WORDS * 4:
        raise ProbeIOError("target observation identity size mismatch")
    if actual["layout"] != OBSERVATION_IDENTITY_LAYOUT:
        raise ProbeIOError("target observation identity layout mismatch")
    if actual["profile"] not in OBSERVATION_PROFILE_NAMES or actual["role"] not in OBSERVATION_ROLE_NAMES:
        raise ProbeIOError("target observation profile or role is unknown")
    if actual["module_mask"] & ~sum(OBSERVATION_MODULES.values()):
        raise ProbeIOError("target observation module mask has unknown bits")
    return actual


def _decode_observation_group(
    words: list[int], before: int, after: int, expected_words: int,
    expected_layout: int, name: str,
) -> tuple[int, int, int, int, list[int]]:
    if len(words) != expected_words or any(
        type(word) is not int or not 0 <= word <= UINT32_MAX for word in words
    ):
        raise ProbeIOError(f"{name} observation block has an invalid word count or value")
    generation = words[1]
    if before == after == generation == words[2] == words[3] == 0:
        return 0, 0, 0, 0, []
    if before != after or before != generation or generation == 0 or generation & 1:
        raise ObservationSnapshotChangedError(f"{name} observation block changed while it was read")
    if words[0] != expected_layout:
        raise ProbeIOError(f"{name} observation layout mismatch")
    if words[2] == 0:
        raise ProbeIOError(f"{name} observation sample id is zero")
    return generation, words[2], words[3], words[4], words[5:]


def decode_performance_snapshot(snapshot: dict, expected_identity: dict) -> dict:
    """验证四组定长统计，并保留各组独立发布时间和更新编号。"""
    expected_keys = set(PERFORMANCE_SNAPSHOT_FIELDS)
    if set(snapshot) != expected_keys:
        raise ProbeIOError("performance record has missing or unexpected fields")
    task_raw = {key: snapshot[key] for key in PERFORMANCE_TASK_FIELDS}
    tasks = decode_rtos_snapshot(task_raw, PERFORMANCE_TASK_FIELDS, allow_unpublished=True)
    timing_generation, timing_id, timing_ms, timing_mask, timing_values = _decode_observation_group(
        snapshot["timing_words"], snapshot["timing_generation_before"],
        snapshot["timing_generation_after"], OBSERVATION_TIMING_WORDS,
        OBSERVATION_TIMING_LAYOUT, "timing",
    )
    can_enabled = bool(expected_identity["module_mask"] & OBSERVATION_MODULES["can"])
    if can_enabled:
        can_generation, can_id, can_ms, active_mask, can_values = _decode_observation_group(
            snapshot["can_words"], snapshot["can_generation_before"],
            snapshot["can_generation_after"], OBSERVATION_CAN_WORDS,
            OBSERVATION_CAN_LAYOUT, "CAN",
        )
    else:
        can_generation, can_id, can_ms, active_mask, can_values = 0, 0, 0, 0, []
    lcd_enabled = bool(expected_identity["module_mask"] & OBSERVATION_MODULES["lcd"])
    if lcd_enabled:
        lcd_generation, lcd_id, lcd_ms, lcd_mask, lcd_values = _decode_observation_group(
        snapshot["lcd_words"], snapshot["lcd_generation_before"],
        snapshot["lcd_generation_after"], OBSERVATION_LCD_WORDS,
        OBSERVATION_LCD_LAYOUT, "LCD",
        )
    else:
        lcd_generation, lcd_id, lcd_ms, lcd_mask, lcd_values = 0, 0, 0, 0, []
    expected_mask = expected_identity["module_mask"]
    if timing_id and timing_mask != expected_mask:
        raise ProbeIOError("observation module mask disagrees with the matched ELF")
    if timing_id and len(timing_values) != len(OBSERVATION_PROBE_NAMES) * 4:
        raise ProbeIOError("timing probe count does not match the fixed layout")
    probes = {}
    if timing_id:
        for index, name in enumerate(OBSERVATION_PROBE_NAMES):
            values = timing_values[index * 4:index * 4 + 4]
            probes[name] = {
                "last_us": values[0], "max_us": values[1],
                "total_us": values[2], "calls": values[3],
            }
    if can_id and len(can_values) != OBSERVATION_CAN_BUS_COUNT * len(OBSERVATION_CAN_FIELDS):
        raise ProbeIOError("CAN bus count does not match the fixed layout")
    buses = []
    for index in range(OBSERVATION_CAN_BUS_COUNT if can_id else 0):
        values = can_values[index * len(OBSERVATION_CAN_FIELDS):(index + 1) * len(OBSERVATION_CAN_FIELDS)]
        bus = dict(zip(OBSERVATION_CAN_FIELDS, values))
        if bus["active"] not in (0, 1) or bus["status"] not in range(4):
            raise ProbeIOError(f"CAN{index + 1} observation status is invalid")
        if bool(bus["active"]) != bool(active_mask & (1 << index)):
            raise ProbeIOError(f"CAN{index + 1} active flag disagrees with the bus mask")
        buses.append(bus)
    if active_mask & ~((1 << OBSERVATION_CAN_BUS_COUNT) - 1):
        raise ProbeIOError("CAN active mask has unknown bits")
    if lcd_id and len(lcd_values) != len(OBSERVATION_LCD_FIELDS):
        raise ProbeIOError("LCD field count does not match the fixed layout")
    lcd = dict(zip(OBSERVATION_LCD_FIELDS, lcd_values)) if lcd_id else {}
    if lcd_id and (lcd["module_absent"] not in (0, 1) or lcd["init_done"] not in (0, 1)):
        raise ProbeIOError("LCD observation flags are invalid")
    return {
        "tick_ms": snapshot["tick_ms"],
        "task": {
            "status": "published" if tasks["published"] else "not-published",
            "generation": snapshot["stats_generation_after"],
            "sample_id": snapshot["stats_sample_id"],
            "captured_ms": snapshot["stats_captured_ms"],
            "count": tasks["task_count"],
            "total": tasks["task_total"],
            "truncated": tasks["task_table_truncated"],
            "tasks": tasks["tasks"],
        },
        "timing": {
            "status": "published" if timing_id else "not-published",
            "generation": timing_generation, "sample_id": timing_id,
            "captured_ms": timing_ms, "module_mask": timing_mask, "probes": probes,
        },
        "can": {
            "status": "not-enabled" if not can_enabled else ("published" if can_id else "not-published"),
            "generation": can_generation, "sample_id": can_id,
            "captured_ms": can_ms, "active_bus_mask": active_mask, "buses": buses,
        },
        "lcd": {
            "status": "not-enabled" if not lcd_enabled else ("published" if lcd_id else "not-published"),
            "generation": lcd_generation, "sample_id": lcd_id,
            "captured_ms": lcd_ms, "module_mask": lcd_mask, "values": lcd,
        },
    }


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


def _validated_address(
    address: int,
    alignment: int,
    ram_bounds: tuple[int, int] = (SNAPSHOT_RAM_START, SNAPSHOT_RAM_END),
) -> int:
    if isinstance(address, bool) or not isinstance(address, int):
        raise ValueError("symbol addresses must be integers")
    if not ram_bounds[0] <= address < ram_bounds[1] or address > UINT32_MAX:
        raise ValueError("symbol address is outside the approved linker RAM region")
    if address % alignment:
        raise ValueError(f"symbol address 0x{address:x} is not {alignment}-byte aligned")
    return address


def _validate_snapshot_addresses(
    addresses: dict[str, int],
    fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS,
    ram_bounds: tuple[int, int] = (SNAPSHOT_RAM_START, SNAPSHOT_RAM_END),
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
            _validated_address(addresses[key], field.alignment_bytes, ram_bounds)
            if not 1 <= field.count <= 128:
                raise ValueError("read length must be between 1 and 128 words")
            if field.count > 1 and field.command != "mdw":
                raise ValueError("multiword reads must use aligned 32-bit words")
            if addresses[key] + field.count * 4 > ram_bounds[1]:
                raise ValueError("read extends beyond the approved linker RAM region")
        except ValueError as error:
            raise ProbeIOError(f"ELF symbol {key} has an invalid address: {error}") from error


def _build_read_command(
    key: str,
    address: int,
    fields: dict[str, SnapshotField] = SNAPSHOT_FIELDS,
    ram_bounds: tuple[int, int] = (SNAPSHOT_RAM_START, SNAPSHOT_RAM_END),
) -> str:
    if key not in fields:
        raise ValueError(f"unsupported snapshot field: {key}")
    field = fields[key]
    validated = _validated_address(address, field.alignment_bytes, ram_bounds)
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
        ram_bounds: tuple[int, int] = (SNAPSHOT_RAM_START, SNAPSHOT_RAM_END),
    ):
        _validate_loopback_host(host)
        _validate_port(port)
        if not 0 < timeout <= 5.0:
            raise ValueError("timeout must be greater than 0 and at most 5 seconds")
        self.host = host
        self.port = port
        self.timeout = timeout
        _validate_snapshot_addresses(allowed_addresses, fields, ram_bounds)
        self.fields = fields
        self.allowed_addresses = dict(allowed_addresses)
        self.ram_bounds = ram_bounds
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
                "adc_version" in self.fields
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
                "ref_magic" in self.fields
                and match.group("command") == "mdw"
                and read[1] == self.allowed_addresses["ref_magic"]
                and count == REFEREE_DIAGNOSTIC_WORD_COUNT
                and self.allowed_addresses["ref_count"] == read[1] + 28
                and read[1] + count * 4 <= self.ram_bounds[1]
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
    ram_bounds: tuple[int, int] = (SNAPSHOT_RAM_START, SNAPSHOT_RAM_END),
) -> dict[str, int | list[int]]:
    return _read_snapshot(addresses, host, port, timeout, fields, ram_bounds)[0]


def read_verified_snapshot(
    addresses: dict[str, int],
    identity_address: int,
    expected_identity: dict,
    host: str,
    port: int,
    timeout: float,
    fields: dict[str, SnapshotField],
    ram_bounds: tuple[int, int],
) -> tuple[dict[str, int | list[int]], list[int]]:
    """先在同一连接核对目标身份，匹配后才读取业务统计。"""
    return _read_snapshot(
        addresses, host, port, timeout, fields, ram_bounds,
        expected_identity=expected_identity, identity_address=identity_address,
    )


def _read_snapshot(
    addresses: dict[str, int],
    host: str,
    port: int,
    timeout: float,
    fields: dict[str, SnapshotField],
    ram_bounds: tuple[int, int],
    expected_identity: dict | None = None,
    identity_address: int | None = None,
) -> tuple[dict[str, int | list[int]], list[int]]:
    client_fields = dict(fields)
    client_addresses = dict(addresses)
    if expected_identity is not None:
        if identity_address is None:
            raise ProbeIOError("identity address is missing")
        client_fields = {**IDENTITY_SNAPSHOT_FIELDS, **client_fields}
        client_addresses = {"identity_words": identity_address, **client_addresses}
    _validate_snapshot_addresses(client_addresses, client_fields, ram_bounds)

    snapshot: dict[str, int | list[int]] = {}
    identity_words: list[int] = []
    try:
        with OpenOCDTelnetClient(
            host, port, timeout, allowed_addresses=client_addresses,
            fields=client_fields, ram_bounds=ram_bounds,
        ) as client:
            _require_running(client.command("targets"), "before the snapshot")
            if expected_identity is not None:
                address = client_addresses["identity_words"]
                command = _build_read_command("identity_words", address, client_fields, ram_bounds)
                output = client.command(command)
                identity_words = _parse_memory_words(output, address, OBSERVATION_IDENTITY_WORDS)
                actual = decode_identity_words(identity_words)
                identity_keys = (
                    "magic", "version", "size_bytes", "layout", "build_id_words",
                    "profile", "role", "module_mask",
                )
                if any(actual[key] != expected_identity[key] for key in identity_keys):
                    raise ProbeIOError("target firmware identity does not match the selected ELF")
                _require_running(client.command("targets"), "after matching firmware identity")
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
                after = client.command(_build_read_command("adc_version", address, client_fields, ram_bounds))
                snapshot["adc_version_after"] = _parse_memory_value(after, address, 32)
                for key in ("tick_ms", "iwdg_reset_flag"):
                    field_address = addresses[key]
                    output = client.command(_build_read_command(key, field_address, client_fields, ram_bounds))
                    snapshot[key] = _parse_memory_value(
                        output, field_address, fields[key].width_bits
                    )
                _require_running(client.command("targets"), "after checking ADC version")
            else:
                for key, address in addresses.items():
                    command = _build_read_command(key, address, client_fields, ram_bounds)
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
    return snapshot, identity_words


def read_verified_performance_snapshot(
    addresses: dict[str, int],
    identity_address: int,
    expected_identity: dict,
    host: str,
    port: int,
    timeout: float,
    fields: dict[str, SnapshotField],
    ram_bounds: tuple[int, int],
) -> tuple[dict[str, int | list[int]], list[int], dict]:
    """Retry a seqlock collision within one total snapshot timeout."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ProbeIOError("timed out waiting for a stable snapshot")
        try:
            raw, target_identity_words = read_verified_snapshot(
                addresses, identity_address, expected_identity, host, port,
                remaining, fields, ram_bounds,
            )
            decoded = decode_performance_snapshot(raw, expected_identity)
            return raw, target_identity_words, decoded
        except ObservationSnapshotChangedError as error:
            if time.monotonic() >= deadline:
                raise ProbeIOError("timed out waiting for a stable snapshot") from error


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


def _counter_delta(old: int, new: int) -> int | None:
    delta = (new - old) & UINT32_MAX
    return None if delta > 0x7FFFFFFF else delta


def _unique_group(records: list[dict], group_name: str) -> list[dict]:
    unique = []
    seen: set[int] = set()
    for record in records:
        group = record[group_name]
        sample_id = group["sample_id"]
        if group.get("status", "published") == "published" and sample_id != 0 and sample_id not in seen:
            seen.add(sample_id)
            unique.append(record)
    return unique


def _group_is_fresh(group: dict, tick_ms: int) -> bool:
    age = (tick_ms - group["captured_ms"]) & UINT32_MAX
    return age <= 10000


def summarize_performance(records: list[dict], expected_identity: dict) -> tuple[str, bool]:
    """接受内存样本或 JSONL 样本，按组去重并重算窗口统计。"""
    # 运行时保留完整样本行以便写入 JSONL；统计字段实际放在 decoded 中。
    # 同时接受已经解码的记录，便于从保存的 JSONL 重算摘要。
    records = [record.get("decoded", record) for record in records]
    lines = [
        "MC02 运行观测摘要",
        f"固件：{expected_identity['profile_name']} / {expected_identity['role_name']}",
        f"Build ID：{expected_identity['build_id']}",
        "窗口：记录中的单调时钟起止时间；任务占比是运行计数差值占比，不是精确 CPU 百分比。",
        "耗时平均值使用 DWT 累计微秒差除以调用数；max 是历史最大观测值，不是 WCET。",
        "FreeRTOS 当前计数来自 CYCCNT/1000；原始 32 位 CYCCNT 约 8.94 秒回绕，任务切换跨回绕的累计有局限。",
    ]
    complete = bool(records)
    latest_tick = records[-1]["tick_ms"] if records else 0

    task_records = _unique_group(records, "task") if records else []
    task_fresh = bool(task_records) and _group_is_fresh(task_records[-1]["task"], latest_tick)
    lines.append("")
    lines.append("任务运行计数与历史最小剩余栈")
    if len(task_records) < 2 or not task_fresh:
        lines.append("  不可计算：不同任务表不足两份或最后一份已超过两个发布周期。")
        complete = False
    else:
        first = task_records[0]["task"]
        last = task_records[-1]["task"]
        if first["truncated"] or last["truncated"]:
            lines.append("  不可计算：任务表被截断。")
            complete = False
        else:
            first_by_name = {task["name"]: task for task in first["tasks"]}
            last_by_name = {task["name"]: task for task in last["tasks"]}
            common = sorted(set(first_by_name) & set(last_by_name))
            if set(first_by_name) != set(last_by_name):
                lines.append("  任务集合在窗口内变化；只列出两端都存在且计数有效的任务。")
                complete = False
            deltas: dict[str, int] = {}
            for name in common:
                delta = _counter_delta(
                    first_by_name[name]["runtime_counter"],
                    last_by_name[name]["runtime_counter"],
                )
                if delta is None:
                    lines.append(f"  {name}: 计数下降，不能配对")
                    complete = False
                else:
                    deltas[name] = delta
            total = sum(deltas.values())
            if total == 0:
                lines.append("  窗口计数总差为 0，无法计算比例。")
                complete = False
            for name, delta in deltas.items():
                ratio = 100.0 * delta / total if total else None
                high_water = last_by_name[name]["stack_high_water_words"]
                stack_bytes = last_by_name[name]["stack_high_water_bytes"]
                pct = "不可计算" if ratio is None else f"{ratio:.1f}%"
                lines.append(f"  {name}: 占比 {pct}，历史最小剩余栈 {high_water} word / {stack_bytes} B")
    if task_records:
        lines.append(f"  任务表发布时间：{task_records[-1]['task']['captured_ms']} ms，更新号 {task_records[-1]['task']['sample_id']}")

    module_for_probe = {
        "motor_task": "motor", "motor_all": "motor", "dji_pid": "dji", "dji_send": "dji",
        "ins_task": "ins", "ins_all": "ins", "ins_read": "ins", "ins_ekf": "ins",
        "ins_temp": "ins", "can_tx": "can", "can_rx_dispatch": "can", "uart_tx_task": "uart",
    }
    timing_records = _unique_group(records, "timing") if records else []
    timing_fresh = bool(timing_records) and _group_is_fresh(timing_records[-1]["timing"], latest_tick)
    lines.append("")
    lines.append("DWT 耗时窗口平均值、调用数与历史最大值")
    if len(timing_records) < 2 or not timing_fresh:
        lines.append("  不可计算：不同耗时记录不足两份或最后一份已过期。")
        complete = False
    else:
        first = timing_records[0]["timing"]
        last = timing_records[-1]["timing"]
        for name in OBSERVATION_PROBE_NAMES:
            module = module_for_probe.get(name)
            if module and not (expected_identity["module_mask"] & OBSERVATION_MODULES[module]):
                lines.append(f"  {name}: 未启用")
                continue
            old = first["probes"][name]
            new = last["probes"][name]
            calls = _counter_delta(old["calls"], new["calls"])
            total_us = _counter_delta(old["total_us"], new["total_us"])
            if calls is None or total_us is None:
                lines.append(f"  {name}: 计数异常，窗口数据不可用")
                complete = False
                continue
            if calls == 0:
                lines.append(f"  {name}: 未采样；历史最大 {new['max_us']} us")
            else:
                average = total_us / calls
                max_observed = max(sample["timing"]["probes"][name]["max_us"] for sample in timing_records)
                lines.append(f"  {name}: 平均 {average:.2f} us / {calls} 次；历史最大观测 {max_observed} us")
    if timing_records:
        lines.append(f"  耗时记录发布时间：{timing_records[-1]['timing']['captured_ms']} ms，更新号 {timing_records[-1]['timing']['sample_id']}")

    can_records = _unique_group(records, "can") if records else []
    can_fresh = bool(can_records) and _group_is_fresh(can_records[-1]["can"], latest_tick)
    lines.append("")
    lines.append("CAN 分总线累计计数差")
    if not (expected_identity["module_mask"] & OBSERVATION_MODULES["can"]):
        lines.append("  未启用")
    elif len(can_records) < 2 or not can_fresh:
        lines.append("  不可计算：不同 CAN 记录不足两份或最后一份已过期。")
        complete = False
    else:
        first = can_records[0]["can"]
        last = can_records[-1]["can"]
        for index, bus in enumerate(last["buses"]):
            if not bus["active"]:
                lines.append(f"  CAN{index + 1}: 服务未启动")
                continue
            old = first["buses"][index]
            differences = {}
            for key in ("tx_ok", "tx_drop", "tx_error", "tx_done", "tx_fifo_empty_events",
                        "busoff", "busoff_recover", "rx_lost", "rx_frames", "rx_queue_drop",
                        "tx_error_count", "rx_error_count"):
                delta = _counter_delta(old[key], bus[key])
                if delta is None:
                    complete = False
                    differences[key] = "计数异常"
                else:
                    differences[key] = delta
            differences["rx_queue_peak_current"] = bus["rx_queue_peak"]
            differences["status_current"] = bus["status"]
            lines.append(f"  CAN{index + 1}: {json.dumps(differences, ensure_ascii=False, sort_keys=True)}")
    if can_records:
        lines.append(f"  CAN 记录发布时间：{can_records[-1]['can']['captured_ms']} ms，更新号 {can_records[-1]['can']['sample_id']}")

    lcd_records = _unique_group(records, "lcd") if records else []
    lcd_fresh = bool(lcd_records) and _group_is_fresh(lcd_records[-1]["lcd"], latest_tick)
    lines.append("")
    lines.append("LCD 分段耗时与累计量")
    if not (expected_identity["module_mask"] & OBSERVATION_MODULES["lcd"]):
        lines.append("  未启用")
    elif len(lcd_records) < 2 or not lcd_fresh:
        lines.append("  不可计算：不同 LCD 记录不足两份或最后一份已过期。")
        complete = False
    else:
        first = lcd_records[0]["lcd"]["values"]
        last = lcd_records[-1]["lcd"]["values"]
        for key in ("sample_last_us", "sample_max_us", "update_last_us", "update_max_us",
                    "draw_last_us", "draw_max_us", "init_us", "recover_us", "clear_us",
                    "row_us", "row_bytes", "spi_us"):
            lines.append(f"  {key}: 当前 {last[key]}")
        for key in ("cycles", "busy_us", "rows", "spi_bytes", "spi_errors", "recover_count"):
            delta = _counter_delta(first[key], last[key])
            if delta is None:
                lines.append(f"  {key}: 计数下降，不能计算窗口差")
                complete = False
            else:
                lines.append(f"  {key}: 窗口增加 {delta}")
    if lcd_records:
        lines.append(f"  LCD 记录发布时间：{lcd_records[-1]['lcd']['captured_ms']} ms，更新号 {lcd_records[-1]['lcd']['sample_id']}")
    lines.append("")
    lines.append("读取范围仅限匹配 ELF 的固定字段和主 RAM；过程不 halt、reset、写内存或发送控制帧。")
    if not complete:
        lines.append("报告不完整：至少一组记录过期、样本不足、计数异常或任务表不完整。")
    return "\n".join(lines) + "\n", complete


def _make_observation_session(root: Path) -> Path:
    base = root / "observations"
    base.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    session = base / f"{stamp}_{os.getpid()}"
    suffix = 1
    while session.exists():
        session = base / f"{stamp}_{os.getpid()}_{suffix:02d}"
        suffix += 1
    session.mkdir()
    return session


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_cmake_configuration(elf: Path) -> dict[str, str]:
    cache = elf.parent / "CMakeCache.txt"
    if not cache.is_file():
        return {"cache": "unavailable"}
    keys = ("MC02_PROFILE", "MC02_BOARD_ROLE", "MC02_VISION_TRANSPORT", "CMAKE_BUILD_TYPE")
    result = {"cache": str(cache.resolve())}
    for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("//") or line.startswith("#") or "=" not in line:
            continue
        left, value = line.split("=", 1)
        key = left.split(":", 1)[0]
        if key in keys:
            result[key] = value
    return result


def _append_jsonl(stream, value: dict) -> None:
    stream.write(json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
    stream.flush()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _short_summary(records: list[dict], identity: dict, error: str | None) -> tuple[str, bool]:
    lines = [
        "MC02 观测摘要",
        f"固件配置：{identity['profile_name']} / {identity['role_name']}",
        f"Build ID?{identity['build_id']}",
        f"原始样本数：{len(records)}",
    ]
    complete = bool(records) and error is None
    if records:
        lines.append(f"窗口状态：{records[-1]['status']}")
    if error:
        lines.append(f"错误：{error}")
    lines.append("地址仅依据所选 ELF 的 RAM 布局；身份匹配不等于整片 Flash 校验。")
    return "\n".join(lines) + "\n", complete


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gdb", default="arm-none-eabi-gdb", help="offline ELF symbol reader")
    parser.add_argument("--readelf", default="arm-none-eabi-readelf", help="offline ELF Build ID reader")
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    parser.add_argument("--host", default=DEFAULT_HOST, help="must resolve to loopback")
    parser.add_argument("--telnet-port", type=int, default=DEFAULT_TELNET_PORT)
    parser.add_argument("--samples", type=int, default=5, help="2..8 snapshots in short mode")
    parser.add_argument(
        "--interval", type=float, default=1.0,
        help="seconds between reads; short mode 0..2, performance mode 0.05..10",
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="total timeout per snapshot, >0..5 seconds")
    parser.add_argument("--duration", type=float, default=30.0, help="performance window, 5..300 seconds")
    parser.add_argument(
        "--output-root", type=Path, default=ROOT / "build",
        help="session files are stored under OUTPUT_ROOT/observations",
    )
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument("--performance", action="store_true", help="collect task/DWT/CAN/LCD statistics for a bounded window")
    profile.add_argument("--imu", action="store_true", help="read the fixed BMI088/INS/heater diagnostic profile")
    profile.add_argument("--adc", action="store_true", help="读取固定 ADC 诊断字段")
    profile.add_argument("--init", action="store_true", help="read the fixed startup-failure diagnostic profile")
    profile.add_argument("--rc", action="store_true", help="read the fixed remote/safety diagnostic profile")
    profile.add_argument("--rc-input", action="store_true", help="read fixed remote/safety plus keyboard and mouse fields (non-atomic)")
    profile.add_argument("--rtos", action="store_true", help="read the fixed task-stack and timing profile")
    profile.add_argument("--crash-v3", action="store_true", help="导出所选 ELF 中固定地址的 v3 崩溃记录")
    profile.add_argument("--crash-record", action="store_true", help="导出固定512字节崩溃记录保留区")
    args = parser.parse_args(argv)

    if not 2 <= args.samples <= 8:
        parser.error("--samples must be between 2 and 8")
    try:
        _validate_loopback_host(args.host)
        _validate_port(args.telnet_port)
        _bounded_float(args.interval, "--interval", 0.05 if args.performance else 0.0,
                       10.0 if args.performance else 2.0)
        _bounded_float(args.timeout, "--timeout", 0.1, 5.0)
        if args.performance:
            _bounded_float(args.duration, "--duration", 5.0, 300.0)
        elif args.duration != 30.0:
            parser.error("--duration is only valid with --performance")
    except ValueError as error:
        parser.error(str(error))
    if not args.elf.is_file():
        parser.error(f"ELF file does not exist: {args.elf}")

    try:
        session = _make_observation_session(args.output_root)
        raw_stream = (session / "raw.jsonl").open("w", encoding="utf-8", newline="\n")
    except OSError as error:
        print(f"observation_io_error: could not create session: {error}", file=sys.stderr)
        return 2

    records: list[dict] = []
    error_message: str | None = None
    interrupted = False
    identity: dict | None = None
    summary = "MC02 观测摘要\n观测尚未完成\n"
    status_code = 2
    start = time.monotonic()
    try:
        elf_sha256 = _sha256_file(args.elf)
        args_record = {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        }
        _append_jsonl(raw_stream, {
            "type": "session",
            "schema_version": 1,
            "created_utc": _utc_now(),
            "arguments": args_record,
            "elf_path": str(args.elf.resolve()),
            "elf_sha256": elf_sha256,
            "cmake_configuration": _load_cmake_configuration(args.elf),
            "mode": "performance" if args.performance else (
                "crash-record" if args.crash_record else ("crash-v3" if args.crash_v3 else "snapshot")
            ),
        })

        identity_info = resolve_observation_identity(args.gdb, args.readelf, str(args.elf), args.timeout)
        identity = identity_info["expected"]
        fields = (
            PERFORMANCE_SNAPSHOT_FIELDS if args.performance else
            RTOS_SNAPSHOT_FIELDS if args.rtos else
            CRASH_RECORD_SNAPSHOT_FIELDS if args.crash_record else
            CRASH_V3_SNAPSHOT_FIELDS if args.crash_v3 else
            ADC_SNAPSHOT_FIELDS if args.adc else
            RC_INPUT_SNAPSHOT_FIELDS if args.rc_input else
            RC_SNAPSHOT_FIELDS if args.rc else
            INIT_SNAPSHOT_FIELDS if args.init else
            IMU_SNAPSHOT_FIELDS if args.imu else
            SNAPSHOT_FIELDS
        )
        addresses = resolve_symbol_addresses(
            args.gdb, str(args.elf), args.timeout, fields, identity_info["ram_bounds"]
        )
        _append_jsonl(raw_stream, {
            "type": "identity",
            "resolved_utc": _utc_now(),
            "build_id": identity["build_id"],
            "elf_identity": identity,
            "identity_address": identity_info["identity_address"],
            "ram_bounds": list(identity_info["ram_bounds"]),
        })

        previous: dict | None = None
        if args.performance:
            window_start = time.monotonic()
            window_end = window_start + args.duration
            deadline = window_start
            sample_index = 0
            skipped_slots = 0
            while deadline <= window_end:
                now = time.monotonic()
                if now < deadline:
                    time.sleep(deadline - now)
                if time.monotonic() > window_end:
                    break
                raw, target_words, decoded = read_verified_performance_snapshot(
                    addresses, identity_info["identity_address"], identity,
                    args.host, args.telnet_port, args.timeout, fields,
                    identity_info["ram_bounds"],
                )
                status = "baseline" if previous is None else classify_change(previous, decoded)
                sample = {
                    "type": "sample",
                    "sample": sample_index + 1,
                    "utc": _utc_now(),
                    "elapsed_seconds": round(time.monotonic() - window_start, 3),
                    "tick_ms": decoded["tick_ms"],
                    "status": status,
                    "missed_schedule_slots_before_sample": skipped_slots,
                    "target_identity_words": target_words,
                    "raw_words": raw,
                    "decoded": decoded,
                }
                records.append(sample)
                _append_jsonl(raw_stream, sample)
                print(f"sample={sample_index + 1} elapsed={sample['elapsed_seconds']:.1f}s status={status}")
                sample_index += 1
                previous = decoded
                if status not in ("baseline", "running"):
                    error_message = f"target state changed during the window: {status}"
                    break
                deadline += args.interval
                now = time.monotonic()
                if deadline <= now:
                    missed = int((now - deadline) // args.interval) + 1
                    deadline += missed * args.interval
                    skipped_slots += missed
            summary, statistics_complete = summarize_performance(records, identity)
            status_code = 0 if statistics_complete and error_message is None else 2
        else:
            for index in range(args.samples):
                raw, target_words = read_verified_snapshot(
                    addresses, identity_info["identity_address"], identity,
                    args.host, args.telnet_port, args.timeout, fields,
                    identity_info["ram_bounds"],
                )
                if args.rc_input:
                    decoded = decode_rc_input(raw)
                elif args.adc:
                    decoded = decode_adc(raw)
                elif args.rtos:
                    decoded = decode_rtos_snapshot(raw)
                else:
                    decoded = raw
                status = "baseline" if previous is None else classify_change(previous, decoded)
                sample = {
                    "type": "sample",
                    "sample": index + 1,
                    "utc": _utc_now(),
                    "elapsed_seconds": round(time.monotonic() - start, 3),
                    "status": status,
                    "target_identity_words": target_words,
                    "raw_words": raw,
                    "decoded": decoded,
                }
                records.append(sample)
                _append_jsonl(raw_stream, sample)
                print(json.dumps({"sample": index + 1, "status": status, **decoded},
                                 ensure_ascii=False, sort_keys=True))
                previous = decoded
                if status not in ("baseline", "running"):
                    error_message = f"target state changed: {status}"
                    break
                if index + 1 < args.samples and args.interval > 0.0:
                    time.sleep(args.interval)
            summary, snapshot_complete = _short_summary(records, identity, error_message)
            status_code = 0 if snapshot_complete and len(records) == args.samples else 2
    except KeyboardInterrupt:
        interrupted = True
        error_message = "interrupted by user; partial data has been saved"
        _append_jsonl(raw_stream, {
            "type": "interrupted", "utc": _utc_now(), "samples_saved": len(records),
        })
    except (ProbeIOError, OSError, ValueError, subprocess.SubprocessError) as error:
        error_message = str(error)
        _append_jsonl(raw_stream, {"type": "error", "utc": _utc_now(), "error": error_message})
    finally:
        if identity is not None and args.performance:
            summary, statistics_complete = summarize_performance(records, identity)
            if error_message:
                summary += f"\n观测不完整：{error_message}\n"
            if not statistics_complete or error_message:
                status_code = 2
        elif identity is not None and error_message:
            summary, _ = _short_summary(records, identity, error_message)
            status_code = 2
        elif identity is None:
            summary = "MC02 观测摘要\n观测未完成：" + (error_message or "ELF 身份未核验") + "\n"
            status_code = 2
        try:
            raw_stream.close()
            (session / "summary.txt").write_text(summary, encoding="utf-8", newline="\n")
        except OSError as error:
            print(f"observation_io_error: could not finish session: {error}", file=sys.stderr)
            status_code = 2

    print(summary, end="")
    print(f"session={session}")
    if interrupted:
        return 130
    if error_message:
        print(f"observation_error: {error_message}", file=sys.stderr)
    return status_code


if __name__ == "__main__":
    raise SystemExit(main())
