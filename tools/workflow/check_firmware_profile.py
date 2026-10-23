#!/usr/bin/env python3
"""检查固件角色隔离，以及清单、编译数据库和实际链接输入的一致性。"""
import argparse
import json
from pathlib import Path
import subprocess

ACTUATORS = {
    "ChassisInit", "GimbalInit", "ShootInit", "RobotCMDInit",
    "ChassisTask", "GimbalTask", "ShootTask", "RobotCMDTask",
    "DJIMotorInit", "DJIMotorControl", "LKMotorInit", "LKMotorControl",
    "HTMotorInit", "HTMotorControlInit", "DMMotorInit", "DMMotorControlInit",
    "XMMotorInit", "XMMotorControlInit", "ServoInit", "ServoSetAngle",
}
CONSUMERS = {"CANProcessRx", "USARTProcessRx", "USBTransportProcessRx", "USBTransportProcess", "MotorControlTask"}
ROLE_INITIALIZERS = {
    "single": {"ChassisInit", "GimbalInit", "ShootInit", "RobotCMDInit"},
    "chassis": {"ChassisInit"},
    "gimbal": {"GimbalInit", "ShootInit", "RobotCMDInit"},
}
ROLE_FORBIDDEN = {
    "single": set(),
    "chassis": {"GimbalInit", "ShootInit", "RobotCMDInit", "GimbalTask", "ShootTask", "RobotCMDTask"},
    "gimbal": {"ChassisInit", "ChassisTask"},
}
OPTIONAL_SOURCES = {
    "bsp/flash/bsp_flash.c", "modules/algorithm/RLS.c",
    "modules/bluetooth/HC05.c", "modules/ist8310/ist8310.c", "modules/oled/oled.c",
    "modules/standard_cmd/std_cmd.c", "modules/unicomm/unicomm.c",
    "modules/TFminiPlus/tfminiplus.c", "modules/motor/DMmotor/dmmotor.c",
    "modules/motor/DRmotor/drmotor.c", "modules/motor/XMmotor/xmmotor.c",
    "modules/motor/servo_motor/servo_motor.c", "modules/motor/step_motor/step_motor.c",
    "Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c",
    "docs/history/seasky/seasky_protocol.c",
}
# DSP 与启动汇编由 CMake 按固定平台依赖单独加入，不属于业务源码清单。
DSP_SOURCES = {
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/CommonTables/arm_common_tables.c",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/FastMathFunctions/arm_sin_f32.c",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/FastMathFunctions/arm_cos_f32.c",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/MatrixFunctions/arm_mat_init_f32.c",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/MatrixFunctions/arm_mat_add_f32.c",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/MatrixFunctions/arm_mat_sub_f32.c",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/MatrixFunctions/arm_mat_mult_f32.c",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/MatrixFunctions/arm_mat_trans_f32.c",
    "Middlewares/Third_Party/CMSIS-DSP-1.7.0/Source/MatrixFunctions/arm_mat_inverse_f32.c",
}
ASSEMBLY_SOURCES = {
    "startup_stm32h723xx.s",
    "Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s",
}
STARTUP_DIAGNOSTICS = {
    "RobotInit", "InitDiagnosticReportFailure", "InitDiagnosticHeartbeat",
    "init_diagnostic_snapshot",
}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--profile", choices=("bench_safe", "robot"), required=True)
    parser.add_argument("--role", choices=("single", "chassis", "gimbal"), default="single")
    parser.add_argument("--vision-transport", choices=("usb", "uart"), default="usb")
    parser.add_argument("--ui-disabled", action="store_true", help="确认裁判UI源码未进入本配置")
    parser.add_argument("--build-dir", type=Path, help="检查源码清单、编译数据库和 Ninja 链接输入")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--rc-diagnostics", action="store_true")
    parser.add_argument("--referee-diagnostics", action="store_true")
    parser.add_argument("--usb-diagnostics", action="store_true")
    args = parser.parse_args()
    if args.usb_diagnostics and (args.profile != "bench_safe" or args.role != "single" or
                                args.vision_transport != "usb" or "_usbtest_" not in args.elf.name or
                                args.rc_diagnostics or args.referee_diagnostics):
        parser.error("USB diagnostics require the separate named bench_safe/single/usb artifact")
    if args.referee_diagnostics and (args.profile != "bench_safe" or "_refdiag_" not in args.elf.name or args.rc_diagnostics):
        parser.error("referee diagnostics require the separate named bench_safe artifact")
    if args.rc_diagnostics and (args.profile != "bench_safe" or "_rcdiag_" not in args.elf.name):
        parser.error("remote diagnostics require the named bench_safe diagnostic artifact")
    if not args.elf.name.endswith("_" + args.profile + ".elf"):
        parser.error("ELF output name does not match the requested profile")
    if args.profile == "bench_safe" and args.role != "single":
        parser.error("bench_safe only supports the single board role")
    result = subprocess.run([args.nm, "-C", str(args.elf)], capture_output=True, text=True, check=True, timeout=15)
    symbols = {line.split()[-1] for line in result.stdout.splitlines() if len(line.split()) >= 3}
    functions = {line.split()[-1] for line in result.stdout.splitlines() if len(line.split()) >= 3 and line.split()[-2] in ("T", "t")}
    missing = CONSUMERS - functions
    # 当前工程用 ELF 文件名和 CMake 选择报告标识配置；旧的全局标记从未由源码定义。
    missing |= STARTUP_DIAGNOSTICS - symbols
    if args.profile == "robot":
        missing |= ROLE_INITIALIZERS[args.role] - functions
    if args.usb_diagnostics:
        missing |= {"USBTransportDiagnosticPoll", "USBTransportDiagnosticSuppressCompletion"} - functions
        missing |= {"usb_diagnostic_record"} - symbols
    elif "usb_diagnostic_record" in symbols:
        raise SystemExit("normal profile contains USB fault-injection diagnostics")
    legacy_usb = {"USBTransmit", "USBInit", "CDC_Transmit_HS", "CDCInitRxbufferNcallback"} & functions
    if legacy_usb:
        raise SystemExit(f"legacy borrowed USB interface remains linked: {sorted(legacy_usb)}")
    if args.rc_diagnostics:
        missing |= {"RemoteControlInit", "RemoteBenchInit", "RemoteBenchTask", "RemoteSafetyGesturesUpdate"} - functions
        missing |= {"remote_control_diag", "usart_rx_oversize_total"} - symbols
    if args.referee_diagnostics:
        missing |= {"RefereeBenchInit", "RefereeBenchTask", "RefereeInit", "RefereeSubmit", "referee_bench_diag"} - symbols
    elif "referee_bench_diag" in symbols:
        raise SystemExit("normal profile contains automatic referee diagnostic traffic")
    forbidden = ACTUATORS & functions if args.profile == "bench_safe" else ROLE_FORBIDDEN[args.role] & functions
    if missing or forbidden:
        raise SystemExit(f"profile check failed: missing={sorted(missing)}, forbidden={sorted(forbidden)}")

    if args.build_dir is not None:
        check_source_selection(args.build_dir.resolve(), args.profile, args.role,
                               args.vision_transport, args.ui_disabled)
    print(f"PASS: {args.profile}/{args.role} linked profile, receive consumers, and selected sources: {args.elf}")


def check_source_selection(build_dir: Path, profile: str, role: str,
                           vision_transport: str, ui_disabled: bool) -> None:
    """与配置报告、编译数据库和 Ninja 命令逐一比较已登记源码。"""
    report = build_dir / "mc02-selected-sources.txt"
    database = build_dir / "compile_commands.json"
    if not report.is_file() or not database.is_file():
        raise SystemExit("build directory is missing the selected-source report or compile database")
    lines = report.read_text(encoding="utf-8").splitlines()
    if (f"profile={profile}" not in lines or f"role={role}" not in lines
            or f"vision={vision_transport}" not in lines):
        raise SystemExit("selected-source report does not match the requested profile and role")
    selected = {line for line in lines if line.endswith(".c")}
    source_root = Path.cwd().resolve()
    compiled: set[str] = set()
    compiler_commands: list[str] = []
    for entry in json.loads(database.read_text(encoding="utf-8")):
        source = Path(entry["file"]).resolve()
        try:
            compiled.add(source.relative_to(source_root).as_posix())
        except ValueError:
            continue
        compiler_commands.append(entry.get("command", ""))
    expected_compiled = selected | DSP_SOURCES | ASSEMBLY_SOURCES
    missing = expected_compiled - compiled
    unexpected_compiled = compiled - expected_compiled
    unexpected = OPTIONAL_SOURCES & compiled
    role_sources = {
        "single": set(),
        "chassis": {"application/gimbal/gimbal.c", "application/shoot/shoot.c", "application/cmd/robot_cmd.c"},
        "gimbal": {"application/chassis/chassis.c", "application/chassis/chassis_motion.c"},
    }
    if profile == "bench_safe":
        unexpected |= {"application/chassis/chassis.c", "application/chassis/chassis_motion.c",
                       "application/gimbal/gimbal.c", "application/shoot/shoot.c",
                       "application/cmd/robot_cmd.c"} & compiled
    else:
        unexpected |= role_sources[role] & compiled
    selected_vision_macro = "VISION_USE_VCP" if vision_transport == "usb" else "VISION_USE_UART"
    rejected_vision_macro = "VISION_USE_UART" if vision_transport == "usb" else "VISION_USE_VCP"
    if not any(selected_vision_macro in command for command in compiler_commands):
        raise SystemExit(f"compile commands are missing the selected transport macro {selected_vision_macro}")
    if any(rejected_vision_macro in command for command in compiler_commands):
        raise SystemExit(f"compile commands contain the unselected transport macro {rejected_vision_macro}")
    if ui_disabled and ("modules/referee/referee_UI.c" in selected
                        or "modules/referee/referee_UI.c" in compiled):
        raise SystemExit("disabled referee UI source was selected or compiled")
    if missing or unexpected or unexpected_compiled:
        raise SystemExit(f"source selection failed: missing={sorted(missing)}, "
                         f"unexpected_optional={sorted(unexpected)}, "
                         f"unregistered={sorted(unexpected_compiled)}")

    target_line = next((line for line in lines if line.startswith("target=")), None)
    if target_line is None:
        raise SystemExit("selected-source report is missing the target name")
    target = target_line.removeprefix("target=")
    commands = subprocess.run(["ninja", "-C", str(build_dir), "-t", "commands", target],
                              capture_output=True, text=True, check=True, timeout=15).stdout
    for source in selected:
        object_path = f"{target}.dir/{source}.obj"
        if object_path not in commands:
            raise SystemExit(f"registered source is absent from the Ninja link command: {source}")

if __name__ == "__main__":
    main()
