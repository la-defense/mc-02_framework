#!/usr/bin/env python3
"""Check the linked firmware profile, including actuator isolation."""
import argparse
from pathlib import Path
import subprocess

ACTUATORS = {
    "ChassisInit", "GimbalInit", "ShootInit", "RobotCMDInit",
    "ChassisTask", "GimbalTask", "ShootTask", "RobotCMDTask",
    "DJIMotorInit", "DJIMotorControl", "LKMotorInit", "LKMotorControl",
    "HTMotorInit", "HTMotorControlInit", "DMMotorInit", "DMMotorControlInit",
    "XMMotorInit", "XMMotorControlInit", "ServoInit", "ServoSetAngle",
}
CONSUMERS = {"CANProcessRx", "USARTProcessRx", "VCPProcessRx", "MotorControlTask"}
ROBOT_INITIALIZERS = {"ChassisInit", "GimbalInit", "ShootInit"}
STARTUP_DIAGNOSTICS = {
    "RobotInit", "InitDiagnosticReportFailure", "InitDiagnosticHeartbeat",
    "init_diagnostic_snapshot",
}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--profile", choices=("bench_safe", "robot"), required=True)
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--rc-diagnostics", action="store_true")
    args = parser.parse_args()
    if args.rc_diagnostics and (args.profile != "bench_safe" or "_rcdiag_" not in args.elf.name):
        parser.error("remote diagnostics require the named bench_safe diagnostic artifact")
    if not args.elf.name.endswith("_" + args.profile + ".elf"):
        parser.error("ELF output name does not match the requested profile")
    result = subprocess.run([args.nm, "-C", str(args.elf)], capture_output=True, text=True, check=True, timeout=15)
    symbols = {line.split()[-1] for line in result.stdout.splitlines() if len(line.split()) >= 3}
    functions = {line.split()[-1] for line in result.stdout.splitlines() if len(line.split()) >= 3 and line.split()[-2] in ("T", "t")}
    missing = CONSUMERS - functions
    if "mc02_build_profile" not in symbols:
        missing.add("mc02_build_profile")
    missing |= STARTUP_DIAGNOSTICS - symbols
    if args.profile == "robot":
        missing |= ROBOT_INITIALIZERS - functions
    if args.rc_diagnostics:
        missing |= {"RemoteControlInit", "RemoteBenchInit", "RemoteBenchTask", "RemoteSafetyGesturesUpdate"} - functions
        missing |= {"remote_control_diag", "usart_rx_oversize_total"} - symbols
    forbidden = ACTUATORS & functions if args.profile == "bench_safe" else set()
    if missing or forbidden:
        raise SystemExit(f"profile check failed: missing={sorted(missing)}, forbidden={sorted(forbidden)}")
    print(f"PASS: {args.profile} linked profile and receive consumers: {args.elf}")

if __name__ == "__main__":
    main()
