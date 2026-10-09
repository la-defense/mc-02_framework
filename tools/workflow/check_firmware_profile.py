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

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--profile", choices=("bench_safe", "robot"), required=True)
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    args = parser.parse_args()
    if not args.elf.name.endswith("_" + args.profile + ".elf"):
        parser.error("ELF output name does not match the requested profile")
    result = subprocess.run([args.nm, "-C", str(args.elf)], capture_output=True, text=True, check=True, timeout=15)
    symbols = {line.split()[-1] for line in result.stdout.splitlines() if len(line.split()) >= 3}
    functions = {line.split()[-1] for line in result.stdout.splitlines() if len(line.split()) >= 3 and line.split()[-2] in ("T", "t")}
    missing = CONSUMERS - functions
    if "mc02_build_profile" not in symbols:
        missing.add("mc02_build_profile")
    if args.profile == "robot":
        missing |= ROBOT_INITIALIZERS - functions
    forbidden = ACTUATORS & functions if args.profile == "bench_safe" else set()
    if missing or forbidden:
        raise SystemExit(f"profile check failed: missing={sorted(missing)}, forbidden={sorted(forbidden)}")
    print(f"PASS: {args.profile} linked profile and receive consumers: {args.elf}")

if __name__ == "__main__":
    main()
