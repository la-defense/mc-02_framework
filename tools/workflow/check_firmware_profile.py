#!/usr/bin/env python3
"""检查固件角色隔离，以及清单、编译数据库和实际链接输入的一致性。"""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess

ACTUATORS = {
    "ChassisInit", "GimbalInit", "ShootInit", "RobotCMDInit",
    "ChassisTask", "GimbalTask", "ShootTask", "RobotCMDTask",
    "DJIMotorInit", "DJIMotorControl", "LKMotorInit", "LKMotorControl",
    "HTMotorInit", "HTMotorControlInit", "DMMotorInit", "DMMotorControlInit",
    "XMMotorInit", "XMMotorControlInit", "ServoInit", "ServoSetAngle",
    "CommissioningChassisCmdInit", "CommissioningChassisCmdTask",
}
COMMISSIONING_CHASSIS_ACTUATORS = {
    "ChassisInit", "ChassisTask", "DJIMotorInit", "DJIMotorControl",
    "CommissioningChassisCmdInit", "CommissioningChassisCmdTask",
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
    "modules/algorithm/RLS.c",
    # 旧版板间格式没有来源时间和序号，只保留源码供历史查阅。
    "modules/can_comm/can_comm.c",
    "modules/bluetooth/HC05.c", "modules/ist8310/ist8310.c", "modules/oled/oled.c",
    "modules/unicomm/unicomm.c",
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
# 数值适配器编入DSP静态库，单独列出以免误认作厂商源码或漏登记的业务源码。
NUMERIC_ADAPTER_SOURCES = {
    "modules/algorithm/fast_trig_cmsis.c",  # 快速三角函数的CMSIS-DSP后端。
    "modules/algorithm/dsp_matrix.c",  # 矩阵接口的CMSIS-DSP后端。
}
# 姿态适配及可选上游实现各自使用独立的窄头路径。
ATTITUDE_ADAPTER_SOURCE = "modules/algorithm/attitude_filter.c"
FUSION_SOURCE = "Middlewares/Third_Party/Fusion/FusionAhrs.c"
# 这些源文件编入无芯片／RTOS 头路径的框架对象目标，仍属于车型源码清单。
PORTABLE_FRAMEWORK_SOURCES = {
    "modules/safety/robot_control_policy.c",  # 整车安全管理共用的三模式策略。
    "bsp/os/platform_startup_wait.c",  # 启动前的固定期限和检查次数。
    "bsp/os/platform_clock_guard.c",  # 主时钟配置的固定期限和停表兜底。
    "bsp/os/platform_periodic.c",  # 周期和完成时限只用标准 C，不依赖 RTOS 头。
    "application/init_diagnostic.c",  # 初始化结果及事件记录。
    "application/vehicle_health.c",  # 整车健康信息采集和发布。
    "application/vehicle_health_diag.c",  # 只读诊断应答。
    "application/robot.c",  # 整车启动和周期协调。
    "application/robot_def.c",  # 车型配置读取与参数校验。
    "application/config/module_plan.c",  # 模块依赖和启动结果。
    "application/config/vehicle_selection.c",  # 车型身份和启动计划。
    "application/config/sentinel_robot/sentinel_config.c",  # 哨兵静态配置。
    "modules/daemon/daemon.c",  # 来源状态和事件队列。
    "modules/daemon/daemon_event_task.c",  # 来源事件任务入口。
    "modules/message_center/message_center.c",  # 消息目录和有界队列。
    "modules/safety/robot_safety.c",  # 整车许可与故障管理。
    "modules/safety/can_recovery.c",  # 核心反馈与新命令恢复条件。
}
# 可选文件只有被车型清单选中时才进入独立框架目标。
OPTIONAL_PORTABLE_FRAMEWORK_SOURCES = {
    "modules/standard_cmd/std_cmd.c",  # 人工 SI 意图及来源期限。
    "modules/standard_cmd/std_cmd_remote.c",  # 遥控与键盘输入映射。
    "application/cmd/robot_manual_motion.c",  # 人工目标到旧应用单位的转换。
    "modules/algorithm/QuaternionEKF.c",  # 四元数姿态估计。
    "modules/algorithm/controller.c",  # 通用 PID 计算。
    "modules/algorithm/crc.c",  # 软件 CRC。
    "modules/algorithm/kalman_filter.c",  # 卡尔曼滤波。
    "modules/algorithm/math_utils.c",  # 标量数值工具。
    "modules/algorithm/sample_filter.c",  # 采样滤波。
    "modules/algorithm/vector3.c",  # 三维向量工具。
    "modules/master_machine/vision_control.c",  # 视觉控制命令换算。
    "modules/master_machine/vision_protocol.c",  # SP 帧解析。
    "modules/power/power_manager.c",  # 功率管理占位。
    "modules/lcd/lcd_font_cn.c",  # LCD 中文字模。
    "modules/lcd/lcd.c",  # LCD 面板协议及故障处理。
    "modules/can_comm/can_topic_wire.c",  # 帧编码与校验。
    "modules/can_comm/can_topic_transport.c",  # 分片传输。
    "modules/can_comm/can_topic_bridge.c",  # 话题桥接。
    "modules/can_comm/can_topic_link.c",  # 逻辑 CAN 端点绑定。
    "modules/can_comm/can_time_sync.c",  # 板间对时。
    "modules/time_sync/time_mapping.c",  # 跨设备时钟映射。
    "modules/referee/referee_UI.c",  # 裁判图形编码及发送状态。
    "modules/referee/referee_task.c",  # 裁判界面状态和字段更新。
    "modules/referee/referee_tx_budget.c",  # 裁判发送额度计算。
    "modules/referee/referee_bench.c",  # 禁热裁判诊断的有限发送流程。
    "modules/referee/rm_referee.c",  # 裁判协议解析及逻辑串口接入。
    "modules/remote/remote_control.c",  # 遥控输入解析及逻辑串口接入。
    "modules/remote/remote_safety.c",  # 遥控安全手势处理。
    "modules/master_machine/master_process.c",  # 上位机业务协议及传输状态。
    "modules/imu/ins_task.c",  # IMU 采样与姿态业务。
    "modules/imu/imu_heater.c",  # 温控策略及逻辑模拟资源。
    "modules/BMI088/bmi088.c",  # BMI088 采样与标定通过框架接口访问平台。
    "modules/remote/remote_bench.c",  # 裸板遥控诊断。
    "modules/master_machine/vision_protocol_v2.c",  # V2 协议编解码。
    "application/chassis/chassis_motion.c",  # 模式、坐标和轮速目标计算。
    "application/cmd/commissioning_chassis_cmd.c",  # 受限底盘专用人工命令。
    "application/config/commissioning_chassis_config.c",  # 受限底盘人工配置预检。
    "application/chassis/chassis_kinematics.c",  # 轮组几何和运动映射。
    "application/chassis/mecanum.c",  # 麦轮几何适配。
    "application/chassis/omni.c",  # 全向轮几何适配。
    "application/topics/chassis_topic_codec.c",  # 底盘话题编解码。
    "application/topics/robot_topics.c",  # 车型话题目录。
    "application/topics/robot_topics_diag.c",  # 话题统计查询。
    "application/topics/robot_can_bridge.c",  # 双板话题与 CAN 适配。
    "modules/motor/DJImotor/dji_motor.c",  # DJI 逻辑端点、控制和反馈。
    "modules/motor/LKmotor/LK9025.c",  # LK 逻辑端点和分组发送。
    "modules/motor/HTmotor/HT04.c",  # HT 逻辑端点和控制。
    "modules/motor/motor_task.c",  # 通信派发顺序及电机周期调用。
    "modules/motor/motor_feedback.c",  # 电机能力及反馈换算。
    "modules/motor/motor_pid.c",  # 电机 PID 周期控制。
    "modules/task_monitor/task_monitor.c",  # 任务期限和故障处理策略。
    "modules/alarm/robot_alarm.c",  # 告警协调与状态发布。
    "modules/alarm/robot_alarm_policy.c",  # 告警仲裁和灯声节奏。
    "modules/alarm/buzzer.c",  # 音符到频率的转换。
    "modules/alarm/led.c",  # RGB 输出状态和发送推进。
    "application/alarm_startup.c",  # 依据模块计划处理告警降级。
    "application/log_startup.c",  # 日志出口的核心／辅助分类。
    "application/robot_can_capacity.c",  # 注册前检查 CAN 过滤器容量。
    "application/robot_can_guard.c",  # 诊断后传播总线隔离影响。
    "application/robot_can_runtime.c",  # 按车型核心依赖通知运行期总线故障。
    "application/robot_can_motor_recovery.c",  # 核对车型核心电机的恢复反馈。
    "application/robot_can_recovery_coordinator.c",  # 组合单板人工输入和核心反馈的恢复检查。
    "application/robot_control_feedback.c",  # 逐台核对来源，按用途交付反馈期限。
    "application/robot_control_feedback_runtime.c",  # 启动时绑定并在周期中提交反馈。
    "application/robot_can_requirements.c",  # 统计车型声明的设备需求。
    "application/robot_param_guard.c",  # 参数维护与温控、监控的配合。
    "application/robot_power.c",  # 受控电源阶段及模块失败传播。
    "application/robot_task.c",  # 常驻任务的周期调用和监控。
    "application/ins_startup.c",  # 姿态初始化结果及核心／辅助故障处理。
    "application/robot_startup.c",  # 调度器运行后的启动协调和任务放行。
    "application/robot_monitor_plan.c",  # 车型依赖到任务监控的映射。
    "application/gimbal/gimbal.c",  # 云台命令和反馈。
    "application/shoot/shoot.c",  # 发射命令和拨弹控制。
    "application/chassis/chassis.c",  # 底盘命令和电机控制。
    "application/cmd/robot_cmd.c",  # 遥控、视觉与控制命令。
    "modules/lcd/lcd_buf.c",  # LCD 有界绘图及像素缓冲。
    "modules/lcd/lcd_key_input.c",  # LCD 按键事件处理。
    "modules/lcd/lcd_referee_rows.c",  # LCD 裁判数据显示。
    "modules/lcd/lcd_ui.c",  # LCD 页面状态展示。
    "modules/lcd/lcd_task.c",  # LCD 页面、按键和运行统计任务。
    "application/robot_task_create.c",  # 按车型配置创建任务。
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
    parser.add_argument("--profile", choices=("bench_safe", "commissioning", "robot"), required=True)
    parser.add_argument("--commissioning-target", choices=("observe", "chassis"), default="observe")
    parser.add_argument("--role", choices=("single", "chassis", "gimbal"), default="single")
    parser.add_argument("--vision-transport", choices=("usb", "uart"), default="usb")
    parser.add_argument("--ui-disabled", action="store_true", help="确认裁判UI源码未进入本配置")
    parser.add_argument("--build-dir", type=Path, help="检查源码清单、编译数据库和 Ninja 链接输入")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--rc-diagnostics", action="store_true")
    parser.add_argument("--referee-diagnostics", action="store_true")
    parser.add_argument("--usb-diagnostics", action="store_true")
    args = parser.parse_args()
    chassis_commissioning = args.profile == "commissioning" and args.commissioning_target == "chassis"
    if args.commissioning_target != "observe" and args.profile != "commissioning":
        parser.error("commissioning target requires the commissioning profile")
    if chassis_commissioning and args.rc_diagnostics:
        parser.error("remote observation cannot be included in the commissioning chassis profile")
    if args.usb_diagnostics and (args.profile != "bench_safe" or args.role != "single" or
                                args.vision_transport != "usb" or "_usbtest_" not in args.elf.name or
                                args.rc_diagnostics or args.referee_diagnostics):
        parser.error("USB diagnostics require the separate named bench_safe/single/usb artifact")
    if args.referee_diagnostics and (args.profile != "bench_safe" or "_refdiag_" not in args.elf.name or args.rc_diagnostics):
        parser.error("referee diagnostics require the separate named bench_safe artifact")
    if args.rc_diagnostics and (args.profile not in ("bench_safe", "commissioning") or "rcdiag" not in args.elf.name):
        parser.error("remote diagnostics require the named non-actuating diagnostic artifact")
    expected_suffix = ("_commissioning_chassis.elf" if chassis_commissioning else
                       "_commissioning_observe_rcdiag.elf" if args.rc_diagnostics
                       else "_commissioning_observe.elf") if args.profile == "commissioning" else "_" + args.profile + ".elf"
    if not args.elf.name.endswith(expected_suffix):
        parser.error("ELF output name does not match the requested profile")
    if args.profile in ("bench_safe", "commissioning") and args.role != "single":
        parser.error("non-actuating profiles only support the single board role")
    result = subprocess.run([args.nm, "-C", str(args.elf)], capture_output=True, text=True, check=True, timeout=15)
    symbols = {line.split()[-1] for line in result.stdout.splitlines() if len(line.split()) >= 3}
    functions = {line.split()[-1] for line in result.stdout.splitlines() if len(line.split()) >= 3 and line.split()[-2] in ("T", "t")}
    missing = CONSUMERS - functions
    if (args.profile == "robot" or chassis_commissioning) and "MotorTaskRunScheduled" in functions:
        missing.discard("MotorControlTask")  # 分开拍点的正式电机任务仍独占通信派发。
    if chassis_commissioning:
        missing -= {"USBTransportProcess", "USBTransportProcessRx"}  # 不启动或派发上位机链路。
        missing |= (COMMISSIONING_CHASSIS_ACTUATORS |
                    {"MotorTaskRunScheduled", "StartCHASSISTASK", "RobotCanRuntimeInstall"}) - functions
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
    if chassis_commissioning:
        forbidden = ((ACTUATORS - COMMISSIONING_CHASSIS_ACTUATORS) |
                     {"VisionInit", "VisionSend", "MX_USB_DEVICE_Init", "USBTransportProcess",
                      "USBTransportProcessRx", "RemoteBenchTask"}) & functions  # 仅链接人工底盘控制。
    elif args.profile in ("bench_safe", "commissioning"):
        forbidden = ACTUATORS & functions
    else:
        forbidden = (ROLE_FORBIDDEN[args.role] | {"CommissioningChassisCmdInit", "CommissioningChassisCmdTask"}) & functions
    if missing or forbidden:
        raise SystemExit(f"profile check failed: missing={sorted(missing)}, forbidden={sorted(forbidden)}")

    if args.build_dir is not None:
        check_source_selection(args.build_dir.resolve(), args.profile, args.role,
                               args.vision_transport, args.ui_disabled, args.commissioning_target)
    print(f"PASS: {args.profile}/{args.role} linked profile, receive consumers, and selected sources: {args.elf}")


def check_source_selection(build_dir: Path, profile: str, role: str,
                           vision_transport: str, ui_disabled: bool, commissioning_target: str = "observe") -> None:
    """与配置报告、编译数据库和 Ninja 命令逐一比较已登记源码。"""
    report = build_dir / "mc02-selected-sources.txt"
    database = build_dir / "compile_commands.json"
    if not report.is_file() or not database.is_file():
        raise SystemExit("build directory is missing the selected-source report or compile database")
    lines = report.read_text(encoding="utf-8").splitlines()
    if (f"profile={profile}" not in lines or f"role={role}" not in lines
            or f"vision={vision_transport}" not in lines):
        raise SystemExit("selected-source report does not match the requested profile and role")
    chassis_commissioning = profile == "commissioning" and commissioning_target == "chassis"
    if profile == "commissioning":
        # 同时检查生成身份与禁止输出的模块，不能只凭产物名称判断用途。
        config = (build_dir / "generated" / "mc02_build_config.h").read_text(encoding="utf-8")
        if f"commissioning_target={commissioning_target}" not in lines:
            raise SystemExit("commissioning target does not match the selected-source report")
        identity = "commission_base" if chassis_commissioning else "commissioning"
        if f'#define MC02_BUILD_ROLE_NAME "{identity}"' not in config:
            raise SystemExit("commissioning build identity is missing")
        motion = ("COMMAND", "CHASSIS", "MOTOR_CONTROL")
        disabled = ("GIMBAL", "SHOOTER", "HEATER") + (("VISION", "SUPERCAP", "UI") if chassis_commissioning else motion)
        for feature in disabled:
            if f"#define MC02_FEATURE_{feature} 0" not in config:
                raise SystemExit(f"commissioning unexpectedly enables {feature}")
        if chassis_commissioning:
            for feature in motion + ("REMOTE", "IMU", "COMM_DISPATCH"):
                if f"#define MC02_FEATURE_{feature} 1" not in config:
                    raise SystemExit(f"commissioning chassis unexpectedly disables {feature}")
        for rail in ("24V_1", "24V_2", "5V"):
            if f"#define MC02_POWER_{rail}_STAGE 0u" not in config:
                raise SystemExit(f"commissioning unexpectedly enables {rail}")
    selected = {line for line in lines if line.endswith(".c")}
    if not PORTABLE_FRAMEWORK_SOURCES <= selected:
        raise SystemExit("selected-source report is missing a portable framework source")
    portable_sources = PORTABLE_FRAMEWORK_SOURCES | (OPTIONAL_PORTABLE_FRAMEWORK_SOURCES & selected)
    source_root = Path.cwd().resolve()
    compiled: set[str] = set()
    selected_counts: dict[str, int] = {}  # 每份配置选中的源码必须恰好编译一次。
    compiler_commands: list[str] = []
    portable_counts: dict[str, int] = {}  # 每个框架源文件必须恰好编译一次。
    attitude_counts: dict[str, int] = {}  # 适配器和可选上游源码也必须恰好编译一次。
    attitude_sources = {ATTITUDE_ADAPTER_SOURCE} | ({FUSION_SOURCE} & selected)
    for entry in json.loads(database.read_text(encoding="utf-8")):
        source = Path(entry["file"]).resolve()
        try:
            relative = source.relative_to(source_root).as_posix()
        except ValueError:
            continue
        compiled.add(relative)
        if relative in selected:
            selected_counts[relative] = selected_counts.get(relative, 0) + 1
        compiler_commands.append(entry.get("command", ""))
        if relative in portable_sources:
            portable_counts[relative] = portable_counts.get(relative, 0) + 1
            check_portable_includes(entry, source_root, relative)
        if relative in attitude_sources:
            attitude_counts[relative] = attitude_counts.get(relative, 0) + 1
            check_attitude_includes(entry, source_root, relative)
    expected_compiled = selected | DSP_SOURCES | NUMERIC_ADAPTER_SOURCES | ASSEMBLY_SOURCES
    missing = expected_compiled - compiled
    unexpected_compiled = compiled - expected_compiled
    unexpected = OPTIONAL_SOURCES & compiled
    role_sources = {
        "single": set(),
        "chassis": {"application/gimbal/gimbal.c", "application/shoot/shoot.c", "application/cmd/robot_cmd.c"},
        "gimbal": {"application/chassis/chassis.c", "application/chassis/chassis_motion.c"},
    }
    commissioning_sources = {
        "application/cmd/commissioning_chassis_cmd.c",  # 只发布本地人工底盘命令。
        "application/config/commissioning_chassis_config.c",  # 只在受限底盘构建中检查人工参数。
    }
    if chassis_commissioning:
        required_chassis_sources = commissioning_sources | {
            "application/chassis/chassis.c",  # 四轮登记和底盘周期计算。
            "modules/motor/DJImotor/dji_motor.c",  # 受限轮组使用的 CAN 电机驱动。
        }
        missing |= required_chassis_sources - compiled
        unexpected |= (role_sources["chassis"] | {"modules/master_machine/master_process.c",
                        "modules/master_machine/vision_control.c", "modules/remote/remote_bench.c"}) & compiled
    elif profile in ("bench_safe", "commissioning"):
        unexpected |= (commissioning_sources | {"application/chassis/chassis.c", "application/chassis/chassis_motion.c",
                       "application/gimbal/gimbal.c", "application/shoot/shoot.c",
                       "application/cmd/robot_cmd.c"}) & compiled
    else:
        unexpected |= (role_sources[role] | commissioning_sources) & compiled
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
    if any(selected_counts.get(source, 0) != 1 for source in selected):
        raise SystemExit(f"selected sources must compile exactly once: {selected_counts}")
    if any(portable_counts.get(source, 0) != 1 for source in portable_sources):
        raise SystemExit(f"portable framework sources must compile exactly once: {portable_counts}")
    if any(attitude_counts.get(source, 0) != 1 for source in attitude_sources):
        raise SystemExit(f"attitude sources must compile exactly once: {attitude_counts}")

    target_line = next((line for line in lines if line.startswith("target=")), None)
    if target_line is None:
        raise SystemExit("selected-source report is missing the target name")
    target = target_line.removeprefix("target=")
    commands = subprocess.run(["ninja", "-C", str(build_dir), "-t", "commands", target],
                              capture_output=True, text=True, check=True, timeout=15).stdout
    link_command = find_link_command(commands, target)
    link_command = expand_link_response(link_command, build_dir)
    for source in selected:
        # 核对每份源码所属目标以及它是否真正进入最终链接。
        owner = source_target(source, portable_sources, target)
        object_path = f"{owner}.dir/{source}.obj"
        if object_path not in link_command:
            raise SystemExit(f"registered source is absent from the Ninja link command: {source}")
    for source in portable_sources:
        # 检查实际预处理依赖，避免相对路径或绝对路径包含绕过编译选项检查。
        object_path = f"CMakeFiles/mc02_portable_framework.dir/{source}.obj"
        deps = subprocess.run(["ninja", "-C", str(build_dir), "-t", "deps", object_path],
                              capture_output=True, text=True, check=True, timeout=15).stdout
        check_portable_dependencies(deps, source_root, build_dir, source)
    for source in attitude_sources:
        # 分别检查适配器和上游实现的真实头依赖，不让芯片或 RTOS 头从相对路径进入。
        owner = "mc02_attitude_adapter" if source == ATTITUDE_ADAPTER_SOURCE else "mc02_fusion_backend"
        object_path = f"CMakeFiles/{owner}.dir/{source}.obj"
        deps = subprocess.run(["ninja", "-C", str(build_dir), "-t", "deps", object_path],
                              capture_output=True, text=True, check=True, timeout=15).stdout
        check_attitude_dependencies(deps, source_root, build_dir, source)


def source_target(source: str, portable_sources: set[str], executable: str) -> str:
    """按 CMake 的分类规则查询源码应归属的目标。"""
    if source in portable_sources:
        return "mc02_portable_framework"
    if source == ATTITUDE_ADAPTER_SOURCE:
        return "mc02_attitude_adapter"
    if source == FUSION_SOURCE:
        return "mc02_fusion_backend"
    if source.startswith(("Core/", "USB_DEVICE/", "bsp/")) or source in {
        "application/vehicle_health_port_stm32.c",  # 整车健康的 STM32 来源适配。
        "application/chassis/chassis_supercap_legacy.c",  # 尚未选型的旧电容板级绑定。
        "modules/super_cap/super_cap.c",  # 尚未验证的旧电容板级协议实现。
    }:
        return "mc02_board_backend"
    if source.startswith(("Drivers/", "Middlewares/")):
        return "mc02_vendor_system"
    raise SystemExit(f"selected source has no isolated target: {source}")


def check_attitude_includes(entry: dict, source_root: Path, source: str) -> None:
    """只允许姿态适配或上游实现使用自身所需的头目录。"""
    allowed = {source_root / "Middlewares/Third_Party/Fusion",
               source_root / "bsp/log"}  # 格式诊断选项需要的公共路径。
    if source == ATTITUDE_ADAPTER_SOURCE:
        allowed.add(source_root / "modules/algorithm")
    arguments = entry.get("arguments") or shlex.split(entry.get("command", ""))
    for index, argument in enumerate(arguments):
        if argument.startswith(("-include", "-imacros", "-iquote", "-idirafter")):
            raise SystemExit(f"attitude source {source} uses unsupported include option: {argument}")
        if argument in ("-I", "-isystem"):
            if index + 1 >= len(arguments):
                raise SystemExit(f"missing include path for {source}")
            directory = arguments[index + 1]
        elif argument.startswith("-I") and len(argument) > 2:
            directory = argument[2:]
        elif argument.startswith("-isystem") and len(argument) > len("-isystem"):
            directory = argument[len("-isystem"):]
        else:
            continue
        resolved = (Path(entry["directory"]) / directory).resolve()
        if resolved not in allowed:
            raise SystemExit(f"attitude source {source} inherits unsupported include: {resolved}")


def check_attitude_dependencies(deps: str, source_root: Path, build_dir: Path, source: str) -> None:
    """根据实际依赖拒绝姿态后端以相对路径包含芯片或 RTOS 头。"""
    lines = deps.splitlines()
    if not lines or "#deps" not in lines[0] or "(VALID)" not in lines[0]:
        raise SystemExit(f"attitude source {source} has no valid Ninja dependencies")
    allowed = [source_root / "Middlewares/Third_Party/Fusion"]
    if source == ATTITUDE_ADAPTER_SOURCE:
        allowed.append(source_root / "modules/algorithm")
    for line in lines[1:]:
        if not line.strip():
            continue
        dependency = (build_dir / line.strip()).resolve()
        if dependency.is_relative_to(source_root) and not any(
                dependency.is_relative_to(directory) for directory in allowed):
            raise SystemExit(f"attitude source {source} uses unsupported project header: {dependency}")


def check_portable_includes(entry: dict, source_root: Path, source: str) -> None:
    """只允许框架公开头和日志格式检查头进入可移植源文件的编译路径。"""
    allowed = {source_root / "modules/daemon", source_root / "modules/message_center", source_root / "modules/safety",
               source_root / "bsp/os", source_root / "bsp/log", source_root / "bsp/can",
               source_root / "modules/can_comm", source_root / "modules/time_sync",
               source_root / "modules/algorithm", source_root / "modules/referee",
               source_root / "modules", source_root / "modules/motor", source_root / "modules/motor/DJImotor", source_root / "modules/motor/LKmotor", source_root / "modules/motor/HTmotor", source_root / "modules/lcd",
               source_root / "bsp/usart", source_root / "modules/remote", source_root / "modules/standard_cmd",
               source_root / "modules/master_machine", source_root / "bsp/usb",
               source_root / "modules/imu", source_root / "modules/BMI088",
               source_root / "modules/task_monitor", source_root / "modules/alarm", source_root / "bsp/board", source_root / "bsp/dwt",
               source_root / "bsp/adc", source_root / "bsp/observation", source_root / "bsp/crash",
               source_root / "bsp", source_root / "bsp/param", source_root / "bsp/watchdog",
               source_root / "bsp/reset_recovery", source_root / "application/chassis",
               source_root / "application/config/sentinel_robot",
               source_root / "application/gimbal", source_root / "application/shoot", source_root / "application/cmd",
               source_root / "bsp/safety", source_root / "application",
               source_root / "application/config",
               Path(entry["directory"]).resolve() / "generated"}
    arguments = entry.get("arguments") or shlex.split(entry.get("command", ""))
    for index, argument in enumerate(arguments):
        # 同时处理 -Ipath、-I path 和 -isystem path，防止换一种写法绕过检查。
        if argument.startswith(("-include", "-imacros", "-iquote", "-idirafter")):
            raise SystemExit(f"portable framework source {source} uses unsupported include option: {argument}")
        if argument in ("-I", "-isystem"):
            if index + 1 >= len(arguments):
                raise SystemExit(f"missing include path for {source}")
            directory = arguments[index + 1]
        elif argument.startswith("-I") and len(argument) > 2:
            directory = argument[2:]
        elif argument.startswith("-isystem") and len(argument) > len("-isystem"):
            directory = argument[len("-isystem"):]
        else:
            continue
        resolved = (Path(entry["directory"]) / directory).resolve()
        if resolved not in allowed:
            raise SystemExit(f"portable framework source {source} inherits unsupported include: {resolved}")


def find_link_command(commands: str, target: str) -> str:
    """从 Ninja 命令中找出生成指定 ELF 的唯一链接命令。"""
    matches = []
    for line in commands.splitlines():
        arguments = shlex.split(line)
        # 编译命令的输出是 .obj，只有最终链接命令输出目标 ELF。
        direct = any(argument == "-o" and index + 1 < len(arguments) and
                     Path(arguments[index + 1]).name == target
                     for index, argument in enumerate(arguments))
        # Windows Ninja 把整个链接命令包进 cmd.exe 的引号，shlex 会把内部参数当成一个字符串。
        wrapped = any(Path((match.group(1) or match.group(2)).replace("\\", "/")).name == target
                      for match in re.finditer(r'(?:^|\s)-o\s+(?:"([^"]+)"|([^\s"]+))', line))
        if direct or wrapped:
            matches.append(line)
    if len(matches) != 1:
        raise SystemExit(f"expected one link command for {target}, got {len(matches)}")
    return matches[0]


def expand_link_response(command: str, build_dir: Path) -> str:
    """读取链接命令实际使用的响应文件；缺失或越出构建目录时拒绝检查。"""
    matches = list(re.finditer(r'(?:^|\s)@(?:"([^"]+)"|([^\s"]+))', command))
    if not matches:
        return command  # 没有响应文件时继续检查原始命令。
    if len(matches) != 1:
        raise SystemExit("link command must reference at most one response file")
    name = (matches[0].group(1) or matches[0].group(2)).replace("\\", "/")
    response = (build_dir / name).resolve()  # 仅允许读取当前构建目录内的实际链接输入。
    if not response.is_relative_to(build_dir.resolve()) or not response.is_file():
        raise SystemExit(f"link response file missing or outside build directory: {name}")
    contents = response.read_text(encoding="utf-8")
    if re.search(r'(?:^|\s)@', contents):
        raise SystemExit("nested link response files are not supported")
    return command.replace("\\", "/") + "\n" + contents.replace("\\", "/")


def check_portable_dependencies(deps: str, source_root: Path, build_dir: Path, source: str) -> None:
    """检查 Ninja 记录的实际头依赖；标准库可以来自工程目录之外。"""
    lines = deps.splitlines()
    if not lines or "#deps" not in lines[0] or "(VALID)" not in lines[0]:
        raise SystemExit(f"portable framework source {source} has no valid Ninja dependencies")
    allowed = (source_root / "modules/daemon", source_root / "modules/message_center", source_root / "modules/safety",
               source_root / "bsp/os", source_root / "bsp/log", source_root / "bsp/can",
               source_root / "modules/can_comm", source_root / "modules/time_sync",
               source_root / "modules/algorithm", source_root / "modules/referee",
               source_root / "modules/motor", source_root / "modules/lcd",
               source_root / "bsp/usart", source_root / "modules/remote",
               source_root / "modules/master_machine", source_root / "bsp/usb",
               source_root / "modules/imu", source_root / "modules/task_monitor",
               source_root / "modules/alarm")
    for line in lines[1:]:
        # 依赖可能为相对构建目录的路径，也可能为绝对路径。
        if not line.strip():
            continue
        dependency = (build_dir / line.strip()).resolve()
        # 限定公开声明文件，禁止借用同目录下的硬件实现头。
        public_files = {source_root / "bsp/board/platform_resources.h",
                        source_root / "bsp/board/platform_analog.h",
                        source_root / "bsp/board/platform_buzzer.h",
                        source_root / "bsp/board/platform_rgb.h",
                        source_root / "bsp/board/platform_lcd.h",  # 面板传输和引脚公开接口。
                        source_root / "application/topics/robot_topics.h",
                        source_root / "application/topics/robot_can_bridge_task.h",
                        source_root / "application/topics/robot_can_bridge.h",
                        source_root / "application/topics/chassis_topic_codec.h",  # 底盘话题字节格式。
                        source_root / "application/topics/robot_topics_diag.h",
                        source_root / "application/config/build_selection.h",
                        source_root / "application/config/vehicle_selection.h",
                        source_root / "application/config/module_plan.h",
                        source_root / "application/robot_task_create.h",
                        source_root / "application/ins_startup.h",  # 姿态启动结果供协调任务使用。
                        source_root / "application/robot_startup.h",  # 启动协调的公开入口。
                        source_root / "application/robot_monitor_plan.h",  # 车型任务监控配置入口。
                        source_root / "application/robot_task.h",
                        source_root / "application/robot.h",
                        source_root / "application/robot_control_tasks.h",
                        source_root / "application/robot_def.h",
                        source_root / "application/config/config.h",
                        source_root / "modules/motor/motor_task.h",  # 周期配置与派发接口不依赖厂商头。
                        source_root / "application/config/chassis_task_config.h",  # 只含底盘周期与栈参数。
                        source_root / "application/config/sentinel_robot/sentinel_config.h",
                        source_root / "application/config/shoot_config_types.h",
                        source_root / "application/chassis/chassis_motion.h",
                        source_root / "application/chassis/chassis.h",
                        source_root / "application/chassis/chassis_supercap_legacy.h",
                        source_root / "application/chassis/chassis_kinematics.h",
                        source_root / "application/chassis/omni.h",
                        source_root / "application/chassis/mecanum.h",
                        source_root / "application/cmd/robot_cmd.h",
                        source_root / "application/cmd/commissioning_chassis_cmd.h",  # 受限人工底盘发布接口。
                        source_root / "application/cmd/robot_manual_motion.h",  # 人工目标的单位转换接口。
                        source_root / "modules/standard_cmd/std_cmd.h",  # 不依赖平台的人工意图定义。
                        source_root / "modules/standard_cmd/std_cmd_remote.h",  # 遥控输入映射接口。
                        source_root / "application/gimbal/gimbal.h",
                        source_root / "application/shoot/shoot.h",
                        source_root / "application/topics/robot_can_bridge_config.h",
                        source_root / "application/init_diagnostic.h",
                        source_root / "application/alarm_startup.h",
                        source_root / "application/log_startup.h",  # 无 HAL 类型的日志启动结果。
                        source_root / "application/robot_can_capacity.h",  # 纯计算容量计划。
                        source_root / "application/robot_can_guard.h",  # 模块与逻辑总线依赖。
                        source_root / "application/robot_can_runtime.h",  # 运行期核心总线故障入口。
                        source_root / "application/robot_can_motor_recovery.h",  # 核心电机恢复身份匹配。
                        source_root / "application/robot_can_recovery_coordinator.h",  # 单板人工恢复协调接口。
                        source_root / "application/robot_control_feedback.h",  # 按用途汇总反馈。
                        source_root / "application/robot_control_feedback_runtime.h",  # 反馈绑定和周期提交。
                        source_root / "application/robot_can_requirements.h",  # 车型设备需求入口。
                        source_root / "application/robot_param_guard.h",  # 维护操作公开回调。
                        source_root / "application/robot_power.h",  # 模块电源阶段声明。
                        source_root / "application/vehicle_health.h",
                        source_root / "application/vehicle_health_source.h",
                        source_root / "modules/motor/HTmotor/ht_motor_task.h",
                        source_root / source,
                        source_root / "modules/general_def.h",
                        source_root / "modules/power/power_manager.h",  # 当前功率占位模块的公开接口。
                        source_root / "modules/BMI088/bmi088_data.h",
                        source_root / "modules/BMI088/bmi088_calibration.h",
                        source_root / "modules/BMI088/bmi088.h",  # 传感器接口不包含板级类型。
                        source_root / "bsp/spi/platform_spi_blocking.h",  # 同步 SPI 设备接口。
                        source_root / "bsp/board/platform_power.h",  # 只含逻辑电源编号与框架函数。
                        source_root / "bsp/board/platform_feature_startup.h",  # 已选外设的启动结果接口。
                        source_root / "bsp/board/platform_clock_startup.h",  # 早期固定期限和时钟失败接口。
                        source_root / "bsp/pwm/platform_pwm_legacy.h",  # 旧 PWM 的不透明设备接口。
                        source_root / "bsp/adc/bsp_adc.h",
                        source_root / "bsp/observation/bsp_observation.h",
                        source_root / "bsp/safety/bsp_safety.h",
                        source_root / "bsp/bsp_init.h",
                        source_root / "bsp/param/bsp_param.h",
                        source_root / "bsp/watchdog/bsp_watchdog.h",
                        source_root / "bsp/reset_recovery/bsp_reset_recovery.h",
                        source_root / "bsp/reset_recovery/reset_recovery_policy.h",
                        source_root / "bsp/crash/bsp_crash.h",
                        source_root / "bsp/crash/bsp_crash_record.h",
                        source_root / "bsp/dwt/platform_probe.h",
                        build_dir.resolve() / "generated/mc02_build_config.h"}
        if source == "modules/BMI088/bmi088.c":
            # 寄存器和内部温度换算只供 BMI088 驱动使用，不放宽其他框架源码的依赖。
            public_files.update({source_root / "modules/BMI088/bmi088_regNdef.h",
                                 source_root / "modules/BMI088/bmi088_temperature.h"})
        if dependency.is_relative_to(source_root) and dependency not in public_files and not any(
                dependency.is_relative_to(directory) for directory in allowed):
            raise SystemExit(f"portable framework source {source} uses unsupported project header: {dependency}")

if __name__ == "__main__":
    main()
