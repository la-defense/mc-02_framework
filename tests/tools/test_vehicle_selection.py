"""核对车型选择、模块映射和禁用模块的生产源码隔离。"""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


class VehicleSelectionTest(unittest.TestCase):
    def test_can_diagnostics_run_before_periodic_tasks(self):
        # 一次性模式切换必须先于任务创建，诊断结束后才可能出现并发派发。
        startup = (ROOT / "application/robot_startup.c").read_text(encoding="utf-8")
        task = (ROOT / "application/robot_task.c").read_text(encoding="utf-8")
        self.assertLess(startup.index("RobotInitializeApplications()"), startup.index("RobotRunStartupDiagnostics()"))
        self.assertLess(startup.index("RobotRunStartupDiagnostics()"), startup.index("PlatformTaskBatchBegin()"))
        diagnostic = task.split("void RobotRunStartupDiagnostics(void)", 1)[1].split("StartDAEMONTASK", 1)[0]
        self.assertIn("PlatformCanRunLoopbackDiagnostic()", diagnostic)
        self.assertIn("CANRunDjiFeedbackInjectTest()", diagnostic)
        daemon = task.split("void StartDAEMONTASK(void const *argument)", 1)[1]
        self.assertNotIn("PlatformCanRunLoopbackDiagnostic()", daemon)
        self.assertNotIn("CANRunDjiFeedbackInjectTest()", daemon)

    def test_usb_fallback_follows_application_registration(self):
        # 默认任务必须先让业务注册回调，空配置只能作为未选业务时的兜底。
        source = (ROOT / "Core/Src/freertos.c").read_bytes()
        body = source.split(b"void StartDefaultTask(void const * argument)", 1)[1]
        self.assertLess(body.index(b"RobotStartupRun()"), body.index(b"MX_USB_DEVICE_Init()"))
        self.assertLess(body.index(b"MX_USB_DEVICE_Init()"), body.index(b"USBTransportProcess()"))

    def test_early_watchdog_precedes_peripheral_setup(self):
        # 入口保留旧编码注释，按字节检查关键调用，不转换或改写源文件。
        main = (ROOT / "Core/Src/main.c").read_bytes()
        body = main.split(b"int main(void)", 1)[1].split(b"void SystemClock_Config(void)", 1)[0]
        self.assertLess(body.index(b"SystemClock_Config();"), body.index(b"BSPStartupProtectionInit()"))
        self.assertLess(body.index(b"BSPStartupProtectionInit()"), body.index(b"MX_GPIO_Init();"))
        self.assertRegex(body, rb"if\s*\(!BSPStartupProtectionInit\(\)\)\s*"
                               rb"PlatformClockStartupFail\(PLATFORM_CLOCK_FAILURE_WATCHDOG\);")
        self.assertLess(body.index(b"PlatformPeripheralStartupFinish()"), body.index(b"BSP_WatchdogFeed();"))

    @staticmethod
    def _evaluate_feature_disabled_condition(expression, feature_values):
        # 未知平台条件按可达处理，避免无关宏掩盖未保护调用。
        expression = re.sub(
            r"defined\s*\(?\s*([A-Za-z_]\w*)\s*\)?",
            lambda match: "1",
            expression,
        )
        values = {
            "MC02_VISION_PROTOCOL_V2": "1",
        }
        values.update(feature_values)
        expression = re.sub(
            r"\b[A-Za-z_]\w*\b",
            lambda match: values.get(match.group(0), "1"),
            expression,
        )
        tokens = re.findall(r"\d+|&&|\|\||==|!=|>=|<=|>|<|!|\(|\)", expression)
        if "".join(tokens) != re.sub(r"\s+", "", expression):
            raise AssertionError(f"unsupported preprocessor condition: {expression}")

        position = 0

        def parse_primary():
            nonlocal position
            if tokens[position] == "(":
                position += 1
                result = parse_or()
                if tokens[position] != ")":
                    raise AssertionError(f"unclosed preprocessor condition: {expression}")
                position += 1
                return result
            value = int(tokens[position])
            position += 1
            return value

        def parse_comparison():
            nonlocal position
            left = parse_primary()
            if position < len(tokens) and tokens[position] in ("==", "!=", ">=", "<=", ">", "<"):
                operator = tokens[position]
                position += 1
                right = parse_primary()
                return {
                    "==": left == right,
                    "!=": left != right,
                    ">=": left >= right,
                    "<=": left <= right,
                    ">": left > right,
                    "<": left < right,
                }[operator]
            return left

        def parse_unary():
            nonlocal position
            if position < len(tokens) and tokens[position] == "!":
                position += 1
                return not parse_unary()
            return parse_comparison()

        def parse_and():
            nonlocal position
            result = bool(parse_unary())
            while position < len(tokens) and tokens[position] == "&&":
                position += 1
                right = bool(parse_unary())
                result = result and right
            return result

        def parse_or():
            nonlocal position
            result = bool(parse_and())
            while position < len(tokens) and tokens[position] == "||":
                position += 1
                right = bool(parse_and())
                result = result or right
            return result

        result = parse_or()
        if position != len(tokens):
            raise AssertionError(f"unparsed preprocessor condition: {expression}")
        return bool(result)

    def assert_feature_calls_disabled(self, relative_path, call_pattern, disabled_feature):
        source = (ROOT / relative_path).read_text(encoding="utf-8")
        feature_values = {
            "MC02_FEATURE_VISION": "1",
            "MC02_FEATURE_HEATER": "1",
        }
        feature_values[f"MC02_FEATURE_{disabled_feature}"] = "0"
        conditions = []
        active = True
        for line in source.splitlines():
            directive = re.match(r"\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)", line)
            if directive:
                kind, expression = directive.groups()
                if kind in ("if", "ifdef", "ifndef"):
                    parent_active = active
                    if kind == "ifdef":
                        expression = f"defined({expression.strip()})"
                    elif kind == "ifndef":
                        expression = f"!defined({expression.strip()})"
                    branch_active = self._evaluate_feature_disabled_condition(expression, feature_values)
                    conditions.append([parent_active, branch_active, parent_active and branch_active])
                    active = parent_active and branch_active
                elif kind == "elif":
                    parent_active, branch_taken, _ = conditions[-1]
                    branch_active = (not branch_taken and
                                     self._evaluate_feature_disabled_condition(expression, feature_values))
                    conditions[-1][1] = branch_taken or branch_active
                    conditions[-1][2] = parent_active and branch_active
                    active = conditions[-1][2]
                elif kind == "else":
                    parent_active, branch_taken, _ = conditions[-1]
                    branch_active = not branch_taken
                    conditions[-1][1] = True
                    conditions[-1][2] = parent_active and branch_active
                    active = conditions[-1][2]
                elif conditions:
                    conditions.pop()
                    active = conditions[-1][2] if conditions else True

            stripped = line.lstrip()
            if (not stripped.startswith(("//", "/*", "*")) and
                    re.search(call_pattern, line)):
                self.assertFalse(active, f"{relative_path}: {line.strip()}")

    def run_case(self, body):
        with tempfile.TemporaryDirectory(prefix="mc02-0054-") as directory:
            script = pathlib.Path(directory) / "check.cmake"
            script.write_text(f'include("{ROOT.as_posix()}/cmake/mc02_vehicle_selection.cmake")\n' + body,
                              encoding="utf-8")
            return subprocess.run(["cmake", "-P", str(script)], text=True,
                                  capture_output=True, timeout=15)

    def test_identity_and_implementations(self):
        for vehicle, chassis, driver in [("hero", "omni", "dji"),
                                         ("unknown", "omni", "dji"),
                                         ("sentinel", "steering", "dji"),
                                         ("sentinel", "wheel_leg", "dji"),
                                         ("sentinel", "half_steering", "dji"),
                                         ("sentinel", "omni", "dr"),
                                         ("sentinel", "omni", "step")]:
            with self.subTest(vehicle=vehicle, chassis=chassis, driver=driver):
                result = self.run_case(f'mc02_validate_vehicle_identity({vehicle} stm32h723 mc02 freertos {chassis} {driver})\n')
                self.assertNotEqual(result.returncode, 0)
        result = self.run_case('mc02_validate_vehicle_identity(sentinel stm32h723 mc02 freertos omni dji)\n')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_adc_resource_follows_users(self):
        # 只要本板需要 VIN、LCD、温控或裸板诊断，就必须准备 ADC。
        cases = [
            ("robot", "FALSE", "0", "0", "0", "0"),  # 生产板没有使用者。
            ("robot", "TRUE", "0", "0", "0", "1"),  # 车型启用 VIN 诊断。
            ("robot", "FALSE", "1", "0", "0", "1"),  # LCD 按键需要 ADC。
            ("robot", "FALSE", "0", "1", "0", "1"),  # 温控电压保护需要 ADC。
            ("robot", "FALSE", "0", "0", "1", "1"),  # 显示诊断请求 ADC。
            ("bench_safe", "FALSE", "0", "0", "0", "1"),  # 裸板保留现有诊断。
        ]
        for profile, vin, lcd, heater, diag, expected in cases:
            with self.subTest(profile=profile, vin=vin, lcd=lcd, heater=heater, diag=diag):
                result = self.run_case(
                    f'mc02_select_adc_resource(actual {profile} {vin} {lcd} {heater} {diag})\n'
                    f'if(NOT "${{actual}}" STREQUAL "{expected}")\n'
                    '  message(FATAL_ERROR "unexpected ADC selection")\nendif()\n')
                self.assertEqual(result.returncode, 0, result.stderr)
        result = self.run_case('mc02_select_adc_resource(actual robot UNKNOWN 0 0 0)\n')
        self.assertNotEqual(result.returncode, 0)  # 不接受未声明或含糊的 VIN 选项。

    def test_unadapted_dm_is_rejected_before_firmware_selection(self):
        # DM 旧驱动尚未进入框架源码清单，不能被能力检查误认为可用。
        result = self.run_case('mc02_validate_vehicle_identity(sentinel stm32h723 mc02 freertos omni dm)\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Motor driver dm', result.stderr)
        for driver in ('dji', 'lk', 'ht'):
            with self.subTest(driver=driver):
                # 已经适配的三个驱动仍可通过身份能力检查。
                accepted = self.run_case(f'mc02_validate_vehicle_identity(sentinel stm32h723 mc02 freertos omni {driver})\n')
                self.assertEqual(accepted.returncode, 0, accepted.stderr)

    def test_module_order_matches_cmake_and_health_mapping(self):
        plan_header = (ROOT / "application/config/module_plan.h").read_text(encoding="utf-8")
        enum_body = re.search(r"typedef enum\s*\{(.*?)ROBOT_MODULE_COUNT", plan_header, re.S)
        self.assertIsNotNone(enum_body)
        c_names = re.findall(r"ROBOT_MODULE_([A-Z_]+)", enum_body.group(1))

        cmake_selection = (ROOT / "cmake/mc02_vehicle_selection.cmake").read_text(encoding="utf-8")
        cmake_body = re.search(r"set\(MC02_MODULE_NAMES\s+(.*?)\)", cmake_selection, re.S)
        self.assertIsNotNone(cmake_body)
        cmake_names = re.findall(r"[A-Z][A-Z_]+", cmake_body.group(1))
        self.assertEqual(c_names, cmake_names)

        health_port = (ROOT / "application/vehicle_health_port_stm32.c").read_text(encoding="utf-8")
        health_body = re.search(
            r"static const VehicleHealthModule_t health_module\[ROBOT_MODULE_COUNT\]\s*=\s*\{(.*?)\};",
            health_port,
            re.S,
        )
        self.assertIsNotNone(health_body)
        health_names = re.findall(r"VEHICLE_HEALTH_MODULE_([A-Z_]+)", health_body.group(1))
        self.assertEqual(c_names, health_names)

    def test_disabled_feature_sources_and_calls_are_excluded(self):
        for protocol in ("sp", "v2"):
            for heater_enabled in (False, True):
                with self.subTest(protocol=protocol, heater=heater_enabled), \
                        tempfile.TemporaryDirectory(prefix="mc02-0054-sources-") as directory:
                    temp = pathlib.Path(directory)
                    source_dir = temp / "source"
                    source_dir.mkdir()
                    build_dir = temp / "build"
                    project = f'''cmake_minimum_required(VERSION 3.22)
project(mc02_role_sources NONE)
# 让生产源码清单的文件存在检查指向仓库，同时把测试构建留在临时目录。
set(CMAKE_SOURCE_DIR "{ROOT.as_posix()}")
set(MC02_PROFILE robot)
set(MC02_BOARD_ROLE chassis)
set(MC02_CHASSIS_TYPE omni)
set(MC02_VISION_PROTOCOL {protocol})
set(MC02_FEATURE_CHASSIS TRUE)
set(MC02_FEATURE_VISION FALSE)
set(MC02_FEATURE_HEATER {str(heater_enabled).upper()})
include("{(ROOT / "cmake/mc02_firmware_sources.cmake").as_posix()}")
file(WRITE "${{CMAKE_BINARY_DIR}}/selected-sources.txt" "${{MC02_SELECTED_SOURCES}}")
'''
                    (source_dir / "CMakeLists.txt").write_text(project, encoding="utf-8")
                    result = subprocess.run(["cmake", "-S", str(source_dir), "-B", str(build_dir)],
                                            text=True, capture_output=True, timeout=30)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    sources = (build_dir / "selected-sources.txt").read_text(encoding="utf-8").split(";")
                    self.assertIn("modules/imu/ins_task.c", sources)
                    for vision_source in (
                        "modules/master_machine/master_process.c",
                        "modules/master_machine/vision_control.c",
                        "modules/master_machine/vision_protocol.c",
                        "modules/master_machine/vision_protocol_v2.c",
                    ):
                        self.assertNotIn(vision_source, sources)
                    self.assertEqual("modules/imu/imu_heater.c" in sources, heater_enabled)

        for feature, path, pattern in (
            ("VISION", "modules/imu/ins_task.c", r"\bVisionUpdate(?:Tx|IMUSample)\s*\("),
            ("VISION", "application/robot.c", r"\bVision(?:Init|Send)\s*\("),
            ("VISION", "application/vehicle_health_port_stm32.c", r"\bVisionGetSnapshot\s*\("),
            ("HEATER", "application/robot.c", r"\bIMUHeater(?:BeginLongOperation|EndLongOperation)\s*\("),
            ("HEATER", "application/robot_task.h", r"\bIMUHeaterForceOff\s*\("),
            ("HEATER", "application/vehicle_health_port_stm32.c", r"\bIMUHeaterGetStatus\s*\("),
            ("HEATER", "modules/imu/ins_task.c", r"\bIMUHeater(?:Init|Update|ForceOff)\s*\("),
            ("HEATER", "modules/BMI088/bmi088.c", r"\bIMUHeater(?:BeginLongOperation|EndLongOperation)\s*\("),
            ("HEATER", "modules/lcd/lcd_ui.c", r"\bIMUHeaterGetStatus\s*\("),
        ):
            with self.subTest(disabled_feature=feature, source=path):
                self.assert_feature_calls_disabled(path, pattern, feature)

    def test_graph(self):
        prefix = 'set(MC02_DEP_SAFETY "")\nset(MC02_DEP_IMU SAFETY)\n'
        self.assertEqual(self.run_case(prefix + 'mc02_validate_module_graph("SAFETY;IMU" "IMU")\n').returncode, 0)
        result = self.run_case(prefix + 'mc02_validate_module_graph("IMU" "IMU")\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("dependency", result.stderr)
        result = self.run_case(prefix + 'set(MC02_DEP_SAFETY IMU)\nmc02_validate_module_graph("SAFETY;IMU" "IMU")\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("cycle", result.stderr)

    def test_unconfirmed_core_rejected(self):
        result = self.run_case('mc02_require_production_core(robot FALSE FALSE)\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("APP-10", result.stderr)
        self.assertEqual(self.run_case('mc02_require_production_core(bench_safe FALSE FALSE)\n').returncode, 0)

    def test_manual_input_confirmation_for_command_boards(self):
        # 仅测试规则；这里的 TRUE 不代表哨兵映射已经过实车确认。
        for role in ("single", "gimbal"):
            result = self.run_case(f'mc02_require_manual_input(robot {role} FALSE)\n')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("manual input", result.stderr)
            self.assertEqual(self.run_case(f'mc02_require_manual_input(robot {role} TRUE)\n').returncode, 0)
        self.assertEqual(self.run_case('mc02_require_manual_input(robot chassis FALSE)\n').returncode, 0)
        self.assertEqual(self.run_case('mc02_require_manual_input(bench_safe single FALSE)\n').returncode, 0)
        self.assertNotEqual(self.run_case('mc02_require_manual_input(robot single UNKNOWN)\n').returncode, 0)

    def test_power_declaration_requires_confirmed_production_rails(self):
        # 裸板保持关闭，可在尚不清楚实车接线时编译。
        result = self.run_case('mc02_validate_power_plan(bench_safe UNCONFIRMED UNCONFIRMED UNCONFIRMED)\n')
        self.assertEqual(result.returncode, 0, result.stderr)
        # 测试声明只用于校验配置规则，不作为真实机器人供电参数。
        result = self.run_case('mc02_validate_power_plan(robot OFF BEFORE_DEVICES AFTER_DEVICES)\n')
        self.assertEqual(result.returncode, 0, result.stderr)
        # 任一路未确认都不能进入生产配置，非法阶段同样明确拒绝。
        for values in ('UNCONFIRMED OFF OFF', 'OFF UNCONFIRMED OFF',
                       'OFF OFF UNCONFIRMED', 'OFF UNKNOWN OFF', 'OFF OFF'):
            with self.subTest(values=values):
                result = self.run_case(f'mc02_validate_power_plan(robot {values})\n')
                self.assertNotEqual(result.returncode, 0)

    def test_power_consumers_and_board_selection(self):
        # 人造声明仅用于测试规则；实际哨兵三路负载仍未确认。
        prefix = 'set(declared "SAFETY;IMU;LCD")\nset(enabled "SAFETY;IMU")\n'
        check = ('if(NOT stage EQUAL 1 OR NOT mask EQUAL 2)\n'
                 '  message(FATAL_ERROR "unexpected selected rail")\nendif()\n')
        call = ('mc02_prepare_power_rail(stage mask robot rail BEFORE_DEVICES '
                '"${declared}" "${enabled}" "IMU;LCD;IMU")\n')
        result = self.run_case(prefix + call + check)
        self.assertEqual(result.returncode, 0, result.stderr)
        call = ('mc02_prepare_power_rail(stage mask robot rail AFTER_DEVICES '
                '"${declared}" "${enabled}" "IMU")\n')
        result = self.run_case(prefix + call +
                               'if(NOT stage EQUAL 2 OR NOT mask EQUAL 2)\n'
                               '  message(FATAL_ERROR "unexpected later stage")\nendif()\n')
        self.assertEqual(result.returncode, 0, result.stderr)
        # 本板没有使用者，以及所有裸板配置，均不产生开启请求。
        for profile, consumers in (('robot', 'LCD'), ('bench_safe', 'IMU')):
            with self.subTest(profile=profile, consumers=consumers):
                call = (f'mc02_prepare_power_rail(stage mask {profile} rail BEFORE_DEVICES '
                        f'"${{declared}}" "${{enabled}}" "{consumers}")\n')
                result = self.run_case(prefix + call +
                                       'if(NOT stage EQUAL 0 OR NOT mask EQUAL 0)\n'
                                       '  message(FATAL_ERROR "rail not disabled")\nendif()\n')
                self.assertEqual(result.returncode, 0, result.stderr)
        # 未声明模块、空负载或关闭电源却声明使用者都要拒绝。
        for stage, consumers in (('BEFORE_DEVICES', ''), ('OFF', 'IMU'),
                                 ('UNCONFIRMED', 'IMU'), ('AFTER_DEVICES', 'UNKNOWN')):
            with self.subTest(stage=stage, consumers=consumers):
                call = (f'mc02_prepare_power_rail(stage mask robot rail {stage} '
                        f'"${{declared}}" "${{enabled}}" "{consumers}")\n')
                self.assertNotEqual(self.run_case(prefix + call).returncode, 0)


if __name__ == "__main__":
    unittest.main()
