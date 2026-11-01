"""核对车型选择、模块映射和禁用模块的生产源码隔离。"""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


class VehicleSelectionTest(unittest.TestCase):
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

    def test_disabled_vision_sources_and_ins_calls_are_excluded(self):
        for protocol in ("sp", "v2"):
            with self.subTest(protocol=protocol), tempfile.TemporaryDirectory(prefix="mc02-0054-sources-") as directory:
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

        ins_source = (ROOT / "modules/imu/ins_task.c").read_text(encoding="utf-8")
        conditions = []
        for line in ins_source.splitlines():
            directive = re.match(r"\s*#\s*(if|ifdef|ifndef|endif)\b(.*)", line)
            if directive:
                kind, expression = directive.groups()
                if kind == "endif":
                    if conditions:
                        conditions.pop()
                else:
                    conditions.append(expression)
            stripped = line.lstrip()
            if (not stripped.startswith(("//", "/*", "*")) and
                    re.search(r"\bVisionUpdate(?:Tx|IMUSample)\s*\(", line)):
                self.assertTrue(any("MC02_FEATURE_VISION" in condition for condition in conditions), line)

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


if __name__ == "__main__":
    unittest.main()
