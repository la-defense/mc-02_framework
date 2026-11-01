"""核对生产CMake配置入口拒绝非法选择和缺失核心条件。"""
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
