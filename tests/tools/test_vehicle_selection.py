"""核对生产CMake配置入口拒绝非法选择和缺失核心条件。"""
import pathlib
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
