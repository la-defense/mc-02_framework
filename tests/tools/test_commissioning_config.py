#!/usr/bin/env python3
"""检查受限观察固件的配置身份和运动入口拒绝规则。"""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(all(shutil.which(tool) for tool in ("cmake", "ninja", "arm-none-eabi-gcc")),
                     "需要 CMake、Ninja 和 ARM GCC")
class CommissioningConfigTest(unittest.TestCase):
    """使用独立临时构建目录，不修改生产车型确认数据。"""

    def test_observation_and_rejections(self):
        with tempfile.TemporaryDirectory(prefix="mc02-commissioning-") as temp:
            build = Path(temp) / "build"
            base = ["cmake", "--preset", "bench-debug", "-B", str(build),
                    "-DMC02_PROFILE=commissioning"]
            # 观察配置必须能生成，并把身份和禁止输出的选择写入生成头。
            result = subprocess.run(base + ["-DMC02_BENCH_RC_DIAGNOSTICS=ON"], cwd=ROOT,
                                    capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            header = (build / "generated/mc02_build_config.h").read_text(encoding="utf-8")
            self.assertIn('#define MC02_BUILD_ROLE_NAME "commissioning"', header)
            for feature in ("COMMAND", "CHASSIS", "GIMBAL", "SHOOTER", "HEATER", "MOTOR_CONTROL"):
                self.assertIn(f"#define MC02_FEATURE_{feature} 0", header)
            for rail in ("24V_1", "24V_2", "5V"):
                self.assertIn(f"#define MC02_POWER_{rail}_STAGE 0u", header)
            sources = (build / "mc02-selected-sources.txt").read_text(encoding="utf-8")
            self.assertIn("target=Basic_Framework_MC02_commissioning_observe_rcdiag.elf", sources)
            self.assertNotIn("application/chassis/chassis.c", sources)
            self.assertNotIn("application/gimbal/gimbal.c", sources)
            self.assertNotIn("application/shoot/shoot.c", sources)
            self.assertNotIn("modules/motor/DJImotor/dji_motor.c", sources)
            # 逐个检查运动请求和无效参数，不能沿用上一次成功的配置绕过拒绝。
            for options, reason in (
                (["-DMC02_COMMISSIONING_TARGET=chassis"], "Commissioning motion requires"),
                (["-DMC02_COMMISSIONING_TARGET=gimbal"], "Commissioning motion requires"),
                (["-DMC02_COMMISSIONING_TARGET=both"], "Commissioning motion requires"),
                (["-DMC02_COMMISSIONING_TARGET=invalid"], "Commissioning target is invalid"),
                (["-DMC02_COMMISSIONING_TARGET=observe", "-DMC02_ENABLE_HEATER=ON"], "IMU"),
                (["-DMC02_COMMISSIONING_TARGET=observe", "-DMC02_ENABLE_HEATER=OFF",
                  "-DMC02_BOARD_ROLE=chassis"], "single MC-02"),
            ):
                with self.subTest(options=options):
                    result = subprocess.run(base + options, cwd=ROOT, capture_output=True,
                                            text=True, timeout=90)
                    self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn(reason, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
