"""官方预设清单、产物命名和 CMake 漂移检查的回归。

测试只使用临时 JSON，不运行 CMake，也不修改仓库预设。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "workflow"))

import presets


class PresetTests(unittest.TestCase):
    def test_official_artifact_names(self) -> None:
        lookup = {preset.name: preset for preset in presets.OFFICIAL}
        self.assertEqual(
            presets.artifact_name(lookup["bench-debug"]),
            "Basic_Framework_MC02_bench_safe.elf",
        )
        self.assertEqual(
            presets.artifact_name(lookup["rcdiag-debug"]),
            "Basic_Framework_MC02_rcdiag_bench_safe.elf",
        )
        self.assertEqual(
            presets.artifact_name(lookup["robot-chassis-debug"]),
            "Basic_Framework_MC02_chassis_robot.elf",
        )
        self.assertEqual(
            presets.artifact_name(lookup["robot-gimbal-rel"]),
            "Basic_Framework_MC02_gimbal_robot.elf",
        )

    def test_profile_arguments_flag_rc_diagnostics(self) -> None:
        lookup = {preset.name: preset for preset in presets.OFFICIAL}
        arguments = presets.profile_arguments(
            lookup["rcdiag-debug"], Path("build"), Path("firmware.elf")
        )
        self.assertIn("--rc-diagnostics", arguments)
        plain = presets.profile_arguments(
            lookup["bench-debug"], Path("build"), Path("firmware.elf")
        )
        self.assertNotIn("--rc-diagnostics", plain)

    def test_github_matrix_has_ten_entries_with_artifacts(self) -> None:
        matrix = presets.github_matrix()
        self.assertEqual(len(matrix), 10)
        for row in matrix:
            self.assertTrue(row["artifact"].endswith(".elf"))

    def test_repository_cmake_matches_official_list(self) -> None:
        self.assertEqual(presets.check_cmake(), [])

    def test_cmake_drift_is_reported(self) -> None:
        data = json.loads((ROOT / "CMakePresets.json").read_text(encoding="utf-8"))
        for entry in data["configurePresets"]:
            if entry["name"] == "robot-chassis-debug":
                entry["cacheVariables"]["MC02_BOARD_ROLE"] = "gimbal"
        with tempfile.TemporaryDirectory(prefix="mc02-presets-") as directory:
            path = Path(directory) / "CMakePresets.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            errors = presets.check_cmake(path)
        self.assertTrue(any("MC02_BOARD_ROLE" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
