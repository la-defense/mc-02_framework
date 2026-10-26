"""固件归档身份、固定文件集合和校验和回归。

测试在临时构建目录生成最小 ELF 产物模型，不连接硬件，也不修改实际固件构建目录。
作者：MC-02 contributors；许可：BSD-3-Clause，详见仓库 LICENSE。
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "workflow"))

import archive_firmware


class FirmwareArchiveTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="mc02-firmware-archive-")
        self.root = Path(self.temporary.name)
        self.build_dir = self.root / "build" / "wsl" / "bench-debug"
        self.build_dir.mkdir(parents=True)
        (self.root / "cmake").mkdir()
        (self.root / "cmake" / "arm-none-eabi-toolchain.cmake").write_text("toolchain fixture\n")
        (self.root / "CMakePresets.json").write_text(json.dumps({
            "version": 3,
            "configurePresets": [
                {"name": "arm-base", "hidden": True, "generator": "Ninja",
                 "cacheVariables": {"MC02_PROFILE": "bench_safe", "MC02_BOARD_ROLE": "single",
                                    "MC02_VISION_TRANSPORT": "usb", "MC02_ENABLE_HEATER": False,
                                    "MC02_BENCH_RC_DIAGNOSTICS": False}},
                {"name": "bench-debug", "inherits": "arm-base",
                 "cacheVariables": {"CMAKE_BUILD_TYPE": "Debug"}},
            ],
            "buildPresets": [{"name": "bench-debug", "configurePreset": "bench-debug"}],
        }), encoding="utf-8")
        (self.build_dir / "CMakeCache.txt").write_text("\n".join((
            f"CMAKE_HOME_DIRECTORY:INTERNAL={self.root}",
            "CMAKE_GENERATOR:INTERNAL=Ninja",
            "CMAKE_BUILD_TYPE:STRING=Debug",
            f"CMAKE_TOOLCHAIN_FILE:FILEPATH={self.root / 'cmake' / 'arm-none-eabi-toolchain.cmake'}",
            "CMAKE_C_COMPILER:FILEPATH=arm-none-eabi-gcc",
            "CMAKE_C_COMPILER_ID:STRING=GNU",
            "CMAKE_C_COMPILER_VERSION:STRING=10.3.1",
            "MC02_PROFILE:STRING=bench_safe",
            "MC02_BOARD_ROLE:STRING=single",
            "MC02_VISION_TRANSPORT:STRING=usb",
            "MC02_VISION_PROTOCOL:STRING=sp",
            "MC02_IMU_BACKEND:STRING=quaternion_ekf",
            "MC02_ENABLE_HEATER:BOOL=OFF",
            "MC02_ENABLE_REFEREE_UI:BOOL=OFF",
            "MC02_BENCH_RC_DIAGNOSTICS:BOOL=OFF",
        )) + "\n", encoding="utf-8")
        (self.build_dir / "mc02-selected-sources.txt").write_text(
            "profile=bench_safe\nrole=single\nvision=usb\nvision_protocol=sp\n"
            "imu_backend=quaternion_ekf\ntarget=Basic_Framework_MC02_bench_safe.elf\n",
            encoding="utf-8")
        for suffix in (".elf", ".hex", ".bin", ".map"):
            (self.build_dir / f"Basic_Framework_MC02_bench_safe{suffix}").write_bytes(
                f"fixture-{suffix}\n".encode("ascii"))
        self.output_dir = self.root / "out"
        self.build_id = "0123456789abcdef" * 2 + "01234567"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def command_runner(self, arguments: list[str], _cwd: Path | None = None) -> str:
        if arguments[0] == "arm-none-eabi-readelf":
            return f"Displaying notes found in: .note.gnu.build-id\nBuild ID: {self.build_id}\n"
        if arguments[-1:] == ["--version"]:
            return f"{Path(arguments[0]).name} test version 1.0\n"
        self.fail(f"测试遇到未预期的命令：{arguments}")

    @staticmethod
    def git_reader(_root: Path) -> dict[str, object]:
        return {"commit": "a" * 40, "commit_time": 1700000000,
                "tracked_changes": False, "tracked_change_count": 0,
                "tracked_change_paths": [], "tracked_change_paths_sha256": None}

    def package(self) -> Path:
        return archive_firmware.create_archive(
            self.root, "bench-debug", self.build_dir, self.output_dir,
            command_runner=self.command_runner, git_reader=self.git_reader)

    def test_archive_contains_matching_identity_and_reproducible_checksums(self) -> None:
        archive_path = self.package()
        self.assertTrue(archive_path.is_file())
        with zipfile.ZipFile(archive_path) as archive:
            names = set(archive.namelist())
            self.assertEqual(names, {
                "Basic_Framework_MC02_bench_safe.elf",
                "Basic_Framework_MC02_bench_safe.hex",
                "Basic_Framework_MC02_bench_safe.bin",
                "Basic_Framework_MC02_bench_safe.map",
                "build-info.json", "SHA256SUMS",
            })
            manifest_bytes = archive.read("build-info.json")
            manifest = json.loads(manifest_bytes)
            self.assertEqual(manifest["source"]["commit"], "a" * 40)
            self.assertEqual(manifest["build"]["identity"]["profile"], "bench_safe")
            self.assertEqual(manifest["build"]["identity"]["heater_enabled"], False)
            self.assertEqual(manifest["firmware"]["gnu_build_id"], self.build_id)
            self.assertNotIn(str(self.root), manifest_bytes.decode("utf-8"))
            self.assertIn("build-info.json", archive.read("SHA256SUMS").decode("ascii"))
        self.assertEqual(self.package(), archive_path)

    def test_existing_different_archive_is_not_overwritten(self) -> None:
        archive_path = self.package()
        archive_path.write_bytes(b"previous archive")
        with self.assertRaisesRegex(archive_firmware.ArchiveError, "拒绝覆盖"):
            self.package()
        self.assertEqual(archive_path.read_bytes(), b"previous archive")

    def test_missing_file_is_reported_without_creating_archive(self) -> None:
        (self.build_dir / "Basic_Framework_MC02_bench_safe.map").unlink()
        with self.assertRaisesRegex(archive_firmware.ArchiveError, "缺少固件归档产物"):
            self.package()
        self.assertFalse(self.output_dir.exists())

    def test_configuration_mismatch_is_rejected(self) -> None:
        cache = self.build_dir / "CMakeCache.txt"
        cache.write_text(cache.read_text(encoding="utf-8").replace(
            "MC02_ENABLE_HEATER:BOOL=OFF", "MC02_ENABLE_HEATER:BOOL=ON"), encoding="utf-8")
        with self.assertRaisesRegex(archive_firmware.ArchiveError, "预设不符|拒绝归档加热器"):
            self.package()
        self.assertFalse(self.output_dir.exists())

    def test_build_type_must_match_selected_preset(self) -> None:
        cache = self.build_dir / "CMakeCache.txt"
        cache.write_text(cache.read_text(encoding="utf-8").replace(
            "CMAKE_BUILD_TYPE:STRING=Debug", "CMAKE_BUILD_TYPE:STRING=RelWithDebInfo"), encoding="utf-8")
        with self.assertRaisesRegex(archive_firmware.ArchiveError, "预设不符"):
            self.package()

    def test_invalid_build_id_is_rejected(self) -> None:
        def no_identity(arguments: list[str], _cwd: Path | None = None) -> str:
            if arguments[0] == "arm-none-eabi-readelf":
                return "ELF header without GNU Build ID\n"
            return self.command_runner(arguments, _cwd)

        with self.assertRaisesRegex(archive_firmware.ArchiveError, "唯一的20字节 GNU Build ID"):
            archive_firmware.create_archive(
                self.root, "bench-debug", self.build_dir, self.output_dir,
                command_runner=no_identity, git_reader=self.git_reader)
        self.assertFalse(self.output_dir.exists())

    def test_non_arm_compiler_is_rejected(self) -> None:
        cache = self.build_dir / "CMakeCache.txt"
        cache.write_text(cache.read_text(encoding="utf-8").replace(
            "CMAKE_C_COMPILER:FILEPATH=arm-none-eabi-gcc",
            "CMAKE_C_COMPILER:FILEPATH=gcc"), encoding="utf-8")
        with self.assertRaisesRegex(archive_firmware.ArchiveError, "只接受 arm-none-eabi-gcc"):
            self.package()
        self.assertFalse(self.output_dir.exists())


if __name__ == "__main__":
    unittest.main()
