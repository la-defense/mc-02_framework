"""验证可移植框架目标不会通过编译选项继承厂商或 RTOS 头路径。"""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/workflow"))
from check_firmware_profile import (  # noqa: E402
    check_portable_dependencies, check_portable_includes, find_link_command,
)


class PortableIncludeContractTest(unittest.TestCase):
    """以真实工作区路径检查允许与拒绝的编译选项。"""

    def entry(self, *arguments):
        """构造编译数据库项，不运行编译器或改写工作区。"""
        return {"directory": str(ROOT), "arguments": ["cc", *arguments]}

    def test_framework_paths_are_accepted(self):
        # 分离和拼接两种写法都应识别为已允许的框架目录。
        entry = self.entry("-Imodules/daemon", "-I", "bsp/os", "-isystem", "bsp/log")
        check_portable_includes(entry, ROOT, "modules/daemon/daemon.c")
        # 安全模块的公开状态定义也可进入框架目标。
        check_portable_includes(self.entry("-Imodules/safety", "-Ibsp/os", "-Ibsp/log"),
                                ROOT, "modules/safety/robot_safety.c")
        # 板间链路可访问逻辑 CAN、时间映射和 CRC 的接口目录。
        check_portable_includes(self.entry("-Ibsp/can", "-Imodules/can_comm",
                                           "-Imodules/time_sync", "-Imodules/algorithm"),
                                ROOT, "modules/can_comm/can_topic_link.c")
        # DJI 控制只能访问自有电机接口与公共单位定义。
        check_portable_includes(self.entry("-Imodules", "-Imodules/motor", "-Ibsp/can"),
                                ROOT, "modules/motor/DJImotor/dji_motor.c")

    def test_vendor_and_rtos_paths_are_rejected(self):
        # 同时覆盖拼接、分离和系统头参数，防止替换选项形式后漏检。
        for flags in (("-IDrivers/CMSIS/Include",),
                      ("-I", "Middlewares/Third_Party/FreeRTOS/Source/include"),
                      ("-isystem", "Core/Inc"),
                      ("-isystemCore/Inc",), ("-I",)):
            with self.subTest(flags=flags), self.assertRaises(SystemExit):
                check_portable_includes(self.entry(*flags), ROOT, "modules/daemon/daemon.c")

    def test_forced_include_is_rejected(self):
        # 隐式强制包含也可能把厂商定义带入，应明确拒绝。
        with self.assertRaises(SystemExit):
            check_portable_includes(self.entry("-include", "Core/Inc/main.h"), ROOT,
                                    "modules/daemon/daemon.c")

    def test_actual_project_dependencies_are_checked(self):
        # 编译选项合规时仍须检查实际头文件，阻止相对路径绕过允许列表。
        source = "modules/daemon/daemon.c"
        valid = f"object: #deps 2, deps mtime 1 (VALID)\n    {ROOT / source}\n    /usr/include/stdint.h\n"
        check_portable_dependencies(valid, ROOT, ROOT / "build", source)
        # 框架的逻辑资源编号可从板级目录单独引用，不能因此放行板级适配头。
        check_portable_dependencies(valid + f"    {ROOT / 'bsp/board/platform_resources.h'}\n",
                                    ROOT, ROOT / "build", source)
        check_portable_dependencies(valid + f"    {ROOT / 'bsp/board/platform_analog.h'}\n",
                                    ROOT, ROOT / "build", source)
        # 电机公开类型可以使用，但依赖链仍不放行外设实现头。
        check_portable_dependencies(valid + f"    {ROOT / 'modules/motor/motor_def.h'}\n",
                                    ROOT, ROOT / "build", source)
        # 上位机 V2 只引用整车话题编号声明，不放行整个 application 目录。
        check_portable_dependencies(valid + f"    {ROOT / 'application/topics/robot_topics.h'}\n",
                                    ROOT, ROOT / "build", source)
        with self.assertRaises(SystemExit):
            check_portable_dependencies(valid + f"    {ROOT / 'application/robot_def.h'}\n",
                                        ROOT, ROOT / "build", source)
        # 任务创建只允许依赖模块选择和初始化诊断的公开声明。
        check_portable_dependencies(valid + f"    {ROOT / 'application/config/vehicle_selection.h'}\n"
                                    f"    {ROOT / 'application/config/module_plan.h'}\n"
                                    f"    {ROOT / 'application/init_diagnostic.h'}\n",
                                    ROOT, ROOT / "build", source)
        # 姿态任务可使用样本类型及短时探针声明，寄存器头仍不可带入。
        check_portable_dependencies(valid + f"    {ROOT / 'modules/BMI088/bmi088_data.h'}\n"
                                    f"    {ROOT / 'bsp/dwt/platform_probe.h'}\n"
                                    f"    {ROOT / 'build/generated/mc02_build_config.h'}\n",
                                    ROOT, ROOT / "build", source)
        with self.assertRaises(SystemExit):
            check_portable_dependencies(valid + f"    {ROOT / 'bsp/dwt/bsp_dwt.h'}\n",
                                        ROOT, ROOT / "build", source)
        with self.assertRaises(SystemExit):
            check_portable_dependencies(valid + f"    {ROOT / 'bsp/board/board_uart_stm32.h'}\n",
                                        ROOT, ROOT / "build", source)
        with self.assertRaises(SystemExit):
            check_portable_dependencies(valid + f"    {ROOT / 'Core/Inc/main.h'}\n",
                                        ROOT, ROOT / "build", source)
        # 可访问 CAN 框架目录也不允许间接引用芯片头。
        with self.assertRaises(SystemExit):
            check_portable_dependencies(valid + f"    {ROOT / 'Drivers/CMSIS/Include/core_cm7.h'}\n",
                                        ROOT, ROOT / "build", source)
        with self.assertRaises(SystemExit):
            check_portable_dependencies("object: deps not found\n", ROOT, ROOT / "build", source)

    def test_link_check_uses_final_link_not_compile_command(self):
        # 只编译了某对象并不表示它进入最终固件，检查必须选中 ELF 链接行。
        commands = "cc -c module.c -o module.c.obj\ncc other.c.obj -o firmware.elf\n"
        link = find_link_command(commands, "firmware.elf")
        self.assertNotIn("module.c.obj", link)
        with self.assertRaises(SystemExit):
            find_link_command(commands, "missing.elf")


if __name__ == "__main__":
    unittest.main()
