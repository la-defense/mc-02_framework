# 自瞄链路课程资料

## Knowledge

- [MC-02 SP 协议收发实现](../../../modules/master_machine/master_process.c)
  本机协议帧长度、CRC、接收计数和离线回调的实际实现；用于解释课程示例和核对协议行为。
- [学习笔记 16：USB CDC 虚拟串口通信](../../学习笔记/16-USB-CDC虚拟串口通信.md)
  本工程 CDC 端点、接收重装、忙状态和发送缓冲约定；用于 CDC 课程及安全审查。
- [STM32 USB Device Library 用户手册 UM1734](https://www.st.com/resource/en/user_manual/dm00108129-stm32cube-usb-device-library-stmicroelectronics.pdf)
  ST USB Device Library 的核心层、Class 层及 CDC 示例；用于区分设备库生命周期和应用层缓冲所有权。
- [Microsoft：在 WSL 2 中连接 USB 设备](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
  usbipd-win 共享、附加、验证和分离设备的官方步骤；用于原生 USB 直通路径。
- [Microsoft：WSL 手动安装与 WSL 2 要求](https://learn.microsoft.com/en-us/windows/wsl/install-manual)
  WSL 2 功能、系统版本与虚拟化前置条件；用于解释主机侧部署边界。
- [CMake CTest 命令参考](https://cmake.org/cmake/help/latest/manual/ctest.1.html)
  CTest 如何发现和运行测试；用于课程练习的主机验证流程。
- [Arm Cortex-M7 Devices Generic User Guide](https://developer.arm.com/documentation/dui0646/latest/)
  第 2.4 节说明异常基本/浮点扩展栈帧、EXC_RETURN 和压栈故障位；用于异常入口及现场读取规则。
- [ST RM0468：STM32H723/733 系列参考手册](https://www.st.com/resource/en/reference_manual/dm00603761.pdf)
  给出 H723 的片上 SRAM 区域与外设寄存器；用于将异常栈地址校验限定到本链接脚本映射的 SRAM。

## Wisdom (Communities)

- [ST Community](https://community.st.com/)
  用于查找 STM32 USB Device Library 的设备相关讨论；涉及具体版本行为时先核对 ST 手册和本工程 vendored 源码。
