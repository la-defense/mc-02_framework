# MC02-0130 CAN 工作区链接记录

对照 `27441a1` 的 ARM Debug 隔离构建和 [MC02-0055 链接记录](../MC02-0055-evidence/README.md#独立控制任务与共享唤醒资源)。正式单板保留 LCD 和裁判 UI，只有隔离源码副本启用测试用的车型构建确认。正式车型源码仍保留原确认条件。

主 RAM 超限时，map 文件中 `rx_lookup` 占 6144 B。它是 CPU 使用的接收索引，与 FDCAN 外设的过滤器和 FIFO 分开。将该表放入板上原有的 `.can_workspace` 后，BSP 在初始化标志发布前清零这段 `NOLOAD` 工作区中的队列和索引，已存在的链接断言继续检查 RAM_D2 容量。

默认单板隔离 Debug 构建的主 RAM 从 136176 B 降到 130032 B，余量为 1040 B。RAM_D2 从 17408 B 增到 23552 B，Flash 从 332944 B 增到 332960 B。以上是链接占用，运行时任务栈和 CAN 中断耗时留待板上测量。

默认单板与受限底盘两个隔离构建均通过 `check_firmware_profile.py` 的源码和 ELF 检查。`arm-none-eabi-objdump -t` 显示两版 `rx_lookup` 位于 `.can_workspace` 的 `0x30000000`，`can_bus` 位于 `0x30001800`。受限底盘主 RAM 为 119888 B，较迁移前少 6144 B。两版测试固件均未烧录。

主机 ASan／UBSan 运行的 72 项 CAN 相关测试全部通过，涵盖早期中断、接收和注册、总线隔离及故障恢复等路径。主机版本依旧由运行库初始化静态内存，目标机上的首次清零和实际时序仍需现场验证。

`verify.py --fast --host --firmware --presets bench-debug,rcdiag-debug --parallel 2` 通过。主机 Debug 和 Release 各通过 849 项。普通禁热及遥控诊断固件通过 ARM 构建、源码和 ELF 检查，主 RAM 分别为 115576 B 和 116088 B，均少 6144 B。六种未确认车型的生产预设仍按预期拒绝。
