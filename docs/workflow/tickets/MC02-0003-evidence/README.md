# MC02-0003：基础人工输入适配验证

本记录针对标准命令接口首个软件切片，不替代 TODO 状态。

- WSL CMake Debug／Release 定向 CTest `production_standard_cmd_manual_and_events` 各1/1通过；测试使用显式检查，Release下仍生效。
- GCC ASan／UBSan直接编译同一实现与测试并执行通过。
- ARM GCC以C11、Cortex-M7、`fpv5-d16`、硬浮点参数独立编译 `std_cmd.c` 通过，未提供HAL或RTOS头路径。
- 覆盖SI换算、正负方向、输出幅值、来源期限取小、边界与回绕、将来时刻、非法枚举／参照／NaN／原始越界／乘法溢出、失败保留整份输出，以及事件重复、回退、回绕和跨来源独立消费。

原始构建目录为 `/tmp/mc02-sentinel-debug`、`/tmp/mc02-sentinel-release`，sanitizer与ARM对象分别为 `/tmp/mc02-std-cmd-san`、`/tmp/mc02-std-cmd-arm.o`。这些是可重新生成的本机产物。

未接入 RobotCMD，未链接进默认固件，未连接硬件，未验证实际机械方向、时间误差和停止时延。完整主机回归与相关ARM固件链接在调用方接线后执行；本记录不能作为MC02-0003全部验收通过的依据。
