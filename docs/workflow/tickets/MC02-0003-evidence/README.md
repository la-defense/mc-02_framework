# MC02-0003：基础人工输入适配验证

本记录针对标准命令接口首个软件切片，不替代 TODO 状态。

- WSL CMake Debug／Release 定向 CTest `production_standard_cmd_manual_and_events` 各1/1通过；测试使用显式检查，Release下仍生效。
- GCC ASan／UBSan直接编译同一实现与测试并执行通过。
- ARM GCC以C11、Cortex-M7、`fpv5-d16`、硬浮点参数独立编译 `std_cmd.c` 通过，未提供HAL或RTOS头路径。
- 覆盖SI换算、正负方向、输出幅值、来源期限取小、边界与回绕、将来时刻、非法枚举／参照／NaN／原始越界／乘法溢出、失败保留整份输出，以及事件重复、回退、回绕和跨来源独立消费。

原始构建目录为 `/tmp/mc02-sentinel-debug`、`/tmp/mc02-sentinel-release`，sanitizer与ARM对象分别为 `/tmp/mc02-std-cmd-san`、`/tmp/mc02-std-cmd-arm.o`。这些是可重新生成的本机产物。

未接入 RobotCMD，未链接进默认固件，未连接硬件，未验证实际机械方向、时间误差和停止时延。完整主机回归与相关ARM固件链接在调用方接线后执行；本记录不能作为MC02-0003全部验收通过的依据。

## 遥控来源时间接线准备

`RemoteControlGetSnapshot()`追加无线来源时刻／本机样本号及键鼠来源时刻／本机样本号，与原数据和有效标记在同一临界区复制。32位样本号跳过保留零，只在模块初始化时归零；键鼠心跳重复、回退或非法帧不推进相应样本号。

WSL Debug／Release 的遥控相关定向用例各14/14、键鼠相关各7/7通过，另重跑标准命令接口各1/1通过。覆盖延迟派发保留接收时间、重复读取不续期、重复键鼠心跳仅无线更新、非法帧不更新，以及已有并发读取／键鼠租约／命令联测。

最初定向选择包含尚未构建的 `pc_vision_integration_test`，导致一项Not Run；补齐该目标后7/7通过。临时构建目录在后续WSL会话中已不存在，最终改用持久的 `/home/mc02/build/sentinel-debug` 与 `sentinel-release`，原始配置／构建日志位于 `build/sentinel-*-config.log`、`build/sentinel-*-build.log`。

遥控诊断禁热 Debug／RelWithDebInfo 固件构建及源码／ELF隔离检查通过；GNU size 的text分别为254548／242616 B、data均900 B、bss为153144／153136 B。与本切片前同工具链产物相比，text各增96 B、bss各增32 B；这不是运行时栈测量。本次未上板。
